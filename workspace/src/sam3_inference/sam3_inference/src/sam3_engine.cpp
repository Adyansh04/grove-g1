#include "sam3_inference/sam3_engine.hpp"

#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>
#include <driver_types.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <format>
#include <fstream>
#include <ios>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <opencv2/imgproc.hpp>
#include <ratio>
#include <source_location>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "sam3_inference/clip_tokenizer.hpp"
#include "sam3_inference/tensor_types.hpp"

namespace sam3_inference
{
namespace
{

constexpr int         kInputSizePx   = 1008;
constexpr std::size_t kContextLength = 32;

using Clock = std::chrono::steady_clock;

double millisecondsSince(Clock::time_point start)
{
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

class Logger : public nvinfer1::ILogger
{
public:
    void log(Severity severity, const char* message) noexcept override
    {
        if (severity <= Severity::kWARNING)
        {
            std::cerr << std::format("[TensorRT] {}\n", message);
        }
    }
};

// Throws for a failed CUDA call, naming the call and where it was made.
void check(
    cudaError_t status, std::string_view what,
    std::source_location where = std::source_location::current())
{
    if (status != cudaSuccess)
    {
        throw CudaFailure(
            std::format(
                "{} ({}:{}): {}",
                what,
                std::filesystem::path(where.file_name()).filename().string(),
                where.line(),
                cudaGetErrorString(status)),
            status == cudaErrorMemoryAllocation);
    }
}

// TensorRT's factories return null on failure, for a missing GPU too.
template <typename T>
T* nonNull(T* created, std::string_view call)
{
    if (created == nullptr)
    {
        throw std::runtime_error(
            std::format("{} failed: no usable GPU, or a driver too old for this CUDA?", call));
    }
    return created;
}

ElementType elementType(nvinfer1::DataType type, const std::string& tensor)
{
    switch (type)
    {
        case nvinfer1::DataType::kFLOAT:
            return ElementType::kFloat32;
        case nvinfer1::DataType::kBF16:
            return ElementType::kBfloat16;
        case nvinfer1::DataType::kINT32:
            return ElementType::kInt32;
        case nvinfer1::DataType::kINT64:
            return ElementType::kInt64;
        case nvinfer1::DataType::kINT8:
            return ElementType::kInt8;
        case nvinfer1::DataType::kUINT8:
            return ElementType::kUint8;
        case nvinfer1::DataType::kBOOL:
            return ElementType::kBool;
        default:
            throw std::runtime_error(std::format(
                "Tensor {} has the unsupported TensorRT type {}",
                tensor,
                static_cast<int>(type)));
    }
}

class DeviceBuffer
{
public:
    DeviceBuffer() = default;
    DeviceBuffer(std::size_t bytes, std::string_view label)
      : bytes_(bytes)
    {
        check(cudaMalloc(&ptr_, bytes), std::format("cudaMalloc of {} ({} bytes)", label, bytes));
    }
    DeviceBuffer(DeviceBuffer&& other) noexcept
      : ptr_(std::exchange(other.ptr_, nullptr))
      , bytes_(std::exchange(other.bytes_, 0))
    {}
    DeviceBuffer& operator=(DeviceBuffer&& other) noexcept
    {
        std::swap(ptr_, other.ptr_);
        std::swap(bytes_, other.bytes_);
        return *this;
    }
    DeviceBuffer(const DeviceBuffer&)            = delete;
    DeviceBuffer& operator=(const DeviceBuffer&) = delete;
    ~DeviceBuffer()
    {
        if (ptr_ != nullptr && cudaFree(ptr_) != cudaSuccess)
        {
            std::cerr << "[sam3] cudaFree failed\n";
        }
    }
    [[nodiscard]] void*       get() const noexcept { return ptr_; }
    [[nodiscard]] std::size_t bytes() const noexcept { return bytes_; }

private:
    void*       ptr_{ nullptr };
    std::size_t bytes_{ 0 };
};

// Page-locked host memory, so copies to and from the GPU run asynchronously.
class PinnedBuffer
{
public:
    PinnedBuffer() = default;
    PinnedBuffer(std::size_t bytes, std::string_view label)
      : bytes_(bytes)
    {
        check(
            cudaMallocHost(&ptr_, bytes),
            std::format("cudaMallocHost of {} ({} bytes)", label, bytes));
    }
    PinnedBuffer(PinnedBuffer&& other) noexcept
      : ptr_(std::exchange(other.ptr_, nullptr))
      , bytes_(std::exchange(other.bytes_, 0))
    {}
    PinnedBuffer& operator=(PinnedBuffer&& other) noexcept
    {
        std::swap(ptr_, other.ptr_);
        std::swap(bytes_, other.bytes_);
        return *this;
    }
    PinnedBuffer(const PinnedBuffer&)            = delete;
    PinnedBuffer& operator=(const PinnedBuffer&) = delete;
    ~PinnedBuffer()
    {
        if (ptr_ != nullptr && cudaFreeHost(ptr_) != cudaSuccess)
        {
            std::cerr << "[sam3] cudaFreeHost failed\n";
        }
    }
    [[nodiscard]] void*       get() const noexcept { return ptr_; }
    [[nodiscard]] std::size_t bytes() const noexcept { return bytes_; }

private:
    void*       ptr_{ nullptr };
    std::size_t bytes_{ 0 };
};

class CudaStream
{
public:
    CudaStream() { check(cudaStreamCreate(&stream_), "cudaStreamCreate"); }
    CudaStream(const CudaStream&)            = delete;
    CudaStream& operator=(const CudaStream&) = delete;
    CudaStream(CudaStream&&)                 = delete;
    CudaStream& operator=(CudaStream&&)      = delete;
    ~CudaStream()
    {
        if (stream_ != nullptr && cudaStreamDestroy(stream_) != cudaSuccess)
        {
            std::cerr << "[sam3] cudaStreamDestroy failed\n";
        }
    }
    [[nodiscard]] cudaStream_t get() const noexcept { return stream_; }

private:
    cudaStream_t stream_{ nullptr };
};

std::vector<char> readFile(const std::filesystem::path& path)
{
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    if (!in)
    {
        throw std::runtime_error(std::format("Cannot open {}", path.string()));
    }
    std::vector<char> data(static_cast<std::size_t>(in.tellg()));
    in.seekg(0);
    in.read(data.data(), static_cast<std::streamsize>(data.size()));
    return data;
}

// What an engine is built from: its graph's size and time and the TensorRT version.
std::string sourceStamp(const std::filesystem::path& onnx)
{
    return std::format(
        "{} bytes, written {}, TensorRT {}",
        std::filesystem::file_size(onnx),
        std::filesystem::last_write_time(onnx).time_since_epoch().count(),
        getInferLibVersion());
}

std::filesystem::path sourcePath(const std::filesystem::path& engine_path)
{
    return engine_path.string() + ".source";
}

std::string readSource(const std::filesystem::path& engine_path)
{
    std::ifstream in(sourcePath(engine_path));
    std::string   stamp;
    std::getline(in, stamp);
    return stamp;
}

void buildEngine(
    Logger& logger, const EngineConfig& config, const std::filesystem::path& onnx,
    const std::filesystem::path& engine_path)
{
    std::cerr << std::format(
        "[sam3] building {} from {} (minutes, once per GPU)\n",
        engine_path.filename().string(),
        onnx.string());
    const std::unique_ptr<nvinfer1::IBuilder> builder(
        nonNull(nvinfer1::createInferBuilder(logger), "createInferBuilder"));
#if NV_TENSORRT_MAJOR < 11
    // TensorRT 10 (JetPack 6) builds a weakly typed network unless told, and would recast the
    // graphs' bf16 layers as it saw fit; 11 keeps the types the graph has.
    constexpr std::uint32_t kNetworkFlags =
        1U << static_cast<std::uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kSTRONGLY_TYPED);
#else
    constexpr std::uint32_t kNetworkFlags = 0;
#endif
    const std::unique_ptr<nvinfer1::INetworkDefinition> network(
        nonNull(builder->createNetworkV2(kNetworkFlags), "createNetworkV2"));
    const std::unique_ptr<nvonnxparser::IParser> parser(
        nonNull(nvonnxparser::createParser(*network, logger), "createParser"));
    if (!parser->parseFromFile(onnx.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kWARNING)))
    {
        std::string errors;
        for (int i = 0; i < parser->getNbErrors(); ++i)
        {
            errors += std::format("\n  {}", parser->getError(i)->desc());
        }
        throw std::runtime_error(std::format("Cannot parse {}{}", onnx.string(), errors));
    }
    const std::unique_ptr<nvinfer1::IBuilderConfig> build_config(
        nonNull(builder->createBuilderConfig(), "createBuilderConfig"));
    build_config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, config.workspace_bytes);
    const std::unique_ptr<nvinfer1::IHostMemory> serialized(
        builder->buildSerializedNetwork(*network, *build_config));
    if (!serialized)
    {
        throw std::runtime_error(std::format("TensorRT could not build {}", onnx.string()));
    }
    std::filesystem::create_directories(engine_path.parent_path());
    // Written beside its final name and renamed: an interrupted write never looks built.
    const std::filesystem::path partial = engine_path.string() + ".tmp";
    {
        std::ofstream out(partial, std::ios::binary);
        out.write(
            static_cast<const char*>(serialized->data()),
            static_cast<std::streamsize>(serialized->size()));
        out.close();
        if (!out)
        {
            std::error_code ignored;
            std::filesystem::remove(partial, ignored);
            throw std::runtime_error(std::format("Cannot write {}", engine_path.string()));
        }
    }
    std::filesystem::rename(partial, engine_path);
    // Without it the next start builds again, which is slow but correct.
    std::ofstream(sourcePath(engine_path)) << sourceStamp(onnx) << '\n';
}

// One engine with its context and the buffers it owns, by tensor name.
struct Graph
{
    std::string                                  name;
    std::unique_ptr<nvinfer1::ICudaEngine>       engine;
    std::unique_ptr<nvinfer1::IExecutionContext> context;
    std::map<std::string, DeviceBuffer>          owned;

    [[nodiscard]] bool has(const std::string& tensor) const
    {
        const std::vector<std::string> all = names();
        return std::ranges::find(all, tensor) != all.end();
    }

    [[nodiscard]] std::size_t count(const std::string& tensor) const
    {
        const nvinfer1::Dims dims = engine->getTensorShape(tensor.c_str());
        std::size_t          n    = 1;
        for (int i = 0; i < dims.nbDims; ++i)
        {
            n *= static_cast<std::size_t>(std::max<std::int64_t>(dims.d[i], 1));
        }
        return n;
    }

    [[nodiscard]] ElementType type(const std::string& tensor) const
    {
        return elementType(engine->getTensorDataType(tensor.c_str()), tensor);
    }

    [[nodiscard]] std::size_t bytes(const std::string& tensor) const
    {
        return count(tensor) * elementSize(type(tensor));
    }

    [[nodiscard]] std::vector<std::string> names() const
    {
        std::vector<std::string> out;
        out.reserve(static_cast<std::size_t>(engine->getNbIOTensors()));
        for (int i = 0; i < engine->getNbIOTensors(); ++i)
        {
            out.emplace_back(engine->getIOTensorName(i));
        }
        return out;
    }

    // Allocates the tensors not listed in `shared` and binds them.
    void allocate(const std::vector<std::string>& shared)
    {
        for (const std::string& tensor : names())
        {
            if (std::ranges::find(shared, tensor) != shared.end())
            {
                continue;
            }
            DeviceBuffer buffer(bytes(tensor), name + "/" + tensor);
            bind(tensor, buffer.get());
            owned.emplace(tensor, std::move(buffer));
        }
    }

    void bind(const std::string& tensor, void* address) const
    {
        if (!context->setTensorAddress(tensor.c_str(), address))
        {
            throw std::runtime_error(std::format("Cannot bind tensor {}/{}", name, tensor));
        }
    }

    [[nodiscard]] void* at(const std::string& tensor) const
    {
        const auto it = owned.find(tensor);
        if (it == owned.end())
        {
            throw std::runtime_error(std::format("No buffer for tensor {}/{}", name, tensor));
        }
        return it->second.get();
    }

    void run(cudaStream_t stream) const
    {
        if (!context->enqueueV3(stream))
        {
            throw std::runtime_error(std::format("TensorRT enqueue of {} failed", name));
        }
    }
};

std::unique_ptr<nvinfer1::ICudaEngine>
deserialize(nvinfer1::IRuntime& runtime, const std::filesystem::path& engine_path)
{
    const std::vector<char> data = readFile(engine_path);
    return std::unique_ptr<nvinfer1::ICudaEngine>(
        runtime.deserializeCudaEngine(data.data(), data.size()));
}

Graph loadGraph(
    nvinfer1::IRuntime& runtime, Logger& logger, const EngineConfig& config, const std::string& name)
{
    const std::filesystem::path engine_path =
        config.engine_dir / (name + config.suffix + ".engine");
    const std::filesystem::path onnx = config.model_dir / (name + config.suffix + ".onnx");
    std::error_code             ignored;
    std::filesystem::remove(engine_path.string() + ".tmp", ignored);  // an interrupted build
    const bool have_onnx   = std::filesystem::exists(onnx);
    const bool have_engine = std::filesystem::exists(engine_path);
    if (!have_engine && !have_onnx)
    {
        throw std::runtime_error(
            std::format("Neither {} nor {} exists", engine_path.string(), onnx.string()));
    }
    bool built = false;
    if (have_onnx)
    {
        const std::string recorded = have_engine ? readSource(engine_path) : std::string{};
        // An engine from before the record was kept is adopted unless its graph is newer.
        const bool adopt =
            have_engine && recorded.empty() &&
            std::filesystem::last_write_time(onnx) <= std::filesystem::last_write_time(engine_path);
        if (adopt)
        {
            std::ofstream(sourcePath(engine_path)) << sourceStamp(onnx) << '\n';
        }
        else if (!have_engine || recorded != sourceStamp(onnx))
        {
            buildEngine(logger, config, onnx, engine_path);
            built = true;
        }
    }
    Graph graph;
    graph.name   = name;
    graph.engine = deserialize(runtime, engine_path);
    if (!graph.engine && have_onnx && !built)
    {
        // Built for another GPU or TensorRT: the graph is here, so build it for this one.
        buildEngine(logger, config, onnx, engine_path);
        graph.engine = deserialize(runtime, engine_path);
    }
    if (!graph.engine)
    {
        throw std::runtime_error(std::format(
            "Cannot load {}: built for another GPU or TensorRT? Delete it to rebuild from ONNX",
            engine_path.string()));
    }
    // The three graphs run one after another, so they share one block of activation memory.
    graph.context.reset(graph.engine->createExecutionContext(
        nvinfer1::ExecutionContextAllocationStrategy::kUSER_MANAGED));
    if (!graph.context)
    {
        throw std::runtime_error(
            std::format("No execution context for {} (GPU memory?)", engine_path.string()));
    }
    return graph;
}

void expectTensor(const Graph& graph, const std::string& tensor, ElementType type, std::size_t count)
{
    if (!graph.has(tensor) || graph.type(tensor) != type || graph.count(tensor) != count)
    {
        throw std::runtime_error(std::format(
            "{}/{} is not the tensor the engine expects ({} elements of the exported type): "
            "export all three graphs again in one run",
            graph.name,
            tensor,
            count));
    }
}

// A tensor one graph writes and the next reads must agree in type and size.
void expectSame(const Graph& producer, const Graph& consumer, const std::string& tensor)
{
    if (!producer.has(tensor) || !consumer.has(tensor) ||
        producer.type(tensor) != consumer.type(tensor) ||
        producer.count(tensor) != consumer.count(tensor))
    {
        throw std::runtime_error(std::format(
            "{} differs between {} and {}: export all three graphs again in one run",
            tensor,
            producer.name,
            consumer.name));
    }
}

}  // namespace

struct Sam3Engine::Impl
{
    struct Prompt
    {
        DeviceBuffer mask;
        DeviceBuffer features;
        // Only when the decoder reads the embeddings too.
        DeviceBuffer embeds;
    };

    Logger                              logger;
    std::size_t                         max_cached_prompts{ 64 };
    std::unique_ptr<nvinfer1::IRuntime> runtime;
    Graph                               image;
    Graph                               text;
    Graph                               decoder;
    ClipTokenizer                       tokenizer;
    CudaStream                          stream;
    DeviceBuffer                        activation;
    PinnedBuffer                        host_image;
    PinnedBuffer                        host_masks;
    // The text encoder's embeddings when the decoder does not read them: every output needs an
    // address.
    DeviceBuffer                  scratch_embeds;
    bool                          decoder_reads_embeds{ false };
    std::map<std::string, Prompt> prompts;
    std::vector<unsigned char>    host_scores;
    std::vector<float>            scores;
    std::size_t                   queries{ 0 };
    int                           mask_size_px{ 0 };
    std::size_t                   mask_bytes{ 0 };
    // Reused between calls: the image at the model's size, as uint8 and as float.
    cv::Mat resized;
    cv::Mat normalised;
    Timing  timing;

    explicit Impl(const EngineConfig& config)
      : max_cached_prompts(std::max<std::size_t>(1, config.max_cached_prompts))
      , tokenizer(config.vocabulary_path)
    {
        int devices = 0;
        check(cudaGetDeviceCount(&devices), "cudaGetDeviceCount");
        if (devices == 0)
        {
            throw std::runtime_error("No CUDA device");
        }
        runtime.reset(nonNull(nvinfer1::createInferRuntime(logger), "createInferRuntime"));
        image   = loadGraph(*runtime, logger, config, "sam3_image_encoder");
        text    = loadGraph(*runtime, logger, config, "sam3_text_encoder");
        decoder = loadGraph(*runtime, logger, config, "sam3_decoder");
        checkTensors();
        shareActivationMemory();

        image.allocate({});
        // The decoder reads the image encoder's outputs and a prompt's cached text features.
        const std::vector<std::string> from_image{ "fpn0", "fpn1", "fpn2", "pos2" };
        std::vector<std::string>       from_text{ "language_mask", "language_features" };
        decoder_reads_embeds = decoder.has("language_embeds");
        if (decoder_reads_embeds)
        {
            expectSame(text, decoder, "language_embeds");
            from_text.emplace_back("language_embeds");
        }
        std::vector<std::string> shared = from_image;
        shared.insert(shared.end(), from_text.begin(), from_text.end());
        decoder.allocate(shared);
        for (const std::string& name : from_image)
        {
            decoder.bind(name, image.at(name));
        }
        text.allocate({ "language_mask", "language_features", "language_embeds" });
        if (!decoder_reads_embeds)
        {
            scratch_embeds = DeviceBuffer(text.bytes("language_embeds"), "text/language_embeds");
            text.bind("language_embeds", scratch_embeds.get());
        }
        host_image = PinnedBuffer(image.bytes("image"), "image/image (host)");
        host_scores.resize(decoder.bytes("scores"));
        const nvinfer1::Dims mask_dims = decoder.engine->getTensorShape("mask_logits");
        queries                        = static_cast<std::size_t>(mask_dims.d[1]);
        mask_size_px                   = static_cast<int>(mask_dims.d[2]);
        mask_bytes                     = static_cast<std::size_t>(mask_size_px) *
                     static_cast<std::size_t>(mask_size_px) *
                     elementSize(decoder.type("mask_logits"));
        host_masks = PinnedBuffer(queries * mask_bytes, "decoder/mask_logits (host)");
        scores.resize(queries);
    }

    // The engines must be the three graphs of one export: the buffers are sized from them.
    void checkTensors() const
    {
        expectTensor(
            image,
            "image",
            ElementType::kFloat32,
            std::size_t{ 3 } * kInputSizePx * kInputSizePx);
        expectTensor(text, "tokens", ElementType::kInt64, kContextLength);
        for (const char* tensor : { "fpn0", "fpn1", "fpn2", "pos2" })
        {
            expectSame(image, decoder, tensor);
        }
        for (const char* tensor : { "language_mask", "language_features" })
        {
            expectSame(text, decoder, tensor);
        }
        if (!text.has("language_embeds") || !decoder.has("scores") || !decoder.has("mask_logits"))
        {
            throw std::runtime_error(
                "The text encoder or decoder lacks an output the engine reads: export again");
        }
    }

    void shareActivationMemory()
    {
        const std::array<const Graph*, 3> graphs{ &image, &text, &decoder };
        std::int64_t                      largest = 0;
        std::string                       sizes;
        for (const Graph* g : graphs)
        {
            const std::int64_t bytes = g->engine->getDeviceMemorySizeV2();
            largest                  = std::max(largest, bytes);
            sizes += std::format(" {} {:.0f} MiB", g->name, static_cast<double>(bytes) / 1048576.0);
        }
        activation = DeviceBuffer(static_cast<std::size_t>(largest), "activation memory");
        for (const Graph* g : graphs)
        {
            g->context->setDeviceMemoryV2(activation.get(), g->engine->getDeviceMemorySizeV2());
        }
        std::cerr
            << std::format("[sam3] activation memory:{}, shared block of the largest\n", sizes);
    }

    // Text features of a prompt, computed on first use.
    const Prompt& prompt(const std::string& phrase)
    {
        const auto it = prompts.find(phrase);
        if (it != prompts.end())
        {
            return it->second;
        }
        if (prompts.size() >= max_cached_prompts)
        {
            prompts.clear();
        }
        const std::vector<std::int64_t> tokens = tokenizer.encode(phrase, kContextLength);
        Prompt p{ .mask = DeviceBuffer(text.bytes("language_mask"), "prompt language_mask"),
                  .features =
                      DeviceBuffer(text.bytes("language_features"), "prompt language_features"),
                  .embeds =
                      decoder_reads_embeds ?
                          DeviceBuffer(text.bytes("language_embeds"), "prompt language_embeds") :
                          DeviceBuffer{} };
        check(
            cudaMemcpyAsync(
                text.at("tokens"),
                tokens.data(),
                tokens.size() * sizeof(std::int64_t),
                cudaMemcpyHostToDevice,
                stream.get()),
            "copy of text/tokens");
        text.bind("language_mask", p.mask.get());
        text.bind("language_features", p.features.get());
        if (decoder_reads_embeds)
        {
            text.bind("language_embeds", p.embeds.get());
        }
        text.run(stream.get());
        return prompts.emplace(phrase, std::move(p)).first->second;
    }
};

Sam3Engine::Sam3Engine(const EngineConfig& config)
  : impl_(std::make_unique<Impl>(config))
{}

Sam3Engine::~Sam3Engine() = default;

const Timing& Sam3Engine::timing() const noexcept { return impl_->timing; }

std::vector<Instance>
Sam3Engine::segment(const cv::Mat& rgb, std::span<const std::string> prompts, float min_score)
{
    if (rgb.empty() || rgb.type() != CV_8UC3)
    {
        throw std::invalid_argument("SAM 3 needs a non-empty CV_8UC3 RGB image");
    }
    Impl& m    = *impl_;
    m.timing   = {};
    auto start = Clock::now();
    // As SAM 3's processor: stretched to 1008 x 1008, scaled to [-1, 1], planar.
    cv::resize(rgb, m.resized, cv::Size(kInputSizePx, kInputSizePx), 0.0, 0.0, cv::INTER_LINEAR);
    m.resized.convertTo(m.normalised, CV_32FC3, 1.0 / 127.5, -1.0);
    auto*             host  = static_cast<float*>(m.host_image.get());
    const std::size_t plane = static_cast<std::size_t>(kInputSizePx) * kInputSizePx;
    // The three planes of the pinned upload buffer: split writes straight into them.
    std::array<cv::Mat, 3> planes{ cv::Mat(kInputSizePx, kInputSizePx, CV_32F, host),
                                   cv::Mat(kInputSizePx, kInputSizePx, CV_32F, host + plane),
                                   cv::Mat(kInputSizePx, kInputSizePx, CV_32F, host + (2 * plane)) };
    cv::split(m.normalised, planes.data());
    check(
        cudaMemcpyAsync(
            m.image.at("image"),
            host,
            m.image.bytes("image"),
            cudaMemcpyHostToDevice,
            m.stream.get()),
        "copy of image/image");
    m.timing.preprocess_ms = millisecondsSince(start);

    start = Clock::now();
    m.image.run(m.stream.get());
    check(cudaStreamSynchronize(m.stream.get()), "image encoder");
    m.timing.image_encoder_ms = millisecondsSince(start);

    std::vector<Instance> instances;
    const ElementType     score_type = m.decoder.type("scores");
    const ElementType     mask_type  = m.decoder.type("mask_logits");
    cv::Mat               logits(m.mask_size_px, m.mask_size_px, CV_32F);
    cv::Mat               upsampled;
    auto*                 host_masks = static_cast<unsigned char*>(m.host_masks.get());
    const auto* device_masks = static_cast<const unsigned char*>(m.decoder.at("mask_logits"));
    for (const std::string& phrase : prompts)
    {
        start                 = Clock::now();
        const Impl::Prompt& p = m.prompt(phrase);
        check(cudaStreamSynchronize(m.stream.get()), "text encoder");
        m.timing.text_encoder_ms += millisecondsSince(start);

        start = Clock::now();
        m.decoder.bind("language_mask", p.mask.get());
        m.decoder.bind("language_features", p.features.get());
        if (m.decoder_reads_embeds)
        {
            m.decoder.bind("language_embeds", p.embeds.get());
        }
        m.decoder.run(m.stream.get());
        check(
            cudaMemcpyAsync(
                m.host_scores.data(),
                m.decoder.at("scores"),
                m.host_scores.size(),
                cudaMemcpyDeviceToHost,
                m.stream.get()),
            "copy of decoder/scores");
        check(cudaStreamSynchronize(m.stream.get()), "decoder");
        m.timing.decoder_ms += millisecondsSince(start);

        start = Clock::now();
        toFloats(m.host_scores.data(), score_type, m.scores);
        // The accepted queries' masks come over in one batch.
        std::vector<std::size_t> accepted;
        for (std::size_t q = 0; q < m.queries; ++q)
        {
            if (m.scores[q] >= min_score)
            {
                check(
                    cudaMemcpyAsync(
                        host_masks + (q * m.mask_bytes),
                        device_masks + (q * m.mask_bytes),
                        m.mask_bytes,
                        cudaMemcpyDeviceToHost,
                        m.stream.get()),
                    "copy of decoder/mask_logits");
                accepted.push_back(q);
            }
        }
        check(cudaStreamSynchronize(m.stream.get()), "copy of the masks");
        const std::size_t first = instances.size();
        for (const std::size_t q : accepted)
        {
            toFloats(
                host_masks + (q * m.mask_bytes),
                mask_type,
                std::span<float>(logits.ptr<float>(), logits.total()));
            // As SAM 3: bilinear to the image size, then the sigmoid above 0.5, i.e. logits above 0.
            cv::resize(logits, upsampled, rgb.size(), 0.0, 0.0, cv::INTER_LINEAR);
            const cv::Mat  full = upsampled > 0.0F;
            const cv::Rect box  = cv::boundingRect(full);
            if (box.area() == 0)
            {
                continue;
            }
            instances.push_back({ phrase, m.scores[q], box, full(box).clone() });
        }
        // Best first within the prompt, whatever order the decoder's queries come in.
        std::stable_sort(
            std::next(instances.begin(), static_cast<std::ptrdiff_t>(first)),
            instances.end(),
            [](const Instance& a, const Instance& b) { return a.score > b.score; });
        m.timing.postprocess_ms += millisecondsSince(start);
    }
    return instances;
}

}  // namespace sam3_inference
