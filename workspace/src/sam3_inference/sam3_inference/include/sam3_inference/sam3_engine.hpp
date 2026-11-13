#pragma once

#include <cstddef>
#include <filesystem>
#include <memory>
#include <opencv2/core.hpp>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace sam3_inference
{

/** @brief One instance SAM 3 found for a prompt. */
struct Instance
{
    /// The prompt that found it.
    std::string label;
    /// Detection score times the prompt's presence score, 0 to 1.
    float score{ 0.0F };
    /// Bounding box of the mask in the image, pixels.
    cv::Rect box;
    /// CV_8UC1 mask cropped to `box`, 255 inside the instance.
    cv::Mat mask;
};

/** @brief Where the model files are and how to build engines from them. */
struct EngineConfig
{
    /// Holds `sam3_image_encoder<suffix>.onnx`, `sam3_text_encoder<suffix>.onnx` and
    /// `sam3_decoder<suffix>.onnx` (with their external weight files).
    std::filesystem::path model_dir;
    /// Engines are loaded from here as `<graph><suffix>.engine`. One is built from its ONNX graph
    /// when it is missing, or when the graph or the TensorRT version changed since it was built
    /// (recorded next to it in `<graph><suffix>.engine.source`). Engines only run on the GPU and
    /// TensorRT version they were built with.
    std::filesystem::path engine_dir;
    /// Precision tag of the file names; must equal `export_sam3.py --suffix`, e.g. ".bf16".
    std::string suffix{ ".bf16" };
    /// CLIP's BPE vocabulary (`bpe_simple_vocab_16e6.txt`, plain or `.gz`).
    std::filesystem::path vocabulary_path;
    /// Scratch memory the builder may use, bytes.
    std::size_t workspace_bytes{ std::size_t{ 2 } << 30 };
    /// Distinct prompts whose text features stay on the GPU; the cache is emptied beyond this.
    std::size_t max_cached_prompts{ 64 };
};

/** @brief Time spent by the last `segment` call, milliseconds. */
struct Timing
{
    /// Resizing and normalising the image, and its upload.
    double preprocess_ms{ 0.0 };
    /// The image encoder, once per call.
    double image_encoder_ms{ 0.0 };
    /// The text encoder, for prompts not cached yet.
    double text_encoder_ms{ 0.0 };
    /// The decoder, once per prompt.
    double decoder_ms{ 0.0 };
    /// Thresholding the scores and turning the accepted masks into instances.
    double postprocess_ms{ 0.0 };
};

/**
 * @brief A failed CUDA call.
 * @details Apart from running out of memory, a CUDA error stays for the life of the process.
 */
class CudaFailure : public std::runtime_error
{
public:
    /**
     * @brief Makes the failure.
     * @param what The failed call and CUDA's text.
     * @param out_of_memory Whether the GPU ran out of memory.
     */
    CudaFailure(const std::string& what, bool out_of_memory)
      : std::runtime_error(what)
      , out_of_memory_(out_of_memory)
    {}

    /** @return Whether the GPU ran out of memory: a later call can work. */
    [[nodiscard]] bool outOfMemory() const noexcept { return out_of_memory_; }

private:
    bool out_of_memory_;
};

/**
 * @class Sam3Engine
 * @brief SAM 3.1's open-vocabulary image detector as three TensorRT engines.
 * @details The image encoder runs once per image, the text encoder once per new prompt (its
 * features stay on the GPU), the decoder once per prompt and image. The three engines share one
 * block of activation memory, since they run one after another.
 *
 * @warning Not thread-safe: one call at a time.
 */
class Sam3Engine
{
public:
    /**
     * @brief Loads the engines, building them from the ONNX graphs first if needed.
     * @param config Model and engine locations.
     * @throws std::runtime_error If there is no usable GPU, a graph or engine is missing or does
     * not load, its tensors do not have the expected types and sizes, or the GPU cannot hold it.
     */
    explicit Sam3Engine(const EngineConfig& config);
    ~Sam3Engine();
    Sam3Engine(const Sam3Engine&)            = delete;
    Sam3Engine& operator=(const Sam3Engine&) = delete;

    /**
     * @brief Finds the instances of each prompt in an image.
     * @param rgb CV_8UC3 image, RGB order, any size. Images larger than the 1008 x 1008 model input
     * are shrunk without antialiasing, unlike SAM 3's PyTorch processor, so their scores and masks
     * can differ slightly from it.
     * @param prompts Phrases to look for, e.g. "red cube".
     * @param min_score Instances scoring below this are left out, 0 to 1.
     * @return Instances of every prompt, prompts in the given order, best first within each.
     * @throws std::invalid_argument If the image is not CV_8UC3 or is empty.
     * @throws std::runtime_error If a CUDA or TensorRT call fails.
     */
    [[nodiscard]] std::vector<Instance>
    segment(const cv::Mat& rgb, std::span<const std::string> prompts, float min_score);

    /** @brief Time spent by the last `segment` call. */
    [[nodiscard]] const Timing& timing() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sam3_inference
