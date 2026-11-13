// Runs SAM 3.1 on one image and prints the instances and timings; writes each mask next to the
// image. The directory holds both the ONNX graphs and the engines (the node keeps them apart).
//
// check_sam3_engine <model and engine dir> <vocabulary> <image> <prompt> [prompt...]

#include <cstdlib>
#include <format>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <vector>

#include "sam3_inference/sam3_engine.hpp"

int main(int argc, char** argv)
{
    constexpr const char* kUsage =
        "usage: check_sam3_engine <model and engine dir> <vocabulary> <image> <prompt> "
        "[prompt...]\n";
    if (argc < 5)
    {
        std::cerr << kUsage;
        return 2;
    }
    // Before the engines, which take seconds to load or minutes to build.
    const cv::Mat bgr = cv::imread(argv[3], cv::IMREAD_COLOR);
    if (bgr.empty())
    {
        std::cerr << std::format("cannot read the image {}\n{}", argv[3], kUsage);
        return 2;
    }
    cv::Mat rgb;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    sam3_inference::EngineConfig config;
    config.engine_dir      = argv[1];
    config.model_dir       = argv[1];
    config.vocabulary_path = argv[2];
    sam3_inference::Sam3Engine     engine(config);
    const std::vector<std::string> prompts(argv + 4, argv + argc);
    for (int run = 0; run < 3; ++run)
    {
        // MIN_SCORE: threshold for the parity check against PyTorch (default 0.5).
        const char* min_score =
            std::getenv("MIN_SCORE");  // NOLINT(concurrency-mt-unsafe): one thread
        const auto instances = engine.segment(
            rgb,
            prompts,
            min_score != nullptr ? std::strtof(min_score, nullptr) : 0.5F);
        const auto& t = engine.timing();
        std::cout << std::format(
            "run {}: {} instances; preprocess {:.1f}, image {:.1f}, text {:.1f}, decoder {:.1f}, "
            "post {:.1f} ms\n",
            run,
            instances.size(),
            t.preprocess_ms,
            t.image_encoder_ms,
            t.text_encoder_ms,
            t.decoder_ms,
            t.postprocess_ms);
        for (std::size_t i = 0; run == 2 && i < instances.size(); ++i)
        {
            const auto& s = instances[i];
            std::cout << std::format(
                "  {} {:.3f} box ({}, {}, {} x {}), {} mask pixels\n",
                s.label,
                s.score,
                s.box.x,
                s.box.y,
                s.box.width,
                s.box.height,
                cv::countNonZero(s.mask));
            const cv::Mat overlay = bgr.clone();
            overlay(s.box).setTo(cv::Scalar(0, 255, 0), s.mask);
            cv::imwrite(std::format("{}.mask{}.png", argv[3], i), overlay);
        }
    }
    return 0;
}
