#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace sam3_inference
{

/**
 * @class ClipTokenizer
 * @brief The byte-level BPE tokenizer of CLIP, as SAM 3 uses it for its text prompts.
 * @details Case and repeated whitespace do not change the ids. The ids start with
 * `<start_of_text>` (49406), end with `<end_of_text>` (49407) and are padded with 0.
 *
 * @note Pieces are split as CLIP's regular expression does for ASCII text; bytes outside ASCII
 * count as letters, which matches it for letters of other scripts, not for their digits or
 * punctuation. `ftfy` text repair is not applied.
 */
class ClipTokenizer
{
public:
    /**
     * @brief Loads the merges.
     * @param vocabulary_path CLIP's `bpe_simple_vocab_16e6.txt`, plain or gzip-compressed (`.gz`).
     * @throws std::runtime_error If the file cannot be read or holds too few merges.
     */
    explicit ClipTokenizer(const std::filesystem::path& vocabulary_path);

    /**
     * @brief Token ids of one text.
     * @param text The prompt, UTF-8.
     * @param context_length Ids returned; longer texts are cut and still end with
     * `<end_of_text>`.
     * @return `context_length` ids.
     * @pre `context_length` is at least 2, room for the start and end ids.
     * @throws std::invalid_argument If `context_length` is less than 2.
     */
    [[nodiscard]] std::vector<std::int64_t>
    encode(std::string_view text, std::size_t context_length = 32) const;

    /** @brief Id of `<start_of_text>`. */
    static constexpr std::int64_t kStartOfText = 49406;
    /** @brief Id of `<end_of_text>`. */
    static constexpr std::int64_t kEndOfText = 49407;

private:
    struct PairHash
    {
        std::size_t operator()(const std::pair<std::string, std::string>& p) const noexcept;
    };

    // Merges the byte symbols of one piece, lowest-ranked pair first, as CLIP does.
    [[nodiscard]] std::vector<std::string> bpe(std::string_view piece) const;

    // UTF-8 of the symbol CLIP gives each byte.
    std::vector<std::string>                                               byte_symbol_;
    std::unordered_map<std::string, std::int64_t>                          encoder_;
    std::unordered_map<std::pair<std::string, std::string>, int, PairHash> ranks_;
};

}  // namespace sam3_inference
