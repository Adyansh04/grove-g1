#include "sam3_inference/clip_tokenizer.hpp"

#include <zlib.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <format>
#include <fstream>
#include <functional>
#include <ios>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace sam3_inference
{
namespace
{

// Merges CLIP's vocabulary file holds after its header line.
constexpr std::size_t kMerges = 49152 - 256 - 2;

std::string utf8(std::uint32_t code_point)
{
    std::string out;
    if (code_point < 0x80)
    {
        out.push_back(static_cast<char>(code_point));
    }
    else if (code_point < 0x800)
    {
        out.push_back(static_cast<char>(0xC0 | (code_point >> 6)));
        out.push_back(static_cast<char>(0x80 | (code_point & 0x3F)));
    }
    else
    {
        out.push_back(static_cast<char>(0xE0 | (code_point >> 12)));
        out.push_back(static_cast<char>(0x80 | ((code_point >> 6) & 0x3F)));
        out.push_back(static_cast<char>(0x80 | (code_point & 0x3F)));
    }
    return out;
}

std::string readText(const std::filesystem::path& path)
{
    if (path.extension() == ".gz")
    {
        gzFile file = gzopen(path.c_str(), "rb");
        if (file == nullptr)
        {
            throw std::runtime_error(std::format("Cannot open {}", path.string()));
        }
        std::string               text;
        std::array<char, 1 << 16> chunk{};
        int                       n = 0;
        while ((n = gzread(file, chunk.data(), static_cast<unsigned>(chunk.size()))) > 0)
        {
            text.append(chunk.data(), static_cast<std::size_t>(n));
        }
        gzclose(file);
        if (n < 0)
        {
            throw std::runtime_error(std::format("Cannot decompress {}", path.string()));
        }
        return text;
    }
    const std::ifstream in(path, std::ios::binary);
    if (!in)
    {
        throw std::runtime_error(std::format("Cannot open {}", path.string()));
    }
    std::ostringstream text;
    text << in.rdbuf();
    return text.str();
}

bool isSpace(unsigned char c) { return c == ' ' || (c >= '\t' && c <= '\r'); }

// After lower-casing; bytes of non-ASCII characters count as letters.
bool isLetter(unsigned char c) { return (c >= 'a' && c <= 'z') || c >= 0x80; }

bool isDigit(unsigned char c) { return c >= '0' && c <= '9'; }

// Lower case, whitespace runs to one space, trimmed.
std::string clean(std::string_view text)
{
    std::string out;
    bool        space = false;
    for (const char ch : text)
    {
        const auto c = static_cast<unsigned char>(ch);
        if (isSpace(c))
        {
            space = !out.empty();
            continue;
        }
        if (space)
        {
            out.push_back(' ');
            space = false;
        }
        out.push_back(c >= 'A' && c <= 'Z' ? static_cast<char>(c - 'A' + 'a') : ch);
    }
    return out;
}

// End of the piece that starts at text[i] (not a space): a contraction ('s 't 're 've 'm 'll 'd),
// a letter run, one digit, or a run of anything else.
std::size_t pieceEnd(std::string_view text, std::size_t i)
{
    static constexpr std::array<std::string_view, 7> kContractions{ "'s", "'t",  "'re", "'ve",
                                                                    "'m", "'ll", "'d" };
    const auto                                       c = static_cast<unsigned char>(text[i]);
    if (c == '\'')
    {
        for (const std::string_view k : kContractions)
        {
            if (text.substr(i, k.size()) == k)
            {
                return i + k.size();
            }
        }
    }
    std::size_t j = i + 1;
    if (isLetter(c))
    {
        while (j < text.size() && isLetter(static_cast<unsigned char>(text[j])))
        {
            ++j;
        }
    }
    else if (!isDigit(c))
    {
        while (j < text.size())
        {
            const auto d = static_cast<unsigned char>(text[j]);
            if (isSpace(d) || isLetter(d) || isDigit(d))
            {
                break;
            }
            ++j;
        }
    }
    return j;
}

// CLIP's pre-tokenizer pattern, split at spaces into pieces.
std::vector<std::string_view> pieces(std::string_view text)
{
    std::vector<std::string_view> out;
    std::size_t                   i = 0;
    while (i < text.size())
    {
        if (isSpace(static_cast<unsigned char>(text[i])))
        {
            ++i;
            continue;
        }
        const std::size_t j = pieceEnd(text, i);
        out.push_back(text.substr(i, j - i));
        i = j;
    }
    return out;
}

}  // namespace

std::size_t
ClipTokenizer::PairHash::operator()(const std::pair<std::string, std::string>& p) const noexcept
{
    const std::size_t a = std::hash<std::string>{}(p.first);
    return a ^ (std::hash<std::string>{}(p.second) + 0x9e3779b97f4a7c15ULL + (a << 6) + (a >> 2));
}

ClipTokenizer::ClipTokenizer(const std::filesystem::path& vocabulary_path)
{
    // The printable Latin-1 bytes keep their code point, the others move to 256 + n.
    std::vector<int> bytes;
    for (int b = '!'; b <= '~'; ++b)
    {
        bytes.push_back(b);
    }
    for (int b = 0xA1; b <= 0xAC; ++b)
    {
        bytes.push_back(b);
    }
    for (int b = 0xAE; b <= 0xFF; ++b)
    {
        bytes.push_back(b);
    }
    std::vector<std::uint32_t> code_points(bytes.begin(), bytes.end());
    std::uint32_t              extra = 0;
    for (int b = 0; b < 256; ++b)
    {
        if (std::ranges::find(bytes, b) == bytes.end())
        {
            bytes.push_back(b);
            code_points.push_back(256 + extra++);
        }
    }
    byte_symbol_.assign(256, "");
    std::vector<std::string> vocabulary;
    for (std::size_t i = 0; i < bytes.size(); ++i)
    {
        byte_symbol_[static_cast<std::size_t>(bytes[i])] = utf8(code_points[i]);
        vocabulary.push_back(byte_symbol_[static_cast<std::size_t>(bytes[i])]);
    }
    for (std::size_t i = 0; i < bytes.size(); ++i)
    {
        vocabulary.push_back(vocabulary[i] + "</w>");
    }

    const std::string  text = readText(vocabulary_path);
    std::istringstream lines(text);
    std::string        line;
    std::getline(lines, line);  // version header
    while (ranks_.size() < kMerges && std::getline(lines, line))
    {
        const std::size_t space = line.find(' ');
        if (space == std::string::npos)
        {
            continue;
        }
        std::pair<std::string, std::string> pair{ line.substr(0, space), line.substr(space + 1) };
        vocabulary.push_back(pair.first + pair.second);
        ranks_.emplace(std::move(pair), static_cast<int>(ranks_.size()));
    }
    if (ranks_.size() < kMerges)
    {
        throw std::runtime_error(std::format(
            "{} holds {} merges, CLIP has {}",
            vocabulary_path.string(),
            ranks_.size(),
            kMerges));
    }
    for (std::size_t i = 0; i < vocabulary.size(); ++i)
    {
        encoder_.emplace(vocabulary[i], static_cast<std::int64_t>(i));
    }
}

std::vector<std::string> ClipTokenizer::bpe(std::string_view piece) const
{
    std::vector<std::string> symbols;
    for (const char c : piece)
    {
        symbols.push_back(byte_symbol_[static_cast<unsigned char>(c)]);
    }
    symbols.back() += "</w>";
    std::vector<std::string> merged;
    while (symbols.size() > 1)
    {
        int         best = std::numeric_limits<int>::max();
        std::size_t at   = 0;
        for (std::size_t i = 0; i + 1 < symbols.size(); ++i)
        {
            const auto it = ranks_.find({ symbols[i], symbols[i + 1] });
            if (it != ranks_.end() && it->second < best)
            {
                best = it->second;
                at   = i;
            }
        }
        if (best == std::numeric_limits<int>::max())
        {
            break;
        }
        // Every occurrence of the best pair merges, left to right.
        const std::string first  = symbols[at];
        const std::string second = symbols[at + 1];
        merged.clear();
        for (std::size_t i = 0; i < symbols.size(); ++i)
        {
            if (i + 1 < symbols.size() && symbols[i] == first && symbols[i + 1] == second)
            {
                merged.push_back(first + second);
                ++i;
            }
            else
            {
                merged.push_back(symbols[i]);
            }
        }
        symbols.swap(merged);
    }
    return symbols;
}

std::vector<std::int64_t>
ClipTokenizer::encode(std::string_view text, std::size_t context_length) const
{
    if (context_length < 2)
    {
        throw std::invalid_argument(
            std::format("context_length must be at least 2, got {}", context_length));
    }
    // As CLIP: lower-cased, whitespace collapsed, split into words, numbers and punctuation runs,
    // each piece encoded with the merges.
    std::vector<std::int64_t> ids{ kStartOfText };
    const std::string         cleaned = clean(text);
    for (const std::string_view piece : pieces(cleaned))
    {
        for (const std::string& symbol : bpe(piece))
        {
            ids.push_back(encoder_.at(symbol));
        }
    }
    ids.push_back(kEndOfText);
    if (ids.size() > context_length)
    {
        ids.resize(context_length);
        ids.back() = kEndOfText;
    }
    ids.resize(context_length, 0);
    return ids;
}

}  // namespace sam3_inference
