/**
 * @file sha256.cpp
 * @brief SHA-256 through OpenSSL's EVP interface.
 */

#include "g1_orchestration/sha256.hpp"

#include <openssl/evp.h>

#include <array>
#include <format>
#include <stdexcept>
#include <string>
#include <string_view>

namespace g1_orchestration
{

std::string sha256Hex(std::string_view bytes)
{
    std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
    unsigned int                               length = 0;
    if (EVP_Digest(bytes.data(), bytes.size(), digest.data(), &length, EVP_sha256(), nullptr) != 1)
    {
        throw std::runtime_error("SHA-256 failed");
    }
    std::string hex;
    hex.reserve(static_cast<std::size_t>(length) * 2);
    for (unsigned int i = 0; i < length; ++i)
    {
        hex += std::format("{:02x}", digest[i]);
    }
    return hex;
}

}  // namespace g1_orchestration
