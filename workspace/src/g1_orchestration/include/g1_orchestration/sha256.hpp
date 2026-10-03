#ifndef G1_ORCHESTRATION__SHA256_HPP_
#define G1_ORCHESTRATION__SHA256_HPP_

/**
 * @file sha256.hpp
 * @brief SHA-256 of a byte string, as the hex text a mission's approval is bound to.
 */

#include <string>
#include <string_view>

namespace g1_orchestration
{

/**
 * @brief SHA-256 of @p bytes.
 *
 * @param bytes What to hash, exactly as given: no normalisation of line ends or whitespace.
 * @return 64 lowercase hex digits.
 * @throws std::runtime_error If OpenSSL fails, which it does not on valid input.
 */
std::string sha256Hex(std::string_view bytes);

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__SHA256_HPP_
