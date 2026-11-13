#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace sam3_inference
{

/** @brief Element types of the engines' tensors that the engine allocates, writes or reads. */
enum class ElementType : std::uint8_t
{
    /** @brief IEEE 754 single precision. */
    kFloat32,
    /** @brief bfloat16: the upper 16 bits of a float32. */
    kBfloat16,
    /** @brief 32-bit signed integer. */
    kInt32,
    /** @brief 64-bit signed integer. */
    kInt64,
    /** @brief 8-bit signed integer. */
    kInt8,
    /** @brief 8-bit unsigned integer. */
    kUint8,
    /** @brief One byte per boolean. */
    kBool
};

/**
 * @brief Bytes per element of a type.
 * @param type The element type.
 * @return 1, 2, 4 or 8.
 */
[[nodiscard]] std::size_t elementSize(ElementType type) noexcept;

/**
 * @brief Converts the elements of a float tensor to float32.
 * @param data Buffer holding at least `out.size()` elements of `type`.
 * @param type kFloat32 or kBfloat16.
 * @param out Receives one float per element; NaN and infinity are kept.
 * @throws std::invalid_argument If `type` is not one of the two float types.
 */
void toFloats(const void* data, ElementType type, std::span<float> out);

}  // namespace sam3_inference
