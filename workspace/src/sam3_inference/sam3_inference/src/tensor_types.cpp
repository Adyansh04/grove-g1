#include "sam3_inference/tensor_types.hpp"

#include <cstdint>
#include <cstring>
#include <span>
#include <stdexcept>

namespace sam3_inference
{

std::size_t elementSize(ElementType type) noexcept
{
    switch (type)
    {
        case ElementType::kFloat32:
        case ElementType::kInt32:
            return 4;
        case ElementType::kBfloat16:
            return 2;
        case ElementType::kInt64:
            return 8;
        case ElementType::kInt8:
        case ElementType::kUint8:
        case ElementType::kBool:
            return 1;
    }
    return 1;
}

void toFloats(const void* data, ElementType type, std::span<float> out)
{
    switch (type)
    {
        case ElementType::kFloat32:
            std::memcpy(out.data(), data, out.size_bytes());
            return;
        case ElementType::kBfloat16:
        {
            const auto* halves = static_cast<const std::uint16_t*>(data);
            for (std::size_t i = 0; i < out.size(); ++i)
            {
                // bf16 is the upper half of a float.
                const std::uint32_t bits = static_cast<std::uint32_t>(halves[i]) << 16;
                std::memcpy(&out[i], &bits, sizeof(float));
            }
            return;
        }
        default:
            throw std::invalid_argument("toFloats: the tensor is not float32 or bfloat16");
    }
}

}  // namespace sam3_inference
