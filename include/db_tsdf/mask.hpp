#ifndef DB_TSDF_MASK_HPP
#define DB_TSDF_MASK_HPP

#include <bit>
#include <concepts>
#include <cstdint>
#include <limits>

namespace db_tsdf
{

template <typename Mask>
concept SupportedMask = std::same_as<Mask, uint16_t> || std::same_as<Mask, uint32_t>;

template <std::unsigned_integral Mask>
inline constexpr int mask_bits_v = std::numeric_limits<Mask>::digits;

template <std::unsigned_integral Mask>
inline constexpr Mask full_mask_v = std::numeric_limits<Mask>::max();

template <std::unsigned_integral Mask>
constexpr Mask rankMask(int rank)
{
    if (rank <= 0) {
        return Mask{0};
    }
    if (rank >= mask_bits_v<Mask>) {
        return full_mask_v<Mask>;
    }
    return static_cast<Mask>(full_mask_v<Mask> >> (mask_bits_v<Mask> - rank));
}

template <std::unsigned_integral Mask>
constexpr int maskRank(Mask mask)
{
    return std::popcount(mask);
}

}  // namespace db_tsdf

#endif
