#include <cstdint>
#include <iostream>

#include <db_tsdf/mask.hpp>

namespace
{

template <typename Actual, typename Expected>
bool expectEqual(const char* label, Actual actual, Expected expected)
{
    if (actual == expected) {
        return true;
    }
    std::cerr << label << ": expected " << expected << ", got " << actual << '\n';
    return false;
}

}  // namespace

int main()
{
    bool ok = true;

    ok &= expectEqual("uint16 width", db_tsdf::mask_bits_v<uint16_t>, 16);
    ok &= expectEqual("uint32 width", db_tsdf::mask_bits_v<uint32_t>, 32);

    ok &= expectEqual("16-bit rank 0", db_tsdf::rankMask<uint16_t>(0), uint16_t{0x0000});
    ok &= expectEqual("16-bit rank 1", db_tsdf::rankMask<uint16_t>(1), uint16_t{0x0001});
    ok &= expectEqual("16-bit rank 15", db_tsdf::rankMask<uint16_t>(15), uint16_t{0x7fff});
    ok &= expectEqual("16-bit rank 16", db_tsdf::rankMask<uint16_t>(16), uint16_t{0xffff});
    ok &= expectEqual("16-bit saturation", db_tsdf::rankMask<uint16_t>(32), uint16_t{0xffff});

    ok &= expectEqual("32-bit rank 16", db_tsdf::rankMask<uint32_t>(16), uint32_t{0x0000ffff});
    ok &= expectEqual("32-bit rank 17", db_tsdf::rankMask<uint32_t>(17), uint32_t{0x0001ffff});
    ok &= expectEqual("32-bit rank 31", db_tsdf::rankMask<uint32_t>(31), uint32_t{0x7fffffff});
    ok &= expectEqual("32-bit rank 32", db_tsdf::rankMask<uint32_t>(32), uint32_t{0xffffffff});
    ok &= expectEqual("32-bit saturation", db_tsdf::rankMask<uint32_t>(64), uint32_t{0xffffffff});

    const auto rank_7 = db_tsdf::rankMask<uint32_t>(7);
    const auto rank_19 = db_tsdf::rankMask<uint32_t>(19);
    ok &= expectEqual("bitwise minimum rank", rank_7 & rank_19, rank_7);
    ok &= expectEqual("32-bit popcount", db_tsdf::maskRank(rank_19), 19);

    return ok ? 0 : 1;
}
