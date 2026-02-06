#ifndef STATUS_DETAIL_FLAG_H
#define STATUS_DETAIL_FLAG_H

#include <cstdint>

// Mirrors HB 5.4.8 Tripper.RecastManaged.Detour.StatusDetailFlag / dtStatus bits
enum class StatusDetailFlag : std::uint32_t
{
    Failure        = 1u << 31,
    Success        = 1u << 30,
    InProgress     = 1u << 29,
    WrongMagic     = 1u << 0,
    WrongVersion   = 1u << 1,
    OutOfMemory    = 1u << 2,
    InvalidParam   = 1u << 3,
    BufferTooSmall = 1u << 4,
    OutOfNodes     = 1u << 5,
    PartialResult  = 1u << 6
};

inline constexpr std::uint32_t StatusBits(StatusDetailFlag flag)
{
    return static_cast<std::uint32_t>(flag);
}

inline bool StatusHasFlag(std::uint32_t status, StatusDetailFlag flag)
{
    return (status & StatusBits(flag)) != 0;
}

#endif
