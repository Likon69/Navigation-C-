#ifndef STRAIGHT_PATH_FLAGS_H
#define STRAIGHT_PATH_FLAGS_H

#include <cstdint>

// Exact HB/Detour straight path flags (dtStraightPathFlags)
enum class StraightPathFlags : std::uint8_t
{
    None             = 0,
    Start            = 1 << 0,
    End              = 1 << 1,
    OffmeshConnection= 1 << 2
};

inline StraightPathFlags operator|(StraightPathFlags lhs, StraightPathFlags rhs)
{
    return static_cast<StraightPathFlags>(static_cast<std::uint8_t>(lhs) | static_cast<std::uint8_t>(rhs));
}

inline StraightPathFlags& operator|=(StraightPathFlags& lhs, StraightPathFlags rhs)
{
    lhs = lhs | rhs;
    return lhs;
}

inline bool HasFlag(StraightPathFlags value, StraightPathFlags flag)
{
    return (static_cast<std::uint8_t>(value) & static_cast<std::uint8_t>(flag)) != 0;
}

#endif
