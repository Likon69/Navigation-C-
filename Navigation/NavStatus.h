#ifndef NAV_STATUS_H
#define NAV_STATUS_H

#include <cstdint>
#include "StatusDetailFlag.h"

// HB-style status flags aligned with dtStatus bits
enum NavStatusFlag : std::uint32_t
{
	NAV_FAILURE = static_cast<std::uint32_t>(StatusDetailFlag::Failure),
	NAV_SUCCESS = static_cast<std::uint32_t>(StatusDetailFlag::Success),
	NAV_IN_PROGRESS = static_cast<std::uint32_t>(StatusDetailFlag::InProgress),
	NAV_WRONG_MAGIC = static_cast<std::uint32_t>(StatusDetailFlag::WrongMagic),
	NAV_WRONG_VERSION = static_cast<std::uint32_t>(StatusDetailFlag::WrongVersion),
	NAV_OUT_OF_MEMORY = static_cast<std::uint32_t>(StatusDetailFlag::OutOfMemory),
	NAV_INVALID_PARAM = static_cast<std::uint32_t>(StatusDetailFlag::InvalidParam),
	NAV_BUFFER_TOO_SMALL = static_cast<std::uint32_t>(StatusDetailFlag::BufferTooSmall),
	NAV_OUT_OF_NODES = static_cast<std::uint32_t>(StatusDetailFlag::OutOfNodes),
	NAV_PARTIAL_RESULT = static_cast<std::uint32_t>(StatusDetailFlag::PartialResult)
};

// PathFind step enum (HB parity)
enum NavPathFindStep : int
{
	NAV_STEP_FIND_START_POLY = 0,
	NAV_STEP_FIND_END_POLY = 1,
	NAV_STEP_INIT_PATHFIND = 2,
	NAV_STEP_UPDATE_PATHFIND = 3,
	NAV_STEP_FINALIZE_PATHFIND = 4,
	NAV_STEP_FIND_STRAIGHT_PATH = 5,
	NAV_STEP_NONE = -1
};

inline bool NavStatusFailed(std::uint32_t status)
{
	return (status & NAV_FAILURE) != 0;
}

inline bool NavStatusSucceeded(std::uint32_t status)
{
	return (status & NAV_SUCCESS) != 0;
}

inline bool NavStatusInProgress(std::uint32_t status)
{
	return (status & NAV_IN_PROGRESS) != 0;
}

inline void NavStatusAddFlag(std::uint32_t& status, NavStatusFlag flag)
{
	status |= static_cast<std::uint32_t>(flag);
}

inline bool NavStatusHasFlag(std::uint32_t status, NavStatusFlag flag)
{
	return (status & static_cast<std::uint32_t>(flag)) != 0;
}

#endif
