#ifndef PATH_RESULT_H
#define PATH_RESULT_H

#include "NavTypes.h"
#include "NavStatus.h"
#include "StraightPathFlags.h"
#include <cstdint>

struct PathResult
{
	XYZ* points;
	StraightPathFlags* straightPathFlags;
	unsigned char* polyTypes;
	unsigned char* abilityFlags;
	std::uint64_t* polyRefs;      // NEW: Polygon references for each waypoint (dtPolyRef)
	int length;
	std::uint32_t status;
	NavPathFindStep failStep;

	PathResult();
};

#endif
