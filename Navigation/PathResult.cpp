#include "PathResult.h"

PathResult::PathResult()
	: points(nullptr),
	  straightPathFlags(nullptr),
	  polyTypes(nullptr),
	  abilityFlags(nullptr),
	  polyRefs(nullptr),
	  length(0),
	  status(static_cast<std::uint32_t>(NAV_SUCCESS)),
	  failStep(NAV_STEP_NONE)
{
}
