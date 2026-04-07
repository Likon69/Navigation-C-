#include "Navigation.h"
#include <windows.h>
#include <vector>

namespace
{
	inline unsigned long long ToExternalRef(dtPolyRef ref)
	{
		return static_cast<unsigned long long>(ref);
	}

	inline dtPolyRef ToInternalRef(unsigned long long ref)
	{
		return static_cast<dtPolyRef>(ref);
	}
}

// Randomization globals use C++ linkage so Detour references can resolve
bool g_dtPathRandomizationEnabled = false;
float g_dtPathRandomizationMagnitude = 0.0f; // 0..1 range (e.g. 0.05 = +/-5%)

// Slope penalty: extra cost per unit of elevation change in getCost().
// Penalizes mountain shortcuts where the path goes uphill/downhill.
// HB avoids these structurally via 133y tiles; we compensate with cost.
float g_dtSlopePenaltyFactor = 5.0f;

extern "C"
{
	__declspec(dllexport) XYZ* CalculatePath(unsigned int mapId, XYZ start, XYZ end, bool smoothPath, int* length)
	{
		return Navigation::GetInstance()->CalculatePath(mapId, start, end, smoothPath, length);
	}

	__declspec(dllexport) void FreePathArr(XYZ* pathArr)
	{
		return Navigation::GetInstance()->FreePathArr(pathArr);
	}
	
	// Advanced Detour Navigation Functions - Like Honorbuddy RecastManaged
	__declspec(dllexport) bool FindNearestPoly(unsigned int mapId, XYZ center, float searchRadius, XYZ* nearestPoint)
	{
		return Navigation::GetInstance()->FindNearestPoly(mapId, center, searchRadius, nearestPoint);
	}
	
	__declspec(dllexport) int FindPolysAroundCircle(unsigned int mapId, XYZ center, float radius, XYZ* results, int maxResults)
	{
		return Navigation::GetInstance()->FindPolysAroundCircle(mapId, center, radius, results, maxResults);
	}
	
	__declspec(dllexport) float FindDistanceToWall(unsigned int mapId, XYZ position, float maxRadius, XYZ* hitPoint)
	{
		return Navigation::GetInstance()->FindDistanceToWall(mapId, position, maxRadius, hitPoint);
	}
	
	__declspec(dllexport) float FindDistanceToWallEx(unsigned int mapId, XYZ position, float maxRadius, XYZ* hitPoint, XYZ* hitNormal)
	{
		return Navigation::GetInstance()->FindDistanceToWallEx(mapId, position, maxRadius, hitPoint, hitNormal);
	}
	
	__declspec(dllexport) float FindDistanceToWallFromPoly(unsigned int mapId, unsigned long long polyRef, XYZ position, float maxRadius, XYZ* hitPoint, XYZ* hitNormal)
	{
		return Navigation::GetInstance()->FindDistanceToWallFromPoly(mapId, static_cast<dtPolyRef>(polyRef), position, maxRadius, hitPoint, hitNormal);
	}
	
	__declspec(dllexport) bool IsPointOnNavMesh(unsigned int mapId, XYZ point, float tolerance)
	{
		return Navigation::GetInstance()->IsPointOnNavMesh(mapId, point, tolerance);
	}
	
	__declspec(dllexport) bool FindRandomPointAroundCircle(unsigned int mapId, XYZ center, float radius, XYZ* outResult)
	{
		if (!outResult) return false;
		*outResult = Navigation::GetInstance()->FindRandomPointAroundCircle(mapId, center, radius);
		return true;
	}
	
	__declspec(dllexport) bool HasLineOfSight(unsigned int mapId, XYZ start, XYZ end)
	{
		return Navigation::GetInstance()->HasLineOfSight(mapId, start, end);
	}

	// ===== Raycast - HB-style full raycast with detailed output =====
	// Note: dtPolyRef is unsigned long long (64-bit) with DT_POLYREF64
	__declspec(dllexport) unsigned int Raycast(unsigned int mapId, unsigned long long startRef,
		XYZ startPos, XYZ endPos, float* outT, XYZ* outHitNormal,
		unsigned long long* outPath, int* outPathCount, int maxPath)
	{
		std::vector<dtPolyRef> pathRefs(static_cast<size_t>(maxPath), 0);
		unsigned int status = Navigation::GetInstance()->Raycast(mapId, ToInternalRef(startRef), startPos, endPos,
			outT, outHitNormal, pathRefs.data(), outPathCount, maxPath);
		if (outPath && outPathCount && *outPathCount > 0)
		{
			for (int i = 0; i < *outPathCount; ++i)
				outPath[i] = ToExternalRef(pathRefs[i]);
		}
		return status;
	}

	__declspec(dllexport) bool FindNearestPolyRef(unsigned int mapId, XYZ position, XYZ extents,
		unsigned long long* outPolyRef, XYZ* nearestPoint)
	{
		dtPolyRef ref = 0;
		bool result = Navigation::GetInstance()->FindNearestPolyRef(mapId, position, extents, &ref, nearestPoint);
		if (result && outPolyRef)
			*outPolyRef = ToExternalRef(ref);
		return result;
	}

	__declspec(dllexport) bool GetPolyHeight(unsigned int mapId, unsigned long long polyRef, XYZ position, float* outHeight)
	{
		return Navigation::GetInstance()->GetPolyHeight(mapId, ToInternalRef(polyRef), position, outHeight);
	}

	__declspec(dllexport) bool ClosestPointOnPoly(unsigned int mapId, unsigned long long polyRef, XYZ position, XYZ* closestPoint)
	{
		return Navigation::GetInstance()->ClosestPointOnPoly(mapId, ToInternalRef(polyRef), position, closestPoint);
	}

	__declspec(dllexport) bool ClosestPointOnPolyBoundary(unsigned int mapId, unsigned long long polyRef, XYZ position, XYZ* closestPoint)
	{
		return Navigation::GetInstance()->ClosestPointOnPolyBoundary(mapId, ToInternalRef(polyRef), position, closestPoint);
	}

	// ===== Polygon area/flags manipulation (like HB Tripper.RecastManaged - for blackspot marking) =====
	
	__declspec(dllexport) unsigned int SetPolyArea(unsigned int mapId, unsigned long long polyRef, unsigned char area)
	{
		return Navigation::GetInstance()->SetPolyArea(mapId, ToInternalRef(polyRef), area);
	}
	
	__declspec(dllexport) unsigned int GetPolyArea(unsigned int mapId, unsigned long long polyRef, unsigned char* outArea)
	{
		return Navigation::GetInstance()->GetPolyArea(mapId, ToInternalRef(polyRef), outArea);
	}
	
	__declspec(dllexport) unsigned int SetPolyFlags(unsigned int mapId, unsigned long long polyRef, unsigned short flags)
	{
		return Navigation::GetInstance()->SetPolyFlags(mapId, ToInternalRef(polyRef), flags);
	}
	
	__declspec(dllexport) unsigned int GetPolyFlags(unsigned int mapId, unsigned long long polyRef, unsigned short* outFlags)
	{
		return Navigation::GetInstance()->GetPolyFlags(mapId, ToInternalRef(polyRef), outFlags);
	}

	__declspec(dllexport) int QueryPolygons(unsigned int mapId, XYZ center, XYZ extents,
		unsigned long long* outPolys, int maxPolys)
	{
		if (!outPolys || maxPolys <= 0)
			return 0;

		std::vector<dtPolyRef> results(static_cast<size_t>(maxPolys), 0);
		int count = Navigation::GetInstance()->QueryPolygons(mapId, center, extents,
			results.data(), maxPolys);
		for (int i = 0; i < count; ++i)
			outPolys[i] = ToExternalRef(results[i]);
		return count;
	}

	__declspec(dllexport) int FindLocalNeighbourhood(unsigned int mapId, unsigned long long startPolyRef,
		XYZ center, float radius, unsigned long long* outPolys, unsigned long long* outParents, int maxResults)
	{
		if (!outPolys || maxResults <= 0)
			return 0;

		std::vector<dtPolyRef> polys(static_cast<size_t>(maxResults), 0);
		std::vector<dtPolyRef> parents(static_cast<size_t>(maxResults), 0);
		int count = Navigation::GetInstance()->FindLocalNeighbourhood(mapId, ToInternalRef(startPolyRef),
			center, radius, polys.data(), parents.data(), maxResults);
		for (int i = 0; i < count; ++i)
		{
			outPolys[i] = ToExternalRef(polys[i]);
			if (outParents)
				outParents[i] = ToExternalRef(parents[i]);
		}
		return count;
	}

	__declspec(dllexport) int GetPolyWallSegments(unsigned int mapId, unsigned long long polyRef,
		XYZ* outSegmentStart, XYZ* outSegmentEnd, int maxSegments)
	{
		if (!outSegmentStart || !outSegmentEnd || maxSegments <= 0)
			return 0;

		std::vector<XYZ> starts(static_cast<size_t>(maxSegments));
		std::vector<XYZ> ends(static_cast<size_t>(maxSegments));
		int count = Navigation::GetInstance()->GetPolyWallSegments(mapId, ToInternalRef(polyRef),
			starts.data(), ends.data(), maxSegments);
		if (count <= 0)
			return 0;

		for (int i = 0; i < count; ++i)
		{
			outSegmentStart[i] = starts[i];
			outSegmentEnd[i] = ends[i];
		}
		return count;
	}
	
	// ===== Honorbuddy-style Sliced PathFinding =====
	__declspec(dllexport) bool InitSlicedFindPath(unsigned int mapId, XYZ start, XYZ end)
	{
		return Navigation::GetInstance()->InitSlicedFindPath(mapId, start, end);
	}
	
	__declspec(dllexport) bool UpdateSlicedFindPath(int maxIterations)
	{
		return Navigation::GetInstance()->UpdateSlicedFindPath(maxIterations);
	}
	
	// QUICK WIN #2: Adaptive sliced pathfinding with ms budget
	__declspec(dllexport) bool UpdateSlicedFindPathMs(float msBudget)
	{
		return Navigation::GetInstance()->UpdateSlicedFindPathMs(msBudget);
	}
	
	__declspec(dllexport) XYZ* FinalizeSlicedFindPath(int maxPathSize, int* length)
	{
		return Navigation::GetInstance()->FinalizeSlicedFindPath(maxPathSize, length);
	}
	
	// ===== Query Filter avec Area Cost =====
	// AMÉLIORATION: Filtre global flags (Action 2)
__declspec(dllexport) void SetIncludeFlags(unsigned short flags)
{
    Navigation::GetInstance()->SetIncludeFlags(flags);
}

__declspec(dllexport) void SetExcludeFlags(unsigned short flags)
{
    Navigation::GetInstance()->SetExcludeFlags(flags);
}

__declspec(dllexport) unsigned short GetIncludeFlags()
{
    return Navigation::GetInstance()->GetIncludeFlags();
}

__declspec(dllexport) unsigned short GetExcludeFlags()
{
    return Navigation::GetInstance()->GetExcludeFlags();
}

__declspec(dllexport) void SetAreaCost(unsigned int areaId, float cost)
	{
		Navigation::GetInstance()->SetAreaCost(areaId, cost);
	}
	
	__declspec(dllexport) float GetAreaCost(unsigned int areaId)
	{
		return Navigation::GetInstance()->GetAreaCost(areaId);
	}
	
	// ===== HB 6.2.3 pattern: tile loaded callback =====
	__declspec(dllexport) void SetTileLoadedCallback(void(__stdcall* callback)(unsigned int, int, int))
	{
		Navigation::GetInstance()->SetTileLoadedCallback(callback);
	}

	// ===== Tile Management OptimisÃ© =====
	__declspec(dllexport) bool IsTileLoaded(unsigned int mapId, int x, int y)
	{
		return Navigation::GetInstance()->IsTileLoaded(mapId, x, y);
	}
	
	__declspec(dllexport) int GetLoadedTilesCount(unsigned int mapId)
	{
		return Navigation::GetInstance()->GetLoadedTilesCount(mapId);
	}
	
	// ===== QUICK WIN #1: Raycast Shortcut During Following =====
	__declspec(dllexport) int UpdatePathFollowing(unsigned int mapId, XYZ currentPos, int pathLength,
		const XYZ* pathPoints, int currentWaypointIndex, int agentId)
	{
		return Navigation::GetInstance()->UpdatePathFollowing(mapId, currentPos, pathLength,
			pathPoints, currentWaypointIndex, agentId);
	}
	
	// ===== AMÃ‰LIORATION #2: Get Navigation Statistics =====
	__declspec(dllexport) void GetNavStats(NavStats* outStats)
	{
		if (outStats)
			*outStats = Navigation::GetInstance()->GetNavStats();
	}
	
	__declspec(dllexport) void ResetNavStats()
	{
		Navigation::GetInstance()->ResetNavStats();
	}

	// ===== Randomized path cost (natural variation to avoid robotic paths) =====
	__declspec(dllexport) void SetPathRandomization(bool enabled, float magnitude)
	{
		g_dtPathRandomizationEnabled = enabled;
		if (magnitude < 0.f) magnitude = 0.f;
		if (magnitude > 1.f) magnitude = 1.f;
		g_dtPathRandomizationMagnitude = magnitude;
	}

	// ===== Slope penalty: penalize elevation changes to avoid mountain shortcuts =====
	__declspec(dllexport) void SetSlopePenalty(float factor)
	{
		if (factor < 0.f) factor = 0.f;
		g_dtSlopePenaltyFactor = factor;
	}

	// ===== EnsureTiles - Load tiles around position (HB-style streaming) =====
	__declspec(dllexport) void EnsureTiles(unsigned int mapId, XYZ position, int ring)
	{
		Navigation::GetInstance()->EnsureTiles(mapId, position, ring);
	}

	// ===== EnsureTilesDirectional - Prefetch tiles in movement direction =====
	__declspec(dllexport) void EnsureTilesDirectional(unsigned int mapId, XYZ position, XYZ velocity, int ring)
	{
		Navigation::GetInstance()->EnsureTilesDirectional(mapId, position, velocity, ring);
	}

	// ===== GetNavMeshQuery - Direct query access for advanced use =====
	__declspec(dllexport) void* GetNavMeshQuery(unsigned int mapId)
	{
		return static_cast<void*>(Navigation::GetInstance()->GetNavMeshQuery(mapId));
	}

	// ===== GetDefaultFilter - Direct filter access =====
	__declspec(dllexport) void* GetDefaultFilter()
	{
		return static_cast<void*>(Navigation::GetInstance()->GetDefaultFilter());
	}

	// ===== NavStatus helpers exposed for managed layer =====
	__declspec(dllexport) std::uint32_t NavStatus_FailureFlag()
	{
		return static_cast<std::uint32_t>(NAV_FAILURE);
	}

	__declspec(dllexport) std::uint32_t NavStatus_SuccessFlag()
	{
		return static_cast<std::uint32_t>(NAV_SUCCESS);
	}

	__declspec(dllexport) std::uint32_t NavStatus_InProgressFlag()
	{
		return static_cast<std::uint32_t>(NAV_IN_PROGRESS);
	}

	__declspec(dllexport) std::uint32_t NavStatus_WrongMagicFlag()
	{
		return StatusBits(StatusDetailFlag::WrongMagic);
	}

	__declspec(dllexport) std::uint32_t NavStatus_WrongVersionFlag()
	{
		return StatusBits(StatusDetailFlag::WrongVersion);
	}

	__declspec(dllexport) std::uint32_t NavStatus_OutOfMemoryFlag()
	{
		return StatusBits(StatusDetailFlag::OutOfMemory);
	}

	__declspec(dllexport) std::uint32_t NavStatus_InvalidParamFlag()
	{
		return StatusBits(StatusDetailFlag::InvalidParam);
	}

	__declspec(dllexport) std::uint32_t NavStatus_BufferTooSmallFlag()
	{
		return StatusBits(StatusDetailFlag::BufferTooSmall);
	}

	__declspec(dllexport) std::uint32_t NavStatus_OutOfNodesFlag()
	{
		return StatusBits(StatusDetailFlag::OutOfNodes);
	}

	__declspec(dllexport) std::uint32_t NavStatus_PartialResultFlag()
	{
		return StatusBits(StatusDetailFlag::PartialResult);
	}

	__declspec(dllexport) bool NavStatus_IsFailure(std::uint32_t status)
	{
		return NavStatusFailed(status);
	}

	__declspec(dllexport) bool NavStatus_IsSuccess(std::uint32_t status)
	{
		return NavStatusSucceeded(status);
	}

	__declspec(dllexport) bool NavStatus_IsInProgress(std::uint32_t status)
	{
		return NavStatusInProgress(status);
	}

	__declspec(dllexport) bool NavStatus_HasFlag(std::uint32_t status, std::uint32_t flag)
	{
		return NavStatusHasFlag(status, static_cast<NavStatusFlag>(flag));
	}

	__declspec(dllexport) std::uint32_t NavStatus_AddFlag(std::uint32_t status, std::uint32_t flag)
	{
		status |= flag;
		return status;
	}
};

BOOL APIENTRY DllMain(HMODULE hModule, DWORD ul_reason_for_call, LPVOID lpReserved)
{
	Navigation* navigation = Navigation::GetInstance();
	switch (ul_reason_for_call)
	{
		case DLL_PROCESS_ATTACH:
			navigation->Initialize();
			break;

		case DLL_PROCESS_DETACH:
			navigation->Release();
			break;

		case DLL_THREAD_ATTACH:
			break;

		case DLL_THREAD_DETACH:
			break;
	}
	return TRUE;
}
