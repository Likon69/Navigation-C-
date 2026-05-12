#pragma once
#ifdef _WIN32
  #define NAV_API __declspec(dllexport)
#else
  #define NAV_API
#endif

#include "PathResult.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct { float x,y,z; } XYZ_C;

typedef struct NavStats_C
{
  float PathfindTimeMs;
  int PolysVisited;
  float PathLength;
  int ShortcutsApplied;
  int StuckRecoveries;
  int PathRecalculations;
} NavStats_C;

// Loader
NAV_API bool Nav_LoadMaps(); // wrapper utilitaire -> appelle Navigation::GetMmapsPath + init
NAV_API void Nav_UnloadMaps();

// Pathfinding basic
NAV_API XYZ_C* CalculatePath_C(unsigned int mapId, XYZ_C start, XYZ_C end, bool straightPath, int* outLength);
NAV_API void FreePathArr_C(XYZ_C* arr);
NAV_API PathResult* CalculatePathEx(unsigned int mapId, XYZ_C start, XYZ_C end, bool straightPath);
NAV_API void FreePathResult(PathResult* result);

// Stubs / futures (implémenter si Navigation possède les méthodes)
// === Extended API (from Honorbuddy / RecastManager) ===
// Raycast: returns true and fills hitPos/tHit when a hit occurs along the segment
NAV_API bool Raycast_C(unsigned int mapId, XYZ_C start, XYZ_C end, XYZ_C* hitPos, float* tHit);
// Find nearest valid navmesh point to a position
NAV_API bool FindNearestPoint_C(unsigned int mapId, XYZ_C position, XYZ_C* nearest);
// Find nearest point with custom extents (for multi-layer search)
NAV_API bool FindNearestPointEx_C(unsigned int mapId, XYZ_C position, float extentX, float extentY, float extentZ, XYZ_C* nearest);
// Find a random navigable point around center within radius
NAV_API bool FindRandomPoint_C(unsigned int mapId, XYZ_C center, float radius, XYZ_C* randomPoint);
// Set cost for area type on specified map
NAV_API bool SetAreaCost_C(unsigned int mapId, int areaType, float cost);

// Agent corridor system (dtPathCorridor wrappers)
NAV_API bool CreateCorridorForAgent_C(int agentId, unsigned int mapId, XYZ_C start, XYZ_C end);
NAV_API bool UpdateCorridorAgentPosition_C(int agentId, XYZ_C newPos);

// HB 4.3.4: OffMesh Connection support
// Check if position is an offmesh connection, returns type and metadata
NAV_API bool IsOffMeshConnection_C(unsigned int mapId, XYZ_C position, 
                                    XYZ_C* outEnd, unsigned char* outType, unsigned int* outInteractId);
// Add custom offmesh connection at runtime
NAV_API void AddOffMeshConnection_C(unsigned int mapId, 
                                     XYZ_C start, XYZ_C end, 
                                     float radius, unsigned char flags,
                                     unsigned char type, unsigned int interactId);
// Load offmesh connections for a specific tile
NAV_API bool LoadTileOffMesh_C(unsigned int mapId, int tileX, int tileY);

// Filter + streaming helpers
NAV_API void SetIncludeFlags_C(unsigned short flags);
NAV_API void SetExcludeFlags_C(unsigned short flags);
NAV_API void WorldToTile_C(float worldX, float worldZ, int* tileX, int* tileY);
NAV_API void EnsureTiles_C(unsigned int mapId, XYZ_C position, int ring);
NAV_API void EnsureTilesDirectional_C(unsigned int mapId, XYZ_C position, XYZ_C velocity, int ring);
NAV_API int UpdatePathFollowing_C(unsigned int mapId, XYZ_C currentPos, int pathLength,
                                   XYZ_C* pathPoints, int currentWaypointIndex, int agentId);

// Corridor + telemetry helpers
NAV_API bool DestroyCorridorForAgent_C(int agentId);
// GetNavStats_C fills outStats in-place (avoids return-by-value struct on x86 Cdecl)
NAV_API void GetNavStats_C(NavStats_C* outStats);
NAV_API void ResetNavStats_C();

// Detour helper exports for HB parity
NAV_API bool FindNearestPolyRef_C(unsigned int mapId, XYZ_C position, XYZ_C extents,
                                   uint64_t* outPolyRef, XYZ_C* nearestPoint);
NAV_API bool GetPolyHeight_C(unsigned int mapId, uint64_t polyRef, XYZ_C position, float* outHeight);
NAV_API bool ClosestPointOnPoly_C(unsigned int mapId, uint64_t polyRef, XYZ_C position, XYZ_C* closestPoint);
NAV_API bool ClosestPointOnPolyBoundary_C(unsigned int mapId, uint64_t polyRef,
                                          XYZ_C position, XYZ_C* closestPoint);
NAV_API int QueryPolygons_C(unsigned int mapId, XYZ_C center, XYZ_C extents,
                             uint64_t* outPolys, int maxPolys);
NAV_API int FindLocalNeighbourhood_C(unsigned int mapId, uint64_t startPolyRef, XYZ_C center, float radius,
                                      uint64_t* outPolys, uint64_t* outParents, int maxResults);
NAV_API int GetPolyWallSegments_C(unsigned int mapId, uint64_t polyRef,
                                   XYZ_C* outSegmentStart, XYZ_C* outSegmentEnd, int maxSegments);

#ifdef __cplusplus
}
#endif

// == Added exports for CopilotBuddy NativeMethods parity ==
#ifdef __cplusplus
extern "C" {
#endif

// Callback convention matches MMAP::TileLoadedCallback (void __stdcall)(uint mapId, int x, int y)
typedef void (__stdcall *NavBridgeTileLoadedCallback)(unsigned int mapId, int x, int y);
NAV_API void SetTileLoadedCallback_C(NavBridgeTileLoadedCallback callback);

// Filter flag accessors
NAV_API unsigned short GetIncludeFlags_C(void);
NAV_API unsigned short GetExcludeFlags_C(void);
NAV_API float          GetAreaCost_C(int areaId);

// LOS + wall distance
NAV_API bool  HasLineOfSight_C(unsigned int mapId, XYZ_C start, XYZ_C end);
NAV_API float FindDistanceToWall_C(unsigned int mapId, XYZ_C position, float maxRadius, XYZ_C* hitPoint);
NAV_API float FindDistanceToWallEx_C(unsigned int mapId, XYZ_C position, float maxRadius, XYZ_C* hitPoint, XYZ_C* outHitNormal);
NAV_API float FindDistanceToWallFromPoly_C(unsigned int mapId, uint64_t polyRef, XYZ_C position, float maxRadius, XYZ_C* hitPoint, XYZ_C* outHitNormal);
NAV_API bool  IsPointOnNavMesh_C(unsigned int mapId, XYZ_C point, float tolerance);

// Tile state
NAV_API bool IsTileLoaded_C(unsigned int mapId, int x, int y);
NAV_API int  GetLoadedTilesCount_C(unsigned int mapId);
// Polygon area/flags manipulation — for blackspot marking (matches HB Tripper.RecastManaged.NavMesh)
NAV_API unsigned int SetPolyArea_C(unsigned int mapId, uint64_t polyRef, unsigned char area);
NAV_API unsigned int GetPolyArea_C(unsigned int mapId, uint64_t polyRef, unsigned char* outArea);
NAV_API unsigned int SetPolyFlags_C(unsigned int mapId, uint64_t polyRef, unsigned short flags);
NAV_API unsigned int GetPolyFlags_C(unsigned int mapId, uint64_t polyRef, unsigned short* outFlags);
// HB-style raycast — resolves start poly and exposes visited poly corridor
// outPath receives uint64_t poly refs (ToExternalRef converted, 32->64 bit)
NAV_API unsigned int Raycast_HB_C(unsigned int mapId, uint64_t startRef,
                                   XYZ_C startPos, XYZ_C endPos,
                                   float* outT, XYZ_C* outHitNormal,
                                   uint64_t* outPath, int* outPathCount, int maxPath);

#ifdef __cplusplus
}
#endif

// == Old-API exports (no _C suffix) — used by NativeMethods.cs P/Invoke ==
// These are defined in DllMain.cpp alongside the _C variants above.
// Do NOT rename — NativeMethods.cs EntryPoint strings must match exactly.
#ifdef __cplusplus
extern "C" {
#endif

// Sliced pathfinding (HB-style async search, splits work across frames)
NAV_API bool     InitSlicedFindPath(unsigned int mapId, XYZ_C start, XYZ_C end);
NAV_API bool     UpdateSlicedFindPath(int maxIterations);
NAV_API bool     UpdateSlicedFindPathMs(float msBudget);
NAV_API XYZ_C*   FinalizeSlicedFindPath(int maxPathSize, int* outLength);

// Poly neighborhood search
NAV_API int      FindPolysAroundCircle(unsigned int mapId, XYZ_C center, float radius,
                                        XYZ_C* results, int maxResults);

// Raw Detour object access (returned as void* — cast by caller)
NAV_API void*    GetNavMeshQuery(unsigned int mapId);
NAV_API void*    GetDefaultFilter(void);

// NavStatus bit accessors (uint32_t mirrors of dtStatus bit fields)
NAV_API uint32_t NavStatus_FailureFlag(void);
NAV_API uint32_t NavStatus_SuccessFlag(void);
NAV_API uint32_t NavStatus_InProgressFlag(void);
NAV_API uint32_t NavStatus_PartialResultFlag(void);
NAV_API bool     NavStatus_IsFailure(uint32_t status);
NAV_API bool     NavStatus_IsSuccess(uint32_t status);
NAV_API bool     NavStatus_IsInProgress(uint32_t status);
NAV_API bool     NavStatus_HasFlag(uint32_t status, uint32_t flag);

#ifdef __cplusplus
}
#endif
