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
NAV_API NavStats_C GetNavStats_C();
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
