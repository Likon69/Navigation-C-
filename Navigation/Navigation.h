#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "MoveMap.h"
#include "NavTypes.h"
#include "NavStatus.h"
#include "PathResult.h"
#include <vector>
#include <string>
#include <unordered_map>
#include <cstdint> // uint64_t for timestamps

// Note: dtPolyRef is unsigned int (32-bit) by default, matching HB WoD
// DT_POLYREF64 has been removed from PreprocessorDefinitions in vcxproj

// AMÃ‰LIORATION #2: Navigation statistics for profiling & debugging
struct NavStats
{
	float pathfindTimeMs;      // Time spent in A* pathfinding
	int polysVisited;          // Number of polygons visited during A*
	float pathLength;          // Total path length in yards
	int shortcutsApplied;      // Number of raycast shortcuts applied
	int stuckRecoveries;       // Number of stuck detection triggers
	int pathRecalculations;    // Number of path recalculations
	int raycastAttempts;       // Number of raycast shortcut tests
	int raycastHits;           // Number of successful raycast shortcuts
	int lastShortcutIndex;     // Last waypoint index selected by shortcut
	float lastShortcutDistance;// Distance gained by last shortcut
	float lastRaycastHitFraction; // Raycast hit fraction (1 = clear)
	int corridorLength;        // True Detour A* poly corridor length (m_polyLength)
	
	NavStats() : pathfindTimeMs(0), polysVisited(0), pathLength(0), 
	             shortcutsApplied(0), stuckRecoveries(0), pathRecalculations(0),
	             raycastAttempts(0), raycastHits(0), lastShortcutIndex(0),
	             lastShortcutDistance(0.0f), lastRaycastHitFraction(1.0f),
	             corridorLength(0) {}
};

class Navigation
{
public:
	static Navigation* GetInstance();
	void Initialize();
	void Release();
	XYZ* CalculatePath(unsigned int mapId, XYZ start, XYZ end, bool straightPath, int* length);
	void FreePathArr(XYZ* length);
	PathResult* CalculatePathEx(unsigned int mapId, XYZ start, XYZ end, bool straightPath);
	void FreePathResult(PathResult* result);
    std::string GetMmapsPath();
    
    // Advanced Detour functions - like Honorbuddy RecastManaged
    bool FindNearestPoly(unsigned int mapId, XYZ center, float searchRadius, XYZ* nearestPoint);
    int FindPolysAroundCircle(unsigned int mapId, XYZ center, float radius, XYZ* results, int maxResults);
    float FindDistanceToWall(unsigned int mapId, XYZ position, float maxRadius, XYZ* hitPoint);
    float FindDistanceToWallEx(unsigned int mapId, XYZ position, float maxRadius, XYZ* hitPoint, XYZ* outHitNormal);
    float FindDistanceToWallFromPoly(unsigned int mapId, dtPolyRef polyRef, XYZ position, float maxRadius, XYZ* hitPoint, XYZ* outHitNormal);
    bool IsPointOnNavMesh(unsigned int mapId, XYZ point, float tolerance);
    XYZ FindRandomPointAroundCircle(unsigned int mapId, XYZ center, float radius);
    bool HasLineOfSight(unsigned int mapId, XYZ start, XYZ end);
    
    // Raycast - HB-style raycast with full output (t, hitNormal, visited polys)
    // Note: dtPolyRef is unsigned int (32-bit) - DT_POLYREF64 removed from vcxproj
    // Returns dtStatus, t=1.0 means no hit (clear path)
    unsigned int Raycast(unsigned int mapId, dtPolyRef startRef, XYZ startPos, XYZ endPos,
        float* outT, XYZ* outHitNormal, dtPolyRef* outPath, int* outPathCount, int maxPath);

    // HB 6.2.3 NavMesh.GetMaxTiles
    int GetMaxTiles(unsigned int mapId);

    // HB 6.2.3 NavMesh.EncodePolyId / DecodePolyId*
    // mapId is needed to resolve the dtNavMesh instance (Detour's decode uses
    // m_saltBits/m_tileBits/m_polyBits which are per-mesh). HB 6.2.3 has a
    // single mesh (this._mesh), CopilotBuddy has many.
    dtPolyRef EncodePolyId(unsigned int mapId, unsigned int salt, unsigned int it, unsigned int ip);
    void DecodePolyId(unsigned int mapId, dtPolyRef polyRef, unsigned int* outSalt, unsigned int* outIt, unsigned int* outIp);
    unsigned int DecodePolyIdSalt(unsigned int mapId, dtPolyRef polyRef);
    unsigned int DecodePolyIdTile(unsigned int mapId, dtPolyRef polyRef);
    unsigned int DecodePolyIdPoly(unsigned int mapId, dtPolyRef polyRef);

    // HB 6.2.3 NavMesh.GetTile* — returns opaque tile pointer (cast on C# side as IntPtr)
    // nullptr when tile is not present at the given index/coord.
    const void* GetTileAt(unsigned int mapId, int x, int y);
    const void* GetTile(unsigned int mapId, int i);

    // HB 6.2.3 NavMesh tile state save/restore — tile is the opaque pointer
    // returned by GetTileAt/GetTile. mapId is needed to resolve the dtNavMesh
    // instance (Detour's getTileStateSize/storeTileState are dtNavMesh methods,
    // not free functions). outData must be at least maxDataSize bytes
    // (caller-allocated buffer, see GetTileStateSize for the required size).
    int GetTileStateSize(unsigned int mapId, const void* tile);
    unsigned int StoreTileState(unsigned int mapId, const void* tile, unsigned char* outData, int maxDataSize);
    unsigned int RestoreTileState(unsigned int mapId, void* tile, const unsigned char* data, int dataSize);

	bool FindNearestPolyRef(unsigned int mapId, XYZ position, XYZ extents,
		dtPolyRef* outPolyRef, XYZ* nearestPoint);
	bool GetPolyHeight(unsigned int mapId, dtPolyRef polyRef, XYZ position, float* outHeight);
	bool ClosestPointOnPoly(unsigned int mapId, dtPolyRef polyRef, XYZ position, XYZ* closestPoint);
	bool ClosestPointOnPolyBoundary(unsigned int mapId, dtPolyRef polyRef, XYZ position, XYZ* closestPoint);
	int QueryPolygons(unsigned int mapId, XYZ center, XYZ extents, dtPolyRef* outPolys, int maxPolys);
	int FindLocalNeighbourhood(unsigned int mapId, dtPolyRef startPolyRef, XYZ center, float radius,
		dtPolyRef* outPolys, dtPolyRef* outParents, int maxResults);
	int GetPolyWallSegments(unsigned int mapId, dtPolyRef polyRef, XYZ* outSegmentStart,
		XYZ* outSegmentEnd, int maxSegments);
    
    // Polygon area/flags manipulation (like HB Tripper.RecastManaged - for blackspot marking)
    unsigned int SetPolyArea(unsigned int mapId, dtPolyRef polyRef, unsigned char area);
    unsigned int GetPolyArea(unsigned int mapId, dtPolyRef polyRef, unsigned char* outArea);
    unsigned int SetPolyFlags(unsigned int mapId, dtPolyRef polyRef, unsigned short flags);
    unsigned int GetPolyFlags(unsigned int mapId, dtPolyRef polyRef, unsigned short* outFlags);
    
    // Honorbuddy-style Sliced PathFinding (pour pathfinding asynchrone)
    bool InitSlicedFindPath(unsigned int mapId, XYZ start, XYZ end);
    bool UpdateSlicedFindPath(int maxIterations);
    bool UpdateSlicedFindPathMs(float msBudget); // QUICK WIN #2: Adaptive sliced with ms budget
    XYZ* FinalizeSlicedFindPath(int maxPathSize, int* length);    // AMÉLIORATION: Filtre global flags (Action 2)
	void SetIncludeFlags(unsigned short flags);
	void SetExcludeFlags(unsigned short flags);
	unsigned short GetIncludeFlags() const { return _includeFlags; }
	unsigned short GetExcludeFlags() const { return _excludeFlags; }


    
    // Query Filter avec Area Cost (optimisation Honorbuddy)
    void SetAreaCost(unsigned int areaId, float cost);
    float GetAreaCost(unsigned int areaId);
    
    // Tile management optimisÃ© (comme Honorbuddy)
    bool IsTileLoaded(unsigned int mapId, int x, int y);
    bool UnloadTile(unsigned int mapId, int x, int y);
    int GetLoadedTilesCount(unsigned int mapId);

    // HB 6.2.3 pattern: callback when a tile is loaded (mirrors SetTileLoaderFunction)
    void SetTileLoadedCallback(MMAP::TileLoadedCallback cb);
    
	// QUICK WIN #1: Raycast shortcut during following
	int UpdatePathFollowing(unsigned int mapId, XYZ currentPos, int pathLength,
		const XYZ* pathPoints, int currentWaypointIndex, int agentId);
    
    // AMÃ‰LIORATION #2: Get navigation statistics
    NavStats GetNavStats() const { return _stats; }
    void ResetNavStats() { _stats = NavStats(); }
    
    // NavBridge support: Allow direct query/navmesh access like Honorbuddy
    dtNavMeshQuery* GetNavMeshQuery(unsigned int mapId);
    dtQueryFilter* GetDefaultFilter() { return &_defaultFilter; }
    
    // Tile streaming — HB 6.2.3 pattern
    void EnsureTiles(unsigned int mapId, XYZ position, int ring = 1);
    void EnsureTilesDirectional(unsigned int mapId, XYZ position, XYZ velocity, int ring = 1);

private:
	static Navigation* s_singletonInstance;
	XYZ* currentPath;
	
	// Honorbuddy-style optimizations: Sliced pathfinding state
	unsigned int _slicedFindPath_mapId;
	dtPolyRef _slicedFindPath_startRef;
	dtPolyRef _slicedFindPath_endRef;
	dtQueryFilter _defaultFilter; // RÃ©utilisable pour Ã©viter new/delete
	float _areaCosts[DT_MAX_AREAS]; // Area cost cache comme Honorbuddy
	unsigned short _includeFlags; // AMÉLIORATION: Include flags (Action 2)
	unsigned short _excludeFlags; // AMÉLIORATION: Exclude flags (Action 2)
	
	// HB 6.2.3 Tripper uses 8192 for FinalizeSlicedFindPath and FindStraightPath.
	static const int MAX_POLYS_BUFFER = 8192;
	dtPolyRef _polysBuffer[MAX_POLYS_BUFFER]; // RÃ©utilisÃ© entre appels
	
	// QUICK WIN #2: Adaptive sliced pathfinding calibration
	float _itersPerMs; // Auto-calibrated iterations per millisecond
	int EstimateIterations(float msBudget);
	void TuneIterationsPerMs(float elapsedMs, int itersExecuted);
	
	// AMÃ‰LIORATION #2: Navigation statistics tracking
	NavStats _stats;
	
	// Tile streaming — time-based GC (HB 6.2.3 pattern: 1 minute idle before eviction)
	struct TileKey {
		unsigned int mapId;
		int x, y;
		bool operator==(const TileKey& o) const { return mapId == o.mapId && x == o.x && y == o.y; }
	};
	struct TileKeyHash {
		std::size_t operator()(const TileKey& k) const {
			std::size_t h = k.mapId;
			h ^= (std::size_t)k.x * 2654435761u + 0x9e3779b9u + (h << 6) + (h >> 2);
			h ^= (std::size_t)k.y * 2246822519u + 0x9e3779b9u + (h << 6) + (h >> 2);
			return h;
		}
	};
	std::unordered_map<TileKey, uint64_t, TileKeyHash> _tileAccessTime;
	static constexpr uint64_t TILE_GC_TIMEOUT_MS = 60000; // 1 minute, like HB GarbageCollectTime
	
	void WorldToTile(float worldX, float worldY, int* tileX, int* tileY);
	void GarbageCollectTiles();
	
};

#endif
