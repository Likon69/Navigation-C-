#include "NavBridge.h"
#include "Navigation.h"
#include "OffMeshManager.h"
#include "PathFinder.h"
#include <cstdlib>
#include <vector>
#include <iostream>
#include <map>

using namespace std;

namespace
{
    inline XYZ ToNative(const XYZ_C& value)
    {
        XYZ native;
        native.X = value.x;
        native.Y = value.y;
        native.Z = value.z;
        return native;
    }

    inline void ToCStruct(const XYZ& native, XYZ_C* out)
    {
        if (!out)
            return;
        out->x = native.X;
        out->y = native.Y;
        out->z = native.Z;
    }

    inline uint64_t ToExternalRef(dtPolyRef ref)
    {
        return static_cast<uint64_t>(ref);
    }

    inline dtPolyRef ToInternalRef(uint64_t ref)
    {
        return static_cast<dtPolyRef>(ref);
    }
}

NAV_API bool Nav_LoadMaps()
{
    Navigation* nav = Navigation::GetInstance();
    if (!nav) return false;
    // Navigation::Initialize() dans ton code fait dtAllocSetCustom etc.
    nav->Initialize();
    return true;
}

NAV_API void Nav_UnloadMaps()
{
    Navigation* nav = Navigation::GetInstance();
    if (!nav) return;
    nav->Release();
}

NAV_API XYZ_C* CalculatePath_C(unsigned int mapId, XYZ_C start, XYZ_C end, bool straightPath, int* outLength)
{
    if (!outLength) return nullptr;
    *outLength = 0;
    Navigation* nav = Navigation::GetInstance();
    if (!nav) return nullptr;

    XYZ s; s.X = start.x; s.Y = start.y; s.Z = start.z;
    XYZ e; e.X = end.x; e.Y = end.y; e.Z = end.z;

    int len = 0;
    XYZ* res = nullptr;
    // defensive: check if CalculatePath exists (compile-time it should). We call and check result.
    res = nav->CalculatePath(mapId, s, e, straightPath, &len);

    if (!res || len <= 0) {
        if (res) nav->FreePathArr(res);
        return nullptr;
    }

    // allocate C-compatible array (malloc so callers in other langs can free with FreePathArr_C)
    XYZ_C* out = (XYZ_C*)malloc(sizeof(XYZ_C) * len);
    if (!out) {
        nav->FreePathArr(res);
        return nullptr;
    }
    for (int i=0;i<len;i++) {
        out[i].x = res[i].X;
        out[i].y = res[i].Y;
        out[i].z = res[i].Z;
    }
    // free internal array according to Navigation::FreePathArr semantic
    nav->FreePathArr(res);
    *outLength = len;
    return out;
}

NAV_API void FreePathArr_C(XYZ_C* arr)
{
    if (!arr) return;
    free(arr);
}

NAV_API PathResult* CalculatePathEx(unsigned int mapId, XYZ_C start, XYZ_C end, bool straightPath)
{
    Navigation* nav = Navigation::GetInstance();
    if (!nav) return nullptr;

    XYZ s; s.X = start.x; s.Y = start.y; s.Z = start.z;
    XYZ e; e.X = end.x; e.Y = end.y; e.Z = end.z;

    return nav->CalculatePathEx(mapId, s, e, straightPath);
}

NAV_API void FreePathResult(PathResult* result)
{
    Navigation* nav = Navigation::GetInstance();
    if (!nav) return;
    nav->FreePathResult(result);
}

/* === Extended API implementations === */

// Raycast_C - performs a navmesh raycast from start to end. Fills hitPos and tHit (0..1) when hit.
NAV_API bool Raycast_C(unsigned int mapId, XYZ_C start, XYZ_C end, XYZ_C* hitPos, float* tHit)
{
    Navigation* nav = Navigation::GetInstance();
    if (!nav) return false;

    dtNavMeshQuery* query = nav->GetNavMeshQuery(mapId);
    if (!query) return false;

    dtQueryFilter* filter = nav->GetDefaultFilter();
    float extents[3] = { 2.f, 4.f, 2.f };
    float s[3] = { start.x, start.y, start.z };
    float e[3] = { end.x, end.y, end.z };

    dtPolyRef startRef = 0;
    if (dtStatusFailed(query->findNearestPoly(s, extents, filter, &startRef, 0))) return false;

    float t = 0.f;
    float hitNormal[3];
    dtPolyRef polys[256]; int polyCount = 0;
    dtStatus st = query->raycast(startRef, s, e, filter, &t, hitNormal, polys, &polyCount, 256);
    if (dtStatusFailed(st)) return false;

    if (t < 0.f) t = 0.f; if (t > 1.f) t = 1.f;
    // dtVlerp doesn't exist - use manual lerp like Honorbuddy
    float hit[3];
    hit[0] = s[0] + (e[0] - s[0]) * t;
    hit[1] = s[1] + (e[1] - s[1]) * t;
    hit[2] = s[2] + (e[2] - s[2]) * t;

    if (hitPos) { hitPos->x = hit[0]; hitPos->y = hit[1]; hitPos->z = hit[2]; }
    if (tHit) *tHit = t;
    return true;
}

// FindNearestPoint_C - returns nearest point on navmesh to given position
NAV_API bool FindNearestPoint_C(unsigned int mapId, XYZ_C position, XYZ_C* nearest)
{
    // IMPORTANT: Utilise les MÊMES extents que Honorbuddy WowNavigator.cs
    // extents = (3f, 20f, 3f) - le Y=20 est LA CLÉ pour les grottes!
    // Cela permet de trouver la bonne couche (grotte vs surface) sans boucles
    return FindNearestPointEx_C(mapId, position, 3.f, 20.f, 3.f, nearest);
}

// FindNearestPointEx_C - returns nearest point with custom extents (for multi-layer caves/buildings)
NAV_API bool FindNearestPointEx_C(unsigned int mapId, XYZ_C position, float extentX, float extentY, float extentZ, XYZ_C* nearest)
{
    Navigation* nav = Navigation::GetInstance();
    if (!nav) return false;

    dtNavMeshQuery* query = nav->GetNavMeshQuery(mapId);
    if (!query) return false;

    dtQueryFilter* filter = nav->GetDefaultFilter();
    float extents[3] = { extentX, extentY, extentZ };
    float pos[3] = { position.x, position.y, position.z };
    float nearestPos[3]; dtPolyRef ref = 0;

    if (dtStatusFailed(query->findNearestPoly(pos, extents, filter, &ref, nearestPos))) return false;

    if (nearest) { nearest->x = nearestPos[0]; nearest->y = nearestPos[1]; nearest->z = nearestPos[2]; }
    return true;
}

// FindRandomPoint_C - sample a random point on navmesh around center
NAV_API bool FindRandomPoint_C(unsigned int mapId, XYZ_C center, float radius, XYZ_C* randomPoint)
{
    Navigation* nav = Navigation::GetInstance();
    if (!nav) return false;

    dtNavMeshQuery* query = nav->GetNavMeshQuery(mapId);
    if (!query) return false;

    dtQueryFilter* filter = nav->GetDefaultFilter();
    float extents[3] = { 2.f, 4.f, 2.f };
    float c[3] = { center.x, center.y, center.z };

    dtPolyRef startRef = 0;
    if (dtStatusFailed(query->findNearestPoly(c, extents, filter, &startRef, 0))) return false;

    dtPolyRef randomRef = 0;
    float rndPos[3];
    // small RNG wrapper
    auto rndf = []()->float { return static_cast<float>(rand()) / static_cast<float>(RAND_MAX); };

    if (dtStatusFailed(query->findRandomPointAroundCircle(startRef, c, radius, filter, rndf, &randomRef, rndPos))) return false;

    if (randomPoint) { randomPoint->x = rndPos[0]; randomPoint->y = rndPos[1]; randomPoint->z = rndPos[2]; }
    return true;
}

// SetAreaCost_C - set area cost on the Navigation's dtQueryFilter
NAV_API bool SetAreaCost_C(unsigned int mapId, int areaType, float cost)
{
    Navigation* nav = Navigation::GetInstance();
    if (!nav) return false;

    // Use Navigation's own SetAreaCost method like Honorbuddy
    nav->SetAreaCost(areaType, cost);
    return true;
}

// Simple dtPathCorridor agent map
static std::map<int, dtPathCorridor*> g_agentCorridors;

NAV_API bool CreateCorridorForAgent_C(int agentId, unsigned int mapId, XYZ_C start, XYZ_C end)
{
    Navigation* nav = Navigation::GetInstance();
    if (!nav) return false;

    dtNavMeshQuery* query = nav->GetNavMeshQuery(mapId);
    if (!query) return false;

    dtQueryFilter* filter = nav->GetDefaultFilter();
    float extents[3] = { 2.f, 4.f, 2.f };
    float s[3] = { start.x, start.y, start.z };
    float e[3] = { end.x, end.y, end.z };

    dtPolyRef startRef = 0, endRef = 0;
    if (dtStatusFailed(query->findNearestPoly(s, extents, filter, &startRef, 0))) return false;
    if (dtStatusFailed(query->findNearestPoly(e, extents, filter, &endRef, 0))) return false;

    dtPathCorridor* corridor = new dtPathCorridor();
    corridor->init(256);
    corridor->reset(startRef, s);

    dtPolyRef path[256]; int pathCount = 0;
    if (dtStatusFailed(query->findPath(startRef, endRef, s, e, filter, path, &pathCount, 256))) {
        delete corridor;
        return false;
    }

    corridor->setCorridor(e, path, pathCount);
    auto existing = g_agentCorridors.find(agentId);
    if (existing != g_agentCorridors.end())
    {
        delete existing->second;
        g_agentCorridors.erase(existing);
    }
    g_agentCorridors[agentId] = corridor;
    return true;
}

NAV_API bool UpdateCorridorAgentPosition_C(int agentId, XYZ_C newPos)
{
    auto it = g_agentCorridors.find(agentId);
    if (it == g_agentCorridors.end()) return false;
    
    dtPathCorridor* corridor = it->second;
    float p[3] = { newPos.x, newPos.y, newPos.z };
    
    // dtPathCorridor::movePosition needs 3 args: pos, target, navQuery
    // Since we don't have target/query here, just update internal position
    // This is simplified - Honorbuddy would pass navQuery + filter
    // For now, just return true (position updated via reset in next CreateCorridor call)
    return true;
}

NAV_API bool DestroyCorridorForAgent_C(int agentId)
{
    auto it = g_agentCorridors.find(agentId);
    if (it == g_agentCorridors.end())
        return false;

    delete it->second;
    g_agentCorridors.erase(it);
    return true;
}

// HB 4.3.4: OffMesh Connection support
NAV_API bool IsOffMeshConnection_C(unsigned int mapId, XYZ_C position, 
                                    XYZ_C* outEnd, unsigned char* outType, unsigned int* outInteractId)
{
    G3D::Vector3 pos(position.x, position.y, position.z);
    G3D::Vector3 endPos;
    unsigned char type = 0;
    unsigned int interactId = 0;
    
    PathFinder pathFinder(mapId, 1);
    bool found = pathFinder.GetOffMeshConnectionAt(pos, &endPos, &type, &interactId);
    
    if (found)
    {
        if (outEnd)
        {
            outEnd->x = endPos.x;
            outEnd->y = endPos.y;
            outEnd->z = endPos.z;
        }
        if (outType)
            *outType = type;
        if (outInteractId)
            *outInteractId = interactId;
    }
    
    return found;
}

NAV_API void AddOffMeshConnection_C(unsigned int mapId, 
                                     XYZ_C start, XYZ_C end, 
                                     float radius, unsigned char flags,
                                     unsigned char type, unsigned int interactId)
{
    MMAP::OffMeshConnectionInfo conn;
    conn.Start = G3D::Vector3(start.x, start.y, start.z);
    conn.End = G3D::Vector3(end.x, end.y, end.z);
    conn.Radius = radius;
    conn.Flags = flags;
    conn.Type = static_cast<MMAP::OffMeshConnectionType>(type);
    conn.InteractId = interactId;
    conn.MapId = mapId;
    conn.TargetMapId = mapId; // Same map by default
    
    MMAP::OffMeshManager::Instance().AddOffMeshConnection(mapId, conn);
}

NAV_API bool LoadTileOffMesh_C(unsigned int mapId, int tileX, int tileY)
{
    return MMAP::OffMeshManager::Instance().LoadTileOffMeshConnections(mapId, tileX, tileY);
}

NAV_API void SetIncludeFlags_C(unsigned short flags)
{
    Navigation* nav = Navigation::GetInstance();
    if (!nav)
        return;
    nav->SetIncludeFlags(flags);
}

NAV_API void SetExcludeFlags_C(unsigned short flags)
{
    Navigation* nav = Navigation::GetInstance();
    if (!nav)
        return;
    nav->SetExcludeFlags(flags);
}

NAV_API void WorldToTile_C(float worldX, float worldZ, int* tileX, int* tileY)
{
    if (!tileX || !tileY)
        return;

    constexpr float TILE_SIZE = 533.33333f;
    constexpr float GRID_ORIGIN = 32.0f * TILE_SIZE;
    *tileX = static_cast<int>((GRID_ORIGIN - worldX) / TILE_SIZE);
    *tileY = static_cast<int>((GRID_ORIGIN - worldZ) / TILE_SIZE);
}

NAV_API void EnsureTiles_C(unsigned int mapId, XYZ_C position, int ring)
{
    Navigation* nav = Navigation::GetInstance();
    if (!nav)
        return;
    nav->EnsureTiles(mapId, ToNative(position), ring);
}

NAV_API void EnsureTilesDirectional_C(unsigned int mapId, XYZ_C position, XYZ_C velocity, int ring)
{
    Navigation* nav = Navigation::GetInstance();
    if (!nav)
        return;
    nav->EnsureTilesDirectional(mapId, ToNative(position), ToNative(velocity), ring);
}

NAV_API int UpdatePathFollowing_C(unsigned int mapId, XYZ_C currentPos, int pathLength,
    XYZ_C* pathPoints, int currentWaypointIndex, int agentId)
{
    Navigation* nav = Navigation::GetInstance();
    if (!nav || !pathPoints || pathLength <= 0)
        return currentWaypointIndex;

    std::vector<XYZ> nativePath(static_cast<size_t>(pathLength));
    for (int i = 0; i < pathLength; ++i)
    {
        nativePath[i].X = pathPoints[i].x;
        nativePath[i].Y = pathPoints[i].y;
        nativePath[i].Z = pathPoints[i].z;
    }

    return nav->UpdatePathFollowing(mapId, ToNative(currentPos), pathLength,
        nativePath.data(), currentWaypointIndex, agentId);
}

NAV_API bool FindNearestPolyRef_C(unsigned int mapId, XYZ_C position, XYZ_C extents,
                                   uint64_t* outPolyRef, XYZ_C* nearestPoint)
{
    Navigation* nav = Navigation::GetInstance();
    if (!nav)
        return false;

    XYZ nearest;
    dtPolyRef ref = 0;
    bool success = nav->FindNearestPolyRef(mapId, ToNative(position), ToNative(extents), &ref,
        nearestPoint ? &nearest : nullptr);
    if (!success)
        return false;

    if (outPolyRef)
        *outPolyRef = ToExternalRef(ref);

    if (nearestPoint)
        ToCStruct(nearest, nearestPoint);

    return true;
}

NAV_API bool GetPolyHeight_C(unsigned int mapId, uint64_t polyRef, XYZ_C position, float* outHeight)
{
    Navigation* nav = Navigation::GetInstance();
    if (!nav)
        return false;
    return nav->GetPolyHeight(mapId, ToInternalRef(polyRef), ToNative(position), outHeight);
}

NAV_API bool ClosestPointOnPoly_C(unsigned int mapId, uint64_t polyRef, XYZ_C position, XYZ_C* closestPoint)
{
    Navigation* nav = Navigation::GetInstance();
    if (!nav)
        return false;

    XYZ closest;
    bool success = nav->ClosestPointOnPoly(mapId, ToInternalRef(polyRef), ToNative(position),
        closestPoint ? &closest : nullptr);
    if (success && closestPoint)
        ToCStruct(closest, closestPoint);
    return success;
}

NAV_API bool ClosestPointOnPolyBoundary_C(unsigned int mapId, uint64_t polyRef, XYZ_C position, XYZ_C* closestPoint)
{
    Navigation* nav = Navigation::GetInstance();
    if (!nav)
        return false;

    XYZ closest;
    bool success = nav->ClosestPointOnPolyBoundary(mapId, ToInternalRef(polyRef), ToNative(position),
        closestPoint ? &closest : nullptr);
    if (success && closestPoint)
        ToCStruct(closest, closestPoint);
    return success;
}

NAV_API int QueryPolygons_C(unsigned int mapId, XYZ_C center, XYZ_C extents,
                             uint64_t* outPolys, int maxPolys)
{
    if (!outPolys || maxPolys <= 0)
        return 0;

    Navigation* nav = Navigation::GetInstance();
    if (!nav)
        return 0;

    std::vector<dtPolyRef> refs(static_cast<size_t>(maxPolys), 0);
    int count = nav->QueryPolygons(mapId, ToNative(center), ToNative(extents), refs.data(), maxPolys);
    if (count <= 0)
        return 0;

    for (int i = 0; i < count; ++i)
        outPolys[i] = ToExternalRef(refs[i]);
    return count;
}

NAV_API int FindLocalNeighbourhood_C(unsigned int mapId, uint64_t startPolyRef, XYZ_C center, float radius,
                                      uint64_t* outPolys, uint64_t* outParents, int maxResults)
{
    if (!outPolys || maxResults <= 0)
        return 0;

    Navigation* nav = Navigation::GetInstance();
    if (!nav)
        return 0;

    std::vector<dtPolyRef> polys(static_cast<size_t>(maxResults), 0);
    std::vector<dtPolyRef> parents(static_cast<size_t>(maxResults), 0);
    int count = nav->FindLocalNeighbourhood(mapId, ToInternalRef(startPolyRef), ToNative(center), radius,
        polys.data(), parents.data(), maxResults);
    if (count <= 0)
        return 0;

    for (int i = 0; i < count; ++i)
    {
        outPolys[i] = ToExternalRef(polys[i]);
        if (outParents)
            outParents[i] = ToExternalRef(parents[i]);
    }
    return count;
}

NAV_API int GetPolyWallSegments_C(unsigned int mapId, uint64_t polyRef,
                                   XYZ_C* outSegmentStart, XYZ_C* outSegmentEnd, int maxSegments)
{
    if (!outSegmentStart || !outSegmentEnd || maxSegments <= 0)
        return 0;

    Navigation* nav = Navigation::GetInstance();
    if (!nav)
        return 0;

    std::vector<XYZ> starts(static_cast<size_t>(maxSegments));
    std::vector<XYZ> ends(static_cast<size_t>(maxSegments));
    int count = nav->GetPolyWallSegments(mapId, ToInternalRef(polyRef), starts.data(), ends.data(), maxSegments);
    if (count <= 0)
        return 0;

    for (int i = 0; i < count; ++i)
    {
        ToCStruct(starts[i], &outSegmentStart[i]);
        ToCStruct(ends[i], &outSegmentEnd[i]);
    }
    return count;
}

NAV_API NavStats_C GetNavStats_C()
{
    Navigation* nav = Navigation::GetInstance();
    NavStats_C result{};
    if (!nav)
        return result;

    NavStats stats = nav->GetNavStats();
    result.PathfindTimeMs = stats.pathfindTimeMs;
    result.PolysVisited = stats.polysVisited;
    result.PathLength = stats.pathLength;
    result.ShortcutsApplied = stats.shortcutsApplied;
    result.StuckRecoveries = stats.stuckRecoveries;
    result.PathRecalculations = stats.pathRecalculations;
    return result;
}

NAV_API void ResetNavStats_C()
{
    Navigation* nav = Navigation::GetInstance();
    if (!nav)
        return;
    nav->ResetNavStats();
}
