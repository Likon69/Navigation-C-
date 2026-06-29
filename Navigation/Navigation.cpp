#include "Navigation.h"
#include "MoveMap.h"
#include "PathFinder.h"
#include <vector>
#include <map>
#include <unordered_map>
#include <iostream>
#include <filesystem>
#include <algorithm>
#include <chrono> // QUICK WIN #2: High-resolution timer for adaptive sliced
#include <stdio.h>
#include <cmath> // ACTION #8: sqrtf, ceilf for LOS distance checks
#include <cstring>

using namespace std;

// ============================================================================
// COORDINATE CONVERSION: WoW <-> Detour/Recast
// ============================================================================
// Trinity/MaNGOS mmaps are generated with a specific coordinate system.
// WoW uses: X = East-West, Y = North-South, Z = Height
// Detour uses: X, Z = horizontal plane, Y = height
// 
// The conversion (same as PathFinder.cpp):
//   Detour[0] = WoW.Y    (North-South -> Detour X)
//   Detour[1] = WoW.Z    (Height -> Detour Y)
//   Detour[2] = WoW.X    (East-West -> Detour Z)
//
// Reverse:
//   WoW.X = Detour[2]
//   WoW.Y = Detour[0]
//   WoW.Z = Detour[1]
// ============================================================================

// Convert WoW XYZ to Detour float[3]
inline void WoWToDetour(const XYZ& wow, float detour[3])
{
    detour[0] = wow.Y;  // Detour X = WoW Y
    detour[1] = wow.Z;  // Detour Y = WoW Z (height)
    detour[2] = wow.X;  // Detour Z = WoW X
}

// Convert Detour float[3] to WoW XYZ
inline void DetourToWoW(const float detour[3], XYZ& wow)
{
    wow.X = detour[2];  // WoW X = Detour Z
    wow.Y = detour[0];  // WoW Y = Detour X
    wow.Z = detour[1];  // WoW Z = Detour Y (height)
}

EXTERN_C IMAGE_DOS_HEADER __ImageBase;

Navigation* Navigation::s_singletonInstance = NULL;

Navigation* Navigation::GetInstance()
{
    if (s_singletonInstance == NULL)
        s_singletonInstance = new Navigation();
    return s_singletonInstance;
}

void Navigation::Initialize()
{
    dtAllocSetCustom(dtCustomAlloc, dtCustomFree);
    
    // Honorbuddy-style initialization: Préparer le filter avec des coûts par défaut
    _slicedFindPath_mapId = 0;
    _slicedFindPath_startRef = 0;
    _slicedFindPath_endRef = 0;
    
    // HB 6.2.3 GetNewDefaultQueryFilter:
    // Include = AbilityFlags.All, Exclude = AbilityFlags.Unwalkable | AbilityFlags.Transport.
    _includeFlags = 0xffff;
    _excludeFlags = 0x0050; // 0x10 (Unwalkable) | 0x40 (Transport)
    
    // CRITICAL FIX: Initialize _defaultFilter properly (HB WoD pattern)
    // This must be done BEFORE any PathFinder is created!
    _defaultFilter.setIncludeFlags(_includeFlags);
    _defaultFilter.setExcludeFlags(_excludeFlags);
    
    // Area costs — valeurs HB 6.2.3 WowNavigator::SetDefaultQueryFilterCosts.
    // Ces valeurs sont la source de vérité pour tous les PathFinder créés.
    for (int i = 0; i < DT_MAX_AREAS; i++)
    {
        _areaCosts[i] = 1.0f;
        _defaultFilter.setAreaCost(i, 1.0f);
    }
    _areaCosts[NAV_GROUND]            = 1.66f;
    _areaCosts[NAV_WATER]             = 3.33f;
    _areaCosts[NAV_LAVA]              = 55.0f;
    _areaCosts[NAV_ROAD]              = 1.0f;
    _areaCosts[NAV_FALL]              = 1.7f;
    _areaCosts[NAV_ELEVATOR]          = 3.16f;
    _areaCosts[NAV_GATE]              = 1.66f;
    _areaCosts[NAV_PORTAL]            = 1.66f;
    _areaCosts[NAV_DEFENDERS_PORTAL]  = 3.16f;
    _areaCosts[NAV_HORDE_PORTAL]      = 1.66f;
    _areaCosts[NAV_ALLIANCE_PORTAL]   = 1.66f;
    _areaCosts[NAV_BLOCKED]           = 100.0f;
    _areaCosts[NAV_INTERACT_UNIT]     = 1.66f;
    _areaCosts[NAV_INTERACT_OBJECT]   = 1.66f;
    _areaCosts[NAV_KNOWN_BUILDING]    = 1.66f;
    _areaCosts[NAV_HORDE]             = 1.66f;
    _areaCosts[NAV_ALLIANCE]          = 1.66f;
    _areaCosts[NAV_BLACKSPOT]         = 60.0f;
    // Sync _defaultFilter avec les mêmes valeurs
    for (int i = 0; i < DT_MAX_AREAS; i++)
        _defaultFilter.setAreaCost(i, _areaCosts[i]);
    
    // QUICK WIN #2: Initialize adaptive sliced calibration
    _itersPerMs = 300.0f; // Conservative estimate: 300 iters per ms (will auto-calibrate)
}

void Navigation::Release()
{
    MMAP::MMapFactory::createOrGetMMapManager()->~MMapManager();
}

void Navigation::FreePathArr(XYZ* pathArr)
{
    delete[] pathArr;
}

XYZ* Navigation::CalculatePath(unsigned int mapId, XYZ start, XYZ end, bool straightPath, int* length)
{
    // AMÃƒâ€°LIORATION #2: Track pathfinding time
    auto t0 = std::chrono::high_resolution_clock::now();
    
    // HB 6.2.3 pattern: GC once per FindPath, load ring=1 around start+end only
    GarbageCollectTiles();
    EnsureTiles(mapId, start, 1);
    EnsureTiles(mapId, end, 1);

    PathFinder pathFinder(mapId, 1);
    pathFinder.setUseStrightPath(straightPath);
    
    // CRITICAL: Sync area costs with global settings (for blackspot support)
    pathFinder.setAreaCosts(_areaCosts, DT_MAX_AREAS);
    pathFinder.setIncludeFlags(_includeFlags);
    pathFinder.setExcludeFlags(_excludeFlags);
    
    pathFinder.calculate(start.X, start.Y, start.Z, end.X, end.Y, end.Z);

    PointsArray pointPath = pathFinder.getPath();
    *length = pointPath.size();
    XYZ* pathArr = new XYZ[pointPath.size()];

    for (unsigned int i = 0; i < pointPath.size(); i++)
    {
        pathArr[i].X = pointPath[i].x;
        pathArr[i].Y = pointPath[i].y;
        pathArr[i].Z = pointPath[i].z;
    }
    
    // AMÃƒâ€°LIORATION #2: Update stats
    auto t1 = std::chrono::high_resolution_clock::now();
    _stats.pathfindTimeMs = std::chrono::duration<float, std::milli>(t1 - t0).count();
    _stats.polysVisited = (int)pointPath.size(); // Poly count
    
    // Calculate path length
    float pathLen = 0.0f;
    for (unsigned int i = 1; i < pointPath.size(); i++)
    {
        float dx = pointPath[i].x - pointPath[i-1].x;
        float dy = pointPath[i].y - pointPath[i-1].y;
        float dz = pointPath[i].z - pointPath[i-1].z;
        pathLen += sqrtf(dx*dx + dy*dy + dz*dz);
    }
    _stats.pathLength = pathLen;
    _stats.pathRecalculations++;

    return pathArr;
}

PathResult* Navigation::CalculatePathEx(unsigned int mapId, XYZ start, XYZ end, bool straightPath)
{
    auto t0 = std::chrono::high_resolution_clock::now();

    // HB 6.2.3 pattern: GC once per FindPath, load ring=1 around start+end only
    GarbageCollectTiles();
    EnsureTiles(mapId, start, 1);
    EnsureTiles(mapId, end, 1);

    PathFinder pathFinder(mapId, 1);
    pathFinder.setUseStrightPath(straightPath);
    
    // CRITICAL: Sync area costs with global settings (for blackspot support)
    pathFinder.setAreaCosts(_areaCosts, DT_MAX_AREAS);
    pathFinder.setIncludeFlags(_includeFlags);
    pathFinder.setExcludeFlags(_excludeFlags);
    
    PathResult* result = new PathResult();
    pathFinder.calculate(start.X, start.Y, start.Z, end.X, end.Y, end.Z);

    PathType pathType = pathFinder.getPathType();
    result->status = pathFinder.GetLastStatus();
    result->failStep = pathFinder.GetFailStep();

    if (pathType & PATHFIND_INCOMPLETE)
    {
        NavStatusAddFlag(result->status, NAV_PARTIAL_RESULT);
    }

    const PointsArray& pointPath = pathFinder.getPath();
    const auto& flags = pathFinder.GetStraightPathFlags();
    const auto& polyTypes = pathFinder.GetPolyTypes();
    const auto& ability = pathFinder.GetAbilityFlags();
    const auto& polyRefs = pathFinder.GetStraightPathRefs();  // NEW: Get polygon references

    bool navFailed = NavStatusFailed(result->status) || (pathType & PATHFIND_NOPATH);

    auto t1 = std::chrono::high_resolution_clock::now();
    _stats.pathfindTimeMs = std::chrono::duration<float, std::milli>(t1 - t0).count();

    result->length = static_cast<int>(pointPath.size());
    if (navFailed || result->length <= 0)
    {
        if (!NavStatusFailed(result->status))
        {
            result->status = NAV_FAILURE | NAV_INVALID_PARAM;
            if (result->failStep == NAV_STEP_NONE)
            {
                result->failStep = NAV_STEP_FINALIZE_PATHFIND;
            }
        }
        result->length = 0;
        _stats.polysVisited = 0;
        _stats.pathLength = 0.0f;
        return result;
    }

    result->points = new XYZ[result->length];
    result->straightPathFlags = new StraightPathFlags[result->length];
    result->polyTypes = new unsigned char[result->length];
    result->abilityFlags = new unsigned char[result->length];
    result->polyRefs = new std::uint64_t[result->length];  // NEW: Allocate polyRefs

    for (int i = 0; i < result->length; ++i)
    {
        const auto& v = pointPath[i];
        result->points[i].X = v.x;
        result->points[i].Y = v.y;
        result->points[i].Z = v.z;
        result->straightPathFlags[i] = (i < static_cast<int>(flags.size())) ?
            flags[i] : StraightPathFlags::None;
        result->polyTypes[i] = (i < static_cast<int>(polyTypes.size())) ? polyTypes[i] : 0;
        result->abilityFlags[i] = (i < static_cast<int>(ability.size())) ? ability[i] : 0;
        // NEW: Copy polygon references (dtPolyRef -> uint64_t)
        result->polyRefs[i] = (i < static_cast<int>(polyRefs.size())) ? 
            static_cast<std::uint64_t>(polyRefs[i]) : 0;
    }

    // Update stats similar to legacy CalculatePath
    _stats.polysVisited = static_cast<int>(pointPath.size());
    _stats.corridorLength = static_cast<int>(pathFinder.getPolyLength()); // true A* corridor
    float pathLen = 0.0f;
    for (size_t i = 1; i < pointPath.size(); ++i)
    {
        float dx = pointPath[i].x - pointPath[i - 1].x;
        float dy = pointPath[i].y - pointPath[i - 1].y;
        float dz = pointPath[i].z - pointPath[i - 1].z;
        pathLen += sqrtf(dx * dx + dy * dy + dz * dz);
    }
    _stats.pathLength = pathLen;
    _stats.pathRecalculations++;

    return result;
}

void Navigation::FreePathResult(PathResult* result)
{
    if (!result)
        return;

    delete[] result->points;
    delete[] result->straightPathFlags;
    delete[] result->polyTypes;
    delete[] result->abilityFlags;
    delete[] result->polyRefs;  // NEW: Free polyRefs
    delete result;
}

string Navigation::GetMmapsPath()
{
    WCHAR DllPath[MAX_PATH] = { 0 };
    GetModuleFileNameW((HINSTANCE)&__ImageBase, DllPath, _countof(DllPath));
    wstring ws(DllPath);
    string pathAndFile(ws.begin(), ws.end());
    char* c = const_cast<char*>(pathAndFile.c_str());
    int strLength = strlen(c);
    int lastOccur = 0;
    for (int i = 0; i < strLength; i++)
    {
        if (c[i] == '\\') lastOccur = i;
    }
    string pathToMmap = pathAndFile.substr(0, lastOccur + 1);
    pathToMmap = pathToMmap.append("mmaps\\");

    return pathToMmap;
}

// Advanced Detour Navigation Functions - Like Honorbuddy RecastManaged
bool Navigation::FindNearestPoly(unsigned int mapId, XYZ center, float searchRadius, XYZ* nearestPoint)
{
    EnsureTiles(mapId, center, 1);

    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    const dtNavMeshQuery* query = manager->GetNavMeshQuery(mapId, 1);
    if (!query) return false;
    
    // FIXED: Convert WoW coordinates to Detour
    float centerPos[3];
    WoWToDetour(center, centerPos);
    
    // ACTION #4: Use consistent extents (prioritize vertical for caves)
    // Note: extents are in Detour space (Y is height)
    float extents[3] = { searchRadius, NavConstants::kExtY, searchRadius };
    float nearestPt[3];
    dtPolyRef nearestRef;
    
    // Phase 9: Use _defaultFilter to respect user-set area costs / blackspot exclusions
    dtStatus status = query->findNearestPoly(centerPos, extents, &_defaultFilter, &nearestRef, nearestPt);
    
    if (dtStatusSucceed(status) && nearestRef != 0)
    {
        // FIXED: Convert Detour coordinates back to WoW
        DetourToWoW(nearestPt, *nearestPoint);
        return true;
    }
    return false;
}

int Navigation::FindPolysAroundCircle(unsigned int mapId, XYZ center, float radius, XYZ* results, int maxResults)
{
    EnsureTiles(mapId, center, 1);

    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    const dtNavMeshQuery* query = manager->GetNavMeshQuery(mapId, 1);
    if (!query) return 0;
    
    // FIXED: Convert WoW coordinates to Detour
    float centerPos[3];
    WoWToDetour(center, centerPos);
    float extents[3] = { radius, radius, radius };
    dtPolyRef startRef;
    
    // Phase 9: Use _defaultFilter to respect user-set area costs / blackspot exclusions
    dtStatus status = query->findNearestPoly(centerPos, extents, &_defaultFilter, &startRef, nullptr);
    if (dtStatusFailed(status) || startRef == 0) return 0;
    
    dtPolyRef* resultRefs = new dtPolyRef[maxResults];
    dtPolyRef* parentRefs = new dtPolyRef[maxResults];
    float* resultCosts = new float[maxResults];
    int resultCount = 0;
    
    status = query->findPolysAroundCircle(startRef, centerPos, radius, &_defaultFilter, 
                                         resultRefs, parentRefs, resultCosts, &resultCount, maxResults);
    
    int actualResults = 0;
    if (dtStatusSucceed(status))
    {
        const dtNavMesh* navMesh = manager->GetNavMesh(mapId);
        for (int i = 0; i < resultCount && i < maxResults; i++)
        {
            float polyCenter[3];
            // Calculate polygon center manually since getPolyCenter doesn't exist in standard Detour
            if (navMesh)
            {
                const dtMeshTile* tile = 0;
                const dtPoly* poly = 0;
                if (dtStatusSucceed(navMesh->getTileAndPolyByRef(resultRefs[i], &tile, &poly)))
                {
                    // Calculate center of polygon from vertices
                    polyCenter[0] = polyCenter[1] = polyCenter[2] = 0.0f;
                    for (int j = 0; j < poly->vertCount; j++)
                    {
                        const float* v = &tile->verts[poly->verts[j] * 3];
                        polyCenter[0] += v[0];
                        polyCenter[1] += v[1];
                        polyCenter[2] += v[2];
                    }
                    polyCenter[0] /= poly->vertCount;
                    polyCenter[1] /= poly->vertCount;
                    polyCenter[2] /= poly->vertCount;
                    
                    // FIXED: Convert Detour coordinates back to WoW
                    DetourToWoW(polyCenter, results[actualResults]);
                    actualResults++;
                }
            }
        }
    }
    
    delete[] resultRefs;
    delete[] parentRefs;
    delete[] resultCosts;
    return actualResults;
}

float Navigation::FindDistanceToWall(unsigned int mapId, XYZ position, float maxRadius, XYZ* hitPoint)
{
    EnsureTiles(mapId, position, 1);

    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    const dtNavMeshQuery* query = manager->GetNavMeshQuery(mapId, 1);
    if (!query)
    {
        return -1.0f;
    }
    
    // FIXED: Convert WoW coordinates to Detour
    float pos[3];
    WoWToDetour(position, pos);
    float extents[3] = { maxRadius, maxRadius, maxRadius };
    dtPolyRef startRef;
    
    // Phase 9: Use _defaultFilter to respect user-set area costs / blackspot exclusions
    dtStatus status = query->findNearestPoly(pos, extents, &_defaultFilter, &startRef, nullptr);
    if (dtStatusFailed(status) || startRef == 0)
    {
        return -1.0f;
    }
    
    float hitDist = 0.0f;
    float hitPos[3];
    float hitNormal[3];
    
    status = query->findDistanceToWall(startRef, pos, maxRadius, &_defaultFilter, 
                                      &hitDist, hitPos, hitNormal);
    
    if (dtStatusSucceed(status))
    {
        if (hitPoint)
        {
            // FIXED: Convert Detour coordinates back to WoW
            DetourToWoW(hitPos, *hitPoint);
        }
        return hitDist;
    }
    return -1.0f;
}

// Extended version that also returns hit normal (like HB WoD)
float Navigation::FindDistanceToWallEx(unsigned int mapId, XYZ position, float maxRadius, XYZ* hitPoint, XYZ* outHitNormal)
{
    EnsureTiles(mapId, position, 1);

    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    const dtNavMeshQuery* query = manager->GetNavMeshQuery(mapId, 1);
    if (!query) return -1.0f;
    
    // FIXED: Convert WoW coordinates to Detour
    float pos[3];
    WoWToDetour(position, pos);
    float extents[3] = { maxRadius, maxRadius, maxRadius };
    dtPolyRef startRef;
    
    // Phase 9: Use _defaultFilter to respect user-set area costs / blackspot exclusions
    dtStatus status = query->findNearestPoly(pos, extents, &_defaultFilter, &startRef, nullptr);
    if (dtStatusFailed(status) || startRef == 0) return -1.0f;
    
    float hitDist = 0.0f;
    float hitPos[3];
    float hitNormal[3];
    
    status = query->findDistanceToWall(startRef, pos, maxRadius, &_defaultFilter, 
                                      &hitDist, hitPos, hitNormal);
    
    if (dtStatusSucceed(status))
    {
        if (hitPoint)
        {
            // FIXED: Convert Detour coordinates back to WoW
            DetourToWoW(hitPos, *hitPoint);
        }
        if (outHitNormal)
        {
            // FIXED: Convert Detour normal to WoW normal
            // Normal is a direction vector, same conversion applies
            DetourToWoW(hitNormal, *outHitNormal);
        }
        return hitDist;
    }
    return -1.0f;
}

// HB WoD style: takes polyRef directly instead of finding nearest poly
float Navigation::FindDistanceToWallFromPoly(unsigned int mapId, dtPolyRef polyRef, XYZ position, float maxRadius, XYZ* hitPoint, XYZ* outHitNormal)
{
    if (polyRef == 0) return -1.0f;
    
    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    EnsureTiles(mapId, position, 1);
    
    const dtNavMeshQuery* query = manager->GetNavMeshQuery(mapId, 1);
    if (!query) return -1.0f;
    
    // FIXED: Convert WoW coordinates to Detour
    float pos[3];
    WoWToDetour(position, pos);
    
    float hitDist = 0.0f;
    float hitPos[3];
    float hitNormal[3];
    
    // Phase 9: Use _defaultFilter to respect user-set area costs / blackspot exclusions
    dtStatus status = query->findDistanceToWall(polyRef, pos, maxRadius, &_defaultFilter, 
                                               &hitDist, hitPos, hitNormal);
    
    if (dtStatusSucceed(status))
    {
        if (hitPoint)
        {
            DetourToWoW(hitPos, *hitPoint);
        }
        if (outHitNormal)
        {
            DetourToWoW(hitNormal, *outHitNormal);
        }
        return hitDist;
    }
    return -1.0f;
}

bool Navigation::IsPointOnNavMesh(unsigned int mapId, XYZ point, float tolerance)
{
    EnsureTiles(mapId, point, 1);

    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    const dtNavMeshQuery* query = manager->GetNavMeshQuery(mapId, 1);
    if (!query) return false;
    
    // FIXED: Convert WoW coordinates to Detour
    float pos[3];
    WoWToDetour(point, pos);
    float extents[3] = { tolerance, tolerance, tolerance };
    dtPolyRef polyRef;
    
    // Phase 9: Use _defaultFilter to respect user-set area costs / blackspot exclusions
    dtStatus status = query->findNearestPoly(pos, extents, &_defaultFilter, &polyRef, nullptr);
    
    return dtStatusSucceed(status) && polyRef != 0;
}

XYZ Navigation::FindRandomPointAroundCircle(unsigned int mapId, XYZ center, float radius)
{
    EnsureTiles(mapId, center, 1);

    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    const dtNavMeshQuery* query = manager->GetNavMeshQuery(mapId, 1);
    XYZ result = center; // Default fallback
    
    if (!query) return result;
    
    // FIXED: Convert WoW coordinates to Detour
    float centerPos[3];
    WoWToDetour(center, centerPos);
    float extents[3] = { radius, radius, radius };
    dtPolyRef startRef;
    
    // Phase 9: Use _defaultFilter to respect user-set area costs / blackspot exclusions
    dtStatus status = query->findNearestPoly(centerPos, extents, &_defaultFilter, &startRef, nullptr);
    if (dtStatusFailed(status) || startRef == 0) return result;
    
    dtPolyRef randomRef;
    float randomPt[3];
    
    // Simple random function - you might want to improve this
    auto frand = []() -> float { return (float)rand() / RAND_MAX; };
    
    status = query->findRandomPointAroundCircle(startRef, centerPos, radius, &_defaultFilter, 
                                               frand, &randomRef, randomPt);
    
    if (dtStatusSucceed(status))
    {
        // FIXED: Convert Detour coordinates back to WoW
        DetourToWoW(randomPt, result);
    }
    
    return result;
}

bool Navigation::HasLineOfSight(unsigned int mapId, XYZ start, XYZ end)
{
    EnsureTiles(mapId, start, 1);
    EnsureTiles(mapId, end, 0);

    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    const dtNavMeshQuery* query = manager->GetNavMeshQuery(mapId, 1);
    if (!query) return false;
    
    // FIXED: Convert WoW coordinates to Detour
    float startPos[3];
    float endPos[3];
    WoWToDetour(start, startPos);
    WoWToDetour(end, endPos);
    
    // ACTION #8: Calculate distance and apply range limits
    float dx = endPos[0] - startPos[0];
    float dy = endPos[1] - startPos[1];
    float dz = endPos[2] - startPos[2];
    float distance = sqrtf(dx*dx + dy*dy + dz*dz);
    
    // If distance > max range, return false immediately (too far for reliable LOS)
    if (distance > NavConstants::kLosMaxRange)
        return false;
    
    // ACTION #4: Use consistent extents for LOS checks
    float extents[3] = { NavConstants::kExtX, NavConstants::kExtY, NavConstants::kExtZ };
    
    dtPolyRef startRef, endRef;
    
    // Phase 9: Use _defaultFilter to respect user-set area costs / blackspot exclusions
    // Find polygons for both points
    dtStatus status1 = query->findNearestPoly(startPos, extents, &_defaultFilter, &startRef, nullptr);
    dtStatus status2 = query->findNearestPoly(endPos, extents, &_defaultFilter, &endRef, nullptr);
    
    if (dtStatusFailed(status1) || dtStatusFailed(status2) || startRef == 0 || endRef == 0)
        return false;
    
    // ACTION #8: Step raycast for distances > step range
    if (distance > NavConstants::kLosStepRange)
    {
        // Break into segments of kLosStepRange
        int numSteps = (int)ceilf(distance / NavConstants::kLosStepRange);
        float stepX = dx / numSteps;
        float stepY = dy / numSteps;
        float stepZ = dz / numSteps;
        
        float currentPos[3] = { startPos[0], startPos[1], startPos[2] };
        dtPolyRef currentRef = startRef;
        
        for (int step = 0; step < numSteps; step++)
        {
            float nextPos[3] = {
                currentPos[0] + stepX,
                currentPos[1] + stepY,
                currentPos[2] + stepZ
            };
            
            // Raycast this segment
            float t = 0.0f;
            float hitNormal[3];
            dtPolyRef path[256];
            int pathCount = 0;
            
            dtStatus rayStatus = query->raycast(currentRef, currentPos, nextPos, &_defaultFilter, 
                                               &t, hitNormal, path, &pathCount, 256);
            
            // If we hit something (t < 1.0), no line of sight
            if (dtStatusFailed(rayStatus) || t < 1.0f)
                return false;
            
            // Move to next segment
            currentPos[0] = nextPos[0];
            currentPos[1] = nextPos[1];
            currentPos[2] = nextPos[2];
            
            // Find poly ref at new position for next raycast
            if (pathCount > 0)
                currentRef = path[pathCount - 1];
        }
        
        return true; // All segments clear
    }
    else
    {
        // Single raycast for short distances
        float t = 0.0f;
        float hitNormal[3];
        dtPolyRef path[256];
        int pathCount = 0;
        
        dtStatus rayStatus = query->raycast(startRef, startPos, endPos, &_defaultFilter, 
                                           &t, hitNormal, path, &pathCount, 256);
        
        // If t >= 1.0, we reached the end without hitting anything
        return dtStatusSucceed(rayStatus) && t >= 1.0f;
    }
}

// ===== Raycast - HB-style full raycast output =====
// Note: dtPolyRef is unsigned int (32-bit) by default, matching HB WoD
unsigned int Navigation::Raycast(unsigned int mapId, dtPolyRef startRef, XYZ startPos, XYZ endPos,
    float* outT, XYZ* outHitNormal, dtPolyRef* outPath, int* outPathCount, int maxPath)
{
    if (!outT || maxPath <= 0)
        return DT_FAILURE | DT_INVALID_PARAM;

    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    EnsureTiles(mapId, startPos, 1);
    EnsureTiles(mapId, endPos, 0);

    const dtNavMeshQuery* query = manager->GetNavMeshQuery(mapId, 1);
    if (!query) return DT_FAILURE;

    // FIXED: Convert WoW coordinates to Detour
    float start[3];
    float end[3];
    WoWToDetour(startPos, start);
    WoWToDetour(endPos, end);
    float t = 0.0f;
    float hitNormal[3] = { 0.0f, 0.0f, 0.0f };

    std::vector<dtPolyRef> pathRefs(static_cast<size_t>(maxPath), 0);
    int pathCount = 0;

    dtStatus status = query->raycast(startRef, start, end, &_defaultFilter,
        &t, hitNormal, pathRefs.data(), &pathCount, maxPath);

    *outT = t;

    if (outHitNormal)
    {
        // FIXED: Convert Detour normal back to WoW
        DetourToWoW(hitNormal, *outHitNormal);
    }

    if (outPathCount)
        *outPathCount = pathCount;

    if (outPath && pathCount > 0)
    {
        std::memcpy(outPath, pathRefs.data(), pathCount * sizeof(dtPolyRef));
    }

    // Update stats
    _stats.raycastAttempts++;
    if (t >= 1.0f)
        _stats.raycastHits++;
    _stats.lastRaycastHitFraction = t;

    return static_cast<unsigned int>(status);
}

bool Navigation::FindNearestPolyRef(unsigned int mapId, XYZ position, XYZ extents,
    dtPolyRef* outPolyRef, XYZ* nearestPoint)
{
    EnsureTiles(mapId, position, 1);

    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    const dtNavMeshQuery* query = manager->GetNavMeshQuery(mapId, 1);
    if (!query)
        return false;

    // FIXED: Convert WoW coordinates to Detour
    float pos[3];
    WoWToDetour(position, pos);
    // FIXED: Convert WoW extents to Detour: WoW(X,Y,Z) -> Detour[Y,Z,X]
    float searchExtents[3] = { extents.Y, extents.Z, extents.X };
    float nearestPos[3];
    dtPolyRef ref = 0;

    dtStatus status = query->findNearestPoly(pos, searchExtents, &_defaultFilter, &ref, nearestPos);
    if (dtStatusFailed(status) || ref == 0)
        return false;

    if (outPolyRef)
        *outPolyRef = ref;

    if (nearestPoint)
    {
        // FIXED: Convert Detour coordinates back to WoW
        DetourToWoW(nearestPos, *nearestPoint);
    }

    return true;
}

bool Navigation::GetPolyHeight(unsigned int mapId, dtPolyRef polyRef, XYZ position, float* outHeight)
{
    if (!outHeight || !polyRef)
        return false;

    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();

    const dtNavMeshQuery* query = manager->GetNavMeshQuery(mapId, 1);
    if (!query)
        return false;

    // FIXED: Convert WoW coordinates to Detour
    float pos[3];
    WoWToDetour(position, pos);
    float height = 0.0f;
    dtStatus status = query->getPolyHeight(polyRef, pos, &height);
    if (dtStatusFailed(status))
        return false;

    // Height in Detour Y = WoW Z, so this is correct
    *outHeight = height;
    return true;
}

bool Navigation::ClosestPointOnPoly(unsigned int mapId, dtPolyRef polyRef, XYZ position, XYZ* closestPoint)
{
    if (!closestPoint || !polyRef)
        return false;

    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    const dtNavMeshQuery* query = manager->GetNavMeshQuery(mapId, 1);
    if (!query)
        return false;

    // FIXED: Convert WoW coordinates to Detour
    float pos[3];
    WoWToDetour(position, pos);
    float closest[3];
    bool posOverPoly = false;
    dtStatus status = query->closestPointOnPoly(polyRef, pos, closest, &posOverPoly);
    if (dtStatusFailed(status))
        return false;

    // FIXED: Convert Detour coordinates back to WoW
    DetourToWoW(closest, *closestPoint);
    return true;
}

bool Navigation::ClosestPointOnPolyBoundary(unsigned int mapId, dtPolyRef polyRef, XYZ position, XYZ* closestPoint)
{
    if (!closestPoint || !polyRef)
        return false;

    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    const dtNavMeshQuery* query = manager->GetNavMeshQuery(mapId, 1);
    if (!query)
        return false;

    // FIXED: Convert WoW coordinates to Detour
    float pos[3];
    WoWToDetour(position, pos);
    float closest[3];
    dtStatus status = query->closestPointOnPolyBoundary(polyRef, pos, closest);
    if (dtStatusFailed(status))
        return false;

    // FIXED: Convert Detour coordinates back to WoW
    DetourToWoW(closest, *closestPoint);
    return true;
}

// ===== Polygon area/flags manipulation (like HB Tripper.RecastManaged - for blackspot marking) =====

unsigned int Navigation::SetPolyArea(unsigned int mapId, dtPolyRef polyRef, unsigned char area)
{
    if (!polyRef)
        return DT_FAILURE | DT_INVALID_PARAM;

    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    dtNavMesh* navMesh = const_cast<dtNavMesh*>(manager->GetNavMesh(mapId));
    if (!navMesh)
        return DT_FAILURE;

    return navMesh->setPolyArea(polyRef, area);
}

unsigned int Navigation::GetPolyArea(unsigned int mapId, dtPolyRef polyRef, unsigned char* outArea)
{
    if (!polyRef || !outArea)
        return DT_FAILURE | DT_INVALID_PARAM;

    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    const dtNavMesh* navMesh = manager->GetNavMesh(mapId);
    if (!navMesh)
        return DT_FAILURE;

    return navMesh->getPolyArea(polyRef, outArea);
}

unsigned int Navigation::SetPolyFlags(unsigned int mapId, dtPolyRef polyRef, unsigned short flags)
{
    if (!polyRef)
        return DT_FAILURE | DT_INVALID_PARAM;

    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    dtNavMesh* navMesh = const_cast<dtNavMesh*>(manager->GetNavMesh(mapId));
    if (!navMesh)
        return DT_FAILURE;

    return navMesh->setPolyFlags(polyRef, flags);
}

unsigned int Navigation::GetPolyFlags(unsigned int mapId, dtPolyRef polyRef, unsigned short* outFlags)
{
    if (!polyRef || !outFlags)
        return DT_FAILURE | DT_INVALID_PARAM;

    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    const dtNavMesh* navMesh = manager->GetNavMesh(mapId);
    if (!navMesh)
        return DT_FAILURE;

    return navMesh->getPolyFlags(polyRef, outFlags);
}

int Navigation::GetMaxTiles(unsigned int mapId)
{
    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    const dtNavMesh* navMesh = manager->GetNavMesh(mapId);
    if (!navMesh)
        return 0;
    return navMesh->getMaxTiles();
}

dtPolyRef Navigation::EncodePolyId(unsigned int mapId, unsigned int salt, unsigned int it, unsigned int ip)
{
    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    const dtNavMesh* navMesh = manager->GetNavMesh(mapId);
    if (!navMesh)
        return 0;
    return navMesh->encodePolyId(salt, it, ip);
}

void Navigation::DecodePolyId(unsigned int mapId, dtPolyRef polyRef, unsigned int* outSalt, unsigned int* outIt, unsigned int* outIp)
{
    if (!polyRef || !outSalt || !outIt || !outIp)
        return;
    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    const dtNavMesh* navMesh = manager->GetNavMesh(mapId);
    if (!navMesh)
        return;
    unsigned int salt = 0, it = 0, ip = 0;
    navMesh->decodePolyId(polyRef, salt, it, ip);
    *outSalt = salt;
    *outIt   = it;
    *outIp   = ip;
}

unsigned int Navigation::DecodePolyIdSalt(unsigned int mapId, dtPolyRef polyRef)
{
    if (!polyRef) return 0;
    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    const dtNavMesh* navMesh = manager->GetNavMesh(mapId);
    if (!navMesh)
        return 0;
    return (unsigned int)navMesh->decodePolyIdSalt(polyRef);
}

unsigned int Navigation::DecodePolyIdTile(unsigned int mapId, dtPolyRef polyRef)
{
    if (!polyRef) return 0;
    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    const dtNavMesh* navMesh = manager->GetNavMesh(mapId);
    if (!navMesh)
        return 0;
    return (unsigned int)navMesh->decodePolyIdTile(polyRef);
}

unsigned int Navigation::DecodePolyIdPoly(unsigned int mapId, dtPolyRef polyRef)
{
    if (!polyRef) return 0;
    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    const dtNavMesh* navMesh = manager->GetNavMesh(mapId);
    if (!navMesh)
        return 0;
    return (unsigned int)navMesh->decodePolyIdPoly(polyRef);
}

const void* Navigation::GetTileAt(unsigned int mapId, int x, int y)
{
    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    const dtNavMesh* navMesh = manager->GetNavMesh(mapId);
    if (!navMesh)
        return nullptr;
    // HB 6.2.3 GetTileAt only takes (x, y) — Detour's getTileAt also takes
    // a layer parameter. V4 mmtiles have a single layer, so we pass 0.
    return navMesh->getTileAt(x, y, 0);
}

const void* Navigation::GetTile(unsigned int mapId, int i)
{
    if (i < 0) return nullptr;
    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    const dtNavMesh* navMesh = manager->GetNavMesh(mapId);
    if (!navMesh)
        return nullptr;
    return navMesh->getTile(i);
}

int Navigation::GetTileStateSize(unsigned int mapId, const void* tile)
{
    if (!tile) return 0;
    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    const dtNavMesh* navMesh = manager->GetNavMesh(mapId);
    if (!navMesh)
        return 0;
    return navMesh->getTileStateSize(static_cast<const dtMeshTile*>(tile));
}

unsigned int Navigation::StoreTileState(unsigned int mapId, const void* tile, unsigned char* outData, int maxDataSize)
{
    if (!tile || !outData || maxDataSize <= 0)
        return static_cast<unsigned int>(DT_FAILURE | DT_INVALID_PARAM);
    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    const dtNavMesh* navMesh = manager->GetNavMesh(mapId);
    if (!navMesh)
        return static_cast<unsigned int>(DT_FAILURE);
    return static_cast<unsigned int>(navMesh->storeTileState(
        static_cast<const dtMeshTile*>(tile), outData, maxDataSize));
}

unsigned int Navigation::RestoreTileState(unsigned int mapId, void* tile, const unsigned char* data, int dataSize)
{
    if (!tile || !data || dataSize <= 0)
        return static_cast<unsigned int>(DT_FAILURE | DT_INVALID_PARAM);
    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    // restoreTileState is non-const on the tile, so the navMesh must be non-const too.
    dtNavMesh* navMesh = const_cast<dtNavMesh*>(manager->GetNavMesh(mapId));
    if (!navMesh)
        return static_cast<unsigned int>(DT_FAILURE);
    return static_cast<unsigned int>(navMesh->restoreTileState(
        static_cast<dtMeshTile*>(tile), data, dataSize));
}

int Navigation::QueryPolygons(unsigned int mapId, XYZ center, XYZ extents, dtPolyRef* outPolys, int maxPolys)
{
    if (!outPolys || maxPolys <= 0)
        return 0;

    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    EnsureTiles(mapId, center, 1);

    const dtNavMeshQuery* query = manager->GetNavMeshQuery(mapId, 1);
    if (!query)
        return 0;

    std::vector<dtPolyRef> results(static_cast<size_t>(maxPolys), 0);
    // FIXED: Convert WoW coordinates to Detour
    float c[3];
    WoWToDetour(center, c);
    // Note: extents are dimensions, not positions - they map Y=height, so convert too
    float e[3] = { extents.Y, extents.Z, extents.X };
    int resultCount = 0;

    dtStatus status = query->queryPolygons(c, e, &_defaultFilter, results.data(), &resultCount, maxPolys);
    if (dtStatusFailed(status) || resultCount <= 0)
        return 0;

    std::memcpy(outPolys, results.data(), sizeof(dtPolyRef) * resultCount);
    return resultCount;
}

int Navigation::FindLocalNeighbourhood(unsigned int mapId, dtPolyRef startPolyRef, XYZ center, float radius,
    dtPolyRef* outPolys, dtPolyRef* outParents, int maxResults)
{
    if (!outPolys || maxResults <= 0 || !startPolyRef)
        return 0;

    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    EnsureTiles(mapId, center, 1);

    const dtNavMeshQuery* query = manager->GetNavMeshQuery(mapId, 1);
    if (!query)
        return 0;

    std::vector<dtPolyRef> polys(static_cast<size_t>(maxResults), 0);
    std::vector<dtPolyRef> parents(static_cast<size_t>(maxResults), 0);
    // FIXED: Convert WoW coordinates to Detour
    float centerPos[3];
    WoWToDetour(center, centerPos);
    int resultCount = 0;

    dtStatus status = query->findLocalNeighbourhood(startPolyRef, centerPos, radius, &_defaultFilter,
        polys.data(), parents.data(), &resultCount, maxResults);

    if (dtStatusFailed(status) || resultCount <= 0)
        return 0;

    std::memcpy(outPolys, polys.data(), sizeof(dtPolyRef) * resultCount);
    if (outParents)
        std::memcpy(outParents, parents.data(), sizeof(dtPolyRef) * resultCount);

    return resultCount;
}

int Navigation::GetPolyWallSegments(unsigned int mapId, dtPolyRef polyRef, XYZ* outSegmentStart,
    XYZ* outSegmentEnd, int maxSegments)
{
    if (!outSegmentStart || !outSegmentEnd || maxSegments <= 0 || !polyRef)
        return 0;

    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();

    const dtNavMeshQuery* query = manager->GetNavMeshQuery(mapId, 1);
    if (!query)
        return 0;

    std::vector<float> segments(static_cast<size_t>(maxSegments) * 6u, 0.0f);
    std::vector<dtPolyRef> neighbours(static_cast<size_t>(maxSegments), 0);
    int segmentCount = 0;
    dtStatus status = query->getPolyWallSegments(polyRef, &_defaultFilter, segments.data(), neighbours.data(), &segmentCount, maxSegments);
    if (dtStatusFailed(status) || segmentCount <= 0)
        return 0;

    for (int i = 0; i < segmentCount; ++i)
    {
        const float* start = &segments[i * 6];
        const float* end = &segments[i * 6 + 3];
        // FIXED: Convert Detour coordinates back to WoW
        // Detour [X,Y,Z] = WoW [Y,Z,X] -> WoW X = Detour[2], Y = Detour[0], Z = Detour[1]
        outSegmentStart[i].X = start[2];
        outSegmentStart[i].Y = start[0];
        outSegmentStart[i].Z = start[1];

        outSegmentEnd[i].X = end[2];
        outSegmentEnd[i].Y = end[0];
        outSegmentEnd[i].Z = end[1];
    }

    return segmentCount;
}

// ===== Honorbuddy-style Sliced PathFinding (pour navigation asynchrone) =====
bool Navigation::InitSlicedFindPath(unsigned int mapId, XYZ start, XYZ end)
{
    // HB 6.2.3 pattern: GC once per FindPath, load ring=1 around start+end only
    GarbageCollectTiles();
    EnsureTiles(mapId, start, 1);
    EnsureTiles(mapId, end, 1);

    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    const dtNavMeshQuery* constQuery = manager->GetNavMeshQuery(mapId, 1);
    if (!constQuery) return false;
    
    // Sliced pathfinding modifie l'état interne, donc const_cast nécessaire
    // Sûr car MMapManager possède ces queries non-const en interne
    dtNavMeshQuery* query = const_cast<dtNavMeshQuery*>(constQuery);
    
    // FIXED: Convert WoW coordinates to Detour
    float startPos[3];
    WoWToDetour(start, startPos);
    float endPos[3];
    WoWToDetour(end, endPos);
    
    // ACTION #4: Use consistent extents for caves/multi-layer
    // Note: extents in Detour = {horizontal, height, horizontal} = {3, 20, 3}
    // Same as POLY_SEARCH_EXTENTS in PathFinder.cpp
    float extents[3] = { NavConstants::kExtX, NavConstants::kExtY, NavConstants::kExtZ };
    
    // Appliquer area costs au filter (optimisation Honorbuddy)
    for (int i = 0; i < DT_MAX_AREAS; i++)
    {
        _defaultFilter.setAreaCost(i, _areaCosts[i]);
    }
    
    // Find start/end polygons
    dtStatus status1 = query->findNearestPoly(startPos, extents, &_defaultFilter, &_slicedFindPath_startRef, nullptr);
    dtStatus status2 = query->findNearestPoly(endPos, extents, &_defaultFilter, &_slicedFindPath_endRef, nullptr);
    
    if (dtStatusFailed(status1) || dtStatusFailed(status2) || 
        _slicedFindPath_startRef == 0 || _slicedFindPath_endRef == 0)
        return false;
    
    // Initialize sliced pathfinding (non-bloquant)
    _slicedFindPath_mapId = mapId;
    dtStatus initStatus = query->initSlicedFindPath(_slicedFindPath_startRef, _slicedFindPath_endRef, 
                                                     startPos, endPos, &_defaultFilter, 0);
    
    return dtStatusSucceed(initStatus);
}

bool Navigation::UpdateSlicedFindPath(int maxIterations)
{
    if (_slicedFindPath_mapId == 0) return false;
    
    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    const dtNavMeshQuery* constQuery = manager->GetNavMeshQuery(_slicedFindPath_mapId, 1);
    if (!constQuery) return false;
    
    // Sliced pathfinding modifie l'ÃƒÂ©tat interne
    dtNavMeshQuery* query = const_cast<dtNavMeshQuery*>(constQuery);
    
    int doneIters;
    dtStatus status = query->updateSlicedFindPath(maxIterations, &doneIters);
    
    // Continue si IN_PROGRESS ou SUCCESS
    return dtStatusSucceed(status) || (status & DT_IN_PROGRESS);
}

// QUICK WIN #2: Adaptive sliced pathfinding with ms budget
// Pattern from Honorbuddy MoP + prompt.json Action 3
bool Navigation::UpdateSlicedFindPathMs(float msBudget)
{
    if (_slicedFindPath_mapId == 0) return false;
    
    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    const dtNavMeshQuery* constQuery = manager->GetNavMeshQuery(_slicedFindPath_mapId, 1);
    if (!constQuery) return false;
    
    dtNavMeshQuery* query = const_cast<dtNavMeshQuery*>(constQuery);
    
    // Estimate iterations based on budget and calibrated rate
    int iters = EstimateIterations(msBudget);
    
    // High-resolution timer for calibration
    auto t0 = std::chrono::high_resolution_clock::now();
    
    int doneIters;
    dtStatus status = query->updateSlicedFindPath(iters, &doneIters);
    
    auto t1 = std::chrono::high_resolution_clock::now();
    float elapsedMs = std::chrono::duration<float, std::milli>(t1 - t0).count();
    
    // Auto-calibrate iters/ms ratio
    TuneIterationsPerMs(elapsedMs, doneIters);
    
    return dtStatusSucceed(status) || (status & DT_IN_PROGRESS);
}

int Navigation::EstimateIterations(float msBudget)
{
    int iters = static_cast<int>(msBudget * _itersPerMs);
    
    // Clamp to reasonable bounds (10-1000 like HB MoP)
    if (iters < 10) iters = 10;
    if (iters > 1000) iters = 1000;
    
    return iters;
}

void Navigation::TuneIterationsPerMs(float elapsedMs, int itersExecuted)
{
    if (elapsedMs < 0.01f) return; // Too fast to measure accurately
    
    float measuredRate = itersExecuted / elapsedMs;
    
    // Exponential moving average: 90% old + 10% new (smooth adaptation)
    _itersPerMs = _itersPerMs * 0.9f + measuredRate * 0.1f;
    
    // Safety clamp (100-2000 iters/ms reasonable range)
    if (_itersPerMs < 100.0f) _itersPerMs = 100.0f;
    if (_itersPerMs > 2000.0f) _itersPerMs = 2000.0f;
}

XYZ* Navigation::FinalizeSlicedFindPath(int maxPathSize, int* length)
{
    *length = 0;
    if (_slicedFindPath_mapId == 0) return nullptr;
    
    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    const dtNavMeshQuery* constQuery = manager->GetNavMeshQuery(_slicedFindPath_mapId, 1);
    if (!constQuery) return nullptr;
    
    // Sliced pathfinding modifie l'ÃƒÂ©tat interne
    dtNavMeshQuery* query = const_cast<dtNavMeshQuery*>(constQuery);
    
    // RÃƒÂ©utiliser le buffer interne pour optimisation (pattern Honorbuddy)
    int pathCount = 0;
    dtStatus status = query->finalizeSlicedFindPath(_polysBuffer, &pathCount, 
                                                     min(maxPathSize, MAX_POLYS_BUFFER));
    
    if (dtStatusFailed(status) || pathCount == 0)
    {
        _slicedFindPath_mapId = 0; // Reset
        return nullptr;
    }
    
    // ACTION #1: Apply funnel algorithm (findStraightPath) to reduce zig-zag
    // This smooths the polygon corridor into a direct polyline
    float straightPath[MAX_POLYS_BUFFER * 3]; // XYZ coords
    unsigned char straightPathFlags[MAX_POLYS_BUFFER];
    dtPolyRef straightPathRefs[MAX_POLYS_BUFFER];
    int straightPathCount = 0;
    
    // Get start/end positions for straight path
    float startPos[3], endPos[3];
    const dtNavMesh* navMesh = manager->GetNavMesh(_slicedFindPath_mapId);
    
    // Start position: center of first poly
    {
        const dtMeshTile* tile = 0;
        const dtPoly* poly = 0;
        if (dtStatusSucceed(navMesh->getTileAndPolyByRef(_polysBuffer[0], &tile, &poly)))
        {
            startPos[0] = startPos[1] = startPos[2] = 0;
            for (int j = 0; j < poly->vertCount; j++)
            {
                const float* v = &tile->verts[poly->verts[j] * 3];
                startPos[0] += v[0];
                startPos[1] += v[1];
                startPos[2] += v[2];
            }
            startPos[0] /= poly->vertCount;
            startPos[1] /= poly->vertCount;
            startPos[2] /= poly->vertCount;
        }
    }
    
    // End position: center of last poly
    {
        const dtMeshTile* tile = 0;
        const dtPoly* poly = 0;
        if (dtStatusSucceed(navMesh->getTileAndPolyByRef(_polysBuffer[pathCount - 1], &tile, &poly)))
        {
            endPos[0] = endPos[1] = endPos[2] = 0;
            for (int j = 0; j < poly->vertCount; j++)
            {
                const float* v = &tile->verts[poly->verts[j] * 3];
                endPos[0] += v[0];
                endPos[1] += v[1];
                endPos[2] += v[2];
            }
            endPos[0] /= poly->vertCount;
            endPos[1] /= poly->vertCount;
            endPos[2] /= poly->vertCount;
        }
    }
    
    // Apply funnel: converts polygon corridor to direct line segments
    status = query->findStraightPath(
        startPos, endPos,
        _polysBuffer, pathCount,
        straightPath, straightPathFlags, straightPathRefs,
        &straightPathCount, MAX_POLYS_BUFFER,
        DT_STRAIGHTPATH_ALL_CROSSINGS // NEW-3: ALL_CROSSINGS for consistency with BuildPointPath
    );
    
    // If funnel succeeds, use straight path; otherwise fall back to polygon centers
    int finalPathCount = 0;
    XYZ* path = nullptr;
    
    if (dtStatusSucceed(status) && straightPathCount > 0)
    {
        // SUCCESS: Use funneled straight path (much smoother!)
        finalPathCount = straightPathCount;
        path = new XYZ[finalPathCount];
        
        for (int i = 0; i < finalPathCount; i++)
        {
            // FIXED: Convert Detour coordinates back to WoW
            // Detour [X,Y,Z] -> WoW [Z,X,Y] (inverse of WoW->Detour)
            path[i].X = straightPath[i * 3 + 2];  // WoW X = Detour Z
            path[i].Y = straightPath[i * 3 + 0];  // WoW Y = Detour X
            path[i].Z = straightPath[i * 3 + 1];  // WoW Z = Detour Y
        }
        
        _stats.shortcutsApplied++; // Track funnel applications
    }
    else
    {
        // FALLBACK: Use polygon centers (old method)
        finalPathCount = pathCount;
        path = new XYZ[finalPathCount];
        
        for (int i = 0; i < pathCount; i++)
        {
            const dtMeshTile* tile = 0;
            const dtPoly* poly = 0;
            if (dtStatusSucceed(navMesh->getTileAndPolyByRef(_polysBuffer[i], &tile, &poly)))
            {
                // Calculer le centre du polygone
                float center[3] = { 0, 0, 0 };
                for (int j = 0; j < poly->vertCount; j++)
                {
                    const float* v = &tile->verts[poly->verts[j] * 3];
                    center[0] += v[0];
                    center[1] += v[1];
                    center[2] += v[2];
                }
                center[0] /= poly->vertCount;
                center[1] /= poly->vertCount;
                center[2] /= poly->vertCount;
                
                // FIXED: Convert Detour coordinates back to WoW
                path[i].X = center[2];  // WoW X = Detour Z
                path[i].Y = center[0];  // WoW Y = Detour X
                path[i].Z = center[1];  // WoW Z = Detour Y
            }
        }
    }
    
    *length = finalPathCount;
    _slicedFindPath_mapId = 0; // Reset pour prochaine utilisation
    return path;
}

// ===== Query Filter avec Area Cost (optimisation Honorbuddy) =====
// AMÉLIORATION: Filtre global flags (Action 2)
void Navigation::SetIncludeFlags(unsigned short flags)
{
    _includeFlags = flags;
    _defaultFilter.setIncludeFlags(flags);
}

void Navigation::SetExcludeFlags(unsigned short flags)
{
    _excludeFlags = flags;
    _defaultFilter.setExcludeFlags(flags);
}

void Navigation::SetAreaCost(unsigned int areaId, float cost)
{
    if (areaId < DT_MAX_AREAS)
    {
        _areaCosts[areaId] = cost;
        _defaultFilter.setAreaCost(areaId, cost);
    }
}

float Navigation::GetAreaCost(unsigned int areaId)
{
    if (areaId < DT_MAX_AREAS)
    {
        return _areaCosts[areaId];
    }
    return 1.0f;
}

// ===== Tile Management OptimisÃƒÂ© (comme Honorbuddy) =====
void Navigation::SetTileLoadedCallback(MMAP::TileLoadedCallback cb)
{
    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    if (manager)
        manager->SetTileLoadedCallback(cb);
}

bool Navigation::IsTileLoaded(unsigned int mapId, int x, int y)
{
    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    return manager && manager->isTileLoaded(mapId, x, y);
}

bool Navigation::UnloadTile(unsigned int mapId, int x, int y)
{
    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    if (!manager)
        return false;

    return manager->unloadTile(mapId, x, y);
}

int Navigation::GetLoadedTilesCount(unsigned int mapId)
{
    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    const dtNavMesh* navMesh = manager->GetNavMesh(mapId);
    if (!navMesh) return 0;
    
    int count = 0;
    int maxTiles = navMesh->getMaxTiles();
    for (int i = 0; i < maxTiles; i++)
    {
        const dtMeshTile* tile = navMesh->getTile(i);
        if (tile && tile->header)
        {
            count++;
        }
    }
    
    return count;
}
// QUICK WIN #1: Raycast shortcut during following
int Navigation::UpdatePathFollowing(unsigned int mapId, XYZ currentPos, int pathLength,
    const XYZ* pathPoints, int currentWaypointIndex, int /*agentId*/)
{
    if (!pathPoints || pathLength <= 0)
        return currentWaypointIndex;

    dtNavMeshQuery* query = GetNavMeshQuery(mapId);
    if (!query)
        return currentWaypointIndex;

    // FIXED: Convert WoW coordinates to Detour
    float pos[3];
    WoWToDetour(currentPos, pos);
    // Note: extents in Detour = {horizontal, height, horizontal} = {3, 20, 3}
    float extents[3] = { NavConstants::kExtX, NavConstants::kExtY, NavConstants::kExtZ };

    dtPolyRef currentRef = 0;
    float nearestPos[3];
    if (dtStatusFailed(query->findNearestPoly(pos, extents, &_defaultFilter, &currentRef, nearestPos)))
        return currentWaypointIndex;

    int clampedIndex = std::max(0, std::min(currentWaypointIndex, pathLength - 1));
    int bestWaypoint = clampedIndex;
    int maxLookahead = std::min(clampedIndex + 5, pathLength - 1);
    _stats.lastShortcutIndex = clampedIndex;
    _stats.lastShortcutDistance = 0.0f;
    _stats.lastRaycastHitFraction = 1.0f;

    for (int idx = clampedIndex + 1; idx <= maxLookahead; ++idx)
    {
        int previousBest = bestWaypoint;
        // FIXED: Convert WoW coordinates to Detour
        float target[3];
        WoWToDetour(pathPoints[idx], target);
        dtPolyRef targetRef = 0;
        float nearestTarget[3];
        if (dtStatusFailed(query->findNearestPoly(target, extents, &_defaultFilter, &targetRef, nearestTarget)))
            continue;

        dtRaycastHit hit{};
        dtStatus rayStatus = query->raycast(currentRef, nearestPos, nearestTarget, &_defaultFilter, 0, &hit);
        bool canShortcut = dtStatusSucceed(rayStatus) && hit.t >= 1.0f;
        _stats.raycastAttempts++;
        _stats.lastRaycastHitFraction = hit.t;

        if (canShortcut)
        {
            bestWaypoint = idx;
            _stats.shortcutsApplied++;
            _stats.raycastHits++;
            _stats.lastShortcutIndex = idx;
            float dx = pathPoints[idx].X - pathPoints[previousBest].X;
            float dy = pathPoints[idx].Y - pathPoints[previousBest].Y;
            float dz = pathPoints[idx].Z - pathPoints[previousBest].Z;
            _stats.lastShortcutDistance = std::sqrt(dx * dx + dy * dy + dz * dz);
            currentRef = targetRef;
            nearestPos[0] = nearestTarget[0];
            nearestPos[1] = nearestTarget[1];
            nearestPos[2] = nearestTarget[2];
        }
        else
        {
            break;
        }
    }

    return bestWaypoint;
}

// NavBridge support: Expose dtNavMeshQuery like Honorbuddy RecastManaged
dtNavMeshQuery* Navigation::GetNavMeshQuery(unsigned int mapId)
{
    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    if (!manager)
        return nullptr;
    
    // MMapManager API: GetNavMeshQuery(mapId, instanceId)
    // For bot use, instanceId = 1 (single instance)
    dtNavMeshQuery const* query = manager->GetNavMeshQuery(mapId, 1);
    
    // Remove const for NavBridge C API compatibility
    return const_cast<dtNavMeshQuery*>(query);
}

// ACTION #5: Tile streaming with LRU cache implementation
void Navigation::WorldToTile(float worldX, float worldY, int* tileX, int* tileY)
{
    // WoW tile grid: 64x64 tiles, each tile is 533.33 yards
    // Origin at center, so range is ±17066.66656 yards
    const float TILE_SIZE = 533.33333f;
    const float GRID_ORIGIN = 32.0f * TILE_SIZE;
    
    *tileX = (int)((GRID_ORIGIN - worldX) / TILE_SIZE);
    *tileY = (int)((GRID_ORIGIN - worldY) / TILE_SIZE);
}

// HB 6.2.3 pattern: time-based tile GC (GarbageCollectTime = 1 minute).
// Called once at the start of each FindPath — never from inside EnsureTiles.
void Navigation::GarbageCollectTiles()
{
    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    if (!manager) return;

    uint64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch()
    ).count();

    auto it = _tileAccessTime.begin();
    while (it != _tileAccessTime.end())
    {
        if (now - it->second > TILE_GC_TIMEOUT_MS)
        {
            manager->unloadTile(it->first.mapId, it->first.x, it->first.y);
            it = _tileAccessTime.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

void Navigation::EnsureTiles(unsigned int mapId, XYZ position, int ring)
{
    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    if (!manager) return;
    
    int centerX, centerY;
    WorldToTile(position.X, position.Y, &centerX, &centerY);
    
    uint64_t currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch()
    ).count();
    
    // Load tiles in a ring around player position
    for (int dx = -ring; dx <= ring; dx++)
    {
        for (int dy = -ring; dy <= ring; dy++)
        {
            int tileX = centerX + dx;
            int tileY = centerY + dy;
            
            // Skip invalid tile coords (64x64 grid)
            if (tileX < 0 || tileX >= 64 || tileY < 0 || tileY >= 64)
                continue;
            
            TileKey key = { mapId, tileX, tileY };
            
            // Update access time (or insert if new)
            _tileAccessTime[key] = currentTime;
            
            // Load tile if not already loaded
            manager->loadMap(mapId, tileX, tileY);
        }
    }
}

void Navigation::EnsureTilesDirectional(unsigned int mapId, XYZ position, XYZ velocity, int ring)
{
    // First ensure basic ring
    EnsureTiles(mapId, position, ring);
    
    // Calculate velocity direction
    float speed = sqrtf(velocity.X * velocity.X + velocity.Y * velocity.Y);
    if (speed < 0.1f) return; // Not moving, no prefetch needed
    
    float dirX = velocity.X / speed;
    float dirY = velocity.Y / speed;
    
    MMAP::MMapManager* manager = MMAP::MMapFactory::createOrGetMMapManager();
    if (!manager) return;
    
    int centerX, centerY;
    WorldToTile(position.X, position.Y, &centerX, &centerY);
    
    uint64_t currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch()
    ).count();
    
    // Prefetch 2-3 tiles ahead in movement direction
    for (int i = ring + 1; i <= ring + 3; i++)
    {
        int prefetchX = centerX + (int)(dirX * i);
        int prefetchY = centerY + (int)(dirY * i);
        
        if (prefetchX < 0 || prefetchX >= 64 || prefetchY < 0 || prefetchY >= 64)
            continue;
        
        TileKey key = { mapId, prefetchX, prefetchY };
        _tileAccessTime[key] = currentTime;
        manager->loadMap(mapId, prefetchX, prefetchY);
    }
}
