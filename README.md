# Navigation C++

C++ wrapper around Recast/Detour that provides pathfinding to CopilotBuddy.

This repository is part of the broader CopilotBuddy project. The bot, its WPF UI and the .NET runtime live in the `CopilotBuddy` repository; the offline extractor tools that produce the mmap tiles this wrapper consumes live in the `Extractor_projects` fork of MaNGOS two. All three repositories evolve together, with updates announced on the CopilotBuddy Discord.

## Origin

This code is ported from Tripper.RecastManager of Honorbuddy (WoD and Legion versions). The public C API imitates Honorbuddy so the CopilotBuddy C# code can call it through P/Invoke without changes.

The porting effort itself goes back to July 2025 and went through a clean restart from scratch in October 2025. This wrapper is the result of that second attempt, kept in sync with the public release of the bot.

## What the wrapper does

Recast/Detour is a C++ navmesh generation and query library. CopilotBuddy is C# / .NET and cannot link it directly. This wrapper:

- loads mmap files in both MaNGOS 1x1 format (MMAP v4, one ADT = one Detour tile of 533 yards) and Trinity 4x4 format (MMAP v5, one ADT = 16 Detour sub-tiles of 133 yards)
- exposes a C API (`Navigation/NavBridge.h`) callable from C# through DllImport
- implements what CopilotBuddy needs: A*, raycast, sliced pathfinding, blackspot area/flags manipulation, HB-style raycast, offmesh connections, tile-loaded callbacks
- produces an x86 DLL (`Navigation.dll`) that sits next to `CopilotBuddy.exe`

The mmap format is auto-detected from the file header (`MMAP_VERSION = 4` or `MMAP_MULTI_TILE_VERSION = 5`). Both formats work side by side at runtime.

## Branches

- `master` : main branch, Trinity 4x4 format (MMAP v5)
- `1x1` : variant for MaNGOS 1x1 mmap files (MMAP v4)
- `test-1x1` : experimental branch for 1x1 mode

## Public C API

The wrapper exposes a flat C ABI declared in `Navigation/NavBridge.h`. Every function uses the Cdecl calling convention and is annotated with `NAV_API` (`__declspec(dllexport)` on Windows). The C# side calls into it through `DllImport` from `CopilotBuddy.NativeMethods`.

### Coordinate and status types

```c
typedef struct { float x, y, z; } XYZ_C;

typedef struct PathResult {
    XYZ* points;                          // waypoint positions
    StraightPathFlags* straightPathFlags; // per-point flags (offmesh start/end, etc.)
    unsigned char* polyTypes;            // dtPolyType per waypoint
    unsigned char* abilityFlags;         // HB-style per-waypoint flags
    uint64_t* polyRefs;                  // dtPolyRef per waypoint
    int length;
    uint32_t status;                     // NavStatusFlag bitfield
    NavPathFindStep failStep;            // step at which a failure occurred
} PathResult;

typedef struct NavStats_C {
    float PathfindTimeMs;
    int   PolysVisited;
    float PathLength;
    int   ShortcutsApplied;
    int   StuckRecoveries;
    int   PathRecalculations;
} NavStats_C;
```

`NavStatusFlag` mirrors the Detour `dtStatus` bit field: `NAV_FAILURE`, `NAV_SUCCESS`, `NAV_IN_PROGRESS`, `NAV_PARTIAL_RESULT`, `NAV_OUT_OF_MEMORY`, `NAV_INVALID_PARAM`, `NAV_BUFFER_TOO_SMALL`, `NAV_OUT_OF_NODES`, `NAV_WRONG_MAGIC`, `NAV_WRONG_VERSION`.

### Loader

| Function | Purpose |
| --- | --- |
| `bool Nav_LoadMaps()` | Resolve the mmap folder (a `mmaps\` directory sitting next to the DLL) and initialise every map file. |
| `void Nav_UnloadMaps()` | Release every loaded map and tile. |

### Pathfinding

| Function | Purpose |
| --- | --- |
| `XYZ_C* CalculatePath_C(mapId, start, end, straightPath, *outLength)` | Basic A* query. Returns a heap-allocated `XYZ_C[]` freed with `FreePathArr_C`. |
| `void FreePathArr_C(arr)` | Free the array returned by `CalculatePath_C`. |
| `PathResult* CalculatePathEx(mapId, start, end, straightPath)` | Full result with status, poly refs, flags. Free with `FreePathResult`. |
| `void FreePathResult(result)` | Free a `PathResult` and every owned buffer. |

### Sliced pathfinding (HB-style, frame-budgeted)

| Function | Purpose |
| --- | --- |
| `bool InitSlicedFindPath(mapId, start, end)` | Begin a sliced search for the given map. |
| `bool UpdateSlicedFindPath(int maxIterations)` | Advance the search by N iterations. |
| `bool UpdateSlicedFindPathMs(float msBudget)` | Advance the search until the time budget is consumed. |
| `XYZ_C* FinalizeSlicedFindPath(int maxPathSize, *outLength)` | Produce the final waypoint array once the search has succeeded. |

### Queries

| Function | Purpose |
| --- | --- |
| `bool Raycast_C(mapId, start, end, *hitPos, *tHit)` | Cast a segment against the navmesh. |
| `bool HasLineOfSight_C(mapId, start, end)` | Pure LOS test (no hit output). |
| `bool FindNearestPoint_C(mapId, position, *nearest)` | Snap to the closest valid navmesh point. |
| `bool FindNearestPointEx_C(mapId, position, ex, ey, ez, *nearest)` | Snap with custom search extents. |
| `bool FindRandomPoint_C(mapId, center, radius, *randomPoint)` | Uniform random navigable point. |
| `bool IsPointOnNavMesh_C(mapId, point, tolerance)` | Membership test. |
| `float FindDistanceToWall_C(mapId, position, maxRadius, *hitPoint)` | Distance to the nearest wall. |
| `float FindDistanceToWallEx_C(...)` | Same plus the wall normal. |
| `float FindDistanceToWallFromPoly_C(mapId, polyRef, ...)` | Distance-to-wall from a known polygon. |

### Low-level Detour helpers

| Function | Purpose |
| --- | --- |
| `bool FindNearestPolyRef_C(mapId, pos, extents, *polyRef, *nearestPoint)` | Resolve a `dtPolyRef`. |
| `bool GetPolyHeight_C(mapId, polyRef, pos, *height)` | Sample polygon height at X/Z. |
| `bool ClosestPointOnPoly_C(mapId, polyRef, pos, *closest)` | Closest point inside the polygon. |
| `bool ClosestPointOnPolyBoundary_C(...)` | Closest point on the polygon boundary. |
| `int QueryPolygons_C(mapId, center, extents, *polys, maxPolys)` | Bulk polygon query. |
| `int FindLocalNeighbourhood_C(mapId, startRef, center, radius, *polys, *parents, maxResults)` | BFS neighbourhood. |
| `int GetPolyWallSegments_C(mapId, polyRef, *segStart, *segEnd, maxSegments)` | Edge segments of a polygon. |
| `int FindPolysAroundCircle(mapId, center, radius, *results, maxResults)` | Polys whose centroid lies inside the circle. |
| `bool Raycast_HB_C(mapId, startRef, start, end, *t, *hitNormal, *path, *pathCount, maxPath)` | HB-style raycast that exposes the visited poly corridor. |

### Tile and area filtering

| Function | Purpose |
| --- | --- |
| `void SetIncludeFlags_C(uint16 flags)` / `GetIncludeFlags_C()` | Restrict A* to matching polygon flags. |
| `void SetExcludeFlags_C(uint16 flags)` / `GetExcludeFlags_C()` | Skip matching polygon flags. |
| `bool SetAreaCost_C(mapId, areaType, cost)` / `GetAreaCost_C(areaId)` | Per-area traversal cost (used for blackspot penalties). |
| `void WorldToTile_C(x, z, *tileX, *tileY)` | Convert world coordinates to ADT tile indices. |
| `void EnsureTiles_C(mapId, position, ring)` | Pre-load a square ring of tiles around a position. |
| `void EnsureTilesDirectional_C(mapId, position, velocity, ring)` | Asymmetric pre-load biased along the movement vector. |
| `int UpdatePathFollowing_C(mapId, currentPos, pathLength, pathPoints, currentWaypointIndex, agentId)` | Advance a stored path through the next waypoint. |
| `bool IsTileLoaded_C(mapId, x, y)` | Tile loaded? |
| `bool UnloadTile_C(mapId, x, y)` | Drop a tile (and every Detour sub-tile it contains for V5). |
| `int GetLoadedTilesCount_C(mapId)` | Detour tiles currently resident. |
| `int GetLoadedAdtCount_C(mapId)` | ADT tiles currently resident. |

### Polygon area and flags (blackspot marking)

| Function | Purpose |
| --- | --- |
| `uint SetPolyArea_C(mapId, polyRef, area)` | Override the polygon area type. |
| `uint GetPolyArea_C(mapId, polyRef, *area)` | Read the current area type. |
| `uint SetPolyFlags_C(mapId, polyRef, flags)` | Override polygon traversal flags. |
| `uint GetPolyFlags_C(mapId, polyRef, *flags)` | Read the current flags. |

### Off-mesh connections

| Function | Purpose |
| --- | --- |
| `bool IsOffMeshConnection_C(mapId, position, *outEnd, *outType, *outInteractId)` | Inspect an offmesh connection at a position. |
| `void AddOffMeshConnection_C(mapId, start, end, radius, flags, type, interactId)` | Register a custom offmesh connection at runtime. |
| `bool LoadTileOffMesh_C(mapId, tileX, tileY)` | Force-load offmesh connections for a tile. |

### Telemetry

| Function | Purpose |
| --- | --- |
| `void GetNavStats_C(NavStats_C* outStats)` | Read counters for the last pathfinding call. |
| `void ResetNavStats_C()` | Zero every counter. |

### Callbacks

```c
typedef void (__stdcall *NavBridgeTileLoadedCallback)(unsigned int mapId, int x, int y);
NAV_API void SetTileLoadedCallback_C(NavBridgeTileLoadedCallback callback);

typedef void (__stdcall *NavLogCallbackFn)(const char* msg);
NAV_API void SetNavLogCallback_C(NavLogCallbackFn callback);
```

Tile-load and log events are forwarded into the C# logger (see `OnNavigatorLogMessage` in CopilotBuddy). The callbacks follow the HB calling convention so the C# side can marshal them directly.

### Status helpers

```c
uint32_t NavStatus_FailureFlag(void);
uint32_t NavStatus_SuccessFlag(void);
uint32_t NavStatus_InProgressFlag(void);
uint32_t NavStatus_PartialResultFlag(void);
bool     NavStatus_IsFailure(uint32_t status);
bool     NavStatus_IsSuccess(uint32_t status);
bool     NavStatus_IsInProgress(uint32_t status);
bool     NavStatus_HasFlag(uint32_t status, uint32_t flag);
```

### Raw object access

```c
void* GetNavMeshQuery(unsigned int mapId); // dtNavMeshQuery*, cast on the C# side
void* GetDefaultFilter(void);              // dtQueryFilter*, cast on the C# side
```

These two pointers expose the underlying Detour objects for advanced callers. They remain valid until `Nav_UnloadMaps` is called.

### Memory ownership

Every pointer returned across the boundary is owned by the DLL and must be released with the matching `_C` / `Ex` free function (`FreePathArr_C`, `FreePathResult`). Buffers filled in-place through `out*` parameters are caller-allocated.

## Tile formats: 1x1 (MaNGOS) vs 4x4 (Trinity)

The mmap files are produced by the offline extractor and live as one file per ADT in the `mmaps\` folder next to the DLL. The wrapper supports two on-disk formats side by side; both are read from the same C API.

### File layout

| | 1x1 (v4) | 4x4 (v5) |
| --- | --- | --- |
| Producer | MaNGOS two `Movemap-Generator` | TrinityCore `mmaps-generator` |
| Header | `MmapTileHeader` (single blob) | `magic` + `dtVersion` + `mmapVersion=5` + `tileCount` + `flags` |
| Detour tiles per ADT | 1 (the whole 533 yard ADT) | up to 16 sub-tiles of ~133 yards |
| Sub-tile storage | n/a | inline blobs after the header, one `dtTileRef` slot each |
| Empty slots | n/a | `blobSize == 0` -> placeholder `dtTileRef = 0` skipped by Detour |
| Version constant | `MMAP_VERSION = 4` | `MMAP_MULTI_TILE_VERSION = 5` |

Both formats use the same world-to-ADT mapping: a 64x64 grid with 533.33-yard tiles, origin at the centre of the map. `WorldToTile_C` returns ADT indices in either case.

### How the wrapper handles both at runtime

`MoveMap.cpp::loadMap` peeks at the first 12 bytes of the file, reads `mmapVersion`, and dispatches:

- `mmapVersion == 5` -> reads `tileCount`, then loops over `tileCount` blobs, registering each Detour sub-tile through `dtNavMesh::addTile`. Empty slots become a zero `dtTileRef` in the internal `mmapLoadedTiles[packedGridPos]` vector.
- `mmapVersion == 4` -> rewinds, reads the legacy `MmapTileHeader`, allocates the single blob, and registers it as one Detour tile.
- anything else -> rejected, the file is not loaded.

There is no compile-time switch: the same binary loads v4 and v5 files, and you can mix them map-by-map on a single runtime (one map v4, another v5, etc.). `UnloadTile_C` walks the full vector of refs for the ADT, so a v4 ADT removes 1 Detour tile and a v5 ADT removes up to 16.

### Differences exposed through the C API

The exported function set is identical between the two formats — same names, same signatures, same call sites. The differences are only observable through the two counters and the call patterns:

| Counter | 1x1 (v4) | 4x4 (v5) |
| --- | --- | --- |
| `GetLoadedAdtCount_C(mapId)` | 1 per loaded ADT | 1 per loaded ADT |
| `GetLoadedTilesCount_C(mapId)` | 1 per loaded ADT | 1 to 16 per loaded ADT (depends on which sub-tiles of the ADT are resident) |
| `IsTileLoaded_C(mapId, x, y)` | ADT presence | ADT presence (true even when only some sub-tiles are resident) |
| `UnloadTile_C(mapId, x, y)` | removes 1 Detour tile | removes every resident sub-tile of the ADT |

Use the ratio `GetLoadedTilesCount_C / GetLoadedAdtCount_C` to detect at runtime which format a given map was loaded with: a ratio of 1 indicates v4, anything above 1 indicates v5.

### Caller-visible behavioural differences

- **Granularity.** v4 queries snap within a single 533-yard tile. v5 queries snap within a 133-yard sub-tile. The latter gives finer boundary handling but exposes more seams in the navmesh (rare edge cases where two polygons from different sub-tiles should be stitched but are not).
- **Preload footprint.** `EnsureTiles_C(mapId, position, ring)` and `EnsureTilesDirectional_C(...)` work at ADT granularity in both formats. For v5, loading one ADT pulls up to 16 sub-tiles into memory at once, so a `ring = 1` pre-load typically brings 9 ADTs (~144 sub-tiles) versus 9 ADTs (9 tiles) under v4. Memory pressure differs accordingly.
- **Unloading.** v5 lets the engine unload a single sub-tile through `dtNavMesh::removeTile`, but the public wrapper API only exposes ADT-level unload (`UnloadTile_C`). Partial sub-tile eviction is therefore not reachable from C#.
- **Off-mesh connections.** Both formats call `OffMeshManager::LoadTileOffMeshConnections(mapId, x, y)` after the ADT is registered. The off-mesh record count is identical between the two formats; only the Detour tile partitioning differs.
- **HB-style raycast and polygon queries.** No code path branches on `mmapVersion`. `Raycast_HB_C`, `FindNearestPolyRef_C`, `QueryPolygons_C`, etc. behave identically.

### Picking a format at build time

The branches in this repository reflect the chosen on-disk format, not the API:

- `master` -> Trinity 4x4 (v5) mmap files
- `1x1` -> MaNGOS 1x1 (v4) mmap files
- `test-1x1` -> experimental 1x1 work

The C API and the public behaviour described in this README are the same on every branch. The only difference between `master` and `1x1` is which format the loader accepts at runtime and which extractor must be used to produce the mmap files in the first place.

## Build with Visual Studio

1. Open `Navigation/Navigation.sln` in Visual Studio 2022 (toolset v145).
2. Select the `Release | Win32` configuration. x86 is mandatory because the bot is x86.
3. Build > Build Solution (Ctrl+Shift+B).
4. The DLL is written to `Bot/Release/Navigation.dll`.
5. Copy `Navigation.dll` into the `bin/` folder of CopilotBuddy, next to `CopilotBuddy.exe`.

The project only depends on Recast/Detour (vendored under `Navigation/Detour/`) and g3dlite (vendored under `Navigation/g3dlite/`). No external dependencies to install.

## Contributing

Appropriate contributions are welcome: bug reports with reproducible mmap files, raycast or pathfinding edge cases, performance improvements, and updates to keep the wrapper aligned with future Honorbuddy API changes.

Updates to this wrapper ship alongside CopilotBuddy and are announced on the Discord. If you are submitting a pull request, please include a small test case (a single .mmtile file plus start and end coordinates) so the change can be validated against the existing behavior.

A sincere thank you to the community around CopilotBuddy. The bug reports and shared test cases on the Discord and the issue tracker are what keep this wrapper moving forward, and the support is very much appreciated.
