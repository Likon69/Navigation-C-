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

## Reference for 1x1

The 1x1 path is also based on the MaNGOS extractor pipeline. The canonical source for the 1x1 movemap generator lives at:

```
C:\Users\Texy6\Desktop\newhcb\Navigation View3D\Extractor_projects-master\mangostwo-server\src\tools\Extractor_projects
```

That repo is a fork of the MaNGOS two (MangosTwo) extractor tools and contains the `Movemap-Generator` that produces the MMAP v4 1x1 tile files this wrapper reads. The MaNGOS project itself is at https://www.getmangos.eu and supports clients 1.12.x, 2.4.3, 3.3.5a, 4.3.4a and 5.4.8.

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
