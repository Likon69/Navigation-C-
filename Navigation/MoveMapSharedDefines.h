/**
* MaNGOS is a full featured server for World of Warcraft, supporting
* the following clients: 1.12.x, 2.4.3, 3.3.5a, 4.3.4a and 5.4.8
*
* Copyright (C) 2005-2015  MaNGOS project <http://getmangos.eu>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*
* World of Warcraft, and all World of Warcraft or Warcraft art, images,
* and lore are copyrighted by Blizzard Entertainment, Inc.
*/

#ifndef MANGOS_H_MOVE_MAP_SHARED_DEFINES
#define MANGOS_H_MOVE_MAP_SHARED_DEFINES

#include "DetourNavMesh.h"

#define MMAP_MAGIC 0x4d4d4150   // 'MMAP'
#define MMAP_VERSION 4
#define SIZE_OF_GRIDS 533.33333f

struct MmapTileHeader
{
	unsigned int mmapMagic;
	unsigned int dtVersion;
	unsigned int mmapVersion;
	unsigned int size;
	bool usesLiquids : 1;

	MmapTileHeader() : mmapMagic(MMAP_MAGIC), dtVersion(DT_NAVMESH_VERSION),
		mmapVersion(MMAP_VERSION), size(0), usesLiquids(false) {}//usesLiquids(true) {} //Remove liquid in paths (not 100% with current maps)
};

// HB 6.2.3 compatible AreaType enum (sequential IDs, NOT bitmask)
// Must match CopilotBuddy/Tripper/Navigation/AreaType.cs
enum NavTerrain
{
	NAV_EMPTY    = 0,   // Unwalkable / null area
	NAV_GROUND   = 1,   // AreaType.Ground  (cost 1.66)
	NAV_WATER    = 2,   // AreaType.Water   (cost 3.33)
	NAV_LAVA     = 3,   // AreaType.Lava    (cost 55.0) — covers both magma and slime
	NAV_ROAD     = 4,   // AreaType.Road    (cost 1.0)  — preferred path
	NAV_FALL     = 5,   // AreaType.Fall    (cost 1.7)
	NAV_ELEVATOR = 6,   // AreaType.Elevator(cost 3.16)
	NAV_GATE     = 7,   // AreaType.Gate    (cost 1.66)
	// 8-63 available for Portal, Blocked, Blackspot, etc.
	// Detour stores area in 6 bits of dtPoly.areaAndtype (max 63)
};

#endif  // _MOVE_MAP_SHARED_DEFINES_H
