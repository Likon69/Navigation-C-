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
#define MMAP_MULTI_TILE_VERSION 5  // 4×4 sub-tiles per ADT (16 Detour blobs)
#define MMAP_SUBTILES_PER_ADT 16   // 4×4 grid = 16 sub-tiles
#define SIZE_OF_GRIDS 533.33333f

// Version 4 — legacy 1-blob-per-ADT format
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

// Version 5 — 4×4 multi-tile format (16 Detour blobs per ADT file)
// File layout: MmapMultiTileHeader + 16 × (uint32 blobSize + byte[blobSize] detourData)
struct MmapMultiTileHeader
{
	unsigned int mmapMagic;     // MMAP_MAGIC (0x4d4d4150)
	unsigned int dtVersion;     // DT_NAVMESH_VERSION (7)
	unsigned int mmapVersion;   // MMAP_MULTI_TILE_VERSION (5)
	unsigned int tileCount;     // Number of sub-tile slots (16)
	unsigned int flags;         // Reserved (1)
};

// HB 6.2.3 compatible AreaType enum (sequential IDs, NOT bitmask)
// Must match CopilotBuddy/Tripper/Navigation/AreaType.cs
enum NavTerrain
{
	NAV_EMPTY            = 0,
	NAV_GROUND           = 1,
	NAV_WATER            = 2,
	NAV_LAVA             = 3,
	NAV_ROAD             = 4,
	NAV_FALL             = 5,
	NAV_ELEVATOR         = 6,
	NAV_GATE             = 7,
	NAV_PORTAL           = 8,
	NAV_DEFENDERS_PORTAL = 9,
	NAV_HORDE_PORTAL     = 10,
	NAV_ALLIANCE_PORTAL  = 11,
	NAV_BLOCKED          = 12,
	NAV_INTERACT_UNIT    = 13,
	NAV_INTERACT_OBJECT  = 14,
	NAV_HORDE            = 15,
	NAV_ALLIANCE         = 16,
	NAV_BLACKSPOT        = 17,
	NAV_KNOWN_BUILDING   = 18,
	NAV_MISC1            = 20,
	NAV_MISC2            = 21,
	NAV_MISC3            = 22,
	NAV_MISC4            = 23,
	NAV_MISC5            = 24,
	NAV_MISC6            = 25,
	NAV_MISC7            = 26,
	NAV_MISC8            = 27,
	NAV_MISC9            = 28,
	NAV_MISC10           = 29,
	// Detour stores area in 6 bits of dtPoly.areaAndtype (max 63)
};

#endif  // _MOVE_MAP_SHARED_DEFINES_H
