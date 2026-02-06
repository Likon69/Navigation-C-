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

#ifndef MANGOS_PATH_FINDER_H
#define MANGOS_PATH_FINDER_H

#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourPathCorridor.h"
#include <vector>
#include <cstdint>

// AMÉLIORATION: Filtre global pour cohérence des coûts/flags (Action 2)
class dtQueryFilter;

#include "MoveMapSharedDefines.h"
#include "G3D/Vector3.h"
#include "NavStatus.h"
#include "StraightPathFlags.h"

namespace Movement
{
	using G3D::Vector2;
	using G3D::Vector3;
	using G3D::Vector4;
	typedef std::vector<Vector3> PointsArray;
}

using Movement::Vector3;
using Movement::PointsArray;

// 74*4.0f=296y  number_of_points*interval = max_path_len
// this is way more than actual evade range
// I think we can safely cut those down even more
#define MAX_PATH_LENGTH         740//74
#define MAX_POINT_PATH_LENGTH   740//74

#define SMOOTH_PATH_STEP_SIZE   4.0f
#define SMOOTH_PATH_SLOP        0.3f

#define VERTEX_SIZE       3
#define INVALID_POLYREF   0

// AMÉLIORATION: Extents cohérents pour grottes/multi-couches (Action 4)
// Format: {horizontal_radius, vertical_range, horizontal_radius}
// Valeurs {3, 20, 3} permettent de gérer les caves et surfaces superposées
static const float POLY_SEARCH_EXTENTS[3] = {3.0f, 20.0f, 3.0f};

// defined in DBC and left shifted for flag usage
#define MAP_LIQUID_TYPE_NO_WATER    0x00
#define MAP_LIQUID_TYPE_MAGMA       0x01
#define MAP_LIQUID_TYPE_OCEAN       0x02
#define MAP_LIQUID_TYPE_SLIME       0x04
#define MAP_LIQUID_TYPE_WATER       0x08

#define MAP_ALL_LIQUIDS   (MAP_LIQUID_TYPE_WATER | MAP_LIQUID_TYPE_MAGMA | MAP_LIQUID_TYPE_OCEAN | MAP_LIQUID_TYPE_SLIME)

#define MAP_LIQUID_TYPE_DARK_WATER  0x10
#define MAP_LIQUID_TYPE_WMO_WATER   0x20

struct GridMapLiquidData
{
	unsigned int type_flags;
	unsigned int entry;
	float level;
	float depth_level;
};

enum PathType
{
	PATHFIND_BLANK = 0x0000,   // path not built yet
	PATHFIND_NORMAL = 0x0001,   // normal path
	PATHFIND_SHORTCUT = 0x0002,   // travel through obstacles, terrain, air, etc (old behavior)
	PATHFIND_INCOMPLETE = 0x0004,   // we have partial path to follow - getting closer to target
	PATHFIND_NOPATH = 0x0008,   // no valid path at all or error in generating one
	PATHFIND_NOT_USING_PATH = 0x0010    // used when we are either flying/swiming or on map w/o mmaps
};

class PathFinder
{
public:
	PathFinder(unsigned int mapId, unsigned int instanceId);
	~PathFinder();

	// Calculate the path from owner to given destination
	// return: true if new path was calculated, false otherwise (no change needed)
	bool calculate(float originX, float originY, float originZ, float destX, float destY, float destZ, bool forceDest = false, bool isSwimming = false);

	// option setters - use optional
	void setUseStrightPath(bool useStraightPath) { m_useStraightPath = useStraightPath; };
	void setPathLengthLimit(float distance) { m_pointPathLimit = std::min<unsigned int>(unsigned int(distance / SMOOTH_PATH_STEP_SIZE), MAX_POINT_PATH_LENGTH); };

	// result getters
	Vector3 getStartPosition()      const { return m_startPosition; }
	Vector3 getEndPosition()        const { return m_endPosition; }
	Vector3 getActualEndPosition()  const { return m_actualEndPosition; }

	PointsArray& getPath() { return m_pathPoints; }
	PathType getPathType() const { return m_type; }
	const std::vector<StraightPathFlags>& GetStraightPathFlags() const { return m_straightPathFlags; }
	const std::vector<dtPolyRef>& GetStraightPathRefs() const { return m_straightPathRefs; }
	const std::vector<unsigned char>& GetPolyTypes() const { return m_polyTypes; }
	const std::vector<unsigned char>& GetAbilityFlags() const { return m_abilityFlags; }
	std::uint32_t GetLastStatus() const { return m_lastStatus; }
	NavPathFindStep GetFailStep() const { return m_failStep; }
	
	// QUICK WIN #1: Raycast shortcut during path following
	// Returns number of waypoints skipped (0 if no shortcut found)
	int UpdateFollowing(const Vector3& currentPos, int currentWaypointIndex);
	
	// QUICK WIN #3: Anti-stuck detection and recovery
	bool IsStuck(const Vector3& currentPos, float deltaTime, float velocityThreshold = 0.5f);
	Vector3 GetRecoveryNudge(const Vector3& currentPos, const Vector3& targetPos, bool alternateLeft);

	// AMÉLIORATION: Filtre global flags (Action 2)
	void setIncludeFlags(unsigned short flags);
	void setExcludeFlags(unsigned short flags);
	
	// AMÉLIORATION: Copy area costs from external array (for blackspot support)
	void setAreaCosts(const float* areaCosts, int count);

	// QUICK WIN #4: dtPathCorridor management
	void InitializeCorridor(const float* startPoint, const float* endPoint);
	void PatchCorridor(dtPolyRef startPoly, dtPolyRef endPoly, const float* startPos, const float* endPos);

	// HB 4.3.4: OffMesh Connection detection
	bool IsOffMeshWaypoint(int waypointIndex) const;
	bool GetOffMeshConnectionAt(const Vector3& pos, Vector3* outEnd, unsigned char* outType, unsigned int* outInteractId) const;

private:

	dtPolyRef      m_pathPolyRefs[MAX_PATH_LENGTH];   // array of detour polygon references
	unsigned int         m_polyLength;                      // number of polygons in the path

	PointsArray    m_pathPoints;       // our actual (x,y,z) path to the target
	PathType       m_type;             // tells what kind of path this is

	std::vector<StraightPathFlags> m_straightPathFlags;
	std::vector<dtPolyRef>     m_straightPathRefs;
	std::vector<unsigned char> m_polyTypes;
	std::vector<unsigned char> m_abilityFlags;

	bool           m_useStraightPath;  // type of path will be generated
	bool           m_forceDestination; // when set, we will always arrive at given point
	unsigned int         m_pointPathLimit;   // limit point path size; min(this, MAX_POINT_PATH_LENGTH)

	Vector3        m_startPosition;    // {x, y, z} of current location
	Vector3        m_endPosition;      // {x, y, z} of the destination
	Vector3        m_actualEndPosition;// {x, y, z} of the closest possible point to given destination

	const unsigned int      m_mapId;       // map id
	const unsigned int      m_instanceId;       // instance id
	const dtNavMesh*        m_navMesh;          // the nav mesh
	const dtNavMeshQuery*   m_navMeshQuery;     // the nav mesh query used to find the path


	void setStartPosition(const Vector3 &point) { m_startPosition = point; }
	void setEndPosition(const Vector3 &point) { m_actualEndPosition = point; m_endPosition = point; }
	void setActualEndPosition(const Vector3 &point) { m_actualEndPosition = point; }

	void clear()
	{
		m_polyLength = 0;
		m_pathPoints.clear();
	}

	bool inRange(const Vector3& p1, const Vector3& p2, float r, float h) const;
	float dist3DSqr(const Vector3& p1, const Vector3& p2) const;
	bool inRangeYZX(const float* v1, const float* v2, float r, float h) const;

	dtPolyRef getPathPolyByPosition(const dtPolyRef* polyPath, unsigned int polyPathSize, const float* point, float* distance = NULL) const;
	dtPolyRef getPolyByLocation(const float* point, float* distance) const;
	bool HaveTile(const Vector3& p) const;

	void BuildPolyPath(const Vector3& startPos, const Vector3& endPos);
	void BuildPointPath(const float* startPoint, const float* endPoint);
    void BuildError();
	void BuildShortcut();

	NavTerrain getNavTerrain(float x, float y, float z);
	void createFilter();
	void updateFilter(bool isSwimming, float x, float y, float z);

	unsigned char ResolveAreaType(dtPolyRef polyRef) const;
	unsigned char ResolveAbilityFlags(dtPolyRef polyRef) const;

	// smooth path aux functions
	unsigned int fixupCorridor(dtPolyRef* path, unsigned int npath, unsigned int maxPath,
		const dtPolyRef* visited, unsigned int nvisited);
	bool getSteerTarget(const float* startPos, const float* endPos, float minTargetDist,
		const dtPolyRef* path, unsigned int pathSize, float* steerPos,
		unsigned char& steerPosFlag, dtPolyRef& steerPosRef);
	dtStatus findSmoothPath(const float* startPos, const float* endPos,
		const dtPolyRef* polyPath, unsigned int polyPathSize,
		float* smoothPath, int* smoothPathSize, unsigned int smoothPathMaxSize);
	
	// QUICK WIN #3: Anti-stuck state tracking
	Vector3 _lastStuckCheckPos;
	float _stuckCheckTimer;

	// QUICK WIN #4: dtPathCorridor for dynamic path following
	dtPathCorridor _corridor;
	bool _corridorInitialized;

	// AMÉLIORATION: Filtre global unifié (Action 2)
	dtQueryFilter* m_filter;

	// HB-style status tracking
	std::uint32_t m_lastStatus;
	NavPathFindStep m_failStep;
	void SetFailureStatus(NavPathFindStep step, std::uint32_t statusFlags);
	void ApplyDtStatus(dtStatus status, NavPathFindStep failureStep);

};

#endif
