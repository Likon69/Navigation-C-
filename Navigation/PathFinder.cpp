/**
* MaNGOS is a full featured server for World of Warcraft, supporting
* the following clients: 1.12.x, 2.4.3, 3.3.5a, 4.3.4a and 5.4.8
*
* Copyright (C) 2005-2015  MaNGOS project <http://getmangos.eu>
	m_pathPoints.clear();
	m_straightPathFlags.clear();
	m_straightPathRefs.clear();
	m_polyTypes.clear();
	m_abilityFlags.clear();
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

#include "DetourCommon.h"

#include "MoveMap.h"
#include "PathFinder.h"
#include "OffMeshManager.h"
#include "Navigation.h"

#include <iostream>
#include <fstream>
#include <vector>

////////////////// PathFinder //////////////////
PathFinder::PathFinder(unsigned int mapId, unsigned int instanceId) :
m_polyLength(0), m_type(PATHFIND_BLANK),
m_useStraightPath(false), m_forceDestination(false), m_pointPathLimit(MAX_POINT_PATH_LENGTH),
m_mapId(mapId), m_instanceId(instanceId), m_navMesh(NULL), m_navMeshQuery(NULL),
_stuckCheckTimer(0.0f), // QUICK WIN #3: Initialize anti-stuck timer
_corridorInitialized(false),
	m_filter(nullptr), // AMÉLIORATION #4: Initialize corridor state
	m_lastStatus(NAV_FAILURE | NAV_INVALID_PARAM),
	m_failStep(NAV_STEP_NONE)
{
	//printf("++ PathFinder::PathInfo for ME \n");

    MMAP::MMapManager* mmap = MMAP::MMapFactory::createOrGetMMapManager();
    m_navMesh = mmap->GetNavMesh(m_mapId);
    m_navMeshQuery = mmap->GetNavMeshQuery(m_mapId, m_instanceId);

	createFilter();
	
	// QUICK WIN #3: Initialize stuck check position to zero
	_lastStuckCheckPos = Vector3(0, 0, 0);
}

PathFinder::~PathFinder()
{
    // AMÉLIORATION: Libérer filtre global (Action 2)
    if (m_filter)
    {
        delete m_filter;
        m_filter = nullptr;
    }
	//printf("++ PathFinder::~PathInfo() for ME \n");
}

bool PathFinder::calculate(float originX, float originY, float originZ, float destX, float destY, float destZ, bool forceDest, bool isSwimming)
{
	Vector3 start(originX, originY, originZ);
	setStartPosition(start);

	Vector3 dest(destX, destY, destZ);
	setEndPosition(dest);

	m_forceDestination = forceDest;
	m_lastStatus = NAV_SUCCESS;
	m_failStep = NAV_STEP_NONE;

	//printf("++ PathFinder::calculate() for Me \n");

	// make sure navMesh works - we can run on map w/o mmap
	// check if the start and end point have a .mmtile loaded (can we pass via not loaded tile on the way?)
	if (!m_navMesh || !m_navMeshQuery || !HaveTile(start) || !HaveTile(dest))
	{
		SetFailureStatus(NAV_STEP_INIT_PATHFIND, NAV_FAILURE | NAV_INVALID_PARAM);
		BuildError();

		//printf("!!!!!!! 1 !!!!!!!\n");

		//printf("1 %i\n", !m_navMesh);
		//printf("2 %i\n", !m_navMeshQuery);
		//printf("3 %i\n", !HaveTile(start));
		//printf("4 %i\n", !HaveTile(dest));


		//printf("!!!!!!! 1 !!!!!!!\n");

		m_type = PathType(PATHFIND_NORMAL | PATHFIND_NOT_USING_PATH);
		return true;
	}

	updateFilter(isSwimming, originX, originY, originZ);

	BuildPolyPath(start, dest);
	return true;
}

dtPolyRef PathFinder::getPathPolyByPosition(const dtPolyRef* polyPath, unsigned int polyPathSize, const float* point, float* distance) const
{
	if (!polyPath || !polyPathSize)
	{
		return INVALID_POLYREF;
	}

	dtPolyRef nearestPoly = INVALID_POLYREF;
	float minDist2d = FLT_MAX;
	float minDist3d = 0.0f;

	for (unsigned int i = 0; i < polyPathSize; i++)
	{
		float closestPoint[VERTEX_SIZE];
		dtStatus dtResult = m_navMeshQuery->closestPointOnPoly(polyPath[i], point, closestPoint, NULL);
		if (dtStatusFailed(dtResult))
			continue;

		float d = dtVdist2DSqr(point, closestPoint);
		if (d < minDist2d)
		{
			minDist2d = d;
			nearestPoly = polyPath[i];
			minDist3d = dtVdistSqr(point, closestPoint);
		}

		if (minDist2d < 1.0f) // shortcut out - close enough for us
		{
			break;
		}
	}

	if (distance)
	{
		*distance = sqrtf(minDist3d);
	}

	return (minDist2d < 3.0f) ? nearestPoly : INVALID_POLYREF;
}

dtPolyRef PathFinder::getPolyByLocation(const float* point, float* distance) const
{
	// first we check the current path
	// if the current path doesn't contain the current poly,
	// we need to use the expensive navMesh.findNearestPoly
	dtPolyRef polyRef = getPathPolyByPosition(m_pathPolyRefs, m_polyLength, point, distance);
	if (polyRef != INVALID_POLYREF)
	{
		return polyRef;
	}

	// we don't have it in our old path
	// try to get it by findNearestPoly()
	// IMPORTANT: Utilise les MÃƒÅ MES extents que Honorbuddy WowNavigator.cs
	// extents = (3f, 20f, 3f) - le Y=20 est LA CLÃƒâ€° pour les grottes!
	const float* extents = POLY_SEARCH_EXTENTS;
	float closestPoint[VERTEX_SIZE] = { 0.0f, 0.0f, 0.0f };
	dtStatus dtResult = m_navMeshQuery->findNearestPoly(point, extents, m_filter, &polyRef, closestPoint);
	if (dtStatusSucceed(dtResult) && polyRef != INVALID_POLYREF)
	{
		*distance = dtVdist(closestPoint, point);
		return polyRef;
	}

	// still nothing ..
	// retry with the same HB extents before giving up
	dtResult = m_navMeshQuery->findNearestPoly(point, extents, m_filter, &polyRef, closestPoint);
	if (dtStatusSucceed(dtResult) && polyRef != INVALID_POLYREF)
	{
		*distance = dtVdist(closestPoint, point);
		return polyRef;
	}

	return INVALID_POLYREF;
}

void PathFinder::BuildPolyPath(const Vector3& startPos, const Vector3& endPos)
{
	// *** getting start/end poly logic ***

	float distToStartPoly, distToEndPoly;
	float startPoint[VERTEX_SIZE] = { startPos.y, startPos.z, startPos.x };
	float endPoint[VERTEX_SIZE] = { endPos.y, endPos.z, endPos.x };

	dtPolyRef startPoly = getPolyByLocation(startPoint, &distToStartPoly);
	dtPolyRef endPoly = getPolyByLocation(endPoint, &distToEndPoly);

	dtStatus dtResult;

	// we have a hole in our mesh
	// make shortcut path and mark it as NOPATH ( with flying exception )
	// its up to caller how he will use this info
	if (startPoly == INVALID_POLYREF || endPoly == INVALID_POLYREF)
	{
		if (startPoly == INVALID_POLYREF)
			SetFailureStatus(NAV_STEP_FIND_START_POLY, NAV_FAILURE | NAV_INVALID_PARAM);
		else
			SetFailureStatus(NAV_STEP_FIND_END_POLY, NAV_FAILURE | NAV_INVALID_PARAM);
		//printf("++ BuildPolyPath :: (startPoly == 0 || endPoly == 0)\n");
        BuildError();
		//printf("!!!!!!! 2 !!!!!!!\n");
		m_type = PathType(PATHFIND_NORMAL | PATHFIND_NOT_USING_PATH);

		return;
	}

	// we may need a better number here
	bool farFromPoly = (distToStartPoly > 7.0f || distToEndPoly > 7.0f);
	if (farFromPoly)
	{
		//printf("++ BuildPolyPath :: farFromPoly distToStartPoly=%.3f distToEndPoly=%.3f\n", distToStartPoly, distToEndPoly);
		
		bool isSwimming = false;

		if (isSwimming)
		{
            SetFailureStatus(NAV_STEP_FIND_START_POLY, NAV_FAILURE | NAV_INVALID_PARAM);
            BuildError();
			//printf("!!!!!!! 3 !!!!!!!\n");
			m_type = PathType(PATHFIND_NORMAL | PATHFIND_NOT_USING_PATH);
			return;
		}
		else
		{
			float closestPoint[VERTEX_SIZE];
			// we may want to use closestPointOnPolyBoundary instead
			dtResult = m_navMeshQuery->closestPointOnPoly(endPoly, endPoint, closestPoint, NULL);
			if (dtStatusSucceed(dtResult))
			{
				dtVcopy(endPoint, closestPoint);
				setActualEndPosition(Vector3(endPoint[2], endPoint[0], endPoint[1]));
			}

			m_type = PATHFIND_INCOMPLETE;
		}
	}

	// *** poly path generating logic ***

	// start and end are on same polygon
	// just need to move in straight line
	if (startPoly == endPoly)
	{
		//printf("++ BuildPolyPath :: (startPoly == endPoly)\n");

		BuildShortcut();

		m_pathPolyRefs[0] = startPoly;
		m_polyLength = 1;

		m_type = farFromPoly ? PATHFIND_INCOMPLETE : PATHFIND_NORMAL;
		//printf("++ BuildPolyPath :: path type %d\n", m_type);
		return;
	}

	// look for startPoly/endPoly in current path
	// TODO: we can merge it with getPathPolyByPosition() loop
	bool startPolyFound = false;
	bool endPolyFound = false;
	unsigned int pathStartIndex, pathEndIndex;

	if (m_polyLength)
	{
		for (pathStartIndex = 0; pathStartIndex < m_polyLength; ++pathStartIndex)
		{
			// here to catch few bugs
			//MANGOS_ASSERT(m_pathPolyRefs[pathStartIndex] != INVALID_POLYREF || m_sourceUnit->PrintEntryError("PathFinder::BuildPolyPath"));

			if (m_pathPolyRefs[pathStartIndex] == startPoly)
			{
				startPolyFound = true;
				break;
			}
		}

		for (pathEndIndex = m_polyLength - 1; pathEndIndex > pathStartIndex; --pathEndIndex)
		if (m_pathPolyRefs[pathEndIndex] == endPoly)
		{
			endPolyFound = true;
			break;
		}
	}

	if (startPolyFound && endPolyFound)
	{
		//printf("++ BuildPolyPath :: (startPolyFound && endPolyFound)\n");

		// we moved along the path and the target did not move out of our old poly-path
		// our path is a simple subpath case, we have all the data we need
		// just "cut" it out

		m_polyLength = pathEndIndex - pathStartIndex + 1;
		memmove(m_pathPolyRefs, m_pathPolyRefs + pathStartIndex, m_polyLength * sizeof(dtPolyRef));
	}
	else if (startPolyFound && !endPolyFound)
	{
		//printf("++ BuildPolyPath :: (startPolyFound && !endPolyFound)\n");

		// we are moving on the old path but target moved out
		// so we have atleast part of poly-path ready

		m_polyLength -= pathStartIndex;

		// try to adjust the suffix of the path instead of recalculating entire length
		// at given interval the target can not get too far from its last location
		// thus we have less poly to cover
		// sub-path of optimal path is optimal

		// take ~80% of the original length
		// TODO : play with the values here
		unsigned int prefixPolyLength = unsigned int(m_polyLength * 0.8f + 0.5f);
		memmove(m_pathPolyRefs, m_pathPolyRefs + pathStartIndex, prefixPolyLength * sizeof(dtPolyRef));

		dtPolyRef suffixStartPoly = m_pathPolyRefs[prefixPolyLength - 1];

		// we need any point on our suffix start poly to generate poly-path, so we need last poly in prefix data
		float suffixEndPoint[VERTEX_SIZE];
		dtResult = m_navMeshQuery->closestPointOnPoly(suffixStartPoly, endPoint, suffixEndPoint, NULL);
		if (dtStatusFailed(dtResult))
		{
			// we can hit offmesh connection as last poly - closestPointOnPoly() don't like that
			// try to recover by using prev polyref
			--prefixPolyLength;
			suffixStartPoly = m_pathPolyRefs[prefixPolyLength - 1];
			dtResult = m_navMeshQuery->closestPointOnPoly(suffixStartPoly, endPoint, suffixEndPoint, NULL);
			if (dtStatusFailed(dtResult))
			{
				// suffixStartPoly is still invalid, error state
                BuildError();
				m_type = PATHFIND_NOPATH;
				return;
			}
		}

		// generate suffix
		unsigned int suffixPolyLength = 0;
	dtResult = m_navMeshQuery->findPath(
		suffixStartPoly,    // start polygon
		endPoly,            // end polygon
		suffixEndPoint,     // start position
		endPoint,           // end position
		m_filter,            // polygon search filter
		m_pathPolyRefs + prefixPolyLength - 1,    // [out] path
		(int*)&suffixPolyLength,
		MAX_PATH_LENGTH - prefixPolyLength); // max number of polygons in output path
		if (!suffixPolyLength || dtStatusFailed(dtResult))
		{
			// this is probably an error state, but we'll leave it
			// and hopefully recover on the next Update
			// we still need to copy our preffix
			//sLog.outError("%u's Path Build failed: 0 length path", m_sourceUnit->GetGUIDLow());
		}

		//printf("++  m_polyLength=%u prefixPolyLength=%u suffixPolyLength=%u \n", m_polyLength, prefixPolyLength, suffixPolyLength);

		// new path = prefix + suffix - overlap
		m_polyLength = prefixPolyLength + suffixPolyLength - 1;
	}
	else
	{
		//printf("++ BuildPolyPath :: (!startPolyFound && !endPolyFound)\n");

		// either we have no path at all -> first run
		// or something went really wrong -> we aren't moving along the path to the target
		// just generate new path

		// free and invalidate old path data
		clear();

	dtResult = m_navMeshQuery->findPath(
		startPoly,          // start polygon
		endPoly,            // end polygon
		startPoint,         // start position
		endPoint,           // end position
		m_filter,           // polygon search filter
		m_pathPolyRefs,     // [out] path
		(int*)&m_polyLength,
		MAX_PATH_LENGTH);   // max number of polygons in output path
		ApplyDtStatus(dtResult, NAV_STEP_UPDATE_PATHFIND);
		if (!m_polyLength || dtStatusFailed(dtResult))
		{
			// only happens if we passed bad data to findPath(), or navmesh is messed up
			//sLog.outError("%u's Path Build failed: 0 length path", m_sourceUnit->GetGUIDLow());
	            BuildError();
			m_type = PATHFIND_NOPATH;
			return;
		}
	}

	// by now we know what type of path we can get
	if (m_pathPolyRefs[m_polyLength - 1] == endPoly && !(m_type & PATHFIND_INCOMPLETE))
	{
		m_type = PATHFIND_NORMAL;
	}
	else
	{
		m_type = PATHFIND_INCOMPLETE;
	}

	// generate the point-path out of our up-to-date poly-path
	BuildPointPath(startPoint, endPoint);
}

void PathFinder::BuildPointPath(const float* startPoint, const float* endPoint)
{
	const int maxStraightPath = static_cast<int>(m_pointPathLimit);
	if (maxStraightPath <= 0)
	{
		SetFailureStatus(NAV_STEP_FIND_STRAIGHT_PATH, NAV_FAILURE | NAV_BUFFER_TOO_SMALL);
		BuildError();
		m_type = PATHFIND_NOPATH;
		return;
	}

	std::vector<float> straightPathPoints(maxStraightPath * VERTEX_SIZE);
	std::vector<unsigned char> straightFlags(maxStraightPath);
	std::vector<dtPolyRef> straightPolys(maxStraightPath);
	int straightCount = 0;

	dtStatus dtResult = m_navMeshQuery->findStraightPath(
		startPoint,
		endPoint,
		m_pathPolyRefs,
		m_polyLength,
		straightPathPoints.data(),
		straightFlags.data(),
		straightPolys.data(),
		&straightCount,
		maxStraightPath,
		m_useStraightPath ? 0 : DT_STRAIGHTPATH_ALL_CROSSINGS);

	ApplyDtStatus(dtResult, NAV_STEP_FIND_STRAIGHT_PATH);
	if (dtStatusFailed(dtResult))
	{
		BuildError();
		m_type = PATHFIND_NOPATH;
		return;
	}
	if (straightCount < 2)
	{
		SetFailureStatus(NAV_STEP_FIND_STRAIGHT_PATH, NAV_FAILURE | NAV_INVALID_PARAM);
		BuildError();
		m_type = PATHFIND_NOPATH;
		return;
	}

	m_pathPoints.resize(straightCount);
	for (int i = 0; i < straightCount; ++i)
	{
		m_pathPoints[i] = Vector3(
			straightPathPoints[i * VERTEX_SIZE + 2],
			straightPathPoints[i * VERTEX_SIZE],
			straightPathPoints[i * VERTEX_SIZE + 1]); // GAP 1: no +0.5f — HB 6.2.3 Tripper.RecastManaged copies Z directly from Detour
	}

	const unsigned int pointCount = static_cast<unsigned int>(straightCount);

	m_straightPathFlags.resize(straightCount);
	for (int i = 0; i < straightCount; ++i)
	{
		m_straightPathFlags[i] = static_cast<StraightPathFlags>(straightFlags[i]);
	}
	m_straightPathRefs.assign(straightPolys.begin(), straightPolys.begin() + straightCount);
	m_polyTypes.resize(straightCount);
	m_abilityFlags.resize(straightCount);
	for (int i = 0; i < straightCount; ++i)
	{
		m_polyTypes[i] = ResolveAreaType(m_straightPathRefs[i]);
		m_abilityFlags[i] = ResolveAbilityFlags(m_straightPathRefs[i]);
	}
	
	// AMÃƒâ€°LIORATION #4: Initialize corridor for stable path following
	InitializeCorridor(startPoint, endPoint);

	// first point is always our current location - we need the next one
	setActualEndPosition(m_pathPoints[pointCount - 1]);

	// force the given destination, if needed
	if (m_forceDestination &&
		(!(m_type & PATHFIND_NORMAL) || !inRange(getEndPosition(), getActualEndPosition(), 1.0f, 1.0f)))
	{
		// we may want to keep partial subpath
		if (dist3DSqr(getActualEndPosition(), getEndPosition()) <
			0.3f * dist3DSqr(getStartPosition(), getEndPosition()))
		{
			setActualEndPosition(getEndPosition());
			m_pathPoints[m_pathPoints.size() - 1] = getEndPosition();
		}
		else
		{
			setActualEndPosition(getEndPosition());
	        SetFailureStatus(NAV_STEP_FINALIZE_PATHFIND, NAV_FAILURE | NAV_INVALID_PARAM);
        BuildError();
		}

		//printf("!!!!!!! 4 !!!!!!!\n");
		m_type = PathType(PATHFIND_NORMAL | PATHFIND_NOT_USING_PATH);
	}

	if (m_type & PATHFIND_INCOMPLETE)
	{
		NavStatusAddFlag(m_lastStatus, NAV_PARTIAL_RESULT);
	}

	//printf("++ PathFinder::BuildPointPath path type %d size %d poly-size %d\n", m_type, pointCount, m_polyLength);
}

void PathFinder::BuildError()
{
    clear();

    m_pathPoints.resize(0);

    m_type = PATHFIND_NOPATH;
}

void PathFinder::BuildShortcut()
{
	//printf("++ PathFinder::BuildShortcut :: making shortcut\n");

	clear();

	// make two point path, our curr pos is the start, and dest is the end
	m_pathPoints.resize(2);

	// set start and a default next position
	m_pathPoints[0] = getStartPosition();
	m_pathPoints[1] = getActualEndPosition();

	m_type = PATHFIND_SHORTCUT;
	m_lastStatus = NAV_SUCCESS;
	m_failStep = NAV_STEP_NONE;
}

void PathFinder::SetFailureStatus(NavPathFindStep step, std::uint32_t statusFlags)
{
	m_lastStatus = statusFlags;
	if (!NavStatusHasFlag(m_lastStatus, NAV_FAILURE))
	{
		NavStatusAddFlag(m_lastStatus, NAV_FAILURE);
	}
	m_failStep = step;
}

void PathFinder::ApplyDtStatus(dtStatus status, NavPathFindStep failureStep)
{
	std::uint32_t flags = static_cast<std::uint32_t>(status);
	if (dtStatusFailed(status))
	{
		SetFailureStatus(failureStep, flags);
		return;
	}

	std::uint32_t preserved = m_lastStatus & (NAV_PARTIAL_RESULT | NAV_BUFFER_TOO_SMALL | NAV_OUT_OF_NODES);
	m_lastStatus = flags | preserved;
	m_failStep = NAV_STEP_NONE;
}

void PathFinder::createFilter()
{
    // AMÉLIORATION: Utiliser filtre global unifié (Action 2)
    if (!m_filter)
    {
        m_filter = new dtQueryFilter();
    }
    
	// HB default query flags: include all, exclude unwalkable and transport.
	m_filter->setIncludeFlags(0xffff);
	m_filter->setExcludeFlags(0x0050); // 0x10 | 0x40
    
    // Area costs par défaut (Honorbuddy pattern)
    for (int i = 0; i < DT_MAX_AREAS; ++i)
        m_filter->setAreaCost(i, 1.0f);
}

void PathFinder::updateFilter(bool isSwimming, float x, float y, float z)
{
	(void)isSwimming;
	(void)x;
	(void)y;
	(void)z;

	// HB WoD keeps polygon AreaType values separate from AbilityFlags.
	// AreaType affects cost through dtQueryFilter::setAreaCost(); it must never
	// be ORed into include flags.
}

NavTerrain PathFinder::getNavTerrain(float x, float y, float z)
{
	// Query navmesh polygon area type at this position.
	// Area IDs now use HB-compatible sequential values:
	// NAV_GROUND=1, NAV_WATER=2, NAV_LAVA=3, NAV_ROAD=4, etc.

	float point[VERTEX_SIZE] = { y, z, x }; // WoW (X,Y,Z) → Detour (Y,Z,X)
	float extents[VERTEX_SIZE] = { 3.0f, 5.0f, 3.0f };
	dtPolyRef polyRef = 0;
	float nearestPoint[VERTEX_SIZE];

	dtStatus status = m_navMeshQuery->findNearestPoly(point, extents, m_filter, &polyRef, nearestPoint);
	if (dtStatusFailed(status) || polyRef == 0)
		return NAV_GROUND; // Fallback if no poly found

	unsigned char polyArea = 0;
	status = m_navMesh->getPolyArea(polyRef, &polyArea);
	if (dtStatusFailed(status))
		return NAV_GROUND;

	// Area values are sequential HB IDs stored directly in dtPoly.areaAndtype
	return (NavTerrain)polyArea;
}

bool PathFinder::HaveTile(const Vector3& p) const
{
    int tx, ty;
    float point[VERTEX_SIZE] = { p.y, p.z, p.x };

    m_navMesh->calcTileLoc(point, &tx, &ty);

    if (m_navMesh->getTileAt(tx, ty, 0) == NULL)
    {
        // Phase 9: Removed hardcoded debug file write (was "C:\Users\Drew\..." path)
        // Tile miss is expected during streaming - caller handles gracefully
    }
        
    return (m_navMesh->getTileAt(tx, ty, 0) != NULL);
}
// NEW-6: Removed fixupCorridor, getSteerTarget, findSmoothPath, and inRangeYZX
// These ~240 lines of dead code were never called by BuildPointPath.
// BuildPointPath uses dtNavMeshQuery::findStraightPath() directly for path funneling.
// The smooth path algorithm was an older iterative approach that was superseded.

bool PathFinder::inRange(const Vector3& p1, const Vector3& p2, float r, float h) const
{
	Vector3 d = p1 - p2;
	return (d.x * d.x + d.y * d.y) < r * r && fabsf(d.z) < h;
}

float PathFinder::dist3DSqr(const Vector3& p1, const Vector3& p2) const
{
	return (p1 - p2).squaredLength();
}

// QUICK WIN #1: Raycast shortcut during path following
// Pattern from Honorbuddy MoP + prompt.json Action 1
// Tries to raycast from current position to waypoint+2, skipping intermediate if clear
int PathFinder::UpdateFollowing(const Vector3& currentPos, int currentWaypointIndex)
{
	// Need at least 3 waypoints ahead to attempt shortcut (current, next, next+1)
	if (currentWaypointIndex + 2 >= (int)m_pathPoints.size())
		return 0;
	
	// Convert current position to Detour format
	float curPos[VERTEX_SIZE] = { currentPos.y, currentPos.z, currentPos.x };
	
	// Get target waypoint (+2 ahead)
	const Vector3& targetWaypoint = m_pathPoints[currentWaypointIndex + 2];
	float targetPos[VERTEX_SIZE] = { targetWaypoint.y, targetWaypoint.z, targetWaypoint.x };
	
	// Find nearest poly for current position
	const float* extents = POLY_SEARCH_EXTENTS;
	dtPolyRef curPoly = 0;
	float nearestPt[VERTEX_SIZE];
	dtStatus status = m_navMeshQuery->findNearestPoly(curPos, extents, m_filter, &curPoly, nearestPt);
	
	if (dtStatusFailed(status) || curPoly == 0)
		return 0;
	
	// Raycast to target waypoint
	dtRaycastHit hit;
	hit.path = nullptr;
	hit.pathCount = 0;
	hit.pathCost = 0.0f;
	
	status = m_navMeshQuery->raycast(curPoly, curPos, targetPos, m_filter, 0, &hit);
	
	// If raycast succeeded and no obstacles (t >= 1.0 means reached target)
	if (dtStatusSucceed(status) && hit.t >= 1.0f)
	{
		// AMÃƒâ€°LIORATION #4: Patch corridor after successful shortcut
		// Maintains valid polygon corridor through shortcut
		if (_corridorInitialized && hit.pathCount > 0)
		{
			dtPolyRef targetPoly = hit.path[hit.pathCount - 1];
			PatchCorridor(curPoly, targetPoly, curPos, targetPos);
		}
		
		// Shortcut possible! Remove intermediate waypoint
		// Erase waypoint at currentWaypointIndex + 1
		m_pathPoints.erase(m_pathPoints.begin() + currentWaypointIndex + 1);
		
		// Return 1 waypoint skipped
		return 1;
	}
	
	// No shortcut possible
	return 0;
}

// QUICK WIN #3: Anti-stuck detection and recovery
// AMÃƒâ€°LIORATION #3: Enhanced with wall distance check
// Pattern from prompt.json Action 9
bool PathFinder::IsStuck(const Vector3& currentPos, float deltaTime, float velocityThreshold)
{
    // Accumulate time
    _stuckCheckTimer += deltaTime;
    
    // Check every 2 seconds (avoid too frequent checks)
    if (_stuckCheckTimer < 2.0f)
        return false;
    
    // Calculate distance moved since last check
    float distMoved = dist3DSqr(currentPos, _lastStuckCheckPos);
    distMoved = sqrtf(distMoved); // Get actual distance not squared
    
    // Calculate velocity (distance / time)
    float velocity = distMoved / _stuckCheckTimer;
    
    // Update state for next check
    _lastStuckCheckPos = currentPos;
    _stuckCheckTimer = 0.0f;
    
    // AMÃƒâ€°LIORATION #3: Check if near wall before declaring stuck
    // Avoids false positives during: cast spell, loot, combat stationary
    if (velocity < velocityThreshold)
    {
        // Convert position to Detour format
        float pos[VERTEX_SIZE] = { currentPos.y, currentPos.z, currentPos.x };
        const float* extents = POLY_SEARCH_EXTENTS;
        dtPolyRef poly = 0;
        
        // Find nearest poly
        dtStatus status = m_navMeshQuery->findNearestPoly(pos, extents, m_filter, &poly, nullptr);
        
        if (dtStatusSucceed(status) && poly != 0)
        {
            float hitDist = 0.0f;
            float hitPos[VERTEX_SIZE];
            float hitNormal[VERTEX_SIZE];
            
            // Check distance to nearest wall
            status = m_navMeshQuery->findDistanceToWall(poly, pos, 5.0f, m_filter, 
                                                        &hitDist, hitPos, hitNormal);
            
            // Only stuck if: low velocity + very close to wall (< 0.5m)
            // This filters out: casting, looting, waiting in open space
            if (dtStatusSucceed(status) && hitDist < 0.5f)
            {
                return true; // Confirmed stuck: near wall and not moving
            }
            
            // Low velocity but NOT near wall = likely intentional stop (cast/loot)
            return false;
        }
    }
    
    // Velocity above threshold = not stuck
    return false;
}

Vector3 PathFinder::GetRecoveryNudge(const Vector3& currentPos, const Vector3& targetPos, bool alternateLeft)
{
    // Calculate direction to target
    Vector3 dir = targetPos - currentPos;
    float len = sqrtf(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
    
    if (len < 0.01f)
        return currentPos; // Too close to target
    
    // Normalize direction
    dir.x /= len;
    dir.y /= len;
    dir.z /= len;
    
    // Calculate perpendicular vector (rotate 90 degrees in XY plane)
    // Perpendicular to (x, y, z) is (-y, x, 0) for 2D rotation
    float perpX = alternateLeft ? -dir.y : dir.y;
    float perpY = alternateLeft ? dir.x : -dir.x;
    float perpZ = 0.0f; // Keep Z stable
    
    // Nudge distance (1.5m as per QUICK_WINS.md)
    float nudgeDistance = 1.5f;
    
    // Calculate nudge position
    Vector3 nudgePos;
    nudgePos.x = currentPos.x + perpX * nudgeDistance;
    nudgePos.y = currentPos.y + perpY * nudgeDistance;
    nudgePos.z = currentPos.z + perpZ * nudgeDistance;
    
    return nudgePos;
}

// AMÃƒâ€°LIORATION #4: Initialize path corridor for stable following
// Honorbuddy MoP pattern: dtPathCorridor maintenance
void PathFinder::InitializeCorridor(const float* startPoint, const float* endPoint)
{
    if (m_polyLength == 0 || !m_navMesh)
    {
        _corridorInitialized = false;
        return;
    }
    
    // Initialize corridor with path
    _corridor.init(m_pointPathLimit);
    _corridor.reset(m_pathPolyRefs[0], startPoint);
    _corridor.setCorridor(endPoint, m_pathPolyRefs, m_polyLength);
    
    _corridorInitialized = true;
}

// AMÃƒâ€°LIORATION #4: Patch corridor after raycast shortcut
void PathFinder::PatchCorridor(dtPolyRef startPoly, dtPolyRef endPoly, 
                               const float* startPos, const float* endPos)
{
    if (!_corridorInitialized)
        return;
    
    // Move corridor over off-mesh connection or through shortcut
    // This maintains valid polygon path after raycast shortcuts
    _corridor.moveOverOffmeshConnection(endPoly, &startPoly, (float*)endPos, (float*)startPos, const_cast<dtNavMeshQuery*>(m_navMeshQuery));
}


// AMÉLIORATION: Méthodes filtre global (Action 2)
void PathFinder::setIncludeFlags(unsigned short flags)
{
    if (m_filter)
        m_filter->setIncludeFlags(flags);
}

void PathFinder::setExcludeFlags(unsigned short flags)
{
    if (m_filter)
        m_filter->setExcludeFlags(flags);
}

void PathFinder::setAreaCosts(const float* areaCosts, int count)
{
    if (m_filter && areaCosts)
    {
        for (int i = 0; i < count && i < DT_MAX_AREAS; ++i)
        {
            m_filter->setAreaCost(i, areaCosts[i]);
        }
    }
}

unsigned char PathFinder::ResolveAreaType(dtPolyRef polyRef) const
{
	if (!polyRef || !m_navMesh)
		return 0;

	const dtMeshTile* tile = nullptr;
	const dtPoly* poly = nullptr;
	if (dtStatusFailed(m_navMesh->getTileAndPolyByRef(polyRef, &tile, &poly)) || !poly)
		return 0;

	return static_cast<unsigned char>(poly->getArea());
}

unsigned char PathFinder::ResolveAbilityFlags(dtPolyRef polyRef) const
{
	if (!polyRef || !m_navMesh)
		return 0;

	const dtMeshTile* tile = nullptr;
	const dtPoly* poly = nullptr;
	if (dtStatusFailed(m_navMesh->getTileAndPolyByRef(polyRef, &tile, &poly)) || !poly)
		return 0;

	return static_cast<unsigned char>(poly->flags);
}
// HB 4.3.4: OffMesh Connection detection
bool PathFinder::IsOffMeshWaypoint(int waypointIndex) const
{
    if (waypointIndex < 0 || waypointIndex >= (int)m_pathPoints.size())
        return false;

    const Vector3& waypoint = m_pathPoints[waypointIndex];
    return MMAP::OffMeshManager::Instance().IsOffMeshWaypoint(m_mapId, waypoint, 3.0f);
}

bool PathFinder::GetOffMeshConnectionAt(const Vector3& pos, Vector3* outEnd, 
                                         unsigned char* outType, unsigned int* outInteractId) const
{
    MMAP::OffMeshConnectionInfo conn;
    if (MMAP::OffMeshManager::Instance().IsOffMeshWaypoint(m_mapId, pos, 3.0f, &conn))
    {
        if (outEnd)
            *outEnd = conn.End;
        if (outType)
            *outType = static_cast<unsigned char>(conn.Type);
        if (outInteractId)
            *outInteractId = conn.InteractId;
        return true;
    }
    return false;
}
