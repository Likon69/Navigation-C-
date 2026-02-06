/**
 * WotLK 3.3.5a - OffMesh Connection Manager Implementation
 * Based on HB 4.3.4 Tripper.RecastManaged.Detour.OffMeshConnection
 */

#include "OffMeshManager.h"
#include "MoveMap.h"
#include "DetourNavMeshBuilder.h"
#include <fstream>
#include <iostream>
#include <cstring>
#include <cstdint>
#include <limits>
#include <array>
#include <algorithm>

namespace MMAP
{
    namespace
    {
        constexpr uint32_t OFFMESH_MAGIC = 0x4D4D4F46;
        constexpr uint32_t OFFMESH_VERSION_LEGACY = 1;
        constexpr uint32_t OFFMESH_VERSION_LATEST = 2;
        constexpr unsigned char AREA_GROUND = 1;
        constexpr unsigned char AREA_ELEVATOR = 6;
        constexpr unsigned char AREA_PORTAL = 8;
        constexpr unsigned char AREA_INTERACT_UNIT = 13;
        constexpr unsigned char AREA_INTERACT_OBJECT = 14;

        unsigned char ResolveAreaForConnection(OffMeshConnectionType type)
        {
            switch (type)
            {
            case OFFMESH_TYPE_ELEVATOR:
                return AREA_ELEVATOR;
            case OFFMESH_TYPE_PORTAL:
                return AREA_PORTAL;
            case OFFMESH_TYPE_INTERACT_UNIT:
                return AREA_INTERACT_UNIT;
            case OFFMESH_TYPE_INTERACT_OBJECT:
                return AREA_INTERACT_OBJECT;
            case OFFMESH_TYPE_NORMAL:
            default:
                return AREA_GROUND;
            }
        }
    }

    // ===========================
    // OffMeshConnectionSet
    // ===========================

    void OffMeshConnectionSet::AddConnection(const OffMeshConnectionInfo& conn)
    {
        m_connections.push_back(conn);
    }

    bool OffMeshConnectionSet::HasConnectionAt(const Vector3& pos, float tolerance) const
    {
        return GetConnectionAt(pos, tolerance) != nullptr;
    }

    const OffMeshConnectionInfo* OffMeshConnectionSet::GetConnectionAt(const Vector3& pos, float tolerance) const
    {
        for (const auto& conn : m_connections)
        {
            // Check distance to start point
            if (conn.IsNearStart(pos, tolerance))
                return &conn;

            // Check bidirectional end point
            if (conn.IsNearEnd(pos, tolerance))
                return &conn;
        }
        return nullptr;
    }

    // ===========================
    // OffMeshManager
    // ===========================

    OffMeshManager& OffMeshManager::Instance()
    {
        static OffMeshManager instance;
        return instance;
    }

    void OffMeshManager::RegisterTileNavParams(unsigned int mapId, const dtMeshHeader* header)
    {
        if (!header)
            return;

        MapNavParams& params = m_navParams[mapId];
        params.walkableHeight = header->walkableHeight;
        params.walkableRadius = header->walkableRadius;
        params.walkableClimb = header->walkableClimb;
        if (header->bvQuantFactor > 0.0f)
            params.cellSize = 1.0f / header->bvQuantFactor;
        // cellHeight is not serialized in dtMeshHeader; keep default 0.2f
    }

    bool OffMeshManager::LoadTileOffMeshConnections(unsigned int mapId, int tileX, int tileY)
    {
        TileKey key { mapId, tileX, tileY };
        if (m_loadedTileKeys.find(key) != m_loadedTileKeys.end())
            return true;

        // Format: mmaps/{mapId:000}_{tileX:00}_{tileY:00}.offmesh
        // Binary format:
        //   [uint32 magic = 0x4D4D4F46]  // "OFFM" magic
        //   [uint32 version = 1]
        //   [uint32 mapId]
        //   [uint32 count]
        //   For each connection (48 bytes):
        //     [float startX, startY, startZ]     (12 bytes)
        //     [float endX, endY, endZ]           (12 bytes)
        //     [float radius]                     (4 bytes)
        //     [ushort polyIndex]                 (2 bytes)
        //     [byte flags]                       (1 byte)
        //     [byte side]                        (1 byte)
        //     [uint32 userId]                    (4 bytes)
        //     [byte type]                        (1 byte)
        //     [byte padding]                     (1 byte)
        //     [ushort padding2]                  (2 bytes)
        //     [uint32 interactId]                (4 bytes)
        //     [uint32 sourceMapId]               (4 bytes)
        //     [uint32 targetMapId]               (4 bytes)

        char fileName[256];
        sprintf_s(fileName, sizeof(fileName), "mmaps/%03u_%02d_%02d.offmesh", mapId, tileX, tileY);

        std::ifstream file(fileName, std::ios::binary);
        if (!file.is_open())
        {
            // Not an error if file doesn't exist (map has no offmesh)
            return true;
        }

        // Read header
        uint32_t magic = 0;
        uint32_t version = 0;
        uint32_t fileMapId = 0;
        uint32_t count = 0;

        file.read(reinterpret_cast<char*>(&magic), sizeof(magic));
        file.read(reinterpret_cast<char*>(&version), sizeof(version));
        file.read(reinterpret_cast<char*>(&fileMapId), sizeof(fileMapId));
        file.read(reinterpret_cast<char*>(&count), sizeof(count));

        // Validate header
        if (magic != OFFMESH_MAGIC)
        {
            std::cerr << "[OffMeshManager] Invalid magic in " << fileName << std::endl;
            file.close();
            return false;
        }

        if (version != OFFMESH_VERSION_LEGACY && version != OFFMESH_VERSION_LATEST)
        {
            std::cerr << "[OffMeshManager] Unsupported version " << version << " in " << fileName << std::endl;
            file.close();
            return false;
        }

        if (fileMapId != mapId)
        {
            std::cerr << "[OffMeshManager] Map ID mismatch in " << fileName 
                      << " (expected " << mapId << ", got " << fileMapId << ")" << std::endl;
            file.close();
            return false;
        }

        // Read connections
        OffMeshConnectionSet& set = m_mapOffMeshData[mapId];
        std::vector<OffMeshConnectionInfo> tileConnections;
        tileConnections.reserve(count);

        for (uint32_t i = 0; i < count; ++i)
        {
            OffMeshConnectionInfo conn;
            if (version == OFFMESH_VERSION_LEGACY)
            {
                // Legacy 48-byte record
                file.read(reinterpret_cast<char*>(&conn.Start.x), sizeof(float));
                file.read(reinterpret_cast<char*>(&conn.Start.y), sizeof(float));
                file.read(reinterpret_cast<char*>(&conn.Start.z), sizeof(float));
                file.read(reinterpret_cast<char*>(&conn.End.x), sizeof(float));
                file.read(reinterpret_cast<char*>(&conn.End.y), sizeof(float));
                file.read(reinterpret_cast<char*>(&conn.End.z), sizeof(float));
                file.read(reinterpret_cast<char*>(&conn.Radius), sizeof(float));

                uint32_t legacyFlags = 0;
                file.read(reinterpret_cast<char*>(&legacyFlags), sizeof(uint32_t));

                unsigned char legacyType = 0;
                file.read(reinterpret_cast<char*>(&legacyType), sizeof(unsigned char));
                unsigned char legacyReserved = 0;
                file.read(reinterpret_cast<char*>(&legacyReserved), sizeof(unsigned char));
                file.read(reinterpret_cast<char*>(&conn.WaitTimeMs), sizeof(unsigned short));
                file.read(reinterpret_cast<char*>(&conn.InteractId), sizeof(uint32_t));
                uint64_t legacyPadding = 0;
                file.read(reinterpret_cast<char*>(&legacyPadding), sizeof(uint64_t));

                if (!file.good())
                {
                    std::cerr << "[OffMeshManager] Truncated legacy offmesh record in " << fileName << std::endl;
                    file.close();
                    return false;
                }

                conn.PolyIndex = 0;
                conn.Flags = (legacyFlags & 1) ? OFFMESH_BIDIRECTIONAL : OFFMESH_ONEWAY;
                conn.Side = 0;
                conn.UserId = 0;
                conn.Type = static_cast<OffMeshConnectionType>(legacyType);
                conn.MapId = mapId;
                conn.TargetMapId = mapId;
            }
            else
            {
                // Version 2 (HB layout, 52 bytes)
                file.read(reinterpret_cast<char*>(&conn.Start.x), sizeof(float));
                file.read(reinterpret_cast<char*>(&conn.Start.y), sizeof(float));
                file.read(reinterpret_cast<char*>(&conn.Start.z), sizeof(float));
                file.read(reinterpret_cast<char*>(&conn.End.x), sizeof(float));
                file.read(reinterpret_cast<char*>(&conn.End.y), sizeof(float));
                file.read(reinterpret_cast<char*>(&conn.End.z), sizeof(float));
                file.read(reinterpret_cast<char*>(&conn.Radius), sizeof(float));
                file.read(reinterpret_cast<char*>(&conn.PolyIndex), sizeof(unsigned short));
                file.read(reinterpret_cast<char*>(&conn.Flags), sizeof(unsigned char));
                file.read(reinterpret_cast<char*>(&conn.Side), sizeof(unsigned char));
                file.read(reinterpret_cast<char*>(&conn.UserId), sizeof(uint32_t));

                unsigned char latestType = 0;
                file.read(reinterpret_cast<char*>(&latestType), sizeof(unsigned char));
                unsigned char latestReserved = 0;
                file.read(reinterpret_cast<char*>(&latestReserved), sizeof(unsigned char));
                file.read(reinterpret_cast<char*>(&conn.WaitTimeMs), sizeof(unsigned short));
                file.read(reinterpret_cast<char*>(&conn.InteractId), sizeof(uint32_t));
                file.read(reinterpret_cast<char*>(&conn.MapId), sizeof(uint32_t));
                file.read(reinterpret_cast<char*>(&conn.TargetMapId), sizeof(uint32_t));

                if (!file.good())
                {
                    std::cerr << "[OffMeshManager] Truncated offmesh record in " << fileName << std::endl;
                    file.close();
                    return false;
                }

                conn.Type = static_cast<OffMeshConnectionType>(latestType);
                if (conn.MapId == 0)
                    conn.MapId = mapId;
                if (conn.TargetMapId == 0)
                    conn.TargetMapId = conn.MapId;
            }

            set.AddConnection(conn);
            tileConnections.push_back(conn);
        }

        file.close();
        if (tileConnections.empty())
        {
            std::cout << "[OffMeshManager] No offmesh data for map " << mapId
                      << " tile (" << tileX << "," << tileY << ")" << std::endl;
            m_loadedTileKeys.insert(key);
            return true;
        }

        if (!InjectConnections(mapId, tileX, tileY, tileConnections))
            return false;

        m_loadedTileKeys.insert(key);
        std::cout << "[OffMeshManager] Injected " << tileConnections.size()
                  << " offmesh connections for map " << mapId
                  << " tile (" << tileX << "," << tileY << ")" << std::endl;
        return true;
    }
    
    bool OffMeshManager::LoadMapOffMeshConnections(unsigned int mapId)
    {
        // Legacy function: Load all tiles for the map (0-63 x 0-63)
        // This is used for backward compatibility or full map preload
        int loadedCount = 0;
        
        for (int x = 0; x < 64; ++x)
        {
            for (int y = 0; y < 64; ++y)
            {
                if (LoadTileOffMeshConnections(mapId, x, y))
                    loadedCount++;
            }
        }
        
        if (loadedCount > 0)
        {
            std::cout << "[OffMeshManager] Loaded offmesh data from " << loadedCount 
                      << " tiles for map " << mapId << std::endl;
        }
        
        return true;
    }

    const OffMeshConnectionInfo* OffMeshManager::GetOffMeshAt(unsigned int mapId, 
                                                                const Vector3& pos, 
                                                                float tolerance) const
    {
        auto it = m_mapOffMeshData.find(mapId);
        if (it == m_mapOffMeshData.end())
            return nullptr;

        return it->second.GetConnectionAt(pos, tolerance);
    }

    void OffMeshManager::AddOffMeshConnection(unsigned int mapId, const OffMeshConnectionInfo& conn)
    {
        m_mapOffMeshData[mapId].AddConnection(conn);
    }

    bool OffMeshManager::IsOffMeshWaypoint(unsigned int mapId, const Vector3& pos, 
                                            float tolerance, OffMeshConnectionInfo* outConn) const
    {
        const OffMeshConnectionInfo* conn = GetOffMeshAt(mapId, pos, tolerance);
        if (conn)
        {
            if (outConn)
                *outConn = *conn;
            return true;
        }
        return false;
    }

    const OffMeshConnectionSet* OffMeshManager::GetMapConnections(unsigned int mapId) const
    {
        auto it = m_mapOffMeshData.find(mapId);
        if (it == m_mapOffMeshData.end())
            return nullptr;
        return &it->second;
    }

    void OffMeshManager::Clear()
    {
        m_mapOffMeshData.clear();
        m_navParams.clear();
        m_injectedTileRefs.clear();
        m_loadedTileKeys.clear();
    }

    uint64_t OffMeshManager::BuildPackedKey(unsigned int mapId, int tileX, int tileY) const
    {
        return (static_cast<uint64_t>(mapId) << 32) |
               ((static_cast<uint64_t>(tileX) & 0xFFFF) << 16) |
               (static_cast<uint64_t>(tileY) & 0xFFFF);
    }

    MapNavParams OffMeshManager::ResolveNavParams(unsigned int mapId) const
    {
        auto it = m_navParams.find(mapId);
        if (it != m_navParams.end())
            return it->second;
        return MapNavParams{};
    }

    int OffMeshManager::ReserveTileLayer(dtNavMesh* navMesh, int tileX, int tileY) const
    {
        if (!navMesh)
            return 1;

        constexpr int MAX_LAYERS = 16;
        std::array<const dtMeshTile*, MAX_LAYERS> tiles{};
        int count = navMesh->getTilesAt(tileX, tileY, tiles.data(), MAX_LAYERS);
        int maxLayer = -1;
        for (int i = 0; i < count; ++i)
        {
            if (tiles[i] && tiles[i]->header)
                maxLayer = std::max(maxLayer, tiles[i]->header->layer);
        }
        return maxLayer + 1;
    }

    bool OffMeshManager::InjectConnections(unsigned int mapId, int tileX, int tileY,
                                           const std::vector<OffMeshConnectionInfo>& connections)
    {
        if (connections.empty())
            return true;

        MMapManager* manager = MMapFactory::createOrGetMMapManager();
        if (!manager)
        {
            std::cerr << "[OffMeshManager] MMapManager unavailable." << std::endl;
            return false;
        }

        dtNavMesh* navMesh = const_cast<dtNavMesh*>(manager->GetNavMesh(mapId));
        if (!navMesh)
        {
            std::cerr << "[OffMeshManager] NavMesh not loaded for map " << mapId << std::endl;
            return false;
        }

        const MapNavParams navParams = ResolveNavParams(mapId);

        std::vector<float> verts;
        std::vector<float> radii;
        std::vector<unsigned short> flags;
        std::vector<unsigned char> areas;
        std::vector<unsigned char> dirs;
        std::vector<unsigned int> userIds;
        verts.reserve(connections.size() * 6);
        radii.reserve(connections.size());
        flags.reserve(connections.size());
        areas.reserve(connections.size());
        dirs.reserve(connections.size());
        userIds.reserve(connections.size());

        float bmin[3] = { std::numeric_limits<float>::max(),
                          std::numeric_limits<float>::max(),
                          std::numeric_limits<float>::max() };
        float bmax[3] = { -std::numeric_limits<float>::max(),
                          -std::numeric_limits<float>::max(),
                          -std::numeric_limits<float>::max() };

        auto expandBounds = [&](const Vector3& pt, float radius)
        {
            const float r = std::max(radius, 1.0f);
            // WoW uses Z-up coordinate system
            // X and Y are horizontal (apply radius)
            // Z is vertical (apply height buffer)
            bmin[0] = std::min(bmin[0], pt.x - r);
            bmin[1] = std::min(bmin[1], pt.y - r);
            bmin[2] = std::min(bmin[2], pt.z - 2.0f);
            bmax[0] = std::max(bmax[0], pt.x + r);
            bmax[1] = std::max(bmax[1], pt.y + r);
            bmax[2] = std::max(bmax[2], pt.z + 2.0f);
        };

        for (size_t i = 0; i < connections.size(); ++i)
        {
            const auto& conn = connections[i];
            verts.push_back(conn.Start.x);
            verts.push_back(conn.Start.y);
            verts.push_back(conn.Start.z);
            verts.push_back(conn.End.x);
            verts.push_back(conn.End.y);
            verts.push_back(conn.End.z);

            const float radius = conn.Radius > 0.01f ? conn.Radius : 2.5f;
            radii.push_back(radius);
            flags.push_back(0x01);
            areas.push_back(ResolveAreaForConnection(conn.Type));
            dirs.push_back(conn.Flags == OFFMESH_BIDIRECTIONAL ? DT_OFFMESH_CON_BIDIR : 0);
            unsigned int userId = conn.UserId;
            if (userId == 0)
            {
                userId = (static_cast<unsigned int>(mapId) << 16) |
                         static_cast<unsigned int>(i & 0xFFFF);
            }
            userIds.push_back(userId);

            expandBounds(conn.Start, radius);
            expandBounds(conn.End, radius);
        }

        // Ensure bounds are sane
        if (bmin[0] > bmax[0])
        {
            bmin[0] = static_cast<float>(tileX) * 533.33333f;
            bmin[1] = -5000.0f;
            bmin[2] = static_cast<float>(tileY) * 533.33333f;
            bmax[0] = bmin[0] + 533.33333f;
            bmax[1] = 5000.0f;
            bmax[2] = bmin[2] + 533.33333f;
        }

        dtNavMeshCreateParams params{};
        params.verts = nullptr;
        params.vertCount = 0;
        params.polys = nullptr;
        params.polyFlags = nullptr;
        params.polyAreas = nullptr;
        params.polyCount = 0;
        params.nvp = DT_VERTS_PER_POLYGON;
        params.detailMeshes = nullptr;
        params.detailVerts = nullptr;
        params.detailVertsCount = 0;
        params.detailTris = nullptr;
        params.detailTriCount = 0;
        params.offMeshConVerts = verts.data();
        params.offMeshConRad = radii.data();
        params.offMeshConFlags = flags.data();
        params.offMeshConAreas = areas.data();
        params.offMeshConDir = dirs.data();
        params.offMeshConUserID = userIds.data();
        params.offMeshConCount = static_cast<int>(connections.size());
        params.userId = BuildPackedKey(mapId, tileX, tileY);
        params.tileX = tileX;
        params.tileY = tileY;
        params.tileLayer = ReserveTileLayer(navMesh, tileX, tileY);
        params.walkableHeight = navParams.walkableHeight;
        params.walkableRadius = navParams.walkableRadius;
        params.walkableClimb = navParams.walkableClimb;
        params.bmin[0] = bmin[0];
        params.bmin[1] = bmin[1];
        params.bmin[2] = bmin[2];
        params.bmax[0] = bmax[0];
        params.bmax[1] = bmax[1];
        params.bmax[2] = bmax[2];
        params.cs = navParams.cellSize;
        params.ch = navParams.cellHeight;
        params.buildBvTree = false;

        unsigned char* outData = nullptr;
        int outDataSize = 0;
        if (!dtCreateNavMeshData(&params, &outData, &outDataSize))
        {
            std::cerr << "[OffMeshManager] Failed to build offmesh tile for map "
                      << mapId << " tile (" << tileX << "," << tileY << ")" << std::endl;
            return false;
        }

        dtTileRef tileRef = 0;
        dtStatus status = navMesh->addTile(outData, outDataSize, DT_TILE_FREE_DATA, 0, &tileRef);
        if (dtStatusFailed(status))
        {
            dtFree(outData);
            std::cerr << "[OffMeshManager] addTile failed for offmesh tile map "
                      << mapId << " tile (" << tileX << "," << tileY << ")" << std::endl;
            return false;
        }

        TileKey key { mapId, tileX, tileY };
        m_injectedTileRefs[key] = tileRef;
        return true;
    }
}
