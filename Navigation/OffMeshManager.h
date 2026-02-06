/**
 * WotLK 3.3.5a - OffMesh Connection Manager
 * Based on HB 4.3.4 Tripper.RecastManaged.Detour.OffMeshConnection
 * 
 * Manages off-mesh connections for special navigation (elevators, portals, etc.)
 * Uses existing Detour dtOffMeshConnection structure but adds custom metadata
 */

#ifndef OFFMESH_MANAGER_H
#define OFFMESH_MANAGER_H

#include "DetourNavMesh.h"
#include "G3D/Vector3.h"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <functional>

// Use G3D Vector3 directly (already defined in PathFinder.h)
typedef G3D::Vector3 Vector3;

namespace MMAP
{
    struct MapNavParams
    {
        float walkableHeight = 4.0f;
        float walkableRadius = 1.5f;
        float walkableClimb = 4.0f;
        float cellSize = 0.3f;
        float cellHeight = 0.2f;
    };

    struct TileKey
    {
        unsigned int mapId;
        int tileX;
        int tileY;

        bool operator==(const TileKey& other) const
        {
            return mapId == other.mapId && tileX == other.tileX && tileY == other.tileY;
        }
    };

    struct TileKeyHasher
    {
        size_t operator()(const TileKey& key) const noexcept
        {
            const uint64_t packed = (static_cast<uint64_t>(key.mapId) << 32) |
                ((static_cast<uint64_t>(key.tileX) & 0xFFFF) << 16) |
                (static_cast<uint64_t>(key.tileY) & 0xFFFF);
            return std::hash<uint64_t>{}(packed);
        }
    };

    // HB 4.3.4 DirectionFlags (already in dtOffMeshConnection::flags)
    enum OffMeshDirectionFlags : unsigned char
    {
        OFFMESH_ONEWAY = 0,          // DT_OFFMESH_CON_FORWARD only
        OFFMESH_BIDIRECTIONAL = 1    // DT_OFFMESH_CON_BIDIR
    };

    // Custom extension: OffMesh connection types (not in original HB)
    enum OffMeshConnectionType : unsigned char
    {
        OFFMESH_TYPE_NORMAL = 0,        // Simple connection (jump, etc)
        OFFMESH_TYPE_ELEVATOR = 1,      // Elevator/lift (Undercity, Thunder Bluff)
        OFFMESH_TYPE_PORTAL = 2,        // Portal/teleport
        OFFMESH_TYPE_INTERACT_UNIT = 3, // NPC interaction (flight master, boat)
        OFFMESH_TYPE_INTERACT_OBJECT = 4 // GameObject interaction (door, lever)
    };

    /**
     * Extended OffMesh Connection Info
     * Wraps dtOffMeshConnection with custom metadata
     */
    struct OffMeshConnectionInfo
    {
        // Native Detour data (from dtOffMeshConnection)
        Vector3 Start;                      // pos[0-2]
        Vector3 End;                        // pos[3-5]
        float Radius;                       // rad
        unsigned short PolyIndex;           // poly
        unsigned char Flags;                // flags (DirectionFlags)
        unsigned char Side;                 // side
        unsigned int UserId;                // userId

        // Custom extensions (stored separately, not in dtOffMeshConnection)
        OffMeshConnectionType Type;         // Connection type
                unsigned short WaitTimeMs;          // Optional wait time for interactives
        unsigned int InteractId;            // NPC/GameObject ID (for INTERACT types)
        unsigned int MapId;                 // Source map
        unsigned int TargetMapId;           // Target map (for portals)

        OffMeshConnectionInfo()
                        : Radius(0), PolyIndex(0), Flags(0), Side(0), UserId(0),
                            Type(OFFMESH_TYPE_NORMAL), WaitTimeMs(0), InteractId(0),
                            MapId(0), TargetMapId(0)
        {}

        // Helper: Check if position is near start
        bool IsNearStart(const Vector3& pos, float tolerance) const
        {
            float dx = Start.x - pos.x;
            float dy = Start.y - pos.y;
            float dz = Start.z - pos.z;
            float distSqr = dx*dx + dy*dy + dz*dz;
            float searchRadiusSqr = (tolerance + Radius) * (tolerance + Radius);
            return distSqr <= searchRadiusSqr;
        }

        // Helper: Check if position is near end (for bidirectional)
        bool IsNearEnd(const Vector3& pos, float tolerance) const
        {
            if (Flags != OFFMESH_BIDIRECTIONAL)
                return false;

            float dx = End.x - pos.x;
            float dy = End.y - pos.y;
            float dz = End.z - pos.z;
            float distSqr = dx*dx + dy*dy + dz*dz;
            float searchRadiusSqr = (tolerance + Radius) * (tolerance + Radius);
            return distSqr <= searchRadiusSqr;
        }
    };

    /**
     * OffMesh Connection Set for a single map
     */
    class OffMeshConnectionSet
    {
    public:
        void AddConnection(const OffMeshConnectionInfo& conn);
        bool HasConnectionAt(const Vector3& pos, float tolerance) const;
        const OffMeshConnectionInfo* GetConnectionAt(const Vector3& pos, float tolerance) const;
        size_t GetCount() const { return m_connections.size(); }

    private:
        std::vector<OffMeshConnectionInfo> m_connections;
    };

    /**
     * Singleton manager for all OffMesh connections
     * Similar to HB 4.3.4 but uses file-based storage instead of embedded in navmesh
     */
    class OffMeshManager
    {
    public:
        static OffMeshManager& Instance();

        // Cache navmesh tile parameters for later injection
        void RegisterTileNavParams(unsigned int mapId, const dtMeshHeader* header);

        // Load offmesh connections from file for a specific tile
        // Format: mmaps/{mapId:000}_{tileX:00}_{tileY:00}.offmesh (binary)
        bool LoadTileOffMeshConnections(unsigned int mapId, int tileX, int tileY);
        
        // Legacy: Load all offmesh for entire map (loads all tiles)
        bool LoadMapOffMeshConnections(unsigned int mapId);

        // Query offmesh at position
        const OffMeshConnectionInfo* GetOffMeshAt(unsigned int mapId, const Vector3& pos, float tolerance = 3.0f) const;

        // Add custom offmesh at runtime
        void AddOffMeshConnection(unsigned int mapId, const OffMeshConnectionInfo& conn);

        // Check if position is an offmesh connection
        bool IsOffMeshWaypoint(unsigned int mapId, const Vector3& pos, float tolerance, 
                                OffMeshConnectionInfo* outConn = nullptr) const;

        // Get all connections for a map
        const OffMeshConnectionSet* GetMapConnections(unsigned int mapId) const;

        // Clear all offmesh data
        void Clear();

    private:
        OffMeshManager() = default;
        ~OffMeshManager() = default;
        OffMeshManager(const OffMeshManager&) = delete;
        OffMeshManager& operator=(const OffMeshManager&) = delete;

        std::unordered_map<unsigned int, OffMeshConnectionSet> m_mapOffMeshData;
        std::unordered_map<unsigned int, MapNavParams> m_navParams;
        std::unordered_map<TileKey, dtTileRef, TileKeyHasher> m_injectedTileRefs;
        std::unordered_set<TileKey, TileKeyHasher> m_loadedTileKeys;

        uint64_t BuildPackedKey(unsigned int mapId, int tileX, int tileY) const;
        bool InjectConnections(unsigned int mapId, int tileX, int tileY,
                               const std::vector<OffMeshConnectionInfo>& connections);
        int ReserveTileLayer(dtNavMesh* navMesh, int tileX, int tileY) const;
        MapNavParams ResolveNavParams(unsigned int mapId) const;
    };
}

#endif // OFFMESH_MANAGER_H
