#pragma once
#include <unordered_map>

using namespace DirectX;
using namespace std;

class SpatialHashGrid {
    struct Cell {
        int x, y, z;
        bool operator==(const Cell& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };
    // Hash function for Cell
    struct CellHash {
        size_t operator()(const Cell& cell) const {
            // Simple spatial hash
            return cell.x * 73856093 ^ cell.y * 19349663 ^ cell.z * 83492791;
        }
    };

    struct Entry
    {
        XMINT3 cellPos;
        UINT indexOffset;
        UINT count;
        UINT occupied = 0;
    };

public:
    const float CellSize = 1.0f;

    // Helper to convert world pos to cell coordinate
    Cell WorldToCell(XMFLOAT3 worldPos) {
        return Cell{
            (int)floor(worldPos.x / CellSize),
            (int)floor(worldPos.y / CellSize),
            (int)floor(worldPos.z / CellSize)
        };
    }
    vector<Entry>& GetHashTable() { return m_hashTable; };
    vector<int>& GetInstanceIndices() { return m_instanceIndices; };

    UINT GetHashTableSize() const { return m_hashTable.size(); };
    UINT GetInstanceIndicesSize() const { return m_instanceIndices.size(); };

    void ClearCells() { m_cells.clear(); };
    void AddCellData(int x, int y, int z, int instanceIndex) { m_cells[{x, y, z}].push_back(instanceIndex); };
    void Build()
    {
        // this is called after cell data is added to m_cells
        m_hashTable.clear();
        m_instanceIndices.clear();

        UINT numOccupiedCells = m_cells.size();
        UINT tableSize = 1;
        while (tableSize < numOccupiedCells * 2) tableSize *= 2;  // Next power of 2

        m_hashTable.resize(tableSize);
        
        // Insert each cell into hash table with linear probing
        for (const auto& [cell, instances] : m_cells)
        {
            UINT hash = CellHash{}(cell) % tableSize;

            // Linear probe to find empty slot
            while (m_hashTable[hash].occupied == 1) {
                hash = (hash + 1) % tableSize;
            }

            // Fill the entry
            m_hashTable[hash].cellPos = {cell.x, cell.y, cell.z};
            m_hashTable[hash].indexOffset = m_instanceIndices.size();
            m_hashTable[hash].count = instances.size();
            m_hashTable[hash].occupied = 1;

            // Append instances to flat list
            m_instanceIndices.insert(m_instanceIndices.end(),
                instances.begin(), instances.end());
        }
    }

private:
    unordered_map<Cell, vector<int>, CellHash> m_cells;

    vector<Entry> m_hashTable;
    vector<int> m_instanceIndices;
};