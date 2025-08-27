#include "stdafx.h"
#include "Box.h"
#include <algorithm>

inline uint32_t PackBrickCoord(uint32_t x, uint32_t y, uint32_t z)
{
    return (x & 0xFF) | ((y & 0xFF) << 8) | ((z & 0xFF) << 16);
}
void Box::ExtractBricks()
{
    const int GRID_SIZE = m_sdfResolution; // 64 for a 64x64x64 grid
    const int BRICK_VOXEL_SIZE = 9;
    const int BRICK_COUNT = (GRID_SIZE - 1 + BRICK_VOXEL_SIZE - 1) / BRICK_VOXEL_SIZE;
    const float PADDING_THRESHOLD = 0.1f;
    const float FRACTION_THRESHOLD = 0.5f;

    // Cube parameters
    const float cubeHalfSize = 0.32f;  // Half the width/height/depth of the cube
    const float cubeCenterX = 0.5f;
    const float cubeCenterY = 0.5f;
    const float cubeCenterZ = 0.5f;

    // Lambda to calculate cube/box SDF at any grid position
    auto sampleSDF = [&](int x, int y, int z) -> float {
        x = clamp(x, 0, GRID_SIZE - 1);
        y = clamp(y, 0, GRID_SIZE - 1);
        z = clamp(z, 0, GRID_SIZE - 1);

        // Convert grid coordinates to normalized space [0, 1]
        float px = static_cast<float>(x) / static_cast<float>(GRID_SIZE - 1);
        float py = static_cast<float>(y) / static_cast<float>(GRID_SIZE - 1);
        float pz = static_cast<float>(z) / static_cast<float>(GRID_SIZE - 1);

        // Calculate distance to cube center (in absolute terms)
        float dx = fabsf(px - cubeCenterX) - cubeHalfSize;
        float dy = fabsf(py - cubeCenterY) - cubeHalfSize;
        float dz = fabsf(pz - cubeCenterZ) - cubeHalfSize;

        // Outside distance (if the point is outside the box)
        float outsideX = fmaxf(dx, 0.0f);
        float outsideY = fmaxf(dy, 0.0f);
        float outsideZ = fmaxf(dz, 0.0f);
        float outsideDist = sqrtf(outsideX * outsideX + outsideY * outsideY + outsideZ * outsideZ);

        // Inside distance (if the point is inside the box, this is negative)
        float insideDist = fminf(fmaxf(dx, fmaxf(dy, dz)), 0.0f);

        return outsideDist + insideDist;
        };

    // Rest of your code remains exactly the same
    for (UINT bz = 0; bz < BRICK_COUNT; bz++) {
        for (UINT by = 0; by < BRICK_COUNT; by++) {
            for (UINT bx = 0; bx < BRICK_COUNT; bx++) {
                bool hasSurface = false;
                int nearSurfaceCount = 0;

                int voxelStartX = bx * BRICK_VOXEL_SIZE;
                int voxelStartY = by * BRICK_VOXEL_SIZE;
                int voxelStartZ = bz * BRICK_VOXEL_SIZE;

                for (int z = 0; z < BRICK_VOXEL_SIZE; z++) {
                    for (int y = 0; y < BRICK_VOXEL_SIZE; y++) {
                        for (int x = 0; x < BRICK_VOXEL_SIZE; x++) {
                            int gx = voxelStartX + x;
                            int gy = voxelStartY + y;
                            int gz = voxelStartZ + z;

                            if (gx >= GRID_SIZE - 1 || gy >= GRID_SIZE - 1 || gz >= GRID_SIZE - 1)
                                continue;

                            float minVal = FLT_MAX, maxVal = -FLT_MAX;
                            for (int cz = 0; cz <= 1; cz++) {
                                for (int cy = 0; cy <= 1; cy++) {
                                    for (int cx = 0; cx <= 1; cx++) {
                                        float val = sampleSDF(gx + cx, gy + cy, gz + cz);
                                        minVal = min(minVal, val);
                                        maxVal = max(maxVal, val);
                                    }
                                }
                            }

                            if (minVal <= 0.0f && maxVal >= 0.0f) {
                                hasSurface = true;
                            }

                            if (minVal >= 0.0f && minVal < PADDING_THRESHOLD) {
                                nearSurfaceCount++;
                            }
                        }
                    }
                }

                bool keepBrick = hasSurface;
                if (!hasSurface && nearSurfaceCount > 0) {
                    float fraction = static_cast<float>(nearSurfaceCount) /
                        (BRICK_VOXEL_SIZE * BRICK_VOXEL_SIZE * BRICK_VOXEL_SIZE);
                    if (fraction >= FRACTION_THRESHOLD) {
                        keepBrick = true;
                    }
                }

                if (keepBrick) {
                    BrickMeta brickMeta;
                    brickMeta.packedCoord = PackBrickCoord(bx, by, bz);
                    brickMeta.sliceStart = static_cast<UINT>(m_brickMeta.size()) * (BRICK_VOXEL_SIZE + 1);
                    m_brickMeta.push_back(brickMeta);
                }
            }
        }
    }
}

void Box::BuildTriangleGeometry(ID3D12Device* device, vector<Index>& indices, vector<Vertex>& vertices)
{
	// Plane indices.
	indices =
	{
		// Front face
		0, 3, 2,
		0, 2, 1,
		// Back face
		4, 5, 6,
		4, 6, 7,
		// Left face
		8, 9, 10,
		8, 10, 11,
		// Right face
		12, 13, 14,
		12, 14, 15,
		// Top face
		16, 17, 18,
		16, 18, 19,
		// Bottom face
		20, 21, 22,
		20, 22, 23
	};
	vertices = {
		// Front face
		{ { -0.31f, -0.31f, -0.31f }, { 0.0f, 0.0f, -1.0f } },  // 0
		{ { -0.31f,  0.31f, -0.31f }, { 0.0f, 0.0f, -1.0f } },  // 1
		{ {  0.31f,  0.31f, -0.31f }, { 0.0f, 0.0f, -1.0f } },  // 2
		{ {  0.31f, -0.31f, -0.31f }, { 0.0f, 0.0f, -1.0f } },  // 3

		// Back face
		{ { -0.31f, -0.31f,  0.31f }, { 0.0f, 0.0f, 1.0f } },   // 4
		{ { -0.31f,  0.31f,  0.31f }, { 0.0f, 0.0f, 1.0f } },   // 5
		{ {  0.31f,  0.31f,  0.31f }, { 0.0f, 0.0f, 1.0f } },   // 6
		{ {  0.31f, -0.31f,  0.31f }, { 0.0f, 0.0f, 1.0f } },   // 7

		// Left face
		{ { -0.31f, -0.31f, -0.31f }, { -1.0f, 0.0f, 0.0f } },  // 8
		{ { -0.31f,  0.31f, -0.31f }, { -1.0f, 0.0f, 0.0f } },  // 9
		{ { -0.31f,  0.31f,  0.31f }, { -1.0f, 0.0f, 0.0f } },  // 10
		{ { -0.31f, -0.31f,  0.31f }, { -1.0f, 0.0f, 0.0f } },  // 11

		// Right face
		{ {  0.31f,  0.31f, -0.31f }, { 1.0f, 0.0f, 0.0f } },   // 12
		{ {  0.31f, -0.31f, -0.31f }, { 1.0f, 0.0f, 0.0f } },   // 13
		{ {  0.31f, -0.31f,  0.31f }, { 1.0f, 0.0f, 0.0f } },   // 14
		{ {  0.31f,  0.31f,  0.31f }, { 1.0f, 0.0f, 0.0f } },   // 15

		// Top face
		{ { -0.31f,  0.31f, -0.31f }, { 0.0f, 1.0f, 0.0f } },   // 16
		{ {  0.31f,  0.31f, -0.31f }, { 0.0f, 1.0f, 0.0f } },   // 17
		{ {  0.31f,  0.31f,  0.31f }, { 0.0f, 1.0f, 0.0f } },   // 18
		{ { -0.31f,  0.31f,  0.31f }, { 0.0f, 1.0f, 0.0f } },   // 19

		// Bottom face
		{ { -0.31f, -0.31f, -0.31f }, { 0.0f, -1.0f, 0.0f } },  // 20
		{ { -0.31f, -0.31f,  0.31f }, { 0.0f, -1.0f, 0.0f } },  // 21
		{ {  0.31f, -0.31f,  0.31f }, { 0.0f, -1.0f, 0.0f } },  // 22
		{ {  0.31f, -0.31f, -0.31f }, { 0.0f, -1.0f, 0.0f } }   // 23
	};

	m_vertexCount = vertices.size();
	m_indexCount = indices.size();
}

inline void UnpackBrickCoord(uint32_t packed, uint32_t& x, uint32_t& y, uint32_t& z)
{
    x = packed & 0xFF;
    y = (packed >> 8) & 0xFF;
    z = (packed >> 16) & 0xFF;
}
void Box::BuildAABBs(ID3D12Device* device, vector<D3D12_RAYTRACING_AABB>& aabbs)
{
	/*aabbs.push_back(
		{ -0.5f, -0.5f, -0.5f, 0.5f,  0.5f,  0.5f }
	);
	m_aabbCount = 1;
    return;*/

    const int GRID_SIZE = m_sdfResolution;
    const float WORLD_SIZE = 1.0f;  // -0.5 to +0.5
    const float VOXEL_SIZE = WORLD_SIZE / GRID_SIZE;
    const float HALF_WORLD = WORLD_SIZE * 0.5f;

    const int BRICK_SIZE = 9;
    m_aabbCount = static_cast<UINT>(m_brickMeta.size());
    for (const auto& brick : m_brickMeta) 
    {
        UINT brickX, brickY, brickZ;
        UnpackBrickCoord(brick.packedCoord, brickX, brickY, brickZ);

        float worldX = (brickX * BRICK_SIZE * VOXEL_SIZE) - HALF_WORLD;
        float worldY = (brickY * BRICK_SIZE * VOXEL_SIZE) - HALF_WORLD;
        float worldZ = (brickZ * BRICK_SIZE * VOXEL_SIZE) - HALF_WORLD;

        aabbs.push_back({
            worldX, worldY, worldZ,
            worldX + BRICK_SIZE * VOXEL_SIZE,
            worldY + BRICK_SIZE * VOXEL_SIZE,
            worldZ + BRICK_SIZE * VOXEL_SIZE
        });
    }
}
