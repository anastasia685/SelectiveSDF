#include "stdafx.h"
#include "Sphere.h"
#include <algorithm>

inline uint32_t PackBrickCoord(uint32_t x, uint32_t y, uint32_t z)
{
	return (x & 0xFF) | ((y & 0xFF) << 8) | ((z & 0xFF) << 16);
}
void Sphere::ExtractBricks()
{
    const int GRID_SIZE = m_sdfResolution; // 64 for a 64x64x64 grid
    const int BRICK_VOXEL_SIZE = 9;
    const int BRICK_COUNT = (GRID_SIZE - 1 + BRICK_VOXEL_SIZE - 1) / BRICK_VOXEL_SIZE;
    const float PADDING_THRESHOLD = 0.1f;
    const float FRACTION_THRESHOLD = 0.4f;

    // Sphere parameters
    const float sphereRadius = 0.38f;
    const XMFLOAT3 sphereCenter = XMFLOAT3(0.5f, 0.5f, 0.5f); // Center in normalized [0,1] space

    // Lambda to calculate sphere SDF at any grid position
    auto sampleSDF = [&](int x, int y, int z) -> float {
        x = clamp(x, 0, GRID_SIZE - 1);
        y = clamp(y, 0, GRID_SIZE - 1);
        z = clamp(z, 0, GRID_SIZE - 1);

        // Convert grid coordinates to normalized space [0, 1]
        float px = static_cast<float>(x) / static_cast<float>(GRID_SIZE - 1);
        float py = static_cast<float>(y) / static_cast<float>(GRID_SIZE - 1);
        float pz = static_cast<float>(z) / static_cast<float>(GRID_SIZE - 1);

        XMVECTOR pos = XMVectorSet(px, py, pz, 0.0f);
        XMVECTOR center = XMLoadFloat3(&sphereCenter);
        XMVECTOR toCenter = XMVectorSubtract(pos, center);

        float distance = XMVectorGetX(XMVector3Length(toCenter));
        return distance - sphereRadius;
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

void Sphere::BuildTriangleGeometry(ID3D12Device* device, vector<Index>& indices, vector<Vertex>& vertices)
{
    const float radius = 0.38f;
    const int segments = 32; // Longitude divisions
    const int rings = 16;    // Latitude divisions

    vertices.clear();
    indices.clear();

    // Generate vertices
    for (int ring = 0; ring <= rings; ++ring)
    {
        float phi = XM_PI * ring / rings; // 0 to PI
        float y = radius * cos(phi);
        float ringRadius = radius * sin(phi);

        for (int segment = 0; segment <= segments; ++segment)
        {
            float theta = 2.0f * XM_PI * segment / segments; // 0 to 2PI
            float x = ringRadius * cos(theta);
            float z = ringRadius * sin(theta);

            XMFLOAT3 position(x, y, z);

            // For a sphere, the normal is just the normalized position
            XMFLOAT3 normal;
            XMStoreFloat3(&normal, XMVector3Normalize(XMLoadFloat3(&position)));

            vertices.push_back({ position, normal });
        }
    }

    // Generate indices
    for (int ring = 0; ring < rings; ++ring)
    {
        for (int segment = 0; segment < segments; ++segment)
        {
            int current = ring * (segments + 1) + segment;
            int next = current + segments + 1;

            // First triangle
            indices.push_back(current);
            indices.push_back(next);
            indices.push_back(current + 1);

            // Second triangle
            indices.push_back(current + 1);
            indices.push_back(next);
            indices.push_back(next + 1);
        }
    }

    m_vertexCount = vertices.size();
    m_indexCount = indices.size();
}

inline void UnpackBrickCoord(uint32_t packed, uint32_t& x, uint32_t& y, uint32_t& z)
{
	x = packed & 0xFF;
	y = (packed >> 8) & 0xFF;
	z = (packed >> 16) & 0xFF;
}
void Sphere::BuildAABBs(ID3D12Device* device, vector<D3D12_RAYTRACING_AABB>& aabbs)
{
	// only one hardcoded AABB for now
	/*m_aabbCount = 1;

	aabbs.push_back(
		{ -0.5f, -0.5f, -0.5f, 0.5f,  0.5f,  0.5f }
	);
    return;*/

	const int GRID_SIZE = m_sdfResolution;
	const float WORLD_SIZE = 1.0f;  // -0.5 to +0.5
	const float VOXEL_SIZE = WORLD_SIZE / GRID_SIZE;
	const float HALF_WORLD = WORLD_SIZE * 0.5f;

	const int BRICK_SIZE = 9;
	m_aabbCount = static_cast<UINT>(m_brickMeta.size());
	for (const auto& brick : m_brickMeta) // m_surfaceBricks 
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
