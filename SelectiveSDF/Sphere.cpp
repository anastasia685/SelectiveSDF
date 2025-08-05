#include "stdafx.h"
#include "Sphere.h"

void Sphere::BuildTriangleGeometry(ID3D12Device* device, vector<Index>& indices, vector<Vertex>& vertices)
{
    const float radius = 0.4f;
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

void Sphere::BuildAABBs(ID3D12Device* device, vector<D3D12_RAYTRACING_AABB>& aabbs)
{
	// only one hardcoded AABB for now
	m_aabbCount = 1;

	aabbs.push_back(
		{ -0.6f, -0.6f, -0.6f, 0.6f,  0.6f,  0.6f }
	);
}
