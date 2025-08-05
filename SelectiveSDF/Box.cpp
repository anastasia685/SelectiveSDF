#include "stdafx.h"
#include "Box.h"

inline XMFLOAT3 getVoxelCorner(const XMFLOAT3& min, const XMFLOAT3& max, int i);
inline float evaluateSDF(const XMFLOAT3& p);

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
		{ { -0.4f, -0.4f, -0.4f }, { 0.0f, 0.0f, -1.0f } },  // 0
		{ { -0.4f,  0.4f, -0.4f }, { 0.0f, 0.0f, -1.0f } },  // 1
		{ {  0.4f,  0.4f, -0.4f }, { 0.0f, 0.0f, -1.0f } },  // 2
		{ {  0.4f, -0.4f, -0.4f }, { 0.0f, 0.0f, -1.0f } },  // 3

		// Back face
		{ { -0.4f, -0.4f,  0.4f }, { 0.0f, 0.0f, 1.0f } },   // 4
		{ { -0.4f,  0.4f,  0.4f }, { 0.0f, 0.0f, 1.0f } },   // 5
		{ {  0.4f,  0.4f,  0.4f }, { 0.0f, 0.0f, 1.0f } },   // 6
		{ {  0.4f, -0.4f,  0.4f }, { 0.0f, 0.0f, 1.0f } },   // 7

		// Left face
		{ { -0.4f, -0.4f, -0.4f }, { -1.0f, 0.0f, 0.0f } },  // 8
		{ { -0.4f,  0.4f, -0.4f }, { -1.0f, 0.0f, 0.0f } },  // 9
		{ { -0.4f,  0.4f,  0.4f }, { -1.0f, 0.0f, 0.0f } },  // 10
		{ { -0.4f, -0.4f,  0.4f }, { -1.0f, 0.0f, 0.0f } },  // 11

		// Right face
		{ {  0.4f,  0.4f, -0.4f }, { 1.0f, 0.0f, 0.0f } },   // 12
		{ {  0.4f, -0.4f, -0.4f }, { 1.0f, 0.0f, 0.0f } },   // 13
		{ {  0.4f, -0.4f,  0.4f }, { 1.0f, 0.0f, 0.0f } },   // 14
		{ {  0.4f,  0.4f,  0.4f }, { 1.0f, 0.0f, 0.0f } },   // 15

		// Top face
		{ { -0.4f,  0.4f, -0.4f }, { 0.0f, 1.0f, 0.0f } },   // 16
		{ {  0.4f,  0.4f, -0.4f }, { 0.0f, 1.0f, 0.0f } },   // 17
		{ {  0.4f,  0.4f,  0.4f }, { 0.0f, 1.0f, 0.0f } },   // 18
		{ { -0.4f,  0.4f,  0.4f }, { 0.0f, 1.0f, 0.0f } },   // 19

		// Bottom face
		{ { -0.4f, -0.4f, -0.4f }, { 0.0f, -1.0f, 0.0f } },  // 20
		{ { -0.4f, -0.4f,  0.4f }, { 0.0f, -1.0f, 0.0f } },  // 21
		{ {  0.4f, -0.4f,  0.4f }, { 0.0f, -1.0f, 0.0f } },  // 22
		{ {  0.4f, -0.4f, -0.4f }, { 0.0f, -1.0f, 0.0f } }   // 23
	};

	m_vertexCount = vertices.size();
	m_indexCount = indices.size();
}

void Box::BuildAABBs(ID3D12Device* device, vector<D3D12_RAYTRACING_AABB>& aabbs)
{
	//UINT gridRes = 64;

	//XMFLOAT3 min = { -0.6f, -0.6f, -0.6f };
	//XMFLOAT3 max = { 0.6f,  0.6f,  0.6f };

	//XMFLOAT3 size = {
	//max.x - min.x,
	//max.y - min.y,
	//max.z - min.z
	//};
	//XMFLOAT3 cellSize = {
	//	size.x / gridRes,
	//	size.y / gridRes,
	//	size.z / gridRes
	//};

	//const float blendRange = 0.1f;

	//for (UINT z = 0; z < gridRes; ++z) {
	//	for (UINT y = 0; y < gridRes; ++y) {
	//		for (UINT x = 0; x < gridRes; ++x) {

	//			XMFLOAT3 cellMin = {
	//				min.x + x * cellSize.x,
	//				min.y + y * cellSize.y,
	//				min.z + z * cellSize.z
	//			};

	//			XMFLOAT3 cellMax = {
	//				cellMin.x + cellSize.x,
	//				cellMin.y + cellSize.y,
	//				cellMin.z + cellSize.z
	//			};

	//			// Check corners
	//			bool keep = false;
	//			float sdfCorners[8];
	//			for (int i = 0; i < 8; ++i) {
	//				XMFLOAT3 corner = getVoxelCorner(cellMin, cellMax, i);
	//				float d = evaluateSDF(corner); // your function
	//				sdfCorners[i] = d;
	//				if (fabsf(d) < blendRange) {
	//					keep = true;
	//					break;
	//				}
	//			}

	//			if (keep) {
	//				// Store AABB for BLAS
	//				aabbs.push_back({ cellMin.x, cellMin.y, cellMin.z,
	//								  cellMax.x, cellMax.y, cellMax.z });

	//				// Optional: store sdfCorners[8] for this voxel
	//				// sdfValues.push_back(sdfCorners); etc
	//			}
	//		}
	//	}
	//}
	//m_aabbCount = static_cast<UINT>(aabbs.size());
	
	aabbs.push_back(
		{ -0.6f, -0.6f, -0.6f, 0.6f,  0.6f,  0.6f }
	);
	m_aabbCount = 1;
}

inline XMFLOAT3 getVoxelCorner(const XMFLOAT3& min, const XMFLOAT3& max, int i)
{
	return {
		(i & 1) ? max.x : min.x,
		(i & 2) ? max.y : min.y,
		(i & 4) ? max.z : min.z
	};
}
inline float evaluateSDF(const XMFLOAT3& p)
{
	XMFLOAT3 d = { fabsf(p.x) - 0.5f, fabsf(p.y) - 0.5f, fabsf(p.z) - 0.5f };
	float dx = max(d.x, 0.0f), dy = max(d.y, 0.0f), dz = max(d.z, 0.0f);
	float outside = sqrtf(dx * dx + dy * dy + dz * dz);
	float inside = min(max(d.x, max(d.y, d.z)), 0.0f);
	return outside + inside;
}
