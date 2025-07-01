#include "stdafx.h"
#include "Box.h"

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
		{ { -0.5f, -0.5f, -0.5f }, { 0.0f, 0.0f, -1.0f } },  // 0
		{ { -0.5f,  0.5f, -0.5f }, { 0.0f, 0.0f, -1.0f } },  // 1
		{ {  0.5f,  0.5f, -0.5f }, { 0.0f, 0.0f, -1.0f } },  // 2
		{ {  0.5f, -0.5f, -0.5f }, { 0.0f, 0.0f, -1.0f } },  // 3

		// Back face
		{ { -0.5f, -0.5f,  0.5f }, { 0.0f, 0.0f, 1.0f } },   // 4
		{ { -0.5f,  0.5f,  0.5f }, { 0.0f, 0.0f, 1.0f } },   // 5
		{ {  0.5f,  0.5f,  0.5f }, { 0.0f, 0.0f, 1.0f } },   // 6
		{ {  0.5f, -0.5f,  0.5f }, { 0.0f, 0.0f, 1.0f } },   // 7

		// Left face
		{ { -0.5f, -0.5f, -0.5f }, { -1.0f, 0.0f, 0.0f } },  // 8
		{ { -0.5f,  0.5f, -0.5f }, { -1.0f, 0.0f, 0.0f } },  // 9
		{ { -0.5f,  0.5f,  0.5f }, { -1.0f, 0.0f, 0.0f } },  // 10
		{ { -0.5f, -0.5f,  0.5f }, { -1.0f, 0.0f, 0.0f } },  // 11

		// Right face
		{ {  0.5f,  0.5f, -0.5f }, { 1.0f, 0.0f, 0.0f } },   // 12
		{ {  0.5f, -0.5f, -0.5f }, { 1.0f, 0.0f, 0.0f } },   // 13
		{ {  0.5f, -0.5f,  0.5f }, { 1.0f, 0.0f, 0.0f } },   // 14
		{ {  0.5f,  0.5f,  0.5f }, { 1.0f, 0.0f, 0.0f } },   // 15

		// Top face
		{ { -0.5f,  0.5f, -0.5f }, { 0.0f, 1.0f, 0.0f } },   // 16
		{ {  0.5f,  0.5f, -0.5f }, { 0.0f, 1.0f, 0.0f } },   // 17
		{ {  0.5f,  0.5f,  0.5f }, { 0.0f, 1.0f, 0.0f } },   // 18
		{ { -0.5f,  0.5f,  0.5f }, { 0.0f, 1.0f, 0.0f } },   // 19

		// Bottom face
		{ { -0.5f, -0.5f, -0.5f }, { 0.0f, -1.0f, 0.0f } },  // 20
		{ { -0.5f, -0.5f,  0.5f }, { 0.0f, -1.0f, 0.0f } },  // 21
		{ {  0.5f, -0.5f,  0.5f }, { 0.0f, -1.0f, 0.0f } },  // 22
		{ {  0.5f, -0.5f, -0.5f }, { 0.0f, -1.0f, 0.0f } }   // 23
	};

	m_vertexCount = vertices.size();
	m_indexCount = indices.size();
}

void Box::BuildAABBs(ID3D12Device* device, vector<D3D12_RAYTRACING_AABB>& aabbs)
{
	// only one hardcoded AABB for now
	m_aabbCount = 1;

	aabbs.push_back(
		{ -0.6f, -0.6f, -0.6f, 0.6f,  0.6f,  0.6f }
	);
}
