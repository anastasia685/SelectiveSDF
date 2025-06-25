#include "stdafx.h"
#include "Plane.h"

void Plane::BuildGeometry(ID3D12Device* device, ID3D12GraphicsCommandList* commandList)
{
    // Plane indices.
    Index indices[] =
    {
        3,1,0,
        2,1,3,
    };

    // Cube vertices positions and corresponding triangle normals.
    Vertex vertices[] =
    {
        { XMFLOAT3(-0.5f, 0.0f, -0.5f), XMFLOAT3(0.0f, 1.0f, 0.0f) },
        { XMFLOAT3( 0.5f, 0.0f, -0.5f), XMFLOAT3(0.0f, 1.0f, 0.0f) },
        { XMFLOAT3( 0.5f, 0.0f,  0.5f), XMFLOAT3(0.0f, 1.0f, 0.0f) },
        { XMFLOAT3(-0.5f, 0.0f,  0.5f), XMFLOAT3(0.0f, 1.0f, 0.0f) },
    };

    m_vertexCount = ARRAYSIZE(vertices);
    m_indexCount = ARRAYSIZE(indices);

    Object::BuildGeometry(device, commandList, indices, vertices);
}