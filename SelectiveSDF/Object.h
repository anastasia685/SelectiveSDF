#pragma once
#include "BufferHelper.hpp"
#include "ResourceManager.h"

using namespace DirectX;

typedef UINT Index;
struct Vertex
{
	XMFLOAT3 position;
	XMFLOAT3 normal;
};

enum ObjectType {
	Triangle = 0,
	Hybrid,
	//SDF only?
};

struct ObjectInstance
{
	UINT objectIndex;

	XMFLOAT3 position;
	XMFLOAT3 rotation;
	//XMFLOAT3 scale;
	float scale; // only allow uniform scaling for now

	XMMATRIX CalculateTransform() const
	{
		return XMMatrixScaling(scale, scale, scale) *
			XMMatrixRotationRollPitchYaw(rotation.x, rotation.y, rotation.z) *
			XMMatrixTranslation(position.x, position.y, position.z);
	}
};


class Object
{
public:
	Object(ObjectType type = ObjectType::Triangle) : m_type(type) {};

	ObjectType GetType() const { return m_type; }
	UINT GetVertexCount() const { return m_vertexCount; }
	UINT GetIndexCount() const { return m_indexCount; }

	BufferHelper::D3DBuffer& GetIndexBuffer() { return m_indexBuffer; };
	BufferHelper::D3DBuffer& GetVertexBuffer() { return m_vertexBuffer; };

	virtual void BuildGeometry(ID3D12Device* device, ID3D12GraphicsCommandList* commandList) = 0; // make child classes implement vertex/index array building
	virtual void ReleaseStagingBuffers()
	{
		m_stagingIndexBuffer.Reset();
		m_stagingVertexBuffer.Reset();
	}


protected:
	UINT m_vertexCount = 0;
	UINT m_indexCount = 0;

	ObjectType m_type;

	BufferHelper::D3DBuffer m_indexBuffer;
	BufferHelper::D3DBuffer m_vertexBuffer;

	ComPtr<ID3D12Resource> m_stagingIndexBuffer, m_stagingVertexBuffer;

	void BuildGeometry(ID3D12Device* device, ID3D12GraphicsCommandList* commandList, Index* indices, Vertex* vertices)
	{
		if(m_indexCount == 0 || m_vertexCount == 0) 
		{
			throw std::runtime_error("Object must have vertex and index count set before building geometry.");
		}

		UINT indexBufferSize = m_indexCount * sizeof(Index);
		UINT vertexBufferSize = m_vertexCount * sizeof(Vertex);


		D3D12_HEAP_TYPE heapType = D3D12_HEAP_TYPE_UPLOAD;
		D3D12_RESOURCE_STATES initialResourceState = D3D12_RESOURCE_STATE_GENERIC_READ;
		D3D12_RESOURCE_FLAGS resourceFlags = D3D12_RESOURCE_FLAG_NONE;
		
		AllocateBuffer(device, heapType, indexBufferSize, &m_stagingIndexBuffer, initialResourceState, resourceFlags, indices);
		AllocateBuffer(device, heapType, m_vertexCount * sizeof(Vertex), &m_stagingVertexBuffer, initialResourceState, resourceFlags, vertices);


		heapType = D3D12_HEAP_TYPE_DEFAULT;
		initialResourceState = D3D12_RESOURCE_STATE_COPY_DEST;
		AllocateBuffer(device, heapType, indexBufferSize, &m_indexBuffer.resource, initialResourceState, resourceFlags, nullptr, L"Index Buffer");
		AllocateBuffer(device, heapType, m_vertexCount * sizeof(Vertex), &m_vertexBuffer.resource, initialResourceState, resourceFlags, nullptr, L"Vertex Buffer");


		commandList->CopyBufferRegion(m_indexBuffer.resource.Get(), 0, m_stagingIndexBuffer.Get(), 0, indexBufferSize);
		commandList->CopyBufferRegion(m_vertexBuffer.resource.Get(), 0, m_stagingVertexBuffer.Get(), 0, vertexBufferSize);

		// Transition default buffers for shader use (StructuredBuffer and ByteAddressBuffer)
		auto barrier1 = CD3DX12_RESOURCE_BARRIER::Transition(
			m_indexBuffer.resource.Get(),
			D3D12_RESOURCE_STATE_COPY_DEST,
			D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE // or GENERIC_READ
		);

		auto barrier2 = CD3DX12_RESOURCE_BARRIER::Transition(
			m_vertexBuffer.resource.Get(),
			D3D12_RESOURCE_STATE_COPY_DEST,
			D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE // or GENERIC_READ
		);

		commandList->ResourceBarrier(1, &barrier1);
		commandList->ResourceBarrier(1, &barrier2);
	}
};
