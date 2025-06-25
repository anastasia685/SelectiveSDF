#pragma once
#include "Object.h"

using namespace std;

class HybridObject : public Object
{
	public:
		HybridObject(UINT hitGroupIndex = 1) : Object(ObjectType::Hybrid), m_hitGroupIndex(hitGroupIndex) {};

		UINT GetHitGroupIndex() const { return m_hitGroupIndex; }
		UINT GetAAbbCount() const { return m_aabbCount; }
		BufferHelper::D3DBuffer& GetAABBBuffer() { return m_aabbBuffer; };

		virtual void BuildGeometry(ID3D12Device* device, ID3D12GraphicsCommandList* commandList) override
		{
			vector<Index> indices;
			vector<Vertex> vertices;
			vector<D3D12_RAYTRACING_AABB> aabbs;

			BuildTriangleGeometry(device, indices, vertices);
			BuildAABBs(device, aabbs);

			Object::BuildGeometry(device, commandList, indices.data(), vertices.data()); // let parent class handle index/vertex buffer creation
			
			AllocateBuffer(
				device,
				D3D12_HEAP_TYPE_UPLOAD,
				m_aabbCount * sizeof(D3D12_RAYTRACING_AABB),
				&m_aabbBuffer.resource,
				D3D12_RESOURCE_STATE_GENERIC_READ,
				D3D12_RESOURCE_FLAG_NONE,
				aabbs.data()
			);
		}

protected:
	UINT m_hitGroupIndex;
	BufferHelper::D3DBuffer m_aabbBuffer;

	UINT m_aabbCount = 0; // number of AABBs in the buffer, used for procedural geometry

	virtual void BuildTriangleGeometry(ID3D12Device* device, vector<Index>& indices, vector<Vertex>& vertices) = 0;
	virtual void BuildAABBs(ID3D12Device* device, vector<D3D12_RAYTRACING_AABB>& aabbs) = 0;
};