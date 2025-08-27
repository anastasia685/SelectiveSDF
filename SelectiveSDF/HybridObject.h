#pragma once
#include "Object.h"
#include "RaytracingHlslCompat.h"

using namespace std;

struct SurfaceVoxel {
	UINT x, y, z;
	float distance;
	//UINT instanceIndex;
	//float s000, s100, s010, s110, s001, s101, s011, s111; // 8 corners
};
struct SurfaceBrick // sbs
{
	UINT bx, by, bz;  // Brick coordinates (each brick covers 7x7x7 voxels)
	// We need 8x8x8 values for the corners
	float values[8][8][8];  // Or store as flat array for texture
};
struct BrickMeta
{
	UINT packedCoord;
	UINT instanceIndex;
	UINT sliceStart;
	UINT padding;
};
struct BrickInfo // vdb
{
	XMUINT3 brickCoord;      // Brick coordinates within its object
	float padding;
};
//struct VoxelPacked {
//	UINT packedData;
//};
struct VoxelAABB {
	XMFLOAT3 min;
	XMFLOAT3 max;
	XMFLOAT2 padding;
};
typedef UINT VoxelPacked;

class HybridObject : public Object
{
	public:
		HybridObject(SDFPrimitive::Enum sdfPrimitiveType = SDFPrimitive::Enum::Box, UINT hitGroupIndex = 1, UINT sdfResolution = 64) :
			Object(ObjectType::Hybrid), m_sdfPrimitiveType(sdfPrimitiveType), m_hitGroupIndex(hitGroupIndex), m_sdfResolution(sdfResolution) {};

		SDFPrimitive::Enum GetSDFPrimitiveType() const { return m_sdfPrimitiveType; }
		UINT GetHitGroupIndex() const { return m_hitGroupIndex; }
		UINT GetAAbbCount() const { return m_aabbCount; }
		UINT GetSDFResolution() const { return m_sdfResolution; };
		BufferHelper::D3DBuffer& GetAABBBuffer() { return m_aabbBuffer; };
		UINT GetCandidateVoxelCount() const { return static_cast<UINT>(m_surfaceVoxels.size()); }
		vector<SurfaceVoxel>& GetCandidateVoxels() { return m_surfaceVoxels; }

		/*UINT GetBrickCount() const { return static_cast<UINT>(m_surfaceBricks.size()); }
		vector<SurfaceBrick>& GetBricks() { return m_surfaceBricks; }*/

		UINT GetBrickCount() const { return static_cast<UINT>(m_brickMeta.size()); }
		vector<BrickMeta>& GetBricks() { return m_brickMeta; }

		virtual void ExtractNarrowBand() = 0;
		virtual void ExtractBricks() = 0;

		virtual void BuildGeometry(ID3D12Device* device, ID3D12GraphicsCommandList* commandList) override
		{
			vector<Index> indices;
			vector<Vertex> vertices;
			vector<D3D12_RAYTRACING_AABB> aabbs;

			BuildTriangleGeometry(device, indices, vertices);
			//ExtractNarrowBand();
			ExtractBricks();
			BuildAABBs(device, aabbs);

			Object::BuildGeometry(device, commandList, indices.data(), vertices.data()); // let parent class handle index/vertex buffer creation


			UINT aabbBufferSize = m_aabbCount * sizeof(D3D12_RAYTRACING_AABB);

			D3D12_HEAP_TYPE heapType = D3D12_HEAP_TYPE_UPLOAD;
			D3D12_RESOURCE_STATES initialResourceState = D3D12_RESOURCE_STATE_GENERIC_READ;
			D3D12_RESOURCE_FLAGS resourceFlags = D3D12_RESOURCE_FLAG_NONE;

			// staging buffer
			AllocateBuffer(
				device, 
				heapType, 
				aabbBufferSize,
				&m_stagingAabbBuffer,
				initialResourceState, 
				resourceFlags, 
				aabbs.data()
			);

			heapType = D3D12_HEAP_TYPE_DEFAULT;
			initialResourceState = D3D12_RESOURCE_STATE_COPY_DEST;
			AllocateBuffer(
				device, 
				heapType, 
				aabbBufferSize,
				&m_aabbBuffer.resource,
				initialResourceState, 
				resourceFlags, 
				nullptr
			);

			commandList->CopyBufferRegion(m_aabbBuffer.resource.Get(), 0, m_stagingAabbBuffer.Get(), 0, aabbBufferSize);


			auto barrier = CD3DX12_RESOURCE_BARRIER::Transition(
				m_aabbBuffer.resource.Get(),
				D3D12_RESOURCE_STATE_COPY_DEST,
				D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE
			);

			commandList->ResourceBarrier(1, &barrier);
		}

		virtual void ReleaseStagingBuffers() override
		{
			Object::ReleaseStagingBuffers();
			m_stagingAabbBuffer.Reset();
		}

protected:
	UINT m_hitGroupIndex;
	BufferHelper::D3DBuffer m_aabbBuffer;
	ComPtr<ID3D12Resource> m_stagingAabbBuffer;

	SDFPrimitive::Enum m_sdfPrimitiveType;
	UINT m_sdfResolution;

	UINT m_aabbCount = 0;

	vector<SurfaceVoxel> m_surfaceVoxels;
	vector<SurfaceBrick> m_surfaceBricks;

	vector<BrickMeta> m_brickMeta;

	virtual void BuildTriangleGeometry(ID3D12Device* device, vector<Index>& indices, vector<Vertex>& vertices) = 0;
	virtual void BuildAABBs(ID3D12Device* device, vector<D3D12_RAYTRACING_AABB>& aabbs) = 0;
};