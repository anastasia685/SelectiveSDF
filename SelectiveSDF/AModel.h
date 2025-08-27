#pragma once

#include "HybridObject.h"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

using namespace DirectX;

// CPU-side structs
struct LeafNode {
	XMINT3 coord;
	//float voxels[8 * 8 * 8]; // LEAF_SIZE
	float voxels[8][8][8]; // LEAF_SIZE
	UINT bitmask[16];
};
struct InternalNode
{
	XMINT3  coord;       // node coord in THIS leve's grid
	UINT    childMask;    // 8 bits: which children exits
	UINT    firstChild;   // index into previous-level array
	XMINT3  padding;
};

// GPU-compatible structs
struct LeafNodeData {
	XMINT3 coord;
	UINT sliceIndex;    // Which 8 slices in the array
	UINT bitmask[16];
};

class AModel : public HybridObject
{
public:
	AModel(const string& fileName, UINT sdfResolution = 64) : HybridObject(SDFPrimitive::Enum::AModel, 2, sdfResolution), m_fileName(string(fileName)) {};
	void BuildSDF(ID3D12Device* device, ID3D12GraphicsCommandList* commandList);
	void BuildSVS(ID3D12Device* device, ID3D12GraphicsCommandList* commandList);

	void BuildLeafAtlas(ID3D12Device* device, ID3D12GraphicsCommandList* commandList);
	void BuildLeafLevel(ID3D12Device* device, ID3D12GraphicsCommandList* commandList);
	void BuildInternalLevel1(ID3D12Device* device, ID3D12GraphicsCommandList* commandList);
	void BuildInternalLevel2(ID3D12Device* device, ID3D12GraphicsCommandList* commandList);
	void BuildRoot(ID3D12Device* device, ID3D12GraphicsCommandList* commandList);
	
	virtual void ReleaseStagingBuffers() override
	{
		m_stagingSdfVoxelBuffer.Reset();
		m_stagingSdfTextureBuffer.Reset();
		Object::ReleaseStagingBuffers();
	};
	BufferHelper::D3DBuffer& GetSDFTextureBuffer() { return m_sdfTextureBuffer; };
	BufferHelper::D3DBuffer& GetSDFVoxelBuffer() { return m_sdfVoxelBuffer; };

	UINT GetLeafCount() const { return static_cast<UINT>(m_leafLevel.size()); }
	UINT GetInternal1Count() const { return static_cast<UINT>(m_internalLevel1.size()); }
	UINT GetInternal2Count() const { return static_cast<UINT>(m_internalLevel2.size()); }
	vector<LeafNode>& GetLeafNodes() { return m_leafLevel; }

	BufferHelper::D3DBuffer& GetLeafAtlasBuffer() { return m_leafAtlasBuffer; }
	BufferHelper::D3DBuffer& GetLeafNodeBuffer() { return m_leafNodeBuffer; }
	BufferHelper::D3DBuffer& GetInternal1NodeBuffer() { return m_internal1NodeBuffer; }
	BufferHelper::D3DBuffer& GetInternal2NodeBuffer() { return m_internal2NodeBuffer; }
	BufferHelper::D3DBuffer& GetRootNodeBuffer() { return m_rootNodeBuffer; }

	virtual void ExtractNarrowBand() override;
	virtual void ExtractBricks() override;

	void ExtractBricksVDB();

protected:
	virtual void BuildTriangleGeometry(ID3D12Device* device, vector<Index>& indices, vector<Vertex>& vertices) override;
	virtual void BuildAABBs(ID3D12Device* device, vector<D3D12_RAYTRACING_AABB>& aabbs) override;

private:
	BufferHelper::D3DBuffer m_sdfTextureBuffer, m_sdfVoxelBuffer;
	ComPtr<ID3D12Resource> m_stagingSdfTextureBuffer, m_stagingSdfVoxelBuffer;
	string m_fileName;


	// none of these are needed. just openvdb tests
	vector<LeafNode> m_leafLevel;
	vector<InternalNode> m_internalLevel1, m_internalLevel2;
	InternalNode m_rootNode;

	BufferHelper::D3DBuffer m_leafAtlasBuffer;
	ComPtr<ID3D12Resource> m_stagingLeafAtlasBuffer;

	BufferHelper::D3DBuffer m_leafNodeBuffer;
	ComPtr<ID3D12Resource> m_stagingLeafNodeBuffer;

	BufferHelper::D3DBuffer m_internal1NodeBuffer, m_internal2NodeBuffer;
	ComPtr<ID3D12Resource> m_stagingInternal1NodeBuffer, m_stagingInternal2NodeBuffer;

	BufferHelper::D3DBuffer m_rootNodeBuffer;
	ComPtr<ID3D12Resource> m_stagingRootNodeBuffer;

	void BuildVDBSBS();
};