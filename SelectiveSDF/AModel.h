#pragma once

#include "HybridObject.h"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

using namespace DirectX;

class AModel : public HybridObject
{
public:
	AModel(const string& fileName) : HybridObject(SDFPrimitive::Enum::AModel, 2), m_fileName(string(fileName)), m_sdfResolution({64, 64, 64}) {};
	void BuildSDF(ID3D12Device* device, ID3D12GraphicsCommandList* commandList);
	virtual void ReleaseStagingBuffers() override
	{
		m_stagingSdfBuffer.Reset();
		Object::ReleaseStagingBuffers();
	};
	BufferHelper::D3DBuffer& GetSDFBuffer() { return m_sdfBuffer; };
	XMUINT3 GetSDFResolution() const { return m_sdfResolution; };

protected:
	virtual void BuildTriangleGeometry(ID3D12Device* device, vector<Index>& indices, vector<Vertex>& vertices) override;
	virtual void BuildAABBs(ID3D12Device* device, vector<D3D12_RAYTRACING_AABB>& aabbs) override;

private:
	BufferHelper::D3DBuffer m_sdfBuffer;
	ComPtr<ID3D12Resource> m_stagingSdfBuffer;
	string m_fileName;
	XMUINT3 m_sdfResolution;
};