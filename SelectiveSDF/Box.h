#pragma once

#include "HybridObject.h"

using namespace DirectX;

class Box : public HybridObject
{
public:
	Box() : HybridObject(SDFPrimitive::Enum::Box, 1) {};

	virtual void ExtractNarrowBand() override {};
	virtual void ExtractBricks() override;

protected:
	virtual void BuildTriangleGeometry(ID3D12Device* device, vector<Index>& indices, vector<Vertex>& vertices) override;
	virtual void BuildAABBs(ID3D12Device* device, vector<D3D12_RAYTRACING_AABB>& aabbs) override;
};