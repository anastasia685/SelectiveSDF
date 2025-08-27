#pragma once

#include "HybridObject.h"

using namespace DirectX;

class Sphere : public HybridObject
{
public:
	Sphere() : HybridObject(SDFPrimitive::Enum::Sphere, 1) {};

	virtual void ExtractNarrowBand() override {};
	virtual void ExtractBricks() override;

protected:
	virtual void BuildTriangleGeometry(ID3D12Device* device, vector<Index>& indices, vector<Vertex>& vertices) override;
	virtual void BuildAABBs(ID3D12Device* device, vector<D3D12_RAYTRACING_AABB>& aabbs) override;
};