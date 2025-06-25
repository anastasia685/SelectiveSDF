#pragma once

#include "HybridObject.h"

using namespace DirectX;

class Box : public HybridObject
{
public:
	Box() : HybridObject(1) {};

protected:
	virtual void BuildTriangleGeometry(ID3D12Device* device, vector<Index>& indices, vector<Vertex>& vertices) override;
	virtual void BuildAABBs(ID3D12Device* device, vector<D3D12_RAYTRACING_AABB>& aabbs) override;
};