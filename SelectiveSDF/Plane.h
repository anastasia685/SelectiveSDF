#pragma once

#include "Object.h"

using namespace DirectX;

class Plane : public Object
{
public:
	Plane() : Object(ObjectType::Triangle) { };

	virtual void BuildGeometry(ID3D12Device* device, ID3D12GraphicsCommandList* commandList) override;
};