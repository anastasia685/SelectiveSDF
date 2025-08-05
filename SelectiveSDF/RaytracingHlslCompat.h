#pragma once

using namespace DirectX;

#define MAX_RAY_RECURSION_DEPTH 1


namespace TrianglePrimitive {
    enum Enum {
        Plane = 0,
        Count
    };
}

namespace SDFPrimitive {
    enum Enum {
        Box = 0,
        Sphere,
        AModel
    };
};

struct SDFInstanceData {
    XMMATRIX world;
    XMMATRIX worldI;
    float scale;
	UINT objectIndex; // for static sdf *object* data like primitive type, etc.
	UINT instanceIndex; // Index in the m_instances vector
    UINT brickStart;
    //SDFPrimitive::Enum sdfPrimitiveType;
    //INT sdfTextureIndex = -1; // only amodel types will have valid texture indices
    //UINT instanceIndex;
};

struct SDFObjectData
{
    SDFPrimitive::Enum primitiveType;
	UINT textureIndex = -1; // Only valid for AModel type
	UINT firstBrickIndex = 0; // First brick index in the atlas
	//UINT brickCount = 0; // Number of bricks in the atlas
    float padding;
};
struct HashTableEntry
{
    XMINT3 cellPos;
    UINT indexOffset;
    UINT count;
    UINT occupied;
    XMFLOAT2 padding;
};
struct BVHNodeGPU
{
    XMFLOAT3 min;
    UINT leftChild; // or ~0 for leaf

    XMFLOAT3 max;
    UINT rightChild; // or ~0 for leaf

    UINT firstInstance;
    UINT instanceCount;
    UINT padding[2]; // pad to 64 bytes
};
struct InstanceIndex
{
    UINT index;
    XMFLOAT3 padding;
};


struct RayPayload
{
    XMFLOAT4 color;
    UINT   recursionDepth;
};

struct ShadowRayPayload
{
    bool hit;
};

struct ProceduralPrimitiveAttributes
{
    XMFLOAT3 normal;
};