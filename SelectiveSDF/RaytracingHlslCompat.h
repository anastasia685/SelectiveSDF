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

struct SDFObjectData {
    XMMATRIX world;
    XMMATRIX worldI;
    float scale;
    SDFPrimitive::Enum sdfPrimitiveType; // why not
    INT sdfTextureIndex = -1; // only amodel types will have valid texture indices
    UINT instanceIndex;
};
struct HashTableEntry
{
    XMINT3 cellPos;
    UINT indexOffset;
    UINT count;
    UINT occupied;
    XMFLOAT2 padding;
};
struct InstanceIndex
{
    INT index;
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