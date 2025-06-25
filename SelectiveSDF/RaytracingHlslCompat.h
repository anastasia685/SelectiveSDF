#ifndef RAYTRACINGHLSLCOMPAT_H
#define RAYTRACINGHLSLCOMPAT_H

using namespace DirectX;

#endif

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
        AModel,
        Count
    };
}

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