struct Ray
{
    float3 origin;
    float3 direction;
};

struct RayPayload
{
    float4 color;
    uint recursionDepth;
};

struct ProceduralPrimitiveAttributes
{
    float3 normal;
};


struct Vertex
{
    float3 position;
    float3 normal;
};
#define INDEX_SIZE 4
#define VERTEX_SIZE 24

struct SceneConstantBuffer
{
    float4x4 viewI;
    float4x4 projectionI;
};

#define MAX_RAY_RECURSION_DEPTH 1


float4 TraceRadianceRay(Ray ray, uint currentRayRecursionDepth, RaytracingAccelerationStructure g_scene)
{
    if (currentRayRecursionDepth >= MAX_RAY_RECURSION_DEPTH)
    {
        return float4(0, 0, 0, 0);
    }

    // Set the ray's extents.
    RayDesc rayDesc;
    rayDesc.Origin = ray.origin;
    rayDesc.Direction = ray.direction;
    // Set TMin to a zero value to avoid aliasing artifacts along contact areas.
    // Note: make sure to enable face culling so as to avoid surface face fighting.
    rayDesc.TMin = 0;
    rayDesc.TMax = 10000;
    RayPayload rayPayload = { float4(0, 0, 0, 0), currentRayRecursionDepth + 1 };
    TraceRay(g_scene,
        RAY_FLAG_NONE,
        0x01,
        0,
        0,
        0,
        rayDesc, rayPayload);

    return rayPayload.color;
}