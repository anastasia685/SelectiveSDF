#include "Common.hlsl"

RWTexture2D<float4> g_renderTarget : register(u0);
RaytracingAccelerationStructure g_scene : register(t0);
ConstantBuffer<SceneConstantBuffer> g_sceneCB : register(b0);


Ray GenerateCameraRay(uint2 launchIndex, float4x4 viewI, float4x4 projectionI)
{
    // launchIndex is essentially screen-space coordinates
    float2 dims = float2(DispatchRaysDimensions().xy);
    float2 ndc = (((launchIndex.xy + 0.5f) / dims.xy) * 2.f - 1.f); // center of the pixel normalized into [0,1], then shifted to [-1,1] NDC space
    ndc.y = -ndc.y; // invert Y to match DirectX coordinate system

    Ray ray;
    ray.origin = mul(g_sceneCB.viewI, float4(0, 0, 0, 1)); // camera position in world space is the origin in view space
    float4 target = mul(projectionI, float4(ndc.x, ndc.y, 1, 1)); // pick a point on the far plane and no need for perspective divide either since we want direction anyways
    ray.direction = mul(viewI, float4(target.xyz, 0));

    return ray;
}
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
        RAY_FLAG_FORCE_OPAQUE,
        0x01 | 0x02,
        0,
        0,
        0,
        rayDesc, rayPayload);

    return rayPayload.color;
}

[shader("raygeneration")]
void RayGen()
{
    // Generate a ray for a camera pixel corresponding to an index from the dispatched 2D grid.
    Ray ray = GenerateCameraRay(DispatchRaysIndex().xy, g_sceneCB.viewI, g_sceneCB.projectionI);
    float4 color = TraceRadianceRay(ray, 0, g_scene);
 
    g_renderTarget[DispatchRaysIndex().xy] = color;
}