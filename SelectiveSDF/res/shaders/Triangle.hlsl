#include "Common.hlsl"

// Triangle resources
ByteAddressBuffer g_indexVertexBuffers[] : register(t7, space0);

ConstantBuffer<SceneConstantBuffer> g_sceneCB : register(b0);

[shader("closesthit")]
void ClosestHit_Triangle(inout RayPayload rayPayload, in BuiltInTriangleIntersectionAttributes attr)
{
    uint instanceID = InstanceID();
    uint triangleIndex = PrimitiveIndex();
    
    // Calculate barycentric coordinates
    float3 barycentrics = float3(1.f - attr.barycentrics.x - attr.barycentrics.y,
                                 attr.barycentrics.x,
                                 attr.barycentrics.y);
    
    Triangle tri = TriangleData(instanceID, triangleIndex, g_indexVertexBuffers);
    
    float3 normal = (tri.v0.normal) * barycentrics.x +
                      (tri.v1.normal) * barycentrics.y +
                      (tri.v2.normal) * barycentrics.z;
    
    float3 hitColor = g_sceneCB.colorMode == 0 ? Phong(normal, float3(0.3, 0.3, 0.3)) : normal * 0.5 + 0.5;
    
    float t = RayTCurrent();
    hitColor = lerp(hitColor, BackgroundColor(), 1.0 - exp(-0.00005 * t * t * t));
    
    rayPayload.color = float4(hitColor , 1.0f);
}