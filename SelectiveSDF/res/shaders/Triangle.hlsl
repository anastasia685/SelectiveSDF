#include "Common.hlsl"

// Triangle resources
ByteAddressBuffer g_indexVertexBuffers[] : register(t4, space0);

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
    
    float3 hitColor = (tri.v0.normal * 0.5 + 0.5) * barycentrics.x +
                      (tri.v1.normal * 0.5 + 0.5) * barycentrics.y +
                      (tri.v2.normal * 0.5 + 0.5) * barycentrics.z;
    
    float t = RayTCurrent();
    hitColor = lerp(hitColor, BackgroundColor(), 1.0 - exp(-0.00005 * t * t * t));
    
    //float3 hitColor = float3(0.0f, 1.0f, 0.0f); // Green color for triangle hit
    rayPayload.color = float4(hitColor , 1.0f);
}