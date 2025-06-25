#include "Common.hlsl"


[shader("closesthit")]
void ClosestHit_AABB(inout RayPayload rayPayload, in ProceduralPrimitiveAttributes attr)
{
    float3 hitColor = float3(1.0f, 0.0f, 0.0f); // Red color for aabb hit
    rayPayload.color = float4(hitColor, 1.0f);
}

[shader("intersection")]
void Intersection_SDF_Box()
{
    // Get AABB bounds (assuming unit cube centered at origin)
    float3 aabbMin = float3(-0.5, -0.5, -0.5);
    float3 aabbMax = float3(0.5, 0.5, 0.5);
    
    // Ray-AABB intersection
    float3 invDir = 1.0 / ObjectRayDirection();
    float3 t0 = (aabbMin - ObjectRayOrigin()) * invDir;
    float3 t1 = (aabbMax - ObjectRayOrigin()) * invDir;
    
    float3 tMin = min(t0, t1);
    float3 tMax = max(t0, t1);
    
    float tEntry = max(max(tMin.x, tMin.y), tMin.z);
    float tExit = min(min(tMax.x, tMax.y), tMax.z);
    
    // Check if ray intersects AABB
    if (tEntry > tExit || tExit < RayTMin())
        return;
        
    // Clamp to ray extents
    tEntry = max(tEntry, RayTMin());
    tExit = min(tExit, RayTCurrent());
    
    ProceduralPrimitiveAttributes attr;
    attr.normal = float3(0, 0, 0);
    
    ReportHit(tEntry, 0, attr);
}

[shader("intersection")]
void Intersection_SDF_Texture()
{
    // Get AABB bounds (assuming unit cube centered at origin)
    float3 aabbMin = float3(-0.5, -0.5, -0.5);
    float3 aabbMax = float3(0.5, 0.5, 0.5);
    
    // Ray-AABB intersection
    float3 invDir = 1.0 / ObjectRayDirection();
    float3 t0 = (aabbMin - ObjectRayOrigin()) * invDir;
    float3 t1 = (aabbMax - ObjectRayOrigin()) * invDir;
    
    float3 tMin = min(t0, t1);
    float3 tMax = max(t0, t1);
    
    float tEntry = max(max(tMin.x, tMin.y), tMin.z);
    float tExit = min(min(tMax.x, tMax.y), tMax.z);
    
    // Check if ray intersects AABB
    if (tEntry > tExit || tExit < RayTMin())
        return;
        
    // Clamp to ray extents
    tEntry = max(tEntry, RayTMin());
    tExit = min(tExit, RayTCurrent());
    
    ProceduralPrimitiveAttributes attr;
    attr.normal = float3(0, 0, 0);
    
    ReportHit(tEntry, 0, attr);
}