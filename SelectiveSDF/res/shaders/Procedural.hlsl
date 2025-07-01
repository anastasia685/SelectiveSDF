#include "Common.hlsl"

RaytracingAccelerationStructure g_scene : register(t0);
StructuredBuffer<SDFObjectData> g_sdfObjectsData : register(t1);
StructuredBuffer<HashTableEntry> g_hashTable : register(t2);
StructuredBuffer<InstanceIndex> g_instanceIndices : register(t3);
ByteAddressBuffer g_indexVertexBuffers[] : register(t4, space0);
Texture3D<float> g_sdfTextures[] : register(t4, space1);
ConstantBuffer<SceneConstantBuffer> g_sceneCB : register(b0);
SamplerState g_samplerClamp : register(s0);

float sdBox(float3 p, float3 b)
{
    float3 d = abs(p) - b;
    return length(max(d, 0.0)) + min(max(d.x, max(d.y, d.z)), 0.0);
}
float sdSphere(float3 p, float s)
{
    return length(p) - s;
}
float sdTexture(float3 localPos, float3 localDir, Texture3D<float> sdfTexture)
{
    float3 boundaryPoint = localPos;
    float distToBoundary = 0;
    
    float3 texDim;
    sdfTexture.GetDimensions(texDim.x, texDim.y, texDim.z);
    float3 texelSize = float3(0.5, 0.5, 0.5) / texDim; // half pixel
    if (any(abs(localPos) > (float3(0.5, 0.5, 0.5) - texelSize))) // local boundaries are -0.5, 0.5
    {
        // Calculate distance to slightly smaller box
        distToBoundary = sdBox(localPos, float3(0.5, 0.5, 0.5) - texelSize);
        boundaryPoint = localPos + localDir * distToBoundary;
    }
    
    float3 boundaryUVW = boundaryPoint + float3(0.5, 0.5, 0.5); // shift to [0, 1] range
    
    float boundarySDFValue = sdfTexture.SampleLevel(g_samplerClamp, boundaryUVW, 0);
    return distToBoundary + boundarySDFValue;
}
float3 boxNormal(float3 p, float3 b)
{
    float3 d = abs(p) - b;
    
    // Outside the box
    if (d.x > 0.0 || d.y > 0.0 || d.z > 0.0)
    {
        // Find which axis has the maximum distance
        float3 absD = abs(d);
        
        if (d.x > 0 && d.x >= max(d.y, d.z))
            return float3(sign(p.x), 0.0, 0.0);
        else if (d.y > 0 && d.y >= max(d.x, d.z))
            return float3(0.0, sign(p.y), 0.0);
        else if (d.z > 0)
            return float3(0.0, 0.0, sign(p.z));
            
        // On edge or corner - use gradient approximation
        return normalize(max(d, 0.0) * sign(p));
    }
    else
    {
        // Inside the box - your existing code is fine
        float3 axisDist = b - abs(p);
        if (axisDist.x < axisDist.y && axisDist.x < axisDist.z)
            return float3(sign(p.x), 0.0, 0.0);
        else if (axisDist.y < axisDist.z)
            return float3(0.0, sign(p.y), 0.0);
        else
            return float3(0.0, 0.0, sign(p.z));
    }
}
float3 sphereNormal(float3 p)
{
    return normalize(p);
}
float3 textureNormal(float3 pLocal, float3 dirLocal, float4x4 world, Texture3D<float> sdfTexture)
{
    float3 texDim;
    sdfTexture.GetDimensions(texDim.x, texDim.y, texDim.z);
    float3 eps = float3(1.0 / texDim.x, 1.0 / texDim.y, 1.0 / texDim.z) * 2; // Adjust epsilon based on texture resolution
    
    // Calculate gradient in local space
    float dx = sdTexture(pLocal + float3(eps.x, 0, 0), dirLocal, sdfTexture) -
               sdTexture(pLocal - float3(eps.x, 0, 0), dirLocal, sdfTexture);
    float dy = sdTexture(pLocal + float3(0, eps.y, 0), dirLocal, sdfTexture) -
               sdTexture(pLocal - float3(0, eps.y, 0), dirLocal, sdfTexture);
    float dz = sdTexture(pLocal + float3(0, 0, eps.z), dirLocal, sdfTexture) -
               sdTexture(pLocal - float3(0, 0, eps.z), dirLocal, sdfTexture);
    
    float3 localNormal = normalize(float3(dx, dy, dz));
    
    // Transform normal back to world space
    return normalize(mul((float3x3) world, localNormal));
}

struct MapResult
{
    float distance;
    float3 normal;
};
MapResult map(float3 p, float3 rd)
{
    const float k = 7; // Smooth min threshold
    
    MapResult result;
    
    float blendExpSum = 0.0;
    float3 blendNormal = float3(0, 0, 0);
    
    for (uint i = 0; i < g_sceneCB.sdfInstanceCount; i++)
    {
        SDFObjectData objectData = g_sdfObjectsData[i];
            
            
        // skip if too far
        float3 pWorld = mul(objectData.world, float4(0, 0, 0, 1)).xyz;
        if (length(pWorld - p) > 2)
            continue;
        
        float3 pLocal = mul(objectData.worldI, float4(p, 1.0)).xyz;
        switch (objectData.sdfPrimitiveType)
        {
            case 0: // box
                {
                    float dist = sdBox(pLocal, float3(0.5, 0.5, 0.5));
                    float w = exp(-k * dist);
                    blendExpSum += w;
                    float3 normal = boxNormal(pLocal, float3(0.5, 0.5, 0.5));
                    normal = normalize(mul((float3x3) objectData.world, normal));
                    blendNormal += w * normal;
                    break;
                }
            case 1: // sphere
                {
                    float dist = sdSphere(pLocal, 0.5);
                    float w = exp(-k * dist);
                    blendExpSum += w;
                    float3 normal = sphereNormal(pLocal);
                    normal = normalize(mul((float3x3) objectData.world, normal));
                    blendNormal += w * normal;
                    break;
                }
            case 2: // amodel
                {
                    float3 rdLocal = mul((float3x3) objectData.worldI, rd);
                    float dist = sdTexture(pLocal, rdLocal, g_sdfTextures[objectData.sdfTextureIndex]);
                    float w = exp(-k * dist);
                    blendExpSum += w;
                    blendNormal += w * textureNormal(pLocal, rdLocal, objectData.world, g_sdfTextures[objectData.sdfTextureIndex]);
                    break;
                }
        }
    }
    
        float blendDist = 1e3;
    if (blendExpSum > 0.0)
    {
        blendDist = -log(blendExpSum + 1e-6) / k;
        blendNormal = normalize(blendNormal / blendExpSum);
    }
    
    result.distance = blendDist;
    result.normal = blendNormal;
    
    return result;
}


[shader("closesthit")]
void ClosestHit_AABB(inout RayPayload rayPayload, in ProceduralPrimitiveAttributes attr)
{
    //float3 hitColor = float3(1.0f, 0.0f, 1.0f);
    float3 hitColor = attr.normal * 0.5 + 0.5;
    rayPayload.color = float4(hitColor, 1.0f);
}

[shader("intersection")]
void Intersection_SDF()
{
    // Get AABB bounds (assuming unit cube centered at origin)
    float3 aabbMin = float3(-0.6, -0.6, -0.6);
    float3 aabbMax = float3(0.6, 0.6, 0.6);
    
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
    
    //ProceduralPrimitiveAttributes attr;
    //attr.normal = float3(1, 0, 0); // world space normal
    //ReportHit(tEntry, 0, attr);
    //return;
    
    
    
    // sphere tracing
    {   
        float t = tEntry;
        const float minDist = 1e-3;
        const float maxDist = tExit;
        bool sdfHit = false;
        float3 sdfNormal = float3(0, 0, 0);
        float3 sdfHitPoint = float3(0, 0, 0);
        
        
        const uint MAX_STEPS = 128;
        uint stepCount = 0;
        while (t < maxDist)
        {
            float3 p = WorldRayOrigin() + WorldRayDirection() * t;
            MapResult mapResult = map(p, WorldRayDirection());
            
            if (mapResult.distance <= minDist)
            {
                // hit
                sdfHit = true;
                sdfNormal = mapResult.normal;
                sdfHitPoint = p;
                break;
            }
            
            if (++stepCount > MAX_STEPS)
                break;

            t += max(mapResult.distance, minDist);
        }
        
        
        if (sdfHit)
        {
            ProceduralPrimitiveAttributes attr;
            attr.normal = sdfNormal; // world space normal
            //ReportHit(t, 0, attr);
            
            
            RayDesc rayDesc;
            rayDesc.Origin = WorldRayOrigin();
            rayDesc.Direction = WorldRayDirection();
            rayDesc.TMin = tEntry;
            rayDesc.TMax = tExit;
    
    
            RayQuery <RAY_FLAG_FORCE_OPAQUE> q;
            q.TraceRayInline(
                g_scene,
                RAY_FLAG_NONE, // or RAY_FLAG_CULL_BACK_FACING_TRIANGLES
                0x04, // Triangle mask
                rayDesc
            );
            // Process all intersections
            while (q.Proceed())
            {
                // Could handle candidates here if needed
            }
    
            if (q.CommittedStatus() == COMMITTED_TRIANGLE_HIT)
            {
                float polyT = q.CommittedRayT();
                float difference = abs(polyT - t);
            
                // Define transition zones
                const float minTransition = 0.05; // Below this, use triangle
                const float maxTransition = 0.1; // Above this, use pure SDF
                
                
                uint primitiveIndex = q.CommittedPrimitiveIndex();
                uint instanceIndex = q.CommittedInstanceID();
                
                Triangle polyTri = TriangleData(q.CommittedInstanceID(), q.CommittedPrimitiveIndex(), g_indexVertexBuffers);
            
                // Calculate hit point in object space
                float3 hitPoint = ObjectRayOrigin() + ObjectRayDirection() * polyT;
                float3 bary = Barycentrics(polyTri, hitPoint);
                
                float3 polyNormal = normalize(polyTri.v0.normal * bary.x + polyTri.v1.normal * bary.y + polyTri.v2.normal * bary.z);
                
                if (difference < minTransition)
                {
                    // too close, fall back to triangle geometry
                    attr.normal = polyNormal;
                    ReportHit(polyT, 0, attr);
                }
                else
                {
                    // Calculate blend factor using smoothstep
                    float blendFactor = smoothstep(minTransition, maxTransition, difference);
            
                    // Interpolate hit distance
                    float reportT = lerp(polyT, t, blendFactor);
                    attr.normal = normalize(lerp(polyNormal, sdfNormal, blendFactor));
            
                    ReportHit(reportT, 0, attr);
                }
            }
            else
            {
                // No triangle hit - use pure SDF
                ReportHit(t, 0, attr);
            }
        }
    }
}