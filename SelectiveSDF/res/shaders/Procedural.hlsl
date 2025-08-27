#include "Common.hlsl"

//RWStructuredBuffer<uint> g_brickVisibility : register(u1);
//RWStructuredBuffer<uint> g_filteredBricks : register(u2);
//RWByteAddressBuffer g_dispatchArgs : register(u3);


RaytracingAccelerationStructure g_scene : register(t0);
StructuredBuffer<SDFObjectData> g_sdfObjectsData : register(t1);
StructuredBuffer<SDFInstanceData> g_sdfInstancesData : register(t2);
StructuredBuffer<BrickMeta> g_bricks : register(t3);
StructuredBuffer<BVHNode> g_bvhNodes : register(t4);
StructuredBuffer<NodeInstanceIndex> g_instanceIndices : register(t5);
StructuredBuffer<uint> g_brickMask : register(t6);

ByteAddressBuffer g_indexVertexBuffers[] : register(t7, space0);
Texture3D<float> g_sdfTextures[] : register(t7, space1);
Texture2DArray<float4> g_sdfBrickTextures[] : register(t7, space2);

ConstantBuffer<SceneConstantBuffer> g_sceneCB : register(b0);

SamplerState g_samplerLinearClamp : register(s0);
SamplerState g_samplerPointClamp : register(s1);


#define MAX_INSTANCES 16


uint SDFObjectIndex(uint objectIndex)
{
    return objectIndex - g_sceneCB.triangleObjectCount;
}

uint SDFInstanceIndex(uint instanceIndex)
{
    return (instanceIndex - g_sceneCB.triangleInstanceCount) / 2;
}

bool PointIntersectsAABB(float3 p, float r, float3 bmin, float3 bmax)
{
    float3 q = clamp(p, bmin, bmax);
    return distance(p, q) <= r;
}
bool AABBOverlap(float3 minA, float3 maxA, float3 minB, float3 maxB)
{
    return (minA.x <= maxB.x && maxA.x >= minB.x) &&
           (minA.y <= maxB.y && maxA.y >= minB.y) &&
           (minA.z <= maxB.z && maxA.z >= minB.z);
}
void TraverseInstanceBVH(
    float3 queryMin, float3 queryMax,
    out uint foundIndices[MAX_INSTANCES],
    out int count)
{
    //foundIndices[0] = SDFInstanceIndex(InstanceIndex());
    //count = 1;
    //return;
    
    count = 0;

    // stack for node indices
    uint stack[64];
    uint stackPtr = 0;
    stack[stackPtr++] = 0; // start at root

    while (stackPtr > 0 && count < MAX_INSTANCES)
    {
        uint nodeIndex = stack[--stackPtr];
        BVHNode node = g_bvhNodes[nodeIndex];
        
        if (!AABBOverlap(queryMin, queryMax, node.min, node.max))
            continue;

        bool isLeaf = (node.leftChild == 0xFFFFFFFF && node.rightChild == 0xFFFFFFFF);

        if (isLeaf)
        {
            for (uint i = 0; i < node.instanceCount && count < MAX_INSTANCES; ++i)
            {
                uint instIdx = g_instanceIndices[node.firstInstance + i].index;
                foundIndices[count++] = instIdx;
            }
        }
        else
        {
            if (node.leftChild != 0xFFFFFFFF && stackPtr < 64)
                stack[stackPtr++] = node.leftChild;
            if (node.rightChild != 0xFFFFFFFF && stackPtr < 64)
                stack[stackPtr++] = node.rightChild;
        }
    }
}
float sdBox(float3 p, float3 b)
{
    float3 d = abs(p) - b;
    return length(max(d, 0.0)) + min(max(d.x, max(d.y, d.z)), 0.0);
}
float sdSphere(float3 p, float s)
{
    return length(p) - s;
}
float sdTexture(float3 localPos, Texture3D<float> sdfTexture)
{   
    float3 texDim;
    sdfTexture.GetDimensions(texDim.x, texDim.y, texDim.z);
    float3 texelSize = float3(0.5, 0.5, 0.5) / texDim; // half pixel
    
    float dist = 0;
    if (any(abs(localPos) > 0.5 + 1e-6)) // local boundaries are -0.5, 0.5
    {
        dist = sdBox(localPos, float3(0.5, 0.5, 0.5));
    }
    
    float3 uvw = clamp(localPos, float3(-0.5, -0.5, -0.5), float3(0.5, 0.5, 0.5)) + float3(0.5, 0.5, 0.5); // shift to [0, 1] range
    dist += sdfTexture.SampleLevel(g_samplerLinearClamp, uvw + texelSize, 0); // sample in the middle of the voxel
    
    return dist;
}
float3 boxNormal(float3 p, float3 b)
{
    float3 d = abs(p) - b;
    
    // outside the box
    if (d.x > 0.0 || d.y > 0.0 || d.z > 0.0)
    {
        // find which axis has the maximum distance
        float3 absD = abs(d);
        
        if (d.x > 0 && d.x >= max(d.y, d.z))
            return float3(sign(p.x), 0.0, 0.0);
        else if (d.y > 0 && d.y >= max(d.x, d.z))
            return float3(0.0, sign(p.y), 0.0);
        else if (d.z > 0)
            return float3(0.0, 0.0, sign(p.z));
            
        // on edge or corner - use gradient approximation
        return normalize(max(d, 0.0) * sign(p));
    }
    else
    {
        // inside the box
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
float3 textureNormal(float3 pLocal, Texture3D<float> sdfTexture)
{
    float3 texDim;
    sdfTexture.GetDimensions(texDim.x, texDim.y, texDim.z);
    float3 eps = float3(1.0 / texDim.x, 1.0 / texDim.y, 1.0 / texDim.z) * 2; // adjust epsilon based on texture resolution (2 texels for now)
    
    // calculate gradient in local space
    float dx = sdTexture(pLocal + float3(eps.x, 0, 0), sdfTexture) -
               sdTexture(pLocal - float3(eps.x, 0, 0), sdfTexture);
    float dy = sdTexture(pLocal + float3(0, eps.y, 0), sdfTexture) -
               sdTexture(pLocal - float3(0, eps.y, 0), sdfTexture);
    float dz = sdTexture(pLocal + float3(0, 0, eps.z), sdfTexture) -
               sdTexture(pLocal - float3(0, 0, eps.z), sdfTexture);
    
    return normalize(float3(dx, dy, dz));
}
float2 smin(float a, float b, float k)
{   
    float h = max(k - abs(a - b), 0.0) / k;
    float m = h * h * 0.5;
    float s = m * k * (1.0 / 2.0);
    return (a < b) ? float2(a - s, m) : float2(b - s, 1.0 - m);
}
float2 smax(float a, float b, float k)
{
    float h = 1.0 - min(abs(a - b) / (6.0 * k), 1.0);
    float w = h * h * h;
    float m = w * 0.5;
    float s = w * k;
    return (a > b) ? float2(a + s, m) : float2(b + s, 1.0 - m);
}
struct MapResult
{
    float distance;
    float3 normal;
};
MapResult map(float3 p, uint foundIndices[MAX_INSTANCES], uint count)
{
    const float kUnion = 16;
    const float kSubtraction = 60;
    
    MapResult result;
    
    float unionExpSum = 0;
    float3 unionNormal = float3(0, 0, 0);
    
    float subtractionExpSum;
    float3 subtractionNormal = float3(0, 0, 0);
    
    for (uint i = 0; i < count; i++)
    {
        SDFInstanceData instanceData = g_sdfInstancesData[foundIndices[i]];
        SDFObjectData objectData = g_sdfObjectsData[SDFObjectIndex(instanceData.objectIndex)];
        
        float3 pLocal = mul(instanceData.worldI, float4(p, 1.0)).xyz;
        
        float dist = 1e3;
        float3 normal = float3(0, 0, 0); // local
        
        switch (objectData.primitiveType)
        {
            case 0: // box
            {
                dist = sdBox(pLocal, float3(0.31, 0.31, 0.31));
                normal = boxNormal(pLocal, float3(0.31, 0.31, 0.31));
                break;
            }
            case 1: // sphere
            {
                dist = sdSphere(pLocal, 0.38);
                normal = sphereNormal(pLocal);
                break;
            }
            case 2: // amodel
            {
                dist = sdTexture(pLocal, g_sdfTextures[objectData.textureIndex]);
                normal = textureNormal(pLocal, g_sdfTextures[objectData.textureIndex]);
                break;
            }
        }
        dist *= instanceData.scale; // to world space units
        normal = normalize(mul((float3x3) instanceData.world, normal));
        
        float wUnion = exp(-kUnion * dist);
        unionExpSum += wUnion;
        unionNormal += wUnion * normal;
        
    }
    
    float unionBlendDist = 1e3;
    {
        unionBlendDist = -log(unionExpSum + 1e-6) / kUnion;
        unionNormal = normalize(unionNormal / unionExpSum);
        
        //unionBlendDist = unionExpSum;
    }
    // smooth xor, cool but not needed for now
    {
        float subtractionBlendDist = 1e3;
        {
            //subtractionBlendDist = log(1.0 + subtractionExpSum) / kSubtraction;
            //subtractionNormal = normalize(subtractionNormal / subtractionExpSum);
        
            subtractionBlendDist = subtractionExpSum;
        }
        //float2 blendResult = -smin(-unionBlendDist, subtractionBlendDist, 0.31f);
    
        //result.distance = blendResult.x;
        //result.normal = normalize(lerp(unionNormal, subtractionNormal, blendResult.y));
    }
    
    result.distance = unionBlendDist;
    result.normal = unionNormal;
    
    return result;
}

float SolveCubic(float c3, float c2, float c1, float c0, float tFar)
{
    tFar += 5e-4; // add a small epsilon to avoid numerical issues at the end
    
    // handle degenerate cases first
    if (abs(c3) < 1e-6)
    {
        if (abs(c2) < 1e-6)
        {
            if (abs(c1) < 1e-6)
            {
                return -1;
            }
            float t = -c0 / c1;
            return (t >= 0 && t <= tFar) ? t : -1;
        }
        
        float discriminant = c1 * c1 - 4 * c2 * c0;
        if (discriminant < 0)
            return -1;
        
        float sqrtDisc = sqrt(discriminant);
        float inv2c2 = 1.0 / (2.0 * c2);
        float t1 = (-c1 - sqrtDisc) * inv2c2;
        float t2 = (-c1 + sqrtDisc) * inv2c2;
        
        float tMin = -1;
        if (t1 >= 0 && t1 <= tFar)
            tMin = t1;
        if (t2 >= 0 && t2 <= tFar && (tMin < 0 || t2 < tMin))
            tMin = t2;
        return tMin;
    }
    
    // differenciate cubic to find critical points
    float a = 3.0 * c3;
    float b = 2.0 * c2;
    // c = c1
    
    float discriminant = b * b - 4.0 * a * c1;
    
    float intervals[4]; // maximum 3 intervals, so 4 endpoints
    int numIntervals = 1;
    intervals[0] = 0;
    intervals[1] = tFar;
    
    if (discriminant > 0)
    {
        float sqrtDisc = sqrt(discriminant);
        float invA = 1.0 / (2.0 * a);
        float t1 = (-b - sqrtDisc) * invA;
        float t2 = (-b + sqrtDisc) * invA;
        
        // add critical points to interval list if it's in [0, tfar]
        if (t1 > 0 && t1 < tFar)
        {
            intervals[numIntervals] = t1;
            numIntervals++;
        }
        if (t2 > 0 && t2 < tFar)
        {
            intervals[numIntervals] = t2;
            numIntervals++;
        }
        
        // sort intervals
        for (int i = 0; i < numIntervals; i++)
        {
            for (int j = i + 1; j < numIntervals + 1; j++)
            {
                if (intervals[j] < intervals[i])
                {
                    float temp = intervals[i];
                    intervals[i] = intervals[j];
                    intervals[j] = temp;
                }
            }
        }
    }
    
    // process intervals in order
    const float epsilon = 1e-6;
    for (int i = 0; i < numIntervals; i++)
    {
        float tStart = intervals[i];
        float tEnd = intervals[i + 1];

        float g0 = ((c3 * tStart + c2) * tStart + c1) * tStart + c0;
        float g1 = ((c3 * tEnd + c2) * tEnd + c1) * tEnd + c0;
        
        if (abs(g0) < epsilon)
            g0 = (g1 >= 0 ? epsilon : -epsilon);
        if (abs(g1) < epsilon)
            g1 = (g0 >= 0 ? epsilon : -epsilon);

        if (g0 * g1 < 0.0) // sign change => root in this interval
        {
            float a = tStart;
            float b = tEnd;
            float fa = g0;
            float fb = g1;

            for (int iter = 0; iter < 20; iter++)
            {
                float denom = fb - fa;
                float tMid = (abs(denom) < 1e-6) ? 0.5 * (a + b) : (fb * a - fa * b) / denom;
                float fMid = ((c3 * tMid + c2) * tMid + c1) * tMid + c0;

                if (abs(fMid) < epsilon || abs(b - a) < epsilon)
                    return tMid;

                if (fa * fMid < 0)
                {
                    b = tMid;
                    fb = fMid;
                }
                else
                {
                    a = tMid;
                    fa = fMid;
                }
            }

            return 0.5 * (a + b);
        }
    }
    
    return -1;
}
TraceResult TraceTriangleInline(float tEntry, float tExit, uint instanceIndex)
{
    TraceResult res;
    res.hit = false;
            
    RayDesc rayDesc;
    rayDesc.Origin = WorldRayOrigin();
    rayDesc.Direction = WorldRayDirection();
    rayDesc.TMin = tEntry;
    rayDesc.TMax = tExit;
    
    RayQuery <RAY_FLAG_NONE> q;
    q.TraceRayInline(
        g_scene,
        RAY_FLAG_NONE,
        0x04, // Hybrid object's triangle geometry mask
        rayDesc
    );
    // Process all intersections
    while (q.Proceed())
    {
        if (q.CandidateType() == CANDIDATE_NON_OPAQUE_TRIANGLE)
        {
            //if (q.CandidateInstanceIndex() == instanceIndex + 1) // actually need any instance hit here, cause blended surface represents multiple instances too
            {
                q.CommitNonOpaqueTriangleHit();
            }
        }
    }
    
    if (q.CommittedStatus() == COMMITTED_TRIANGLE_HIT)
    {
        float polyT = q.CommittedRayT();
                
        uint primitiveIndex = q.CommittedPrimitiveIndex();
        uint instanceIndex = q.CommittedInstanceIndex();
        uint instanceID = q.CommittedInstanceID();
                
        Triangle polyTri = TriangleData(instanceID, primitiveIndex, g_indexVertexBuffers);
                
        // transform the hit point to local space of that triangle instance
        float3 worldHitPoint = rayDesc.Origin + rayDesc.Direction * polyT;
        float3 objectHitPoint = mul(g_sdfInstancesData[SDFInstanceIndex(instanceIndex)].worldI, float4(worldHitPoint, 1.0)).xyz;
            
        float3 bary = Barycentrics(polyTri, objectHitPoint);
                
        float3 polyNormal = normalize(polyTri.v0.normal * bary.x + polyTri.v1.normal * bary.y + polyTri.v2.normal * bary.z);
                
        res.hit = true;
        res.t = polyT;
        res.normal = polyNormal;
    }
    return res;
}
void ComposeHits(TraceResult poly, TraceResult sdf)
{   
    ProceduralPrimitiveAttributes attr;
    
    if(!poly.hit && !sdf.hit)
    {
        return;
    }
    
    if(!sdf.hit)
    {
        attr.normal = poly.normal;
        ReportHit(poly.t, 0, attr);
        return;
    }
    if (!poly.hit)
    {
        attr.normal = sdf.normal;
        ReportHit(sdf.t, 0, attr);
        return;
    }
    
    float difference = poly.t - sdf.t;
            
    // transition thresholds
    const float minTransition = 0.010;
    const float maxTransition = 0.025;
        
    if (difference < minTransition)
    {
        // too close, fall back to triangle geometry
        attr.normal = poly.normal;
        ReportHit(poly.t, 0, attr);
    }
    else
    {
        float s = clamp((difference - minTransition) / (maxTransition - minTransition), 0.0, 1.0);
        float blendFactor = s;
            
        float reportT = lerp(poly.t, sdf.t, blendFactor);
        attr.normal = normalize(lerp(poly.normal, sdf.normal, blendFactor));
            
        ReportHit(reportT, 0, attr);
    }
}

void SphereTrace()
{
    SDFInstanceData thisInstanceData = g_sdfInstancesData[SDFInstanceIndex(InstanceIndex())];
    SDFObjectData thisObjectData = g_sdfObjectsData[SDFObjectIndex(thisInstanceData.objectIndex)];
    
    float3 localMin = float3(-0.5, -0.5, -0.5);
    float3 localMax = float3(0.5, 0.5, 0.5);
    
    uint GRID_SIZE = 64;
    float GRID_SPAN = 1.0f;
    float VOXEL_SIZE = GRID_SPAN / GRID_SIZE;
    float HALF_SPAN = GRID_SPAN * 0.5f;
    
    int primitiveIndex = PrimitiveIndex();
    
    float3 worldMin = mul(thisInstanceData.world, float4(localMin, 1)).xyz;
    float3 worldMax = mul(thisInstanceData.world, float4(localMax, 1)).xyz;

    
    float3 rayOrigin = WorldRayOrigin();
    float3 rayDir = WorldRayDirection();

    
    float3 aabbMin = min(worldMin, worldMax);
    float3 aabbMax = max(worldMin, worldMax);
    float3 corners[8] =
    {
        mul(thisInstanceData.world, float4(localMin.x, localMin.y, localMin.z, 1)).xyz,
        mul(thisInstanceData.world, float4(localMax.x, localMin.y, localMin.z, 1)).xyz,
        mul(thisInstanceData.world, float4(localMin.x, localMax.y, localMin.z, 1)).xyz,
        mul(thisInstanceData.world, float4(localMax.x, localMax.y, localMin.z, 1)).xyz,
        mul(thisInstanceData.world, float4(localMin.x, localMin.y, localMax.z, 1)).xyz,
        mul(thisInstanceData.world, float4(localMax.x, localMin.y, localMax.z, 1)).xyz,
        mul(thisInstanceData.world, float4(localMin.x, localMax.y, localMax.z, 1)).xyz,
        mul(thisInstanceData.world, float4(localMax.x, localMax.y, localMax.z, 1)).xyz
    };
    aabbMin = corners[0];
    aabbMax = corners[0];
    for (int i = 1; i < 8; i++)
    {
        aabbMin = min(aabbMin, corners[i]);
        aabbMax = max(aabbMax, corners[i]);
    }

    
    float3 invDir = 1.0 / rayDir;
    float3 t0 = (aabbMin - rayOrigin) * invDir;
    float3 t1 = (aabbMax - rayOrigin) * invDir;
    float3 tMin3 = min(t0, t1);
    float3 tMax3 = max(t0, t1);
    float tEntry = max(max(tMin3.x, tMin3.y), tMin3.z);
    float tExit = min(min(tMax3.x, tMax3.y), tMax3.z);
    
    
    if (tEntry > tExit || tExit < RayTMin())
        return;
        
    tEntry = max(tEntry, RayTMin());
    tExit = min(tExit, 1000);
        
    uint foundIndices[MAX_INSTANCES];
    uint count = 0;
    TraverseInstanceBVH(worldMin, worldMax, foundIndices, count);
        
    float t = tEntry;
    uint MAX_STEPS = 128;
    uint stepCount = 0;
        
    TraceResult polyResult = TraceTriangleInline(tEntry, tExit + 0.01, InstanceIndex()); // small offset/extension to avoid numerical errors
    
    TraceResult sdfResult;
    sdfResult.hit = false;
        
    while (t < tExit && stepCount <= MAX_STEPS)
    {
        float3 p = rayOrigin + rayDir * t;
        MapResult mapResult = map(p, foundIndices, count);
            
        if (mapResult.distance <= 1e-3)
        {
            sdfResult.hit = true;
            sdfResult.t = t + mapResult.distance;
            sdfResult.normal = mapResult.normal;
            break;
        }

        // Step in voxel-sized increments
        t += mapResult.distance;
        stepCount++;
    }
    
    ComposeHits(polyResult, sdfResult);
}
uint3 UnpackBrickCoord(uint packedCoord)
{
    return uint3(packedCoord & 0xFF, (packedCoord >> 8) & 0xFF, (packedCoord >> 16) & 0xFF);
}
void GetVoxelCorners(uint sdfInstanceIdx, uint sliceStart, int3 voxelPos, out float corners[8])
{
    float2 texCoord = float2(voxelPos.xy) + float2(0.5, 0.5);
    float2 normalizedCoord = texCoord / 10.0; // normalize to [0,1]
    
    // First gather: z slice
    float4 gather0 = g_sdfBrickTextures[sdfInstanceIdx].Gather(
        g_samplerPointClamp,
        float3(normalizedCoord, sliceStart + voxelPos.z)
    );
    
    // Second gather: z+1 slice  
    float4 gather1 = g_sdfBrickTextures[sdfInstanceIdx].Gather(
        g_samplerPointClamp,
        float3(normalizedCoord, sliceStart + voxelPos.z + 1)
    );
    
    // Map gather results to corners
    corners[0] = gather0.w; // (x,y,z)
    corners[1] = gather0.z; // (x+1,y,z)
    corners[2] = gather0.x; // (x,y+1,z)
    corners[3] = gather0.y; // (x+1,y+1,z)
    corners[4] = gather1.w; // (x,y,z+1)
    corners[5] = gather1.z; // (x+1,y,z+1)
    corners[6] = gather1.x; // (x,y+1,z+1)
    corners[7] = gather1.y; // (x+1,y+1,z+1)
}
void GetVoxelNormals(uint sdfInstanceIdx, uint sliceStart, int3 voxelPos, out float3 normals[8])
{
    const float brickResolution = 10.0; // assuming 10×10 brick slices
    float2 baseCoord = (float2(voxelPos.xy) + float2(0.5, 0.5)) / brickResolution;

    // Sample manually — we assume normals are packed in .yzw
    normals[0] = g_sdfBrickTextures[sdfInstanceIdx].SampleLevel(g_samplerPointClamp,
        float3(baseCoord, sliceStart + voxelPos.z + 0), 0).yzw; // (x,y,z)

    normals[1] = g_sdfBrickTextures[sdfInstanceIdx].SampleLevel(g_samplerPointClamp,
        float3(baseCoord + float2(1, 0) / brickResolution, sliceStart + voxelPos.z + 0), 0).yzw; // (x+1,y,z)

    normals[2] = g_sdfBrickTextures[sdfInstanceIdx].SampleLevel(g_samplerPointClamp,
        float3(baseCoord + float2(0, 1) / brickResolution, sliceStart + voxelPos.z + 0), 0).yzw; // (x,y+1,z)

    normals[3] = g_sdfBrickTextures[sdfInstanceIdx].SampleLevel(g_samplerPointClamp,
        float3(baseCoord + float2(1, 1) / brickResolution, sliceStart + voxelPos.z + 0), 0).yzw; // (x+1,y+1,z)

    normals[4] = g_sdfBrickTextures[sdfInstanceIdx].SampleLevel(g_samplerPointClamp,
        float3(baseCoord, sliceStart + voxelPos.z + 1), 0).yzw; // (x,y,z+1)

    normals[5] = g_sdfBrickTextures[sdfInstanceIdx].SampleLevel(g_samplerPointClamp,
        float3(baseCoord + float2(1, 0) / brickResolution, sliceStart + voxelPos.z + 1), 0).yzw; // (x+1,y,z+1)

    normals[6] = g_sdfBrickTextures[sdfInstanceIdx].SampleLevel(g_samplerPointClamp,
        float3(baseCoord + float2(0, 1) / brickResolution, sliceStart + voxelPos.z + 1), 0).yzw; // (x,y+1,z+1)

    normals[7] = g_sdfBrickTextures[sdfInstanceIdx].SampleLevel(g_samplerPointClamp,
        float3(baseCoord + float2(1, 1) / brickResolution, sliceStart + voxelPos.z + 1), 0).yzw; // (x+1,y+1,z+1)
}
//void MarkBrickVisible(uint brickIdx)
//{
//    uint prev;
//    InterlockedExchange(g_brickVisibility[brickIdx], g_sceneCB.frameIndex, prev);
//    if (prev == g_sceneCB.frameIndex)
//        return; // already added this frame
    
//    if (prev != g_sceneCB.frameIndex)
//    {
//        uint idx;
//        g_dispatchArgs.InterlockedAdd(0, 1, idx);
//    }
//}
void BrickSolve()
{
    uint brickIndex = PrimitiveIndex();
    
    SDFInstanceData instanceData = g_sdfInstancesData[SDFInstanceIndex(InstanceIndex())];
    
    BrickMeta meta = g_bricks[instanceData.brickStart + brickIndex];

    uint3 brickCoord = UnpackBrickCoord(meta.packedCoord);
    uint instanceIdx = meta.instanceIndex;
    uint sliceStart = meta.sliceStart;
    
    float3 rayOrigin = ObjectRayOrigin();
    float3 rayDir = ObjectRayDirection();
    
    const float GRID_SIZE = 64.0f;
    const float VOXEL_SIZE = 1.0f / GRID_SIZE;
    const float HALF_WORLD = 0.5f;
    const float BRICK_SIZE = 9.0 * VOXEL_SIZE;
    
    float3 brickMin = float3(brickCoord.x, brickCoord.y, brickCoord.z) * BRICK_SIZE - HALF_WORLD;
    float3 brickMax = brickMin + BRICK_SIZE;
        
    float3 invDir = 1.0 / rayDir;
    float3 t0 = (brickMin - rayOrigin) * invDir;
    float3 t1 = (brickMax - rayOrigin) * invDir;
    float3 tMin3 = min(t0, t1);
    float3 tMax3 = max(t0, t1);
    
    float tEntry = max(max(tMin3.x, tMin3.y), tMin3.z);
    float tExit = min(min(tMax3.x, tMax3.y), tMax3.z);
    
    if (tEntry > tExit || tExit < RayTMin())
        return;
    
    tEntry = max(tEntry, RayTMin());
    tExit = min(tExit, 1000);
    
    // brick uvw
    if (g_sceneCB.surfaceMode == 1)
    {
        ProceduralPrimitiveAttributes attr;
        float3 p = rayOrigin + rayDir * tEntry;
        attr.normal = (p - brickMin) / BRICK_SIZE;
        ReportHit(tEntry, 0, attr);
        return;
    }
    
    //MarkBrickVisible(instanceData.brickStart + brickIndex);
    
    // skip bricks with no zero-crossing voxels
        uint brickActive = g_brickMask[instanceData.brickStart + brickIndex];
    if (brickActive == 0)
        return;
    
    if (g_sceneCB.surfaceMode == 2)
    {
        ProceduralPrimitiveAttributes attr;
        float3 p = rayOrigin + rayDir * tEntry;
        attr.normal = (p - brickMin) / BRICK_SIZE;
        ReportHit(tEntry, 0, attr);
        return;
    }
    
    // DDA setup
    float3 entryPoint = rayOrigin + tEntry * rayDir;
    float3 voxelPosFloat = (entryPoint - brickMin) / VOXEL_SIZE;
    int3 voxelPos = clamp(int3(voxelPosFloat), 0, 8); // 0-8 range for 9x9x9
    
    float3 tDelta = abs(VOXEL_SIZE / rayDir);
    float3 tMax;
    int3 step;
    
    // initialize DDA
    [unroll]
    for (int i = 0; i < 3; i++)
    {
        if (rayDir[i] > 0)
        {
            tMax[i] = tEntry + ((voxelPos[i] + 1) * VOXEL_SIZE + brickMin[i] - entryPoint[i]) / rayDir[i];
            step[i] = 1;
        }
        else if (rayDir[i] < 0)
        {
            tMax[i] = tEntry + (voxelPos[i] * VOXEL_SIZE + brickMin[i] - entryPoint[i]) / rayDir[i];
            step[i] = -1;
        }
        else
        {
            tMax[i] = 1e10;
            step[i] = 0;
        }
    }
    
    // DDA traversal
    float currentT = tEntry;
    const int maxSteps = 27; // maximum 9 steps per axis
    
    TraceResult polyResult = TraceTriangleInline(tEntry - 0.5, tExit + 0.5, InstanceIndex());
                    
    TraceResult sdfResult;
    sdfResult.hit = false;

    for (int iter = 0; iter < maxSteps && currentT < tExit; iter++)
    {
        float corners[8];
        GetVoxelCorners(instanceIdx - g_sceneCB.triangleInstanceCount, sliceStart, voxelPos, corners);
    
        float minVal = corners[0];
        float maxVal = corners[0];
        [unroll]
        for (int i = 1; i < 8; i++)
        {
            minVal = min(minVal, corners[i]);
            maxVal = max(maxVal, corners[i]);
        }
    
        if (minVal <= 0 && maxVal >= 0) // sign change => surface inside voxel
        {   
            float3 voxelMin = brickMin + float3(voxelPos) * VOXEL_SIZE;
            float3 voxelMax = voxelMin + VOXEL_SIZE;
    
            // entry/exit for this specific voxel
            float3 vt0 = (voxelMin - rayOrigin) * invDir;
            float3 vt1 = (voxelMax - rayOrigin) * invDir;
            float3 vtMin = min(vt0, vt1);
            float3 vtMax = max(vt0, vt1);
            float voxelTEntry = max(max(vtMin.x, vtMin.y), vtMin.z);
            float voxelTExit = min(min(vtMax.x, vtMax.y), vtMax.z);
    
            voxelTEntry = max(voxelTEntry, currentT);
            voxelTExit = min(voxelTExit, tExit);
    
            if (voxelTEntry < voxelTExit)
            {
                float3 voxelEntryPoint = rayOrigin + voxelTEntry * rayDir;
                float3 localOrigin = (voxelEntryPoint - voxelMin) / VOXEL_SIZE;
                float localTFar = (voxelTExit - voxelTEntry) / VOXEL_SIZE;
                float3 localDir = rayDir;
                
                // voxel uvws
                if (g_sceneCB.surfaceMode == 3)
                {
                    ProceduralPrimitiveAttributes attr;
                    attr.normal = localOrigin;
                    ReportHit(max(voxelTEntry, 0), 0, attr);
                    return;
                }
                
                            
                // Compute k constants from corners
                float k0 = corners[0]; // s000
                float k1 = corners[1] - corners[0]; // s100 - s000
                float k2 = corners[2] - corners[0]; // s010 - s000
                float a = corners[5] - corners[4]; // s101 - s001
                float k4 = k0 - corners[4]; // s000 - s001
                float k5 = k1 - a;
                float k6 = k2 - (corners[6] - corners[4]); // k2 - (s011 - s001)
                float k3 = corners[3] - corners[2] - k1; // s110 - s010 - k1
                float k7 = k3 - (corners[7] - corners[6] - a); // k3 - (s111 - s011 - a)
        
                // m constants from ray equation
                float ox = localOrigin.x, oy = localOrigin.y, oz = localOrigin.z;
                float dx = localDir.x, dy = localDir.y, dz = localDir.z;
        
                float m0 = ox * oy;
                float m1 = dx * dy;
                float m2 = ox * dy + oy * dx;
                float m3 = k5 * oz - k1;
                float m4 = k6 * oz - k2;
                float m5 = k7 * oz - k3;
        
                // cubic coefficients
                float c0 = (k4 * oz - k0) + ox * m3 + oy * m4 + m0 * m5;
                float c1 = dx * m3 + dy * m4 + m2 * m5 + dz * (k4 + k5 * ox + k6 * oy + k7 * m0);
                float c2 = m1 * m5 + dz * (k5 * dx + k6 * dy + k7 * m2);
                float c3 = k7 * m1 * dz;
        
                float t = SolveCubic(c3, c2, c1, c0, localTFar);
        
                if (t >= 0 && t <= localTFar)
                {
                    sdfResult.hit = true;
                    sdfResult.t = voxelTEntry + t * VOXEL_SIZE;
                    
                    //compute normal
                    float3 localHit = localOrigin + t * localDir;
                    
                    float2 texCoordXY = (float2(voxelPos.xy) + float2(0.5, 0.5) + localHit.xy) / 10.0f;
                    // let hardware do bilinear filtering in xy
                    float3 normal0 = g_sdfBrickTextures[instanceIdx - g_sceneCB.triangleInstanceCount].SampleLevel(
                        g_samplerLinearClamp,
                        float3(texCoordXY, sliceStart + voxelPos.z),
                        0
                    ).yzw;
    
                    float3 normal1 = g_sdfBrickTextures[instanceIdx - g_sceneCB.triangleInstanceCount].SampleLevel(
                        g_samplerLinearClamp,
                        float3(texCoordXY, sliceStart + voxelPos.z + 1),
                        0
                    ).yzw;
    
                    // manually interpolate in z
                    sdfResult.normal = lerp(normal0, normal1, localHit.z);
                    break;
                }
            }
        }
    
        // step to next voxel using DDA
        if (tMax.x < tMax.y)
        {
            if (tMax.x < tMax.z)
            {
                currentT = tMax.x;
                tMax.x += tDelta.x;
                voxelPos.x += step.x;
                if (voxelPos.x < 0 || voxelPos.x >= 9)
                    break;
            }
            else
            {
                currentT = tMax.z;
                tMax.z += tDelta.z;
                voxelPos.z += step.z;
                if (voxelPos.z < 0 || voxelPos.z >= 9)
                    break;
            }
        }
        else
        {
            if (tMax.y < tMax.z)
            {
                currentT = tMax.y;
                tMax.y += tDelta.y;
                voxelPos.y += step.y;
                if (voxelPos.y < 0 || voxelPos.y >= 9)
                    break;
            }
            else
            {
                currentT = tMax.z;
                tMax.z += tDelta.z;
                voxelPos.z += step.z;
                if (voxelPos.z < 0 || voxelPos.z >= 9)
                    break;
            }
        }
    }
    
    if (g_sceneCB.surfaceMode != 3)
        ComposeHits(polyResult, sdfResult);
}


[shader("closesthit")]
void ClosestHit_AABB(inout RayPayload rayPayload, in ProceduralPrimitiveAttributes attr)
{
    float3 hitColor = g_sceneCB.colorMode == 0 ? Phong(attr.normal, float3(1, 1, 1)) : attr.normal * 0.5 + 0.5;
    rayPayload.color = float4(hitColor, 1.0f);
}

[shader("intersection")]
void Intersection_SDF()
{   
    //--- TESTING ---
    //{
    //    ProceduralPrimitiveAttributes attr;
    //    attr.normal = float3(1, 0, 0); // world space normal
    //    ReportHit(RayTMin(), 0, attr);
    //    return;
    //}
    
    //SphereTrace();
    BrickSolve();
}