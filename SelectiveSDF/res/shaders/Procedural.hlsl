#include "Common.hlsl"

RaytracingAccelerationStructure g_scene : register(t0);
StructuredBuffer<SDFObjectData> g_sdfObjectsData : register(t1);
StructuredBuffer<SDFInstanceData> g_sdfInstancesData : register(t2);
StructuredBuffer<BrickMeta> g_bricks : register(t3);
//StructuredBuffer<SDFBrickData> g_brickTable : register(t3);
StructuredBuffer<BVHNode> g_bvhNodes : register(t4);
StructuredBuffer<NodeInstanceIndex> g_instanceIndices : register(t5);
//Texture2DArray<float> g_brickAtlas : register(t6);

ByteAddressBuffer g_indexVertexBuffers[] : register(t7, space0);
StructuredBuffer<SurfaceVoxel> g_sdfVoxels[] : register(t7, space1);
//Texture2DArray<float> g_leafAtlas[] : register(t7, space1);
Texture3D<float> g_sdfTextures[] : register(t7, space2);
Texture2DArray<float4> g_sdfBrickTextures[] : register(t7, space3);
//StructuredBuffer<LeafNodeData> g_leafNodes[] : register(t7, space2);
//StructuredBuffer<InternalNodeData> g_internal1Nodes[] : register(t7, space3);
//StructuredBuffer<InternalNodeData> g_internal2Nodes[] : register(t7, space4);
//StructuredBuffer<InternalNodeData> g_rootNodes[] : register(t7, space5);

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

    // Stack for node indices
    uint stack[64];
    uint stackPtr = 0;
    stack[stackPtr++] = 0; // Start at root

    while (stackPtr > 0 && count < MAX_INSTANCES)
    {
        uint nodeIndex = stack[--stackPtr];
        BVHNode node = g_bvhNodes[nodeIndex];

        // Use AABB-AABB intersection test
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
//float TraverseVoxelBVH(float3 queryPoint, out int closestVoxelIndex)
//{
//    float closestDist = 1000;
//    closestVoxelIndex = -1;
    
//    // Priority queue would be better, but stack works too
//    uint stack[64];
//    uint stackPtr = 0;
//    stack[stackPtr++] = 0;
    
//    while (stackPtr > 0)
//    {
//        uint nodeIndex = stack[--stackPtr];
//        BVHNode node = g_bvhNodes[nodeIndex];
        
//        // Key difference: distance to AABB, not overlap test
//        float distToNode = DistanceToAABB(queryPoint, node.min, node.max);
        
//        // Prune if this node is farther than current best
//        if (distToNode > closestDist)
//            continue;
            
//        if (isLeaf)
//        {
//            // Check actual voxels, update closest
//            for (uint i = 0; i < node.instanceCount; i++)
//            {
//                uint voxelIdx = g_instanceIndices[node.firstInstance + i].index;
//                float3 voxelCenter = GetVoxelCenter(voxelIdx);
//                float dist = distance(queryPoint, voxelCenter);
//                if (dist < closestDist)
//                {
//                    closestDist = dist;
//                    closestVoxelIndex = voxelIdx;
//                }
//            }
//        }
//        else
//        {
//            // Push children (ideally sorted by distance)
//            if (node.leftChild != 0xFFFFFFFF)
//                stack[stackPtr++] = node.leftChild;
//            if (node.rightChild != 0xFFFFFFFF)
//                stack[stackPtr++] = node.rightChild;
//        }
//    }
    
//    return closestDist;
//}
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
float3 textureNormal(float3 pLocal, Texture3D<float> sdfTexture)
{
    float3 texDim;
    sdfTexture.GetDimensions(texDim.x, texDim.y, texDim.z);
    float3 eps = float3(1.0 / texDim.x, 1.0 / texDim.y, 1.0 / texDim.z) * 2; // Adjust epsilon based on texture resolution
    
    // Calculate gradient in local space
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
    //k *= 6.0;
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
    const float kUnion = 10;
    const float kSubtraction = 60;
    
    MapResult result;
    
    float unionExpSum;
    float3 unionNormal = float3(0, 0, 0);
    
    float subtractionExpSum;
    float3 subtractionNormal = float3(0, 0, 0);
    
    for (uint i = 0; i < g_sceneCB.sdfInstanceCount; i++)
    {
        //SDFInstanceData instanceData = g_sdfInstancesData[foundIndices[i]];
        SDFInstanceData instanceData = g_sdfInstancesData[i];
        SDFObjectData objectData = g_sdfObjectsData[SDFObjectIndex(instanceData.objectIndex)];
            
        // skip if too far
        //float3 pWorld = mul(instanceData.world, float4(0, 0, 0, 1)).xyz;
        //if (length(pWorld - p) > 2)
        //    continue;
        
        float3 pLocal = mul(instanceData.worldI, float4(p, 1.0)).xyz;
        
        float dist = 1e3;
        float3 normal = float3(0, 0, 0); // local
        
        switch (objectData.primitiveType)
        {
            case 0: // box
                {
                    dist = sdBox(pLocal, float3(0.4, 0.4, 0.4));
                    normal = boxNormal(pLocal, float3(0.4, 0.4, 0.4));
                    break;
                }
            case 1: // sphere
                {
                    dist = sdSphere(pLocal, 0.4);
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
        
        //float wUnion = exp(-kUnion * dist);
        //unionExpSum += wUnion;
        //unionNormal += wUnion * normal;
        
        //float wSubtraction = exp(kSubtraction * dist);
        //subtractionExpSum += wSubtraction;
        //subtractionNormal += wSubtraction * normal;
        
        if (i == 0)
        {
            unionExpSum = dist;
            unionNormal = normal;

            subtractionExpSum = dist;
            subtractionNormal = normal;
        }
        else
        {
            //float2 unionRes = smin(unionExpSum, dist, 0.47f);
            float2 unionRes = smin(unionExpSum, dist, 0.25f);
            unionExpSum = unionRes.x;
            unionNormal = normalize(lerp(unionNormal, normal, unionRes.y));
            
            //float2 subtractionRes = -smin(-subtractionExpSum, -dist, 0.3f);
            //subtractionExpSum = subtractionRes.x;
            //subtractionNormal = normalize(lerp(subtractionNormal, normal, subtractionRes.y));
            
            // just max for maximizing "carved out" area
            if(dist > subtractionExpSum)
            {
                subtractionExpSum = dist;
                subtractionNormal = normal;
            }
        }
        
    }
    
    float unionBlendDist = 1e3;
    //if (unionExpSum > 0.0)
    {
        //unionBlendDist = -log(unionExpSum + 1e-6) / kUnion;
        //unionNormal = normalize(unionNormal / unionExpSum);
        
        unionBlendDist = unionExpSum;
    }
    float subtractionBlendDist = 1e3;
    //if (subtractionExpSum > 0.0)
    {
        //subtractionBlendDist = log(1.0 + subtractionExpSum) / kSubtraction;
        //subtractionNormal = normalize(subtractionNormal / subtractionExpSum);
        
        subtractionBlendDist = subtractionExpSum;
    }
    
    
    float2 blendResult = -smin(-unionBlendDist, subtractionBlendDist, 0.31f);
    
    //result.distance = blendResult.x;
    //result.normal = normalize(lerp(unionNormal, subtractionNormal, blendResult.y));
    
    result.distance = unionBlendDist;
    result.normal = unionNormal;
    
    return result;
}

float evalCubic(float c3, float c2, float c1, float c0, float t)
{
    return c3 * t * t * t + c2 * t * t + c1 * t + c0;
}
float _SolveCubic(float c3, float c2, float c1, float c0, float tMax)
{   
    float criticalT[4];
    int numCritical = 0;
    
    // Always include boundaries
    criticalT[numCritical++] = 0.0;
    criticalT[numCritical++] = tMax;
    
    // Find derivative roots g'(t) = 3*c3*t² + 2*c2*t + c1 = 0
    float a = 3.0 * c3;
    float b = 2.0 * c2;
    float c = c1;
    
    if (abs(a) > 1e-6)
    {
        float discriminant = b * b - 4.0 * a * c;
        if (discriminant >= 0.0)
        {
            float sqrtDisc = sqrt(discriminant);
            float t1 = (-b - sqrtDisc) / (2.0 * a);
            float t2 = (-b + sqrtDisc) / (2.0 * a);
            
            // Add roots that lie within our interval
            if (t1 > 1e-6 && t1 < tMax - 1e-6)
            {
                criticalT[numCritical++] = t1;
            }
            if (abs(t2 - t1) > 1e-6 && t2 > 1e-6 && t2 < tMax - 1e-6)
            {
                criticalT[numCritical++] = t2;
            }
        }
    }
    
    // Sort critical points (insertion sort for small arrays)
    int i;
    for (i = 1; i < numCritical; i++)
    {
        float key = criticalT[i];
        int j = i - 1;
        while (j >= 0 && criticalT[j] > key)
        {
            criticalT[j + 1] = criticalT[j];
            j--;
        }
        criticalT[j + 1] = key;
    }
    
    
    
    for (i = 0; i < numCritical - 1; i++)
    {
        float tStart = criticalT[i];
        float tEnd = criticalT[i + 1];
        
        float valueStart = evalCubic(c3, c2, c1, c0, tStart);
        float valueEnd = evalCubic(c3, c2, c1, c0, tEnd);
        
        // Check for sign change (root exists in this interval)
        if (valueStart * valueEnd <= 0.0)
        {
            // Use repeated linear interpolation to find root
            float t = tStart;
            float dt = tEnd - tStart;
            
            for (int iter = 0; iter < 10; iter++)
            {
                float midT = t + dt * 0.5;
                float midValue = evalCubic(c3, c2, c1, c0, midT);
                
                if (abs(midValue) < 1e-6)
                {
                    return midT;
                }
                
                if (valueStart * midValue < 0.0)
                {
                    tEnd = midT;
                    valueEnd = midValue;
                }
                else
                {
                    t = midT;
                    valueStart = midValue;
                }
                dt *= 0.5;
            }
            
            return t + dt * 0.5;
        }
    }
    
    return -1.0; // No root found
}
float SolveCubic(float c3, float c2, float c1, float c0, float tFar)
{
    tFar += 1e-4; // Add a small epsilon to avoid numerical issues at the end
    
    // Handle degenerate cases first
    if (abs(c3) < 1e-6)
    {
        // Quadratic case
        if (abs(c2) < 1e-6)
        {
            // Linear case
            if (abs(c1) < 1e-6)
            {
                return -1; // No solution or constant
            }
            float t = -c0 / c1;
            return (t >= 0 && t <= tFar) ? t : -1;
        }
        // Solve quadratic
        float discriminant = c1 * c1 - 4 * c2 * c0;
        if (discriminant < 0)
            return -1;
        
        float sqrtDisc = sqrt(discriminant);
        float inv2c2 = 1.0 / (2.0 * c2);
        float t1 = (-c1 - sqrtDisc) * inv2c2;
        float t2 = (-c1 + sqrtDisc) * inv2c2;
        
        // Return smallest positive root in range
        float tMin = -1;
        if (t1 >= 0 && t1 <= tFar)
            tMin = t1;
        if (t2 >= 0 && t2 <= tFar && (tMin < 0 || t2 < tMin))
            tMin = t2;
        return tMin;
    }
    
    // Step 1: Find critical points by solving g'(t) = 3c3t² + 2c2t + c1 = 0
    float a = 3.0 * c3;
    float b = 2.0 * c2;
    // c = c1 (reusing variable name)
    
    float discriminant = b * b - 4.0 * a * c1;
    
    // Create interval list
    float intervals[4]; // Maximum 3 intervals, so 4 endpoints
    int numIntervals = 1;
    intervals[0] = 0;
    intervals[1] = tFar;
    
    if (discriminant > 0)
    {
        // Two critical points
        float sqrtDisc = sqrt(discriminant);
        float invA = 1.0 / (2.0 * a);
        float t1 = (-b - sqrtDisc) * invA;
        float t2 = (-b + sqrtDisc) * invA;
        
        // Add critical points to interval list if they're in [0, tfar]
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
        
        // Sort intervals
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
    
    // Step 2: Process intervals in order, looking for sign changes
    const float epsilon = 1e-6;
    for (int i = 0; i < numIntervals; i++)
    {
        float tStart = intervals[i];
        float tEnd = intervals[i + 1];

        float g0 = ((c3 * tStart + c2) * tStart + c1) * tStart + c0;
        float g1 = ((c3 * tEnd + c2) * tEnd + c1) * tEnd + c0;

        // Clamp small values to avoid zero crossings from noise
        if (abs(g0) < epsilon)
            g0 = (g1 >= 0 ? epsilon : -epsilon);
        if (abs(g1) < epsilon)
            g1 = (g0 >= 0 ? epsilon : -epsilon);

        if (g0 * g1 < 0.0)
        {
            // Sign change -> root in interval
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

            return 0.5 * (a + b); // Fallback
        }
    }
    
    return -1; // No intersection found
}
float3 ComputeAnalyticNormal(float3 localPos, float corners[8])
{
    float x = localPos.x, y = localPos.y, z = localPos.z;
    
    // Make sure these are in the right order based on your corner indexing
    float y0 = lerp(corners[1] - corners[0], corners[3] - corners[2], y);
    float y1 = lerp(corners[5] - corners[4], corners[7] - corners[6], y);
    float dfdx = lerp(y0, y1, z);
    
    float x0 = lerp(corners[2] - corners[0], corners[3] - corners[1], x);
    float x1 = lerp(corners[6] - corners[4], corners[7] - corners[5], x);
    float dfdy = lerp(x0, x1, z);
    
    float x0z = lerp(corners[4] - corners[0], corners[5] - corners[1], x);
    float x1z = lerp(corners[6] - corners[2], corners[7] - corners[3], x);
    float dfdz = lerp(x0z, x1z, y);
    
    return normalize(float3(dfdx, dfdy, dfdz));
}
void TraceTriangleInline(float3 sdfNormal, float tEntry, float t)
{
    ProceduralPrimitiveAttributes attr;
    attr.normal = sdfNormal; // world space normal
    //ReportHit(t, 0, attr);
    //return;
            
            
    RayDesc rayDesc;
    rayDesc.Origin = WorldRayOrigin();
    rayDesc.Direction = WorldRayDirection();
    rayDesc.TMin = tEntry;
    rayDesc.TMax = 1000;
    
    RayQuery < RAY_FLAG_FORCE_OPAQUE > q;
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
        const float minTransition = 0.005; // Below this, use triangle
        const float maxTransition = 0.025;
                
        uint primitiveIndex = q.CommittedPrimitiveIndex();
        uint instanceIndex = q.CommittedInstanceIndex();
        uint instanceID = q.CommittedInstanceID();
                
        Triangle polyTri = TriangleData(instanceID, primitiveIndex, g_indexVertexBuffers);
                
                
        // Transform the hit point to object space of that triangle instance
        float3 worldHitPoint = rayDesc.Origin + rayDesc.Direction * polyT;
        float3 objectHitPoint = mul(g_sdfInstancesData[SDFInstanceIndex(instanceIndex)].worldI, float4(worldHitPoint, 1.0)).xyz;
            
        // Calculate hit point in object space
        //float3 hitPoint = ObjectRayOrigin() + ObjectRayDirection() * polyT;
        float3 bary = Barycentrics(polyTri, objectHitPoint);
                
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
            //float blendFactor = smoothstep(0, maxTransition, difference);
            float s = clamp((difference - minTransition) / (maxTransition - minTransition), 0.0, 1.0);
            float blendFactor = s * s * s * s;
            
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

void SphereTrace()
{
    SDFInstanceData thisInstanceData = g_sdfInstancesData[SDFInstanceIndex(InstanceIndex())];
    SDFObjectData thisObjectData = g_sdfObjectsData[SDFObjectIndex(thisInstanceData.objectIndex)];
    
    
    float3 localMin = float3(-0.5, -0.5, -0.5);
    float3 localMax = float3(0.5, 0.5, 0.5);
    
    uint GRID_SIZE = 64;
    float GRID_SPAN = 1.0f; // Assuming the world is -0.5 to 0.5 in each dimension
    float VOXEL_SIZE = GRID_SPAN / GRID_SIZE; // Size of each voxel in world space
    float HALF_SPAN = GRID_SPAN * 0.5f;
    
    int primitiveIndex = PrimitiveIndex();
    
    SurfaceVoxel voxel = g_sdfVoxels[thisObjectData.textureIndex][primitiveIndex];
    
    //float3 localMin = float3(voxel.x, voxel.y, voxel.z) * VOXEL_SIZE - HALF_SPAN;
    //float3 localMax = localMin + VOXEL_SIZE;
    
    
    float3 worldMin = mul(thisInstanceData.world, float4(localMin, 1)).xyz;
    float3 worldMax = mul(thisInstanceData.world, float4(localMax, 1)).xyz;
    
    float3 rayOrigin = ObjectRayOrigin();
    float3 rayDir = ObjectRayDirection();
    
    float3 invDir = 1.0 / rayDir;
    float3 t0 = (localMin - rayOrigin) * invDir;
    float3 t1 = (localMax - rayOrigin) * invDir;

    float3 tMin3 = min(t0, t1);
    float3 tMax3 = max(t0, t1);

    float tEntry = max(max(tMin3.x, tMin3.y), tMin3.z);
    float tExit = min(min(tMax3.x, tMax3.y), tMax3.z);
    
    // Check if ray intersects AABB
    if (tEntry > tExit || tExit < RayTMin())
        return;
        
    // Clamp to ray extents
    tEntry = max(tEntry, RayTMin());
    tExit = min(tExit, 1000);
    
    
    {
        rayOrigin = WorldRayOrigin();
        rayDir = WorldRayDirection();
        
        uint foundIndices[MAX_INSTANCES];
        uint count = 0;
        TraverseInstanceBVH(worldMin, worldMax, foundIndices, count);
        
        float t = tEntry;
        uint MAX_STEPS = 128;
        uint stepCount = 0;
        
        bool sdfHit = false;
        float3 sdfNormal = float3(0, 0, 0);
        
        while (t < tExit && stepCount <= MAX_STEPS)
        {
            float3 p = rayOrigin + rayDir * t;
            MapResult mapResult = map(p, foundIndices, count);
            
            if (mapResult.distance <= 1e-3)
            {
                sdfHit = true;
                sdfNormal = mapResult.normal;
                break;
            }
            

            // Step in voxel-sized increments
            t += mapResult.distance;
            stepCount++;
        }
        
        if (sdfHit)
        {
            TraceTriangleInline(sdfNormal, tEntry, t);
            return;
        }
    }
}
uint3 UnpackBrickCoord(uint packedCoord)
{
    return uint3(packedCoord & 0xFF, (packedCoord >> 8) & 0xFF, (packedCoord >> 16) & 0xFF);
}
void GetVoxelCorners(uint sdfInstanceIdx, uint sliceStart, int3 voxelPos, out float corners[8])
{
    float2 texCoord = float2(voxelPos.xy) + float2(0.5, 0.5);
    float2 normalizedCoord = texCoord / 10.0; // Normalize to [0,1]
    
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
float3 ComputeGradientForVoxel(float corners[8], float3 uvw)
{
    // For df/dx (equation 9)
    float y0 = lerp(corners[1] - corners[0], corners[3] - corners[2], uvw.y);
    float y1 = lerp(corners[5] - corners[4], corners[7] - corners[6], uvw.y);
    float dx = lerp(y0, y1, uvw.z);
    
    // For df/dy (equation 10)
    float x0 = lerp(corners[2] - corners[0], corners[3] - corners[1], uvw.x);
    float x1 = lerp(corners[6] - corners[4], corners[7] - corners[5], uvw.x);
    float dy = lerp(x0, x1, uvw.z);
    
    // For df/dz (equation 11)
    float x0_z = lerp(corners[4] - corners[0], corners[5] - corners[1], uvw.x);
    float x1_z = lerp(corners[6] - corners[2], corners[7] - corners[3], uvw.x);
    float dz = lerp(x0_z, x1_z, uvw.y);
    
    return float3(dx, dy, dz);
}
float3 ComputeContinuousNormal(
    uint sdfInstanceIdx,
    uint sliceStart,
    float3 hitPointBrickLocal)  // Hit point in brick-local voxel coordinates [0,7]
{
    // Find which dual voxel the hit point is in
    // Dual voxel grid is offset by 0.5 from regular grid
    // Regular voxels: [0,1), [1,2), [2,3), etc.
    // Dual voxels:    [-0.5,0.5), [0.5,1.5), [1.5,2.5), etc.
    float3 dualVoxelPos = floor(hitPointBrickLocal - float3(0.5, 0.5, 0.5));
    
    // Compute position within dual voxel (u,v,w) -> [0,1]^3
    float3 dualVoxelUVW = hitPointBrickLocal - float3(0.5, 0.5, 0.5) - dualVoxelPos;
    
    // The dual voxel overlaps 8 regular voxels
    // We need to compute normals from all 8 and interpolate
    float3 normals[8];
    
    for (int i = 0; i < 8; i++)
    {
        // Get offset for this corner of the dual voxel
        int3 offset = int3(i & 1, (i >> 1) & 1, (i >> 2) & 1);
        int3 voxelPos = int3(dualVoxelPos) + offset;
        
        // The brick stores voxels from -1 to 7 (9x9x9 voxels)
        // So voxelPos can range from -1 to 7 and still be valid
        // Add 1 to convert from spatial position to texture position
        // (spatial -1 -> texture 0, spatial 0 -> texture 1, etc.)
        // Note: No clamping needed! The padding handles edge cases
        float corners[8];
        GetVoxelCorners(sdfInstanceIdx, sliceStart, voxelPos + int3(1, 1, 1), corners);
        
        // Compute hit point position relative to THIS voxel
        // This is key: we evaluate the gradient at the same world space point
        // but using each voxel's local coordinate system
        float3 localHitInVoxel = hitPointBrickLocal - float3(voxelPos);
        
        // Compute gradient at hit point using this voxel's implicit function
        float3 gradient = ComputeGradientForVoxel(corners, localHitInVoxel);
        
        // Normalize to get normal
        normals[i] = normalize(gradient);
    }
    
    // Trilinearly interpolate the 8 normals based on position in dual voxel
    float3 normal =
        (1 - dualVoxelUVW.x) * (1 - dualVoxelUVW.y) * (1 - dualVoxelUVW.z) * normals[0] +
        dualVoxelUVW.x * (1 - dualVoxelUVW.y) * (1 - dualVoxelUVW.z) * normals[1] +
        (1 - dualVoxelUVW.x) * dualVoxelUVW.y * (1 - dualVoxelUVW.z) * normals[2] +
        dualVoxelUVW.x * dualVoxelUVW.y * (1 - dualVoxelUVW.z) * normals[3] +
        (1 - dualVoxelUVW.x) * (1 - dualVoxelUVW.y) * dualVoxelUVW.z * normals[4] +
        dualVoxelUVW.x * (1 - dualVoxelUVW.y) * dualVoxelUVW.z * normals[5] +
        (1 - dualVoxelUVW.x) * dualVoxelUVW.y * dualVoxelUVW.z * normals[6] +
        dualVoxelUVW.x * dualVoxelUVW.y * dualVoxelUVW.z * normals[7];
    
    return normalize(normal);
}
void BrickSolve()
{
    //{
    //    ProceduralPrimitiveAttributes attr;
    //    attr.normal = float3(1, 0, 0); // world space normal
    //    ReportHit(RayTMin(), 0, attr);
    //    return;
    //}
    
    uint brickIndex = PrimitiveIndex();
    
    // TODO: get instance data and brick slice start
    SDFInstanceData instanceData = g_sdfInstancesData[SDFInstanceIndex(InstanceIndex())];
    
    BrickMeta meta = g_bricks[instanceData.brickStart + brickIndex];

    uint3 brickCoord = UnpackBrickCoord(meta.packedCoord);
    uint instanceIdx = meta.instanceIndex;
    uint sliceStart = meta.sliceStart;
    
    float3 rayOrigin = ObjectRayOrigin();
    float3 rayDir = ObjectRayDirection();
    
    const float VOXEL_SIZE = 1.0f / 64.0f;
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
    //ProceduralPrimitiveAttributes attr;
    //float3 p = rayOrigin + rayDir * tEntry;
    //attr.normal = (p - brickMin) / BRICK_SIZE;
    //ReportHit(tEntry, 0, attr);
    //return;
    
    
    // DDA setup
    float3 entryPoint = rayOrigin + tEntry * rayDir;
    float3 voxelPosFloat = (entryPoint - brickMin) / VOXEL_SIZE;
    int3 voxelPos = clamp(int3(voxelPosFloat), 0, 8); // 0-8 range for 9x9x9
    
    float3 tDelta = abs(VOXEL_SIZE / rayDir);
    float3 tMax;
    int3 step;
    
    // Initialize DDA
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
    const int maxSteps = 27; // Maximum 9 steps per axis

    for (int iter = 0; iter < maxSteps && currentT < tExit; iter++)
    {
        // Get 8 corner values for current voxel from texture atlas
        float corners[8];
        GetVoxelCorners(instanceIdx - g_sceneCB.triangleInstanceCount, sliceStart, voxelPos, corners);
    
        // Check if surface crosses this voxel
        float minVal = corners[0];
        float maxVal = corners[0];
        [unroll]
        for (int i = 1; i < 8; i++)
        {
            minVal = min(minVal, corners[i]);
            maxVal = max(maxVal, corners[i]);
        }
    
        if (minVal <= 0 && maxVal >= 0)
        {
            // Surface in this voxel! Calculate exact voxel bounds
            float3 voxelMin = brickMin + float3(voxelPos) * VOXEL_SIZE;
            float3 voxelMax = voxelMin + VOXEL_SIZE;
    
            // Calculate entry/exit for this specific voxel
            float3 vt0 = (voxelMin - rayOrigin) * invDir;
            float3 vt1 = (voxelMax - rayOrigin) * invDir;
            float3 vtMin = min(vt0, vt1);
            float3 vtMax = max(vt0, vt1);
            float voxelTEntry = max(max(vtMin.x, vtMin.y), vtMin.z);
            float voxelTExit = min(min(vtMax.x, vtMax.y), vtMax.z);
    
            // Clamp to current traversal bounds
            voxelTEntry = max(voxelTEntry, currentT);
            voxelTExit = min(voxelTExit, tExit);
    
            if (voxelTEntry < voxelTExit)
            {
                // Advance origin if needed
                float3 actualOrigin = rayOrigin;
                if (voxelTEntry > 0)
                {
                    actualOrigin = rayOrigin + voxelTEntry * rayDir;
                }
        
                // Transform to canonical voxel space [0,1]³
                float3 voxelEntryPoint = clamp(rayOrigin + voxelTEntry * rayDir, voxelMin, voxelMax);
                float3 localOrigin = (voxelEntryPoint - voxelMin) / VOXEL_SIZE;
                
                //float3 localOrigin = (actualOrigin - voxelMin) / VOXEL_SIZE;
                float3 localDir = rayDir / VOXEL_SIZE;
                float localTFar = (voxelTExit - max(voxelTEntry, 0));
                
                // voxel uvws
                //ProceduralPrimitiveAttributes attr;
                //attr.normal = localOrigin;
                //ReportHit(max(voxelTEntry, 0), 0, attr);
                //return;
                            
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
        
                // Compute m constants
                float ox = localOrigin.x, oy = localOrigin.y, oz = localOrigin.z;
                float dx = localDir.x, dy = localDir.y, dz = localDir.z;
        
                float m0 = ox * oy;
                float m1 = dx * dy;
                float m2 = ox * dy + oy * dx;
                float m3 = k5 * oz - k1;
                float m4 = k6 * oz - k2;
                float m5 = k7 * oz - k3;
        
                // Compute cubic coefficients
                float c0 = (k4 * oz - k0) + ox * m3 + oy * m4 + m0 * m5;
                float c1 = dx * m3 + dy * m4 + m2 * m5 + dz * (k4 + k5 * ox + k6 * oy + k7 * m0);
                float c2 = m1 * m5 + dz * (k5 * dx + k6 * dy + k7 * m2);
                float c3 = k7 * m1 * dz;
        
                // Solve cubic
                float t = SolveCubic(c3, c2, c1, c0, localTFar);
        
                if (t >= 0 && t <= localTFar)
                {
                    // Found valid intersection!
                    float hitT = (max(voxelTEntry, 0) + t);
                    
                    //compute normal
                    float3 localHit = localOrigin + t * localDir;
                        
                    //float3 cornerNormals[8];
                    //GetVoxelNormals(instanceIdx - g_sceneCB.triangleInstanceCount, sliceStart, voxelPos, cornerNormals);
                    //float3 c00 = lerp(cornerNormals[0], cornerNormals[1], localHit.x);
                    //float3 c01 = lerp(cornerNormals[4], cornerNormals[5], localHit.x);
                    //float3 c10 = lerp(cornerNormals[2], cornerNormals[3], localHit.x);
                    //float3 c11 = lerp(cornerNormals[6], cornerNormals[7], localHit.x);

                    //float3 c0 = lerp(c00, c10, localHit.y);
                    //float3 c1 = lerp(c01, c11, localHit.y);

                    //float3 sdfNormal = normalize(lerp(c0, c1, localHit.z));
                    
                    
                    float2 texCoordXY = (float2(voxelPos.xy) + float2(0.5, 0.5) + localHit.xy) / 10.0f;
                    // Sample two slices with hardware bilinear
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
    
                    // Manually interpolate in Z
                    float3 sdfNormal = lerp(normal0, normal1, localHit.z);
            
                    //ProceduralPrimitiveAttributes attr;
                    //attr.normal = sdfNormal;
                    //ReportHit(hitT, 0, attr);
                    //return;
                    
                    TraceTriangleInline(sdfNormal, tEntry - 0.1, hitT); // TODO: adjust tEntry to reflect actual entry point
                    return;
                }
            }
        }
    
        // Step to next voxel using DDA
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
}

[shader("closesthit")]
void ClosestHit_AABB(inout RayPayload rayPayload, in ProceduralPrimitiveAttributes attr)
{
    //float3 hitColor = float3(1.0f, 0.0f, 1.0f);
    float3 hitColor = attr.normal;// * 0.5 + 0.5;
    rayPayload.color = float4(hitColor, 1.0f);
}

[shader("intersection")]
void _Intersection_SDF()
{
    SDFInstanceData thisInstanceData = g_sdfInstancesData[SDFInstanceIndex(InstanceIndex())];
    
    float3 localMin = float3(-0.6, -0.6, -0.6);
    float3 localMax = float3(0.6, 0.6, 0.6);

    float3 corners[8] =
    {
        float3(localMin.x, localMin.y, localMin.z),
        float3(localMin.x, localMin.y, localMax.z),
        float3(localMin.x, localMax.y, localMin.z),
        float3(localMin.x, localMax.y, localMax.z),
        float3(localMax.x, localMin.y, localMin.z),
        float3(localMax.x, localMin.y, localMax.z),
        float3(localMax.x, localMax.y, localMin.z),
        float3(localMax.x, localMax.y, localMax.z)
    };

    float3 worldMin = float3(1000, 1000, 1000);
    float3 worldMax = float3(-1000, -1000, -1000);

    for (int i = 0; i < 8; ++i)
    {
        float3 p = mul(thisInstanceData.world, float4(corners[i], 1)).xyz;
        worldMin = min(worldMin, p);
        worldMax = max(worldMax, p);
    }
    
    float3 invDir = 1.0 / WorldRayDirection();
    float3 t0 = (worldMin - WorldRayOrigin()) * invDir;
    float3 t1 = (worldMax - WorldRayOrigin()) * invDir;

    float3 tMin3 = min(t0, t1);
    float3 tMax3 = max(t0, t1);

    float tEntry = max(max(tMin3.x, tMin3.y), tMin3.z);
    float tExit = min(min(tMax3.x, tMax3.y), tMax3.z);
    
    // Check if ray intersects AABB
    if (tEntry > tExit || tExit < RayTMin())
        return;
        
    // Clamp to ray extents
    tEntry = max(tEntry, RayTMin());
    tExit = min(tExit, 1000);
    
    //ProceduralPrimitiveAttributes attr;
    //attr.normal = float3(1, 0, 0); // world space normal
    //ReportHit(tEntry, 0, attr);
    //return;
    
    float voxelSize = (localMax.x - localMin.x) / 64.0; // assuming 64³ texture
    float worldVoxelSize = voxelSize * thisInstanceData.scale;
    
    // sphere tracing
    {   
        float t = tEntry;
        const float minDist = 1e-3;
        const float maxDist = tExit;
        bool sdfHit = false;
        float3 sdfNormal = float3(0, 0, 0);
        float3 sdfHitPoint = float3(0, 0, 0);
        
        uint foundIndices[MAX_INSTANCES];
        uint count = 0;
        float3 influenceRange = float3(0.1, 0.1, 0.1);
        TraverseInstanceBVH(worldMin - influenceRange, worldMax + influenceRange, foundIndices, count);
        
        const uint MAX_STEPS = 128;
        uint stepCount = 0;
        
        while (t < maxDist)
        {
            float3 p = WorldRayOrigin() + WorldRayDirection() * t;
            MapResult mapResult = map(p, foundIndices, count);
            
            if (mapResult.distance <= 0)
            {
                // hit
                sdfHit = true;
                sdfNormal = mapResult.normal;
                sdfHitPoint = p;
                break;
            }
            
            if (++stepCount > MAX_STEPS)
                break;

            // Step in voxel-sized increments
            t += max(mapResult.distance, minDist);
        }
        
        //float prevT = tEntry;
        //MapResult prevResult;
        //prevResult.distance = 1e3; // Initialize as "outside"

        //while (t < maxDist)
        //{
        //    float3 p = WorldRayOrigin() + WorldRayDirection() * t;
        //    MapResult mapResult = map(p, foundIndices, count);
    
        //    if (mapResult.distance <= 0)
        //    {
        //        // Step back toward surface (distance is negative, so this moves back)
        //        float surfaceT = t + mapResult.distance;
        //        sdfHitPoint = WorldRayOrigin() + WorldRayDirection() * surfaceT;
    
        //        MapResult surfaceResult = map(sdfHitPoint, foundIndices, count);
        //        sdfNormal = surfaceResult.normal;
        //        sdfHit = true;
        //        break;
        //    }
    
        //    if (++stepCount > MAX_STEPS)
        //        break;
        
        //    // Store previous values for next iteration
        //    prevT = t;
        //    prevResult = mapResult;
    
        //    // Step by voxel size
        //    t += max(mapResult.distance, worldVoxelSize);
        //}
        
        
        if (sdfHit)
        {
            ProceduralPrimitiveAttributes attr;
            attr.normal = sdfNormal; // world space normal
            ReportHit(t, 0, attr);
            return; 
            
            RayDesc rayDesc;
            rayDesc.Origin = WorldRayOrigin();
            rayDesc.Direction = WorldRayDirection();
            rayDesc.TMin = tEntry;
            rayDesc.TMax = 1000;
    
    
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
                const float minTransition = 5 * worldVoxelSize; // Below this, use triangle
                const float maxTransition = 10 * worldVoxelSize; // Above this, use pure SDF
                //const float minTransition = 0.05; // Below this, use triangle
                //const float maxTransition = 0.1;
                
                uint primitiveIndex = q.CommittedPrimitiveIndex();
                uint instanceIndex = q.CommittedInstanceIndex();
                uint instanceID = q.CommittedInstanceID();
                
                Triangle polyTri = TriangleData(instanceID, primitiveIndex, g_indexVertexBuffers);
                
                
                // Transform the hit point to object space of that triangle instance
                float3 worldHitPoint = rayDesc.Origin + rayDesc.Direction * polyT;
                float3 objectHitPoint = mul(g_sdfInstancesData[SDFInstanceIndex(instanceIndex)].worldI, float4(worldHitPoint, 1.0)).xyz;
            
                // Calculate hit point in object space
                //float3 hitPoint = ObjectRayOrigin() + ObjectRayDirection() * polyT;
                float3 bary = Barycentrics(polyTri, objectHitPoint);
                
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


[shader("intersection")]
void __Intersection_SDF()
{ 
    float bound = 0.0078125; // 1/128, assuming 64³ texture
    float3 localMin = float3(-bound, -bound, -bound);
    float3 localMax = float3(bound, bound, bound);
    
    float3 invDir = 1.0 / ObjectRayDirection();
    float3 t0 = (localMin - ObjectRayOrigin()) * invDir;
    float3 t1 = (localMax - ObjectRayOrigin()) * invDir;

    float3 tMin3 = min(t0, t1);
    float3 tMax3 = max(t0, t1);

    float tEntry = max(max(tMin3.x, tMin3.y), tMin3.z);
    float tExit = min(min(tMax3.x, tMax3.y), tMax3.z);
    
    // Check if ray intersects AABB
    if (tEntry > tExit || tExit < RayTMin())
        return;
        
    // Clamp to ray extents
    tEntry = max(tEntry, RayTMin());
    tExit = min(tExit, 1000);
    
    ProceduralPrimitiveAttributes attr;
    attr.normal = float3(1, 0, 0); // world space normal
    ReportHit(tEntry, 0, attr);
}

[shader("intersection")]
void ___Intersection_SDF()
{
    uint gridSize = 64;
    float worldSize = 1.0f; // Assuming the world is -0.5 to 0.5 in each dimension
    float voxelSize = worldSize / gridSize; // Size of each voxel in world space
    
    SDFInstanceData thisInstanceData = g_sdfInstancesData[SDFInstanceIndex(InstanceIndex())];
    int primitiveIndex = PrimitiveIndex();
    
    // Convert linear index back to 3D voxel coordinates
    int z = primitiveIndex / (gridSize * gridSize);
    int y = (primitiveIndex % (gridSize * gridSize)) / gridSize;
    int x = primitiveIndex % gridSize;
    
    
    float3 rayOrigin = ObjectRayOrigin();
    float3 rayDir = ObjectRayDirection();
    
    // Voxel bounds in object space
    float halfWorld = worldSize * 0.5f;
    float3 voxelMin = float3(
        (x * voxelSize) - halfWorld,
        (y * voxelSize) - halfWorld,
        (z * voxelSize) - halfWorld
    );
    float3 voxelMax = voxelMin + voxelSize;
    
    // Calculate ray-voxel intersection
    float3 invDir = 1.0 / rayDir;
    float3 t0 = (voxelMin - rayOrigin) * invDir;
    float3 t1 = (voxelMax - rayOrigin) * invDir;
    
    float3 tMin3 = min(t0, t1);
    float3 tMax3 = max(t0, t1);

    float tEntry = max(max(tMin3.x, tMin3.y), tMin3.z);
    float tExit = min(min(tMax3.x, tMax3.y), tMax3.z);
    
    // Check if ray intersects AABB
    if (tEntry > tExit || tExit < RayTMin())
        return;
        
    // Clamp to ray extents
    tEntry = max(tEntry, RayTMin());
    tExit = min(tExit, 1000);
    
    
    
    
    float3 p = rayOrigin + rayDir * tEntry;
           
    // Convert local position to texture coordinates (0-1)
    float3 texCoord = (p + halfWorld) / worldSize;
        
    // Sample SDF at current position
    //float sdfValue = g_sdfTextures[thisObjectData.sdfTextureIndex].SampleLevel(g_samplerClamp, texCoord, 0);
    float sdfValue = 0;
            
    if (sdfValue <= 0.001)
    {
        // Hit detected
        ProceduralPrimitiveAttributes attr;
        attr.normal = float3(1, 0, 0);
                
        ReportHit(tEntry, 0, attr);
        return;
    }
    
    //if (tEntry < RayTCurrent())
    //{
    //    // Calculate surface normal
    //    float3 normal = float3(1, 0, 0);
    //    normal = texCoord * 2 - 1;
        
    //    ProceduralPrimitiveAttributes attr;
    //    attr.normal = normal;
        
    //    ReportHit(tEntry, 0, attr);
    //}
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

//[shader("intersection")]
//void Intersection_SDF()
//{
//    // --TESTING--
//    //ProceduralPrimitiveAttributes __attr;
//    //__attr.normal = float3(1, 0, 0);
//    //ReportHit(RayTMin(), 0, __attr);
//    //return;
    
//    SDFInstanceData thisInstanceData = g_sdfInstancesData[SDFInstanceIndex(InstanceIndex())];
//    SDFObjectData thisObjectData = g_sdfObjectsData[SDFObjectIndex(thisInstanceData.objectIndex)];
//    int primitiveIndex = PrimitiveIndex();
    
//    SurfaceVoxel voxel = g_sdfVoxels[thisObjectData.textureIndex][primitiveIndex];
    
//    float3 rayOrigin = ObjectRayOrigin();
//    float3 rayDir = ObjectRayDirection();
    
//    float gridSize = 1.0f;
//    float voxelSize = gridSize / 64.0f;
//    float halfWorld = 0.5f;
//    float3 voxelMin = float3(voxel.x, voxel.y, voxel.z) * voxelSize - halfWorld;
//    float3 voxelMax = voxelMin + voxelSize;
        
//    float3 invDir = 1.0 / rayDir;
//    float3 t0 = (voxelMin - rayOrigin) * invDir;
//    float3 t1 = (voxelMax - rayOrigin) * invDir;
//    float3 tMin3 = min(t0, t1);
//    float3 tMax3 = max(t0, t1);
    
//    float tEntry = max(max(tMin3.x, tMin3.y), tMin3.z);
//    float tExit = min(min(tMax3.x, tMax3.y), tMax3.z);
    
//    if (tEntry > tExit || tExit < RayTMin())
//        return;
    
//    tEntry = max(tEntry, RayTMin());
//    tExit = min(tExit, 1000);
    
    
//    //float corners[8] = { voxel.s000, voxel.s001, voxel.s010, voxel.s011, voxel.s100, voxel.s101, voxel.s110, voxel.s111 };
//    //float minVal = corners[0];
//    //float maxVal = corners[0];
//    //[unroll]
//    //for (int i = 1; i < 8; i++)
//    //{
//    //    minVal = min(minVal, corners[i]);
//    //    maxVal = max(maxVal, corners[i]);
//    //}
    
//    //// surface
//    //if (minVal <= 0 && maxVal >= 0)
//    //{
//    //    ProceduralPrimitiveAttributes __attr;
//    //    __attr.normal = float3(1, 0, 0);
//    //    ReportHit(tEntry, 0, __attr);
//    //}
//    //// padding
//    //else
//    //{
//    //    ProceduralPrimitiveAttributes __attr;
//    //    __attr.normal = float3(1, 1, 0);
//    //    ReportHit(tEntry, 0, __attr);
//    //}
//    //return;
    
    
//    //{
//    //    uint foundIndices[MAX_INSTANCES];
//    //    uint count = 0;
//    //    float3 influenceRange = float3(0.1, 0.1, 0.1);
        
//    //    float3 worldMin = mul(thisObjectData.world, float4(voxelMin, 1.0)).xyz;
//    //    float3 worldMax = mul(thisObjectData.world, float4(voxelMax, 1.0)).xyz;
        
//    //    TraverseInstanceBVH(worldMin - influenceRange, worldMax + influenceRange, foundIndices, count);
        
//    //    float3 p = rayOrigin + rayDir * tEntry;
//    //    float3 pWorld = mul(thisObjectData.world, float4(p, 1.0)).xyz;
        
//    //    MapResult mapResult;
        
//    //    [unroll]
//    //    for (int i = 0; i < 8; i++)
//    //    {
//    //        mapResult = map(pWorld, foundIndices, count);
//    //    }
        
//    //    if (mapResult.distance <= voxelSize)
//    //    {
//    //        ProceduralPrimitiveAttributes attr;
//    //        attr.normal = mapResult.normal;
                
//    //        ReportHit(tEntry + mapResult.distance / thisObjectData.scale, 0, attr);
//    //    }
        
//    //    return;
//    //}
    
    
    
    
//    //{
//    //    float3 p = rayOrigin + rayDir * tEntry;
//    //    // Convert to local coordinates within the voxel [0,1]³
//    //    float3 localCoord = (p - voxelMin) / voxelSize;
//    //    float u = localCoord.x;
//    //    float v = localCoord.y;
//    //    float w = localCoord.z;

//    //    // trilinear interpolation using stored corner values
//    //    float sdfValue =
//    //    (1 - u) * (1 - v) * (1 - w) * voxel.s000 +
//    //    u * (1 - v) * (1 - w) * voxel.s100 +
//    //    (1 - u) * v * (1 - w) * voxel.s010 +
//    //    u * v * (1 - w) * voxel.s110 +
//    //    (1 - u) * (1 - v) * w * voxel.s001 +
//    //    u * (1 - v) * w * voxel.s101 +
//    //    (1 - u) * v * w * voxel.s011 +
//    //    u * v * w * voxel.s111;
            
//    //    if (sdfValue <= 0.001)
//    //    {
//    //        //Hit detected
//    //        ProceduralPrimitiveAttributes attr;
//    //        attr.normal = float3(1, 0, 0);
                
//    //        ReportHit(tEntry + sdfValue, 0, attr);
//    //    }
//    //    return;
//    //}
//    {
        
//        float3 actualOrigin = rayOrigin;
//        if (tEntry > 0)
//        {
//            actualOrigin = rayOrigin + tEntry * rayDir;
//        }
        
//        float3 localOrigin = (actualOrigin - voxelMin) / voxelSize;
//        float3 localDir = rayDir / voxelSize;
//        float tFar = tExit - max(tEntry, 0);
        
//        float k0 = voxel.s000;
//        float k1 = voxel.s100 - voxel.s000;
//        float k2 = voxel.s010 - voxel.s000;
//        float a = voxel.s101 - voxel.s001;
//        float k4 = k0 - voxel.s001;
//        float k5 = k1 - a;
//        float k6 = k2 - (voxel.s011 - voxel.s001);
//        float k3 = voxel.s110 - voxel.s010 - k1;
//        float k7 = k3 - (voxel.s111 - voxel.s011 - a);
        
//        float ox = localOrigin.x, oy = localOrigin.y, oz = localOrigin.z;
//        float dx = localDir.x, dy = localDir.y, dz = localDir.z;
        
        
//        float m0 = ox * oy;
//        float m1 = dx * dy;
//        float m2 = ox * dy + oy * dx;
//        float m3 = k5 * oz - k1;
//        float m4 = k6 * oz - k2;
//        float m5 = k7 * oz - k3;
        
        
//        float c0 = (k4 * oz - k0) + ox * m3 + oy * m4 + m0 * m5;
//        float c1 = dx * m3 + dy * m4 + m2 * m5 + dz * (k4 + k5 * ox + k6 * oy + k7 * m0);
//        float c2 = m1 * m5 + dz * (k5 * dx + k6 * dy + k7 * m2);
//        float c3 = k7 * m1 * dz;
        
        
//        float t = SolveCubic(c3, c2, c1, c0, tFar);
        
//        if (t < 0 || t > tFar)
//        {
//            return;
//        }
            
        
//        ProceduralPrimitiveAttributes _attr;
//        //_attr.normal = float3(1, 0, 0);
//        _attr.normal = localOrigin;
//        ReportHit(max(tEntry, 0) + t, 0, _attr);
//        return;
//    }
    
    
    
//    //{
//    //    // Transform ray to canonical voxel space [0,1]³
//    //    float3 voxelRayOrigin = (rayOrigin + rayDir * tEntry - voxelMin) / voxelSize;
//    //    float3 voxelRayDir = rayDir / voxelSize;
//    //    float voxelTFar = (tExit - tEntry) / voxelSize;
    
//    //    // Calculate coefficients for trilinear interpolation
//    //    float k0 = voxel.s000;
//    //    float k1 = voxel.s100 - voxel.s000;
//    //    float k2 = voxel.s010 - voxel.s000;
//    //    float a = voxel.s101 - voxel.s001;
//    //    float k4 = k0 - voxel.s001;
//    //    float k5 = k1 - a;
//    //    float k6 = k2 - (voxel.s011 - voxel.s001);
//    //    float k3 = voxel.s110 - voxel.s010 - k1;
//    //    float k7 = k3 - (voxel.s111 - voxel.s011 - a);
    
    
    
//    //    // Ray components in voxel space
//    //    float ox = voxelRayOrigin.x, oy = voxelRayOrigin.y, oz = voxelRayOrigin.z;
//    //    float dx = voxelRayDir.x, dy = voxelRayDir.y, dz = voxelRayDir.z;
    
//    //    // Calculate intermediate values (from paper, Equations 6-7)
//    //    float m0 = ox * oy;
//    //    float m1 = dx * dy;
//    //    float m2 = ox * dy + oy * dx;
//    //    float m3 = k5 * oz - k1;
//    //    float m4 = k6 * oz - k2;
//    //    float m5 = k7 * oz - k3;

//    //    // Calculate cubic coefficients c3*t³ + c2*t² + c1*t + c0 = 0 (Equation 5)
//    //    float c0 = (k4 * oz - k0) + ox * m3 + oy * m4 + m0 * m5;
//    //    float c1 = dx * m3 + dy * m4 + m2 * m5 + dz * (k4 + k5 * ox + k6 * oy + k7 * m0);
//    //    float c2 = m1 * m5 + dz * (k5 * dx + k6 * dy + k7 * m2);
//    //    float c3 = k7 * m1 * dz;
    
    
//    //    float t = SolveCubic(c3, c2, c1, c0, voxelTFar);
//    //    if (t < 0)
//    //        return;

//    //    // If we get here, cubic solver found something
//    //    ProceduralPrimitiveAttributes _attr;
//    //    _attr.normal = float3(1, 0, 0); // Red for "cubic root found"
//    //    float objectT = tEntry + t * voxelSize;
//    //    ReportHit(objectT, 0, _attr);
//    //}
    
    
//    // Check if this voxel contains surface
//    //if (voxel.s000 <= 0.0f)
//    //{
//    //    // Calculate ray-AABB intersection for hit distance
//    //    float3 rayOrigin = ObjectRayOrigin();
//    //    float3 rayDir = ObjectRayDirection();
        
//    //    // You'll need voxel bounds - calculate from voxel.x, voxel.y, voxel.z
//    //    float voxelSize = 1.0f / 64.0f;
//    //    float halfWorld = 0.5f;
//    //    float3 voxelMin = float3(voxel.x, voxel.y, voxel.z) * voxelSize - halfWorld;
//    //    float3 voxelMax = voxelMin + voxelSize;
        
//    //    float3 invDir = 1.0 / rayDir;
//    //    float3 t0 = (voxelMin - rayOrigin) * invDir;
//    //    float3 t1 = (voxelMax - rayOrigin) * invDir;
//    //    float3 tMin3 = min(t0, t1);
//    //    float tEntry = max(max(tMin3.x, tMin3.y), tMin3.z);
//    //    tEntry = max(tEntry, RayTMin());
        
//    //    ProceduralPrimitiveAttributes attr;
//    //    attr.normal = float3(1, 0, 0);
        
//    //    ReportHit(tEntry, 0, attr);
//    //}
//}

//void GetVoxelCorners(uint sliceStart, int3 voxelPos, out float corners[8])
//{
//    // Texture coordinates for gather (add 0.5 for texel centers)
//    float2 texCoord = float2(voxelPos.xy) + float2(0.5, 0.5);
//    float2 normalizedCoord = texCoord / 8.0; // Normalize to [0,1]
    
//    // First gather: z slice
//    float4 gather0 = g_brickAtlas.Gather(
//        g_samplerPointClamp,
//        float3(normalizedCoord, sliceStart + voxelPos.z)
//    );
    
//    // Second gather: z+1 slice  
//    float4 gather1 = g_brickAtlas.Gather(
//        g_samplerPointClamp,
//        float3(normalizedCoord, sliceStart + voxelPos.z + 1)
//    );
    
//    // Map gather results to corners
//    corners[0] = gather0.w; // (x,y,z)
//    corners[1] = gather0.z; // (x+1,y,z)
//    corners[2] = gather0.x; // (x,y+1,z)
//    corners[3] = gather0.y; // (x+1,y+1,z)
//    corners[4] = gather1.w; // (x,y,z+1)
//    corners[5] = gather1.z; // (x+1,y,z+1)
//    corners[6] = gather1.x; // (x,y+1,z+1)
//    corners[7] = gather1.y; // (x+1,y+1,z+1)
//}
//[shader("intersection")]
//void _____Intersection_SDF()
//{
//    //{
//    //    // --TESTING--
//    //    ProceduralPrimitiveAttributes __attr;
//    //    __attr.normal = float3(1, 0, 0);
//    //    ReportHit(RayTMin(), 0, __attr);
//    //    return;
//    //}
    
//    int primitiveIndex = PrimitiveIndex();
//    SDFInstanceData thisInstanceData = g_sdfInstancesData[SDFInstanceIndex(InstanceIndex())];
//    SDFObjectData thisObjectData = g_sdfObjectsData[SDFObjectIndex(thisInstanceData.objectIndex)];
//    uint globalBrickIndex = thisObjectData.firstBrickIndex + primitiveIndex;
//    SDFBrickData thisBrickData = g_brickTable[globalBrickIndex];
    
//    uint sliceStart = globalBrickIndex * 8;
    
    
//    float3 rayOrigin = ObjectRayOrigin();
//    float3 rayDir = ObjectRayDirection();
    
//    const float VOXEL_SIZE = 1.0f / 64.0f;
//    const float HALF_WORLD = 0.5f;
//    const float BRICK_SIZE = 7.0 * VOXEL_SIZE;
    
//    float3 brickMin = float3(thisBrickData.brickCoord.x, thisBrickData.brickCoord.y, thisBrickData.brickCoord.z) * BRICK_SIZE - HALF_WORLD;
//    float3 brickMax = brickMin + BRICK_SIZE;
        
//    float3 invDir = 1.0 / rayDir;
//    float3 t0 = (brickMin - rayOrigin) * invDir;
//    float3 t1 = (brickMax - rayOrigin) * invDir;
//    float3 tMin3 = min(t0, t1);
//    float3 tMax3 = max(t0, t1);
    
//    float tEntry = max(max(tMin3.x, tMin3.y), tMin3.z);
//    float tExit = min(min(tMax3.x, tMax3.y), tMax3.z);
    
//    if (tEntry > tExit || tExit < RayTMin())
//        return;
    
//    tEntry = max(tEntry, RayTMin());
//    tExit = min(tExit, 1000);
    

    
//    //{   
//    //    //ProceduralPrimitiveAttributes attr;
//    //    //float3 p = rayOrigin + rayDir * tEntry;
//    //    //attr.normal = (p - brickMin) / BRICK_SIZE;
//    //    //ReportHit(tEntry, 0, attr);
//    //    //return;
        
        
//    //    float bestT = 1e30;
//    //    int3 bestVoxel = int3(-1, -1, -1);
//    //    float3 bestHit = float3(0, 0, 0);

//    //    for (int z = 0; z < 7; z++)
//    //        for (int y = 0; y < 7; y++)
//    //            for (int x = 0; x < 7; x++)
//    //            {
//    //                int3 voxelIndex = int3(x, y, z);
//    //                float3 voxelMin = brickMin + float3(voxelIndex) * VOXEL_SIZE;
//    //                float3 voxelMax = voxelMin + VOXEL_SIZE;

//    //                float3 vt0 = (voxelMin - rayOrigin) * invDir;
//    //                float3 vt1 = (voxelMax - rayOrigin) * invDir;
//    //                float3 vtMin = min(vt0, vt1);
//    //                float3 vtMax = max(vt0, vt1);

//    //                float voxelTEntry = max(max(vtMin.x, vtMin.y), vtMin.z);
//    //                float voxelTExit = min(min(vtMax.x, vtMax.y), vtMax.z);

//    //                if (voxelTEntry <= voxelTExit && voxelTExit >= RayTMin())
//    //                {
//    //                    float corners[8];
//    //                    GetVoxelCorners(sliceStart, voxelIndex, corners);
                        
//    //                    float minVal = corners[0];
//    //                    float maxVal = corners[0];
//    //                    [unroll]
//    //                    for (int i = 1; i < 8; i++)
//    //                    {
//    //                        minVal = min(minVal, corners[i]);
//    //                        maxVal = max(maxVal, corners[i]);
//    //                    }
    
//    //                    if (minVal <= 0 && maxVal >= 0)
//    //                    {
//    //                        float3 localP = rayOrigin + voxelTEntry * rayDir;
//    //                        float3 localOrigin = (localP - voxelMin) / VOXEL_SIZE;
//    //                        float3 localDir = rayDir / VOXEL_SIZE;
//    //                        float localTFar = (voxelTExit - max(voxelTEntry, 0));
                            
//    //                        // Compute k constants from corners
//    //                        float k0 = corners[0]; // s000
//    //                        float k1 = corners[1] - corners[0]; // s100 - s000
//    //                        float k2 = corners[2] - corners[0]; // s010 - s000
//    //                        float a = corners[5] - corners[4]; // s101 - s001
//    //                        float k4 = k0 - corners[4]; // s000 - s001
//    //                        float k5 = k1 - a;
//    //                        float k6 = k2 - (corners[6] - corners[4]); // k2 - (s011 - s001)
//    //                        float k3 = corners[3] - corners[2] - k1; // s110 - s010 - k1
//    //                        float k7 = k3 - (corners[7] - corners[6] - a); // k3 - (s111 - s011 - a)
        
//    //                        // Compute m constants
//    //                        float ox = localOrigin.x, oy = localOrigin.y, oz = localOrigin.z;
//    //                        float dx = localDir.x, dy = localDir.y, dz = localDir.z;
        
//    //                        float m0 = ox * oy;
//    //                        float m1 = dx * dy;
//    //                        float m2 = ox * dy + oy * dx;
//    //                        float m3 = k5 * oz - k1;
//    //                        float m4 = k6 * oz - k2;
//    //                        float m5 = k7 * oz - k3;
        
//    //                        // Compute cubic coefficients
//    //                        float c0 = (k4 * oz - k0) + ox * m3 + oy * m4 + m0 * m5;
//    //                        float c1 = dx * m3 + dy * m4 + m2 * m5 + dz * (k4 + k5 * ox + k6 * oy + k7 * m0);
//    //                        float c2 = m1 * m5 + dz * (k5 * dx + k6 * dy + k7 * m2);
//    //                        float c3 = k7 * m1 * dz;
        
//    //                        // Solve cubic
//    //                        float t = SolveCubic(c3, c2, c1, c0, localTFar);
        
//    //                        if (t >= 0 && t <= localTFar)
//    //                        {
//    //                            if (voxelTEntry < bestT)
//    //                            {
//    //                                bestT = voxelTEntry;
//    //                                bestVoxel = voxelIndex;
//    //                                bestHit = rayOrigin + voxelTEntry * rayDir;
//    //                            }
//    //                        }
//    //                    }
                        
//    //                }
//    //            }

//    //    if (bestT < 1e30)
//    //    {
//    //        float3 voxelMin = brickMin + float3(bestVoxel) * VOXEL_SIZE;
//    //        float3 localUVW = (bestHit - voxelMin) / VOXEL_SIZE;

//    //        ProceduralPrimitiveAttributes attr;
//    //        attr.normal = saturate(localUVW); // Use UVW as debug normal
//    //        ReportHit(bestT, 0, attr);
//    //        return;
//    //    }

//    //}
//    //return;
    
    
    
    
    
    
    
//    // DDA setup
//    float3 entryPoint = rayOrigin + tEntry * rayDir;
//    float3 voxelPosFloat = (entryPoint - brickMin) / VOXEL_SIZE;
//    int3 voxelPos = clamp(int3(voxelPosFloat), 0, 6); // 0-6 range for 7x7x7
    
//    float3 tDelta = abs(VOXEL_SIZE / rayDir);
//    float3 tMax;
//    int3 step;
    
//    // Initialize DDA
//    [unroll]
//    for (int i = 0; i < 3; i++)
//    {
//        if (rayDir[i] > 0)
//        {
//            tMax[i] = tEntry + ((voxelPos[i] + 1) * VOXEL_SIZE + brickMin[i] - entryPoint[i]) / rayDir[i];
//            step[i] = 1;
//        }
//        else if (rayDir[i] < 0)
//        {
//            tMax[i] = tEntry + (voxelPos[i] * VOXEL_SIZE + brickMin[i] - entryPoint[i]) / rayDir[i];
//            step[i] = -1;
//        }
//        else
//        {
//            tMax[i] = 1e30;
//            step[i] = 0;
//        }
//    }
    
//    // DDA traversal
//    float currentT = tEntry;
//    const int maxSteps = 21; // Maximum 7 steps per axis

//    for (int iter = 0; iter < maxSteps && currentT < tExit; iter++)
//    {
//        // Get 8 corner values for current voxel from texture atlas
//        float corners[8];
//        GetVoxelCorners(sliceStart, voxelPos, corners);
    
//        // Check if surface crosses this voxel
//        float minVal = corners[0];
//        float maxVal = corners[0];
//        [unroll]
//        for (int i = 1; i < 8; i++)
//        {
//            minVal = min(minVal, corners[i]);
//            maxVal = max(maxVal, corners[i]);
//        }
    
//        if (minVal <= 0 && maxVal >= 0)
//        {
//            // Surface in this voxel! Calculate exact voxel bounds
//            float3 voxelMin = brickMin + float3(voxelPos) * VOXEL_SIZE;
//            float3 voxelMax = voxelMin + VOXEL_SIZE;
    
//            // Calculate entry/exit for this specific voxel
//            float3 vt0 = (voxelMin - rayOrigin) * invDir;
//            float3 vt1 = (voxelMax - rayOrigin) * invDir;
//            float3 vtMin = min(vt0, vt1);
//            float3 vtMax = max(vt0, vt1);
//            float voxelTEntry = max(max(vtMin.x, vtMin.y), vtMin.z);
//            float voxelTExit = min(min(vtMax.x, vtMax.y), vtMax.z);
    
//            // Clamp to current traversal bounds
//            voxelTEntry = max(voxelTEntry, currentT);
//            voxelTExit = min(voxelTExit, tExit);
    
//            if (voxelTEntry < voxelTExit)
//            {
//                // Advance origin if needed
//                float3 actualOrigin = rayOrigin;
//                if (voxelTEntry > 0)
//                {
//                    actualOrigin = rayOrigin + voxelTEntry * rayDir;
//                }
        
//                // Transform to canonical voxel space [0,1]³
//                float3 voxelHitPoint = clamp(rayOrigin + voxelTEntry * rayDir, voxelMin, voxelMax);
//                float3 localOrigin = (voxelHitPoint - voxelMin) / VOXEL_SIZE;
                
//                //float3 localOrigin = (actualOrigin - voxelMin) / VOXEL_SIZE;
//                float3 localDir = rayDir / VOXEL_SIZE;
//                float localTFar = (voxelTExit - max(voxelTEntry, 0));
                
//                //{
//                //    ProceduralPrimitiveAttributes attr;
//                //    attr.normal = localOrigin;
//                //    ReportHit(max(voxelTEntry, 0), 0, attr);
//                //    return;
//                //}
        
//                // Compute k constants from corners
//                float k0 = corners[0]; // s000
//                float k1 = corners[1] - corners[0]; // s100 - s000
//                float k2 = corners[2] - corners[0]; // s010 - s000
//                float a = corners[5] - corners[4]; // s101 - s001
//                float k4 = k0 - corners[4]; // s000 - s001
//                float k5 = k1 - a;
//                float k6 = k2 - (corners[6] - corners[4]); // k2 - (s011 - s001)
//                float k3 = corners[3] - corners[2] - k1; // s110 - s010 - k1
//                float k7 = k3 - (corners[7] - corners[6] - a); // k3 - (s111 - s011 - a)
        
//                // Compute m constants
//                float ox = localOrigin.x, oy = localOrigin.y, oz = localOrigin.z;
//                float dx = localDir.x, dy = localDir.y, dz = localDir.z;
        
//                float m0 = ox * oy;
//                float m1 = dx * dy;
//                float m2 = ox * dy + oy * dx;
//                float m3 = k5 * oz - k1;
//                float m4 = k6 * oz - k2;
//                float m5 = k7 * oz - k3;
        
//                // Compute cubic coefficients
//                float c0 = (k4 * oz - k0) + ox * m3 + oy * m4 + m0 * m5;
//                float c1 = dx * m3 + dy * m4 + m2 * m5 + dz * (k4 + k5 * ox + k6 * oy + k7 * m0);
//                float c2 = m1 * m5 + dz * (k5 * dx + k6 * dy + k7 * m2);
//                float c3 = k7 * m1 * dz;
        
//                // Solve cubic
//                float t = SolveCubic(c3, c2, c1, c0, localTFar);
        
//                if (t >= 0 && t <= localTFar)
//                {
//                    // Found valid intersection!
//                    float worldHitT = (max(voxelTEntry, 0) + t);
            
//                    ProceduralPrimitiveAttributes attr;
//                    attr.normal = float3(1, 0, 1); // TODO: Compute actual normal
//                    ReportHit(worldHitT, 0, attr);
//                    return;
//                }
//            }
//        }
    
//        // Step to next voxel using DDA
//        if (tMax.x < tMax.y)
//        {
//            if (tMax.x < tMax.z)
//            {
//                currentT = tMax.x;
//                tMax.x += tDelta.x;
//                voxelPos.x += step.x;
//                if (voxelPos.x < 0 || voxelPos.x >= 7)
//                    break;
//            }
//            else
//            {
//                currentT = tMax.z;
//                tMax.z += tDelta.z;
//                voxelPos.z += step.z;
//                if (voxelPos.z < 0 || voxelPos.z >= 7)
//                    break;
//            }
//        }
//        else
//        {
//            if (tMax.y < tMax.z)
//            {
//                currentT = tMax.y;
//                tMax.y += tDelta.y;
//                voxelPos.y += step.y;
//                if (voxelPos.y < 0 || voxelPos.y >= 7)
//                    break;
//            }
//            else
//            {
//                currentT = tMax.z;
//                tMax.z += tDelta.z;
//                voxelPos.z += step.z;
//                if (voxelPos.z < 0 || voxelPos.z >= 7)
//                    break;
//            }
//        }
//    }
    
//    //ProceduralPrimitiveAttributes attr;
//    //attr.normal = float3(1, 1, 0); // TODO: Compute actual normal
//    //ReportHit(tEntry, 0, attr);
//    //return;
    
//    return;
    
//    {
//        RayDesc rayDesc;
//        rayDesc.Origin = WorldRayOrigin();
//        rayDesc.Direction = WorldRayDirection();
//        rayDesc.TMin = tEntry;
//        rayDesc.TMax = 1000;
    
//        RayQuery < RAY_FLAG_FORCE_OPAQUE > q;
//        q.TraceRayInline(
//            g_scene,
//            RAY_FLAG_NONE, // or RAY_FLAG_CULL_BACK_FACING_TRIANGLES
//            0x04, // Triangle mask
//            rayDesc
//        );
//        // Process all intersections
//        while (q.Proceed())
//        {
//            // Could handle candidates here if needed
//        }
    
//        if (q.CommittedStatus() == COMMITTED_TRIANGLE_HIT)
//        {
//            float polyT = q.CommittedRayT();
        
//            uint tri_primitiveIndex = q.CommittedPrimitiveIndex();
//            uint tri_instanceIndex = q.CommittedInstanceIndex();
//            uint tri_instanceID = q.CommittedInstanceID();
                
//            Triangle polyTri = TriangleData(tri_instanceID, tri_primitiveIndex, g_indexVertexBuffers);
                
                
//            // Transform the hit point to object space of that triangle instance
//            float3 worldHitPoint = rayDesc.Origin + rayDesc.Direction * polyT;
//            float3 objectHitPoint = mul(thisInstanceData.worldI, float4(worldHitPoint, 1.0)).xyz;
            
//            float3 bary = Barycentrics(polyTri, objectHitPoint);
                
//            float3 polyNormal = normalize(polyTri.v0.normal * bary.x + polyTri.v1.normal * bary.y + polyTri.v2.normal * bary.z);
        
//            ProceduralPrimitiveAttributes __attr;
//            __attr.normal = polyNormal;
//            ReportHit(polyT, 0, __attr);
//            return;
//        }
//    }
//}