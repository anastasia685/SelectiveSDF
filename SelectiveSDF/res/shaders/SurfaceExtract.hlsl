struct CandidateVoxel
{
    uint x, y, z;
    uint instanceIndex;
    float s000, s100, s010, s110, s001, s101, s011, s111;
};
struct VoxelAABB
{
    float3 min;
    float3 max;
    float2 padding;
};
struct BrickAAB
{
    float3 min;
    float3 max;
    float2 padding;
};
struct ComputeConstantBuffer
{
    uint triangleObjectCount;
    uint sdfObjectCount;
    uint triangleInstanceCount;
    uint sdfInstanceCount;
    uint frameIndex;
    uint padding[3];
};
struct BrickMeta
{
    uint packedCoord;
    uint instanceIndex;
    uint sliceStart;
    uint padding;
};
struct SDFObjectData
{
    int primitiveType;
    uint textureIndex;
    uint firstBrickIndex;
    uint brickCount;
};
struct SDFInstanceData
{
    float4x4 world;
    float4x4 worldI;
    float scale;
    uint objectIndex;
    uint _instanceIndex;
    float padding;
};
struct BVHNode
{
    float3 min;
    uint leftChild; // or ~0 for leaf

    float3 max;
    uint rightChild; // or ~0 for leaf

    uint firstInstance;
    uint instanceCount;
    float2 padding; // pad to 64 bytes
};
struct NodeInstanceIndex
{
    int index;
    float3 padding;
};

RWTexture2DArray<float4> g_outputBrickTextures[] : register(u1, space0);
RWStructuredBuffer<uint> g_brickMask : register(u0);
StructuredBuffer<BrickMeta> g_candidateBricks : register(t0);
StructuredBuffer<SDFObjectData> g_sdfObjectsData : register(t1);
StructuredBuffer<SDFInstanceData> g_sdfInstancesData : register(t2);
StructuredBuffer<BVHNode> g_bvhNodes : register(t3);
StructuredBuffer<NodeInstanceIndex> g_instanceIndices : register(t4);
//StructuredBuffer<uint> g_brickVisibility : register(t5);
//StructuredBuffer<uint> g_filteredBricks : register(t6);
Texture3D<float> g_sdfTextures[] : register(t7, space0);
ConstantBuffer<ComputeConstantBuffer> g_computeCB : register(b0);
SamplerState g_samplerLinearClamp : register(s0);


#define BRICK_RES 10  // 10x10x10 data points
#define BRICK_SIZE 9
#define MAX_INSTANCES 16

uint3 UnpackBrickCoord(uint packedCoord)
{
    return uint3(packedCoord & 0xFF, (packedCoord >> 8) & 0xFF, (packedCoord >> 16) & 0xFF);
}
uint SDFObjectIndex(uint objectIndex)
{
    return objectIndex - g_computeCB.triangleObjectCount;
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
    
    float3 uvw = localPos + float3(0.5, 0.5, 0.5); // shift to [0, 1] range
    dist += sdfTexture.SampleLevel(g_samplerLinearClamp, uvw + texelSize, 0); // sample in the middle of the voxel
    
    return dist;
}
float3 boxNormal(float3 p, float3 b)
{
    float3 d = abs(p) - b;
    
    // outside the box
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
    float h = max(k - abs(a - b), 0.0) / k;
    float m = h * h * 0.5;
    float s = m * k * (1.0 / 2.0);
    return (a < b) ? float2(a - s, m) : float2(b - s, 1.0 - m);
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
    
    float unionBlendDist = -log(unionExpSum + 1e-6) / kUnion;
    unionNormal = normalize(unionNormal / unionExpSum);
    
    result.distance = unionBlendDist;
    result.normal = unionNormal;
    
    return result;
}

groupshared uint hasPositive;
groupshared uint hasNegative;

[numthreads(BRICK_RES, BRICK_RES, BRICK_RES)]
void CS(uint3 threadID : SV_DispatchThreadID, uint groupIndex : SV_GroupIndex, uint3 groupID : SV_GroupID, uint3 groupThreadID : SV_GroupThreadID)
{
    uint brickIndex = groupID.x; // Each thread group corresponds to a single brick
    
    //uint brickFrameIndex = g_brickVisibility[brickIndex];
    //if (brickFrameIndex != g_computeCB.frameIndex - 1)
    //{
    //    g_brickMask[brickIndex] = 0; // not really necessary, idk
    //    return; // skip if brick is not visible
    //}
    
    
    if (groupIndex == 0) // groupIndex = linear thread ID inside group
    {
        hasPositive = 0;
        hasNegative = 0;
    }
    GroupMemoryBarrierWithGroupSync(); // ensure all threads see the initialized values
    
    BrickMeta meta = g_candidateBricks[brickIndex];

    uint3 brickCoord = UnpackBrickCoord(meta.packedCoord);
    uint sdfInstanceIndex = meta.instanceIndex - g_computeCB.triangleInstanceCount; // Adjust for triangle instances
    
    SDFInstanceData sdfInstanceData = g_sdfInstancesData[sdfInstanceIndex];
    SDFObjectData sdfObjectData = g_sdfObjectsData[SDFObjectIndex(sdfInstanceData.objectIndex)];
    
    // local coord inside brick (0-9)^3
    uint3 local = groupThreadID;

    // convert brick space to dense grid texel space
    uint GRID_SIZE = 64;
    uint3 denseTexelCoord = int3(brickCoord * BRICK_SIZE) + int3(local);
    float3 localPos = float3(denseTexelCoord) / GRID_SIZE - float3(0.5f, 0.5f, 0.5f);
    float3 worldPos = mul(sdfInstanceData.world, float4(localPos, 1.0)).xyz;
    
    float3 worldMin = mul(sdfInstanceData.world, float4(-0.5, -0.5, -0.5, 1)).xyz;
    float3 worldMax = mul(sdfInstanceData.world, float4( 0.5,  0.5,  0.5, 1)).xyz;

    // Sample original dense SDF
    //float sdf = g_sdfTextures[sdfObjectData.textureIndex].Load(int4(denseTexelCoord, 0));
    //if (sdf >= 0.0f)
    //    InterlockedOr(hasPositive, 1);
    //else if (sdf < 0.0f)
    //    InterlockedOr(hasNegative, 1);
    //g_outputBrickTextures[sdfInstanceIndex][uint3(local.xy, meta.sliceStart + local.z)] = float4(sdf, float3(1, 0, 0));
    
    uint foundIndices[MAX_INSTANCES];
    uint count = 0;
    TraverseInstanceBVH(worldMin, worldMax, foundIndices, count);
    MapResult mapResult = map(worldPos, foundIndices, count);
    
    if (mapResult.distance >= 0.0f)
        InterlockedOr(hasPositive, 1);
    else if (mapResult.distance < 0.0f)
        InterlockedOr(hasNegative, 1);

    g_outputBrickTextures[sdfInstanceIndex][uint3(local.xy, meta.sliceStart + local.z)] = float4(mapResult.distance, mapResult.normal);
    
    GroupMemoryBarrierWithGroupSync(); // wait for all threads to finish writing

    if (groupIndex == 0)
    {
        uint hasSurface = (hasPositive & hasNegative) ? 1 : 0;
        g_brickMask[brickIndex] = hasSurface;
    }
}