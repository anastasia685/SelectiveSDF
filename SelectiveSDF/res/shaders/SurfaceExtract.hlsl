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
    //uint candidateVoxelCount;
    //float3 padding;
    uint triangleObjectCount;
    uint sdfObjectCount;
    uint triangleInstanceCount;
    uint sdfInstanceCount;
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

//RWStructuredBuffer<VoxelAABB> outputAABBs : register(u0);
RWTexture2DArray<float4> g_outputBrickTextures[] : register(u0, space0);
//StructuredBuffer<uint> candidateVoxels : register(t0);
StructuredBuffer<BrickMeta> g_candidateBricks : register(t0);
StructuredBuffer<SDFObjectData> g_sdfObjectsData : register(t1);
StructuredBuffer<SDFInstanceData> g_sdfInstancesData : register(t2);
Texture3D<float> g_sdfTextures[] : register(t3, space0);
ConstantBuffer<ComputeConstantBuffer> g_computeCB : register(b0);
SamplerState g_samplerLinearClamp : register(s0);

//[numthreads(64, 1, 1)]
//void CS( uint3 DTid : SV_DispatchThreadID )
//{   
//    uint index = DTid.x;

//    if (index >= g_computeCB.candidateVoxelCount)
//        return;
    
    
//    //{
//    //    VoxelAABB box;
//    //    box.min = float3(0, 0, 0);
//    //    box.max = float3(0.001, 0, 0);
//    //    box.padding = float2(0, 0);

//    //    outputAABBs[index] = box;
//    //    return;
//    //}
    
//    uint candidateVoxelData = candidateVoxels[index];
    
//    uint3 voxelIdx = uint3(
//        candidateVoxelData & 0x3F, // bits 0-5 (0x3F = 63)
//        (candidateVoxelData >> 6) & 0x3F, // bits 6-11
//        (candidateVoxelData >> 12) & 0x3F // bits 12-17
//    );
//    // instanceIndex would be: candidateVoxelData >> 18

//    float voxelSize = 1.0f / 64.0f;
//    float3 gridOrigin = float3(-0.5f, -0.5f, -0.5f);
    
//    float3 voxelPos = float3(voxelIdx) * voxelSize + gridOrigin;

//    VoxelAABB box;
//    box.min = voxelPos;
//    box.max = voxelPos + voxelSize;
//    box.padding = float2(0, 0);

//    outputAABBs[index] = box;
//}

#define BRICK_RES 10  // 10x10x10 data points
#define BRICK_SIZE 9

uint3 UnpackBrickCoord(uint packedCoord)
{
    return uint3(packedCoord & 0xFF, (packedCoord >> 8) & 0xFF, (packedCoord >> 16) & 0xFF);
}
uint SDFObjectIndex(uint objectIndex)
{
    return objectIndex - g_computeCB.triangleObjectCount;
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
struct MapResult
{
    float distance;
    float3 normal;
};
MapResult map(float3 p)
{
    const float kUnion = 10;
    const float kSubtraction = 60;
    
    MapResult result;
    
    float unionExpSum;
    float3 unionNormal = float3(0, 0, 0);
    
    float subtractionExpSum;
    float3 subtractionNormal = float3(0, 0, 0);
    
    for (uint i = 0; i < g_computeCB.sdfInstanceCount; i++)
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
            float2 unionRes = smin(unionExpSum, dist, 0.23f);
            unionExpSum = unionRes.x;
            unionNormal = normalize(lerp(unionNormal, normal, unionRes.y));
            
            //float2 subtractionRes = -smin(-subtractionExpSum, -dist, 0.3f);
            //subtractionExpSum = subtractionRes.x;
            //subtractionNormal = normalize(lerp(subtractionNormal, normal, subtractionRes.y));
            
            // just max for maximizing "carved out" area
            if (dist > subtractionExpSum)
            {
                subtractionExpSum = dist;
                subtractionNormal = normal;
            }
        }
        
    }
    
    float unionBlendDist = unionExpSum;
    float subtractionBlendDist = subtractionExpSum;
    
    
    float2 blendResult = -smin(-unionBlendDist, subtractionBlendDist, 0.27f);
    
    result.distance = unionBlendDist;
    result.normal = unionNormal;
    
    return result;
}

[numthreads(BRICK_RES, BRICK_RES, BRICK_RES)]
void CS(uint3 threadID : SV_DispatchThreadID, uint groupIndex : SV_GroupIndex, uint3 groupID : SV_GroupID, uint3 groupThreadID : SV_GroupThreadID)
{
    uint brickIndex = groupID.x; // Each thread group corresponds to a single brick
    BrickMeta meta = g_candidateBricks[brickIndex];

    uint3 brickCoord = UnpackBrickCoord(meta.packedCoord);
    uint sdfInstanceIndex = meta.instanceIndex - g_computeCB.triangleInstanceCount; // Adjust for triangle instances
    
    SDFInstanceData sdfInstanceData = g_sdfInstancesData[sdfInstanceIndex];
    SDFObjectData sdfObjectData = g_sdfObjectsData[SDFObjectIndex(sdfInstanceData.objectIndex)];
    
    // Local coord inside brick (0-9)^3
    uint3 local = groupThreadID;

    // Convert brick space to dense grid texel space
    uint3 denseTexelCoord = int3(brickCoord * BRICK_SIZE) + int3(local);
    float3 localPos = float3(denseTexelCoord) / 64 - float3(0.5f, 0.5f, 0.5f);
    float3 worldPos = mul(sdfInstanceData.world, float4(localPos, 1.0)).xyz;

    // Sample original dense SDF
    //float sdf = g_sdfTextures[sdfObjectData.textureIndex].Load(int4(denseTexelCoord, 0));
    MapResult mapResult = map(worldPos);

    // Write to output Texture2DArray
    g_outputBrickTextures[sdfInstanceIndex][uint3(local.xy, meta.sliceStart + local.z)] = float4(mapResult.distance, mapResult.normal);
}