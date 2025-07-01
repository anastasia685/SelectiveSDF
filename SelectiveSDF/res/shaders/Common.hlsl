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
struct Triangle
{
    Vertex v0;
    Vertex v1;
    Vertex v2;
};
#define INDEX_SIZE 4
#define VERTEX_SIZE 24

struct SceneConstantBuffer
{
    float4x4 viewI;
    float4x4 projectionI;
    uint sdfInstanceCount;
};
struct SDFObjectData
{
    float4x4 world;
    float4x4 worldI;
    float scale;
    int sdfPrimitiveType; // why not
    int sdfTextureIndex;
    int _instanceIndex;
};
struct HashTableEntry
{
    int3 cellPos;
    uint indexOffset;
    uint count;
    uint occupied;
    float2 padding;
};
struct InstanceIndex
{
    int index;
    float3 padding;
};

#define MAX_RAY_RECURSION_DEPTH 1

float3 BackgroundColor()
{
    uint2 launchIndex = DispatchRaysIndex().xy;
    float2 dims = float2(DispatchRaysDimensions().xy);

    float ramp = launchIndex.y / dims.y;
    return float3(0.0f, 0.2f, 0.7f - 0.3f * ramp);
}

Triangle TriangleData(uint instanceID, uint triangleIndex, ByteAddressBuffer indexVertexBuffers[])
{
    Triangle tri;
    
    uint indexBufferIndex = instanceID * 2;
    uint vertexBufferIndex = instanceID * 2 + 1;
    
    uint3 indices = indexVertexBuffers[NonUniformResourceIndex(indexBufferIndex)].Load3(triangleIndex * 3 * INDEX_SIZE);
    
    // Load vertices (assuming Vertex is 24 bytes: float3 pos + float3 normal)
    tri.v0 = indexVertexBuffers[NonUniformResourceIndex(vertexBufferIndex)].Load<Vertex>(indices.x * VERTEX_SIZE);
    tri.v1 = indexVertexBuffers[NonUniformResourceIndex(vertexBufferIndex)].Load<Vertex>(indices.y * VERTEX_SIZE);
    tri.v2 = indexVertexBuffers[NonUniformResourceIndex(vertexBufferIndex)].Load<Vertex>(indices.z * VERTEX_SIZE);
    
    return tri;
}

float3 Barycentrics(Triangle tri, float3 p)
{
    float3 v0v1 = tri.v1.position - tri.v0.position;
    float3 v0v2 = tri.v2.position - tri.v0.position;
    float3 v0p = p - tri.v0.position;
    
    float d00 = dot(v0v1, v0v1);
    float d01 = dot(v0v1, v0v2);
    float d11 = dot(v0v2, v0v2);
    float d20 = dot(v0p, v0v1);
    float d21 = dot(v0p, v0v2);
    
    float denom = d00 * d11 - d01 * d01;
    float v = (d11 * d20 - d01 * d21) / denom;
    float w = (d00 * d21 - d01 * d20) / denom;
    float u = 1.0f - v - w;
    
    return float3(u, v, w);
}