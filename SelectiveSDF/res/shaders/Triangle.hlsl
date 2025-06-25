#include "Common.hlsl"

// Triangle resources
ByteAddressBuffer g_indexVertexBuffers[] : register(t1, space0);

[shader("closesthit")]
void ClosestHit_Triangle(inout RayPayload rayPayload, in BuiltInTriangleIntersectionAttributes attr)
{
    uint instanceID = InstanceID();
    uint indexBufferIndex = instanceID * 2;
    uint vertexBufferIndex = instanceID * 2 + 1;
    
    uint triangleIndex = PrimitiveIndex();
    
    // Calculate barycentric coordinates
    float3 barycentrics = float3(1.f - attr.barycentrics.x - attr.barycentrics.y,
                                 attr.barycentrics.x,
                                 attr.barycentrics.y);
    
    // Load indices (each index is 4 bytes)
    uint3 indices = g_indexVertexBuffers[NonUniformResourceIndex(indexBufferIndex)].Load3(triangleIndex * 3 * INDEX_SIZE);
    
    // Load vertices (assuming Vertex is 24 bytes: float3 pos + float3 normal)
    Vertex v0 = g_indexVertexBuffers[NonUniformResourceIndex(vertexBufferIndex)].Load<Vertex>(indices.x * VERTEX_SIZE);
    Vertex v1 = g_indexVertexBuffers[NonUniformResourceIndex(vertexBufferIndex)].Load<Vertex>(indices.y * VERTEX_SIZE);
    Vertex v2 = g_indexVertexBuffers[NonUniformResourceIndex(vertexBufferIndex)].Load<Vertex>(indices.z * VERTEX_SIZE);
    
    float3 hitColor = (v0.normal * 0.5 + 0.5) * barycentrics.x +
                      (v1.normal * 0.5 + 0.5) * barycentrics.y +
                      (v2.normal * 0.5 + 0.5) * barycentrics.z;
    
    //float3 hitColor = float3(0.0f, 1.0f, 0.0f); // Green color for triangle hit
    rayPayload.color = float4(hitColor , 1.0f);
}