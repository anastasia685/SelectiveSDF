#include "Common.hlsl"

[shader("miss")]
void Miss(inout RayPayload rayPayload)
{
    rayPayload.color = float4(BackgroundColor(), 1.0f);
}