// Test shader: one red triangle covering the centre of the target.

struct VertexOutput
{
    float4 position : SV_Position;
    float3 color;
};

[shader("vertex")]
VertexOutput vsMain(uint vertexId : SV_VertexID)
{
    static const float2 corners[3] = { float2(0.0, -0.8), float2(0.8, 0.8), float2(-0.8, 0.8) };
    VertexOutput output;
    output.position = float4(corners[vertexId], 0.0, 1.0);
    output.color = float3(1.0, 0.0, 0.0);
    return output;
}

[shader("fragment")]
float4 fsMain(VertexOutput input) : SV_Target
{
    return float4(input.color, 1.0);
}
