// Unlit coloured lines: grid, axes and other overlays.

struct LinePush
{
    float4 viewProjection[4]; // rows; view is camera-relative
    float4 offset;            // position of the line set's origin relative to the camera
    float4 color;
};

[[vk::push_constant]]
ConstantBuffer<LinePush> push;

[shader("vertex")]
float4 vsMain(float3 position : POSITION) : SV_Position
{
    float4 relative = float4(position + push.offset.xyz, 1.0);
    return float4(dot(push.viewProjection[0], relative), dot(push.viewProjection[1], relative),
                  dot(push.viewProjection[2], relative), dot(push.viewProjection[3], relative));
}

[shader("fragment")]
float4 fsMain() : SV_Target
{
    return push.color;
}
