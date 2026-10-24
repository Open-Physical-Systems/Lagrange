// Opaque mesh shading shared by the viewer's viewport (fsMain) and the camera sensor (fsSensor)
// (invariant 13: the same code draws both).
//
// Matrices are passed as rows and applied with dot products, so the result does not depend
// on the compiler's matrix layout convention.

struct MeshPush
{
    float4 viewProjection[4]; // rows; view is camera-relative (no translation)
    float4 model[3];          // rows of the 3x4 model matrix; translation is relative to the camera
    float4 baseColor; // linear RGB; alpha for fsMain, the instance id (as a float) for fsSensor
};

[[vk::push_constant]]
ConstantBuffer<MeshPush> push;

struct VertexInput
{
    float3 position : POSITION;
    float3 normal : NORMAL;
};

struct VertexOutput
{
    float4 position : SV_Position;
    float3 normal;
    float viewDepth; // clip w: the distance along the view axis, m
};

[shader("vertex")]
VertexOutput vsMain(VertexInput input)
{
    float4 local = float4(input.position, 1.0);
    float4 relative = float4(dot(push.model[0], local), dot(push.model[1], local), dot(push.model[2], local), 1.0);

    VertexOutput output;
    output.position = float4(dot(push.viewProjection[0], relative), dot(push.viewProjection[1], relative),
                             dot(push.viewProjection[2], relative), dot(push.viewProjection[3], relative));
    output.normal = float3(dot(push.model[0].xyz, input.normal), dot(push.model[1].xyz, input.normal),
                           dot(push.model[2].xyz, input.normal));
    output.viewDepth = output.position.w;
    return output;
}

// One directional light plus a sky/ground ambient term, encoded for display (the colour
// targets are UNORM). Deliberately simple: the viewport is for inspection, and the raster
// camera level has no light model yet (sensors.md §2: exposure comes with `sensor_model`).
float3 shade(float3 normal)
{
    static const float3 lightDirection = normalize(float3(0.35, 0.25, 0.9));
    static const float3 skyColor = float3(0.55, 0.60, 0.70);
    static const float3 groundColor = float3(0.20, 0.19, 0.18);

    normal = normalize(normal);
    float diffuse = max(dot(normal, lightDirection), 0.0);
    float3 ambient = lerp(groundColor, skyColor, normal.z * 0.5 + 0.5);
    float3 linearColor = push.baseColor.rgb * (ambient * 0.6 + diffuse * 0.8);
    return pow(saturate(linearColor), float3(1.0 / 2.2));
}

[shader("fragment")]
float4 fsMain(VertexOutput input) : SV_Target
{
    return float4(shade(input.normal), push.baseColor.a);
}

// The camera sensor draws twice: the image, multisampled and averaged so that a pixel
// integrates over its area; then depth and instance id at one sample, so that an edge pixel
// never mixes two surfaces.
[shader("fragment")]
float4 fsSensorColor(VertexOutput input) : SV_Target
{
    return float4(shade(input.normal), 1.0);
}

struct SensorLabels
{
    float depth : SV_Target0;   // R32_SFLOAT, m along the optical axis
    uint instance : SV_Target1; // R32_UINT
};

[shader("fragment")]
SensorLabels fsSensorLabels(VertexOutput input)
{
    SensorLabels output;
    output.depth = input.viewDepth;
    output.instance = uint(push.baseColor.a);
    return output;
}
