// Realistic shading of the viewport: sky and sun, physically based materials, cascaded sun shadows,
// water, and the final image. Colours are linear and in HDR until fsComposite. Colour model:
// Cook-Torrance with GGX (Walter et al. 2007), Smith visibility and Schlick's Fresnel; image-based
// light by Lazarov's analytic split-sum (2013). Sky: an analytic gradient. Tone map: ACES, Narkowicz's
// fit (2015). Water: Gerstner waves (Tessendorf 2001, §2) and the Fresnel of water, n = 1.33.
//
// Everything is relative to the camera, whose eye is at the origin: the GPU never sees a large
// coordinate. Noise is anchored to `worldWrap`, the eye's world position wrapped at 1024 m.

struct Frame
{
    float4 viewProjection[4]; // rows; for the mirror pass, the same (the geometry is mirrored)
    float4 sunDirection;      // xyz towards the sun; w: sun illuminance scale
    float4 sunColor;          // rgb; w: exposure
    float4 worldWrap;         // xyz; w: seconds, for what moves
    float4 cameraRight;       // xyz; w: tan(fov/2) * aspect
    float4 cameraUp;          // xyz; w: tan(fov/2)
    float4 cameraForward;     // xyz; w: near plane, m
    float4 shadow0[3];        // rows of the near cascade: world relative to camera -> atlas uv, depth
    float4 shadow1[3];        // the far cascade
    float4 water;             // x: surface height relative to the eye; y: wind m/s; z: has water;
                              // w: bottom of the water relative to the eye
    float4 waterRect;         // xy centre relative to the eye; zw half sizes
    float4 screen;            // xy: size in pixels; zw: 1 / size
    float4 mode;              // x: 1 in the mirror pass (y: the plane's relative height there)
};

[[vk::binding(0)]] ConstantBuffer<Frame> frame;
[[vk::binding(1)]] Sampler2D shadowMap;   // the sun's depth, two cascades side by side
[[vk::binding(2)]] Sampler2D sceneColor;  // HDR, what was drawn before the water
[[vk::binding(3)]] Sampler2D sceneDepth;
[[vk::binding(4)]] Sampler2D mirrorColor; // what the water reflects; alpha 0 where only sky

struct Push
{
    float4 model[3];  // rows; translation relative to the camera
    float4 baseColor; // linear RGB, alpha
    float4 surface;   // metallic, roughness, detail (1: stone-like, 0: smooth), unused
    float4 emissive;  // rgb
};

[[vk::push_constant]] ConstantBuffer<Push> push;

static const float PI = 3.14159265;

float4 project(float3 relative)
{
    float4 p = float4(relative, 1.0);
    return float4(dot(frame.viewProjection[0], p), dot(frame.viewProjection[1], p),
                  dot(frame.viewProjection[2], p), dot(frame.viewProjection[3], p));
}

// --- noise ----------------------------------------------------------------------------------

float hash(float3 p)
{
    p = frac(p * 0.3183099 + 0.1);
    p *= 17.0;
    return frac(p.x * p.y * p.z * (p.x + p.y + p.z));
}

float noise3(float3 x)
{
    float3 i = floor(x);
    float3 f = frac(x);
    f = f * f * (3.0 - 2.0 * f);
    return lerp(lerp(lerp(hash(i), hash(i + float3(1, 0, 0)), f.x),
                     lerp(hash(i + float3(0, 1, 0)), hash(i + float3(1, 1, 0)), f.x), f.y),
                lerp(lerp(hash(i + float3(0, 0, 1)), hash(i + float3(1, 0, 1)), f.x),
                     lerp(hash(i + float3(0, 1, 1)), hash(i + float3(1, 1, 1)), f.x), f.y),
                f.z);
}

float fbm(float3 p, int octaves)
{
    float sum = 0.0;
    float amplitude = 0.5;
    for (int i = 0; i < octaves; ++i)
    {
        sum += amplitude * noise3(p);
        p = p * 2.03 + float3(11.3, 7.1, 3.7);
        amplitude *= 0.5;
    }
    return sum;
}

// --- sky ------------------------------------------------------------------------------------

// A clear midday sky: deep blue overhead, paler to the horizon, brighter round the sun. An analytic
// gradient in the spirit of Preetham et al. (1999), tuned by eye rather than from their tables.
float3 clearSky(float3 dir)
{
    float up = saturate(dir.z);
    float toHorizon = pow(1.0 - up, 3.2);
    float3 zenith = float3(0.085, 0.205, 0.46);
    float3 horizon = float3(0.56, 0.68, 0.82);
    float3 color = lerp(zenith, horizon, toHorizon);
    float toSun = saturate(dot(dir, frame.sunDirection.xyz));
    color += float3(0.32, 0.27, 0.20) * pow(toSun, 6.0) * 0.35;
    color += float3(0.9, 0.8, 0.65) * pow(toSun, 90.0) * 0.5;
    return color;
}

// Clouds on a plane far overhead, drifting.
float cloudAmount(float3 dir)
{
    if (dir.z <= 0.02)
        return 0.0;
    float2 at = dir.xy / (dir.z + 0.12) * 1.3 + frame.worldWrap.w * float2(0.004, 0.002);
    float shape = fbm(float3(at, 0.0), 5);
    return smoothstep(0.52, 0.74, shape) * smoothstep(0.03, 0.2, dir.z);
}

// What comes from direction `dir`; `sharp` 1 with sun and clouds in detail, 0 for reflections off
// rough surfaces, where only the broad colour matters.
float3 sky(float3 dir, float sharp)
{
    float3 horizonGround = float3(0.56, 0.68, 0.82) * 0.92;
    float3 clear = clearSky(float3(dir.xy, max(dir.z, 0.02)));
    if (sharp > 0.0)
    {
        float clouds = cloudAmount(dir) * sharp;
        float lit = 0.75 + 0.5 * saturate(dot(dir, frame.sunDirection.xyz));
        clear = lerp(clear, float3(1.0, 1.0, 1.02) * 1.0 * lit, clouds);
    }
    return lerp(horizonGround, clear, smoothstep(-0.03, 0.02, dir.z));
}

float3 fogged(float3 color, float3 relative)
{
    float distance = length(relative);
    float amount = 1.0 - exp(-distance * 0.0022);
    float3 dir = relative / max(distance, 1e-3);
    float3 far = sky(float3(dir.xy, max(dir.z, 0.03)), 0.0);
    return lerp(color, far, amount);
}

// --- shadow ---------------------------------------------------------------------------------

float shadowTap(float3 uvw, float2 offset)
{
    // Reversed depth: nearer is larger. Lit when this point is at least as near as what is stored.
    float stored = shadowMap.SampleLevel(uvw.xy + offset, 0.0).r;
    return uvw.z >= stored ? 1.0 : 0.0;
}

float sunLight(float3 relative, float3 normal)
{
    if (frame.mode.x > 0.5)
        relative.z = 2.0 * frame.mode.y - relative.z;  // the mirrored point's real place
    float3 lifted = relative + normal * 0.09;  // past a texel of the cascades, against acne
    float4 p = float4(lifted, 1.0);
    float3 a = float3(dot(frame.shadow0[0], p), dot(frame.shadow0[1], p), dot(frame.shadow0[2], p));
    float3 uvw;
    float radius;
    // The near cascade is the left half of the atlas.
    if (a.x > 0.002 && a.x < 0.498 && a.y > 0.004 && a.y < 0.996 && a.z > 0.0 && a.z < 1.0)
    {
        uvw = a;
        radius = 1.5 / 2048.0;
    }
    else
    {
        uvw = float3(dot(frame.shadow1[0], p), dot(frame.shadow1[1], p), dot(frame.shadow1[2], p));
        if (uvw.x < 0.502 || uvw.x > 0.998 || uvw.y < 0.004 || uvw.y > 0.996)
            return 1.0;
        radius = 1.0 / 2048.0;
    }
    uvw.z += 0.0004;
    static const float2 poisson[8] = {float2(-0.94, -0.40), float2(0.95, -0.77), float2(-0.09, -0.93),
                                      float2(0.34, 0.29),   float2(-0.92, 0.46), float2(-0.38, 0.92),
                                      float2(0.80, 0.60),   float2(0.14, -0.38)};
    float sum = 0.0;
    for (int i = 0; i < 8; ++i)
        sum += shadowTap(uvw, poisson[i] * radius);
    return sum / 8.0;
}

// --- surfaces -------------------------------------------------------------------------------

float3 shadeSurface(float3 relative, float3 N, float3 albedo, float metallic, float roughness, float3 glow)
{
    float3 V = normalize(-relative);
    float NV = saturate(dot(N, V)) + 1e-3;
    float3 L = frame.sunDirection.xyz;
    float3 F0 = lerp(float3(0.04, 0.04, 0.04), albedo, metallic);
    float3 diffuseColor = albedo * (1.0 - metallic);

    // The sun, by GGX.
    float NL = saturate(dot(N, L));
    float3 color = float3(0.0, 0.0, 0.0);
    if (NL > 0.0)
    {
        float3 H = normalize(V + L);
        float NH = saturate(dot(N, H));
        float VH = saturate(dot(V, H));
        float a = max(roughness * roughness, 0.002);
        float a2 = a * a;
        float d = NH * NH * (a2 - 1.0) + 1.0;
        float D = a2 / (PI * d * d);
        float k = (roughness + 1.0) * (roughness + 1.0) / 8.0;
        float G = 1.0 / ((NV * (1.0 - k) + k) * (NL * (1.0 - k) + k) * 4.0);
        float3 F = F0 + (1.0 - F0) * pow(1.0 - VH, 5.0);
        float3 specular = min(D * G, 40.0) * F;
        color = (diffuseColor / PI + specular) * frame.sunColor.rgb * frame.sunDirection.w * NL *
                sunLight(relative, N);
    }

    // The sky: broad light from above and the ground's bounce, and its reflection, blurred with
    // roughness (Lazarov's analytic split-sum).
    float3 irradiance = lerp(float3(0.12, 0.115, 0.105), float3(0.38, 0.46, 0.60), N.z * 0.5 + 0.5);
    float3 R = reflect(-V, N);
    float3 sharpSky = sky(R, roughness < 0.25 ? 1.0 - roughness * 4.0 : 0.0);
    float3 envSpec = lerp(sharpSky, irradiance, saturate(roughness * 1.3));
    float4 c0 = float4(-1.0, -0.0275, -0.572, 0.022);
    float4 c1 = float4(1.0, 0.0425, 1.04, -0.04);
    float4 r = roughness * c0 + c1;
    float a004 = min(r.x * r.x, exp2(-9.28 * NV)) * r.x + r.y;
    float2 AB = float2(-1.04, 1.04) * a004 + r.zw;
    color += diffuseColor * irradiance + envSpec * (F0 * AB.x + AB.y);
    return color + glow;
}

struct VertexInput
{
    float3 position : POSITION;
    float3 normal : NORMAL;
};

struct MeshOutput
{
    float4 position : SV_Position;
    float3 relative;
    float3 normal;
};

[shader("vertex")]
MeshOutput vsMesh(VertexInput input)
{
    float4 local = float4(input.position, 1.0);
    float3 relative = float3(dot(push.model[0], local), dot(push.model[1], local), dot(push.model[2], local));
    MeshOutput output;
    output.relative = relative;
    output.normal = float3(dot(push.model[0].xyz, input.normal), dot(push.model[1].xyz, input.normal),
                           dot(push.model[2].xyz, input.normal));
    output.position = project(relative);
    return output;
}

// The sun's depth pass: `frame.viewProjection` is not used; the push's first rows carry the sun's
// matrix applied to the model already.
[shader("vertex")]
float4 vsShadow(VertexInput input) : SV_Position
{
    float4 local = float4(input.position, 1.0);
    return float4(dot(push.model[0], local), dot(push.model[1], local), dot(push.model[2], local), 1.0);
}

[shader("fragment")]
void fsShadow()
{
}

// Fine bumps and blotches where a surface is rough, none where it is polished: a concrete floor is
// not a flat colour, a chrome ball has no grain.
void surfaceDetail(float3 world, inout float3 N, inout float3 albedo, inout float roughness)
{
    float amount = push.surface.z;
    if (amount <= 0.0)
        return;
    float wide = fbm(world * 0.35, 3);
    float fine = fbm(world * 7.0, 3);
    albedo *= 1.0 + amount * ((wide - 0.5) * 0.42 + (fine - 0.5) * 0.32);
    roughness = saturate(roughness + amount * (fine - 0.5) * 0.12);
    const float e = 0.012;
    float dx = fbm((world + float3(e, 0, 0)) * 7.0, 2) - fbm((world - float3(e, 0, 0)) * 7.0, 2);
    float dy = fbm((world + float3(0, e, 0)) * 7.0, 2) - fbm((world - float3(0, e, 0)) * 7.0, 2);
    float dz = fbm((world + float3(0, 0, e)) * 7.0, 2) - fbm((world - float3(0, 0, e)) * 7.0, 2);
    float3 gradient = float3(dx, dy, dz);
    gradient -= N * dot(gradient, N);
    N = normalize(N - gradient * amount * 0.9);
}

[shader("fragment")]
float4 fsMesh(MeshOutput input, bool front : SV_IsFrontFace) : SV_Target
{
    float3 N = normalize(input.normal);
    float3 V = normalize(-input.relative);
    if (dot(N, V) < 0.0)
        N = -N;  // imported meshes do not have reliable winding
    if (frame.mode.x > 0.5 && input.relative.z > frame.mode.y)
        discard;  // the mirror pass: what is under the water does not reflect
    float3 albedo = push.baseColor.rgb;
    float roughness = push.surface.y;
    float3 world = input.relative + frame.worldWrap.xyz;
    surfaceDetail(world, N, albedo, roughness);
    if (push.baseColor.a < 1.0)
        albedo *= push.baseColor.a;  // glass shows little of its own colour, mostly reflections
    float3 color = shadeSurface(input.relative, N, albedo, push.surface.x, roughness, push.emissive.rgb);
    if (frame.mode.x < 0.5)
        color = fogged(color, input.relative);
    float alpha = push.baseColor.a;
    if (alpha < 1.0)
    {
        // Glass: its reflection stays whatever its transparency.
        float fresnel = 0.04 + 0.96 * pow(1.0 - saturate(dot(N, V)), 5.0);
        alpha = saturate(lerp(alpha, 1.0, fresnel));
    }
    return float4(color, alpha);
}

// --- full screen ----------------------------------------------------------------------------

struct ScreenOutput
{
    float4 position : SV_Position;
    float2 uv;
};

[shader("vertex")]
ScreenOutput vsScreen(uint vertex : SV_VertexID)
{
    ScreenOutput output;
    output.uv = float2((vertex << 1) & 2, vertex & 2);
    output.position = float4(output.uv * 2.0 - 1.0, 0.0, 1.0);  // far, in reversed depth
    return output;
}

float3 viewRay(float2 uv)
{
    float2 ndc = uv * 2.0 - 1.0;
    return frame.cameraForward.xyz + frame.cameraRight.xyz * (ndc.x * frame.cameraRight.w) -
           frame.cameraUp.xyz * (ndc.y * frame.cameraUp.w);
}

[shader("fragment")]
float4 fsSky(ScreenOutput input) : SV_Target
{
    float3 dir = normalize(viewRay(input.uv));
    float3 color = sky(dir, 1.0);
    float cosSun = dot(dir, frame.sunDirection.xyz);
    float disc = smoothstep(0.99994, 0.99998, cosSun);
    color += frame.sunColor.rgb * disc * 60.0 * smoothstep(0.0, 0.03, dir.z) * (1.0 - cloudAmount(dir));
    color += frame.sunColor.rgb * pow(saturate(cosSun), 900.0) * 0.5;  // the glow round it
    return float4(color, 1.0);
}

// The picture as a person sees it: exposure, ACES film response, display encoding.
float3 display(float3 hdr)
{
    float3 x = hdr * frame.sunColor.w;
    float3 mapped = saturate((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14));
    return pow(mapped, float3(1.0 / 2.2));
}

float luminance(float3 c)
{
    return dot(c, float3(0.299, 0.587, 0.114));
}

// Screen-space ambient occlusion at this pixel: how much of what is near hides the sky.
float occlusion(float2 uv, float depth)
{
    if (depth <= 0.0)
        return 1.0;
    float near = frame.cameraForward.w;
    float z = near / depth;
    float3 here = viewRay(uv) * z;
    float3 dx = ddx(here);
    float3 dy = ddy(here);
    float3 n = normalize(cross(dx, dy));
    float radius = 0.35;
    float noiseAngle = frac(52.9829189 * frac(dot(input_pixel(uv), float2(0.06711056, 0.00583715)))) * 2.0 * PI;
    float pixels = clamp(radius / z / frame.cameraUp.w * frame.screen.y * 0.5, 3.0, 60.0);
    float hidden = 0.0;
    const int samples = 10;
    for (int i = 0; i < samples; ++i)
    {
        float t = (float(i) + 0.5) / float(samples);
        float angle = noiseAngle + t * 2.4 * float(samples);
        float2 offset = float2(cos(angle), sin(angle)) * sqrt(t) * pixels * frame.screen.zw;
        float d = sceneDepth.SampleLevel(uv + offset, 0.0).r;
        if (d <= 0.0)
            continue;
        float3 there = viewRay(uv + offset) * (near / d);
        float3 w = there - here;
        float length2 = dot(w, w);
        if (length2 > radius * radius * 4.0)
            continue;
        hidden += saturate(dot(n, w) / sqrt(length2 + 1e-5) - 0.22);
    }
    return saturate(1.0 - 1.15 * hidden / float(samples));
}

float2 input_pixel(float2 uv)
{
    return floor(uv * frame.screen.xy);
}

// Bright things bleed a little into their surroundings, as in an eye or a lens.
float3 bloom(float2 uv)
{
    float3 sum = float3(0, 0, 0);
    static const float2 ring[8] = {float2(1, 0), float2(0.7, 0.7), float2(0, 1), float2(-0.7, 0.7),
                                   float2(-1, 0), float2(-0.7, -0.7), float2(0, -1), float2(0.7, -0.7)};
    for (int i = 0; i < 8; ++i)
    {
        float3 near = sceneColor.SampleLevel(uv + ring[i] * frame.screen.zw * 1.5, 0.0).rgb;
        float3 far = sceneColor.SampleLevel(uv + ring[i] * frame.screen.zw * 3.5, 0.0).rgb;
        sum += max(near - 3.0, 0.0) + 0.5 * max(far - 3.0, 0.0);
    }
    return min(sum / 14.0, float3(3.0, 3.0, 3.0));
}

// FXAA in the manner of Lottes (2009): smooths edges by their local contrast.
[shader("fragment")]
float4 fsComposite(ScreenOutput input) : SV_Target
{
    float2 uv = input.uv;
    float2 px = frame.screen.zw;
    float depth = sceneDepth.SampleLevel(uv, 0.0).r;
    float ao = occlusion(uv, depth);
    float3 here = sceneColor.SampleLevel(uv, 0.0).rgb * ao + bloom(uv);
    float3 center = display(here);
    float lumaC = luminance(center);
    float lumaN = luminance(display(sceneColor.SampleLevel(uv + float2(0, -px.y), 0.0).rgb));
    float lumaS = luminance(display(sceneColor.SampleLevel(uv + float2(0, px.y), 0.0).rgb));
    float lumaW = luminance(display(sceneColor.SampleLevel(uv + float2(-px.x, 0), 0.0).rgb));
    float lumaE = luminance(display(sceneColor.SampleLevel(uv + float2(px.x, 0), 0.0).rgb));
    float low = min(lumaC, min(min(lumaN, lumaS), min(lumaW, lumaE)));
    float high = max(lumaC, max(max(lumaN, lumaS), max(lumaW, lumaE)));
    float range = high - low;
    if (range < max(0.04, high * 0.12))
        return float4(center, 1.0);
    float2 dir = float2(lumaS - lumaN, lumaW - lumaE);
    float reduce = max(range * 0.25, 1.0 / 128.0);
    dir = clamp(dir / (min(abs(dir.x), abs(dir.y)) + reduce), -8.0, 8.0) * px;
    float3 a = 0.5 * (sceneColor.SampleLevel(uv + dir * (1.0 / 3.0 - 0.5), 0.0).rgb +
                      sceneColor.SampleLevel(uv + dir * (2.0 / 3.0 - 0.5), 0.0).rgb);
    float3 b = 0.5 * a + 0.25 * (sceneColor.SampleLevel(uv - dir * 0.5, 0.0).rgb +
                                 sceneColor.SampleLevel(uv + dir * 0.5, 0.0).rgb);
    float lumaB = luminance(display(b));
    float3 smoothed = (lumaB < low || lumaB > high) ? a : b;
    return float4(display(smoothed * ao + bloom(uv)), 1.0);
}

// --- water ----------------------------------------------------------------------------------

// Eight Gerstner waves whose height follows the wind: long swell, then chop. A wave's speed is that
// of deep-water gravity waves, c = sqrt(g / k).
struct Wave
{
    float2 direction;
    float length;
    float steepness;
};

static const Wave waves[8] = {
    {float2(0.83, 0.56), 9.1, 0.32},  {float2(0.60, 0.80), 5.3, 0.30},
    {float2(0.97, -0.24), 3.1, 0.28}, {float2(-0.31, 0.95), 2.2, 0.26},
    {float2(0.71, -0.70), 1.37, 0.24}, {float2(-0.86, 0.51), 0.91, 0.22},
    {float2(0.28, 0.96), 0.61, 0.2},  {float2(-0.64, -0.77), 0.43, 0.18},
};

// How rough the water is, from calm to a strong wind: a metre-scale amplitude factor.
float waveScale()
{
    float wind = frame.water.y;
    return 0.05 + 0.0065 * min(wind * wind, 200.0);
}

void gerstner(float2 world, float t, out float3 displacement, out float3 normal)
{
    displacement = float3(0, 0, 0);
    float3 tangentX = float3(1, 0, 0);
    float3 tangentY = float3(0, 1, 0);
    float scale = waveScale();
    for (int i = 0; i < 8; ++i)
    {
        Wave w = waves[i];
        float k = 2.0 * PI / w.length;
        float c = sqrt(9.81 / k);
        float amplitude = min(w.length * 0.045, scale * w.length * 0.16);
        float phase = k * (dot(w.direction, world) - c * t);
        float s = sin(phase);
        float co = cos(phase);
        float q = w.steepness;
        displacement += float3(q * amplitude * w.direction * co, amplitude * s);
        float wa = k * amplitude;
        tangentX += float3(-q * w.direction.x * w.direction.x * wa * s,
                           -q * w.direction.x * w.direction.y * wa * s, w.direction.x * wa * co);
        tangentY += float3(-q * w.direction.x * w.direction.y * wa * s,
                           -q * w.direction.y * w.direction.y * wa * s, w.direction.y * wa * co);
    }
    normal = normalize(cross(tangentX, tangentY));
}

struct WaterOutput
{
    float4 position : SV_Position;
    float3 relative;
    float3 normal;
    float2 world;
    float side; // 1 on the sides of the water, 0 on its surface
};

// A grid of `kGrid` cells over the surface, drawn without a vertex buffer, and a skirt down to the
// bottom at its edge, so that water standing above the ground has sides.
static const uint kGrid = 256;

[shader("vertex")]
WaterOutput vsWater(uint vertex : SV_VertexID)
{
    static const uint2 corner[6] = {uint2(0, 0), uint2(1, 0), uint2(0, 1), uint2(0, 1), uint2(1, 0), uint2(1, 1)};
    uint quad = vertex / 6;
    uint2 cell = uint2(quad % (kGrid + 2), quad / (kGrid + 2));
    uint2 at = cell + corner[vertex % 6];
    // The outermost ring of vertices lies at the rim, hanging down.
    bool skirt = at.x == 0 || at.y == 0 || at.x == kGrid + 2 || at.y == kGrid + 2;
    uint2 inner = clamp(at, uint2(1, 1), uint2(kGrid + 1, kGrid + 1)) - uint2(1, 1);
    float2 unit = float2(inner) / float(kGrid) * 2.0 - 1.0;
    // Cells bunch up near the camera, where the detail shows.
    float2 toCamera = -frame.waterRect.xy / max(frame.waterRect.zw, 0.01);
    toCamera = clamp(toCamera, -1.0, 1.0);
    float2 warped = unit;
    bool rim = any(inner == uint2(0, 0)) || any(inner == uint2(kGrid, kGrid));
    if (!rim)
    {
        float2 d = unit - toCamera;
        warped = toCamera + d * (0.35 + 0.65 * abs(d) / 2.0);
        warped = clamp(warped, -0.999, 0.999);
    }
    float2 relativeXY = frame.waterRect.xy + warped * frame.waterRect.zw;
    float2 world = relativeXY + frame.worldWrap.xy;
    float3 displacement;
    float3 normal;
    gerstner(world, frame.worldWrap.w, displacement, normal);
    WaterOutput output;
    if (skirt)
    {
        output.relative = float3(relativeXY, frame.water.w);
        output.normal = normalize(float3(unit, 0.2));
        output.side = 1.0;
    }
    else if (rim)
    {
        output.relative = float3(relativeXY, frame.water.x + displacement.z);
        output.normal = normal;
        output.side = 0.0;
    }
    else
    {
        output.relative = float3(relativeXY + displacement.xy, frame.water.x + displacement.z);
        output.normal = normal;
        output.side = 0.0;
    }
    output.world = world;
    output.position = project(output.relative);
    return output;
}

// Small ripples that the mesh is too coarse to hold: the slope of several scrolling noise layers.
float2 ripple(float2 world, float t, float distance)
{
    float wind = 1.0 + 0.1 * min(frame.water.y, 16.0);
    float2 slope = float2(0, 0);
    float scale = 2.6;
    float amount = 0.11 * wind;
    for (int i = 0; i < 4; ++i)
    {
        float3 p = float3(world * scale + float2(0.7, 0.4) * t * (0.55 + 0.25 * float(i)) + float(i) * 7.1, t * 0.45);
        const float e = 0.08;
        float dx = noise3(p + float3(e, 0, 0)) - noise3(p - float3(e, 0, 0));
        float dy = noise3(p + float3(0, e, 0)) - noise3(p - float3(0, e, 0));
        slope += float2(dx, dy) / (2.0 * e) * amount;
        scale *= 2.3;
        amount *= 0.72;
    }
    return slope / (1.0 + distance * 0.018);
}

// A dancing web of light on what lies under the water.
float caustics(float2 p, float t)
{
    float2 a = p * 2.1 + float2(t * 0.31, t * 0.23);
    float2 b = p * 2.7 - float2(t * 0.27, -t * 0.21);
    float n = abs(noise3(float3(a, t * 0.2)) - 0.5) + abs(noise3(float3(b, t * 0.25)) - 0.5);
    return pow(saturate(1.0 - n * 2.4), 5.0);
}

[shader("fragment")]
float4 fsWater(WaterOutput input) : SV_Target
{
    float distance = length(input.relative);
    float3 V = -input.relative / max(distance, 1e-3);
    float2 uv = input.position.xy * frame.screen.zw;
    float t = frame.worldWrap.w;

    float above = frame.water.x + 0.0 < 0.0 ? 1.0 : -1.0;  // the eye above the surface?
    float3 N = normalize(input.normal);
    if (input.side < 0.5)
    {
        // The waves' slope per pixel: the mesh only holds their shape.
        float3 lifted;
        gerstner(input.world, t, lifted, N);
        float2 slope = ripple(input.world, t, distance);
        N = normalize(float3(N.xy - slope, N.z));
        N *= above;
    }
    else if (dot(N, V) < 0.0)
    {
        N = -N;
    }

    // Reflection: what stands above the water, mirrored, and the sky behind it.
    float NV = saturate(dot(N, V));
    float fresnel = 0.02 + 0.98 * pow(1.0 - NV, 5.0);
    float3 R = reflect(-V, N);
    R.z = abs(R.z) + 0.01;
    float2 mirrorUv = saturate(uv + N.xy * float2(0.035, 0.07) / (1.0 + distance * 0.02));
    float4 mirrored = mirrorColor.SampleLevel(mirrorUv, 0.0);
    float3 reflection = lerp(sky(normalize(R), 1.0), mirrored.rgb, mirrored.a);

    // Refraction: what lies below, seen through the water, losing red first.
    float near = frame.cameraForward.w;
    float2 bentUv = saturate(uv + N.xy * 0.04 / (1.0 + distance * 0.06));
    float depthThere = sceneDepth.SampleLevel(bentUv, 0.0).r;
    float waterDepth = input.position.z;
    if (depthThere > waterDepth)
    {
        // That sample is in front of the water: look straight through instead.
        bentUv = uv;
        depthThere = sceneDepth.SampleLevel(uv, 0.0).r;
    }
    float3 below = sceneColor.SampleLevel(bentUv, 0.0).rgb;
    float viewZ = near / max(waterDepth, 1e-6);
    float belowZ = depthThere > 0.0 ? near / depthThere : viewZ + 200.0;
    float forward = max(dot(-V, frame.cameraForward.xyz), 0.05);
    float path = max(belowZ - viewZ, 0.0) / forward;
    if (input.side > 0.5)
        path = max(path, 0.8);
    static const float3 kAbsorption = float3(0.46, 0.14, 0.09);  // 1/m: clear coastal water
    float3 transmitted = exp(-kAbsorption * path);
    float3 point = viewRay(bentUv) * belowZ;
    float submerged = max(frame.water.x - point.z, 0.0);
    float3 lightOnBottom = caustics(point.xy + frame.worldWrap.xy, t) * exp(-submerged * 0.6) *
                           saturate(submerged * 4.0) * frame.sunDirection.z * 1.3;
    below *= 1.0 + lightOnBottom;
    // Light scattered back by the water itself: its own colour.
    float sunUp = saturate(frame.sunDirection.z);
    float3 deep = float3(0.004, 0.045, 0.075) * (0.45 + 1.2 * sunUp);
    float3 water = below * transmitted + deep * (1.0 - transmitted);

    // Light through wave crests, turned green.
    float crest = saturate((input.relative.z - frame.water.x) / max(waveScale(), 0.01) * 0.8);
    water += float3(0.015, 0.09, 0.075) * crest * crest * sunUp * pow(1.0 - NV, 2.0) * 2.0;

    float3 color = lerp(water, reflection, fresnel);

    // The sun's glitter, by GGX on the wavy surface.
    float3 L = frame.sunDirection.xyz;
    float3 H = normalize(V + L);
    float NH = saturate(dot(N, H));
    float a = 0.045 + 0.03 * saturate(frame.water.y * 0.1);
    float a2 = a * a;
    float d = NH * NH * (a2 - 1.0) + 1.0;
    float D = a2 / (PI * d * d);
    float visibility = sunLight(input.relative, float3(0, 0, 1));
    color += min(D * 0.25 * fresnelSun(saturate(dot(V, H))), 6.0) * frame.sunColor.rgb * frame.sunDirection.w * saturate(dot(N, L)) * visibility;

    // Foam: where the water meets something, and on the crests in a strong wind.
    float shore = 1.0 - saturate(path * forward / 0.22);
    float tearing = saturate((crest - 0.72) * 3.0) * saturate((frame.water.y - 6.0) / 8.0);
    float cell = fbm(float3(input.world * 3.3 + float2(0.3, 0.2) * t, t * 0.25), 3);
    float streak = fbm(float3(input.world * 11.0 - float2(0.25, 0.1) * t, t * 0.4), 2);
    float foam = saturate(smoothstep(0.42, 0.62, cell * (0.6 + streak)) * saturate(shore * 1.5 + tearing));
    if (input.side < 0.5)
        color = lerp(color, float3(0.78, 0.82, 0.84) * (0.45 + 0.65 * visibility * sunUp), foam * 0.85);

    return float4(fogged(color, input.relative), 1.0);
}

float fresnelSun(float VH)
{
    return 0.02 + 0.98 * pow(1.0 - VH, 5.0);
}
