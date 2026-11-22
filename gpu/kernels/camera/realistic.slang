// Realistic shading of the viewport: sky and sun, physically based materials, cascaded sun shadows,
// water, and the final image. Colours are linear and in HDR until fsComposite. Colour model:
// Cook-Torrance with GGX (Walter et al. 2007), Smith visibility and Schlick's Fresnel; image-based
// light by Lazarov's analytic split-sum (2013). Sky: an analytic gradient. Tone map: ACES, Narkowicz's
// fit (2015). Water: Gerstner waves (Tessendorf 2001, §2), the physics' surface, Cox and Munk's
// ripples and the exact Fresnel of water, n = 1.333 (see the water's optics below).
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
    float4 mode;              // x: 1 in the mirror pass (y: the plane's relative height there);
                              // z: the season; w: 1 with surfaceAlbedo's photographs
    float4 weather;           // x: rain, mm/h; y: snow, mm/h of water; zw: the wind, m/s, world
    float4 light;             // x: the daylight's share (1: the drawn day; the sky and the sun's
                              // light scale by it); y: lamps; z: 1 to write the linear HDR value
                              // (calibration tests); w: rotor ground jets in airJets
    float4 surface;           // the water's moving surface (water.md): xy the first cell's centre
                              // relative to the eye; z: its cell, m; w: cells a side (0: none)
    float4 surfaceLook;       // x: relief, the slopes' scale (1: the physics' slopes as they are)
    float4 sprayBox;          // the spray's water (RealisticSpray): xyz the first cell's centre
                              // relative to the eye; w: its cell, m
    float4 sprayDims;         // xyz: cells (0: none); w: extinction per kg/m³ of water, 1/m
    float4 waterOptics;       // xyz: dissolved organic matter's absorption at 620, 550, 465 nm,
                              // 1/m; w: unused
    float4 airJets[16];       // per jet (GroundAir.hpp): xyz its ground hit relative to the eye,
                              // w arriving speed m/s; then x its arriving radius m
    float4 cutGrid;           // plants cut by propellers (RealisticCutPlants): xy the first cell's
                              // centre relative to the eye; z its cell, m; w cells along x (0: none)
    float4 cutLevel;          // x: the tops' reference height relative to the eye; y: cells along y;
                              // z: 1 when grass blades are drawn near the eye (vsGrassBlade)
    float4 trees;             // x: cut cells in treeCuts (0: none cut); y: flying leaves in
                              // flyingLeaves; zw: unused
};
static const int kAirJets = 8; // kMaxAirJets

[[vk::binding(0)]] ConstantBuffer<Frame> frame;
[[vk::binding(1)]] Sampler2D shadowMap;   // the sun's depth, two cascades side by side
[[vk::binding(2)]] Sampler2D sceneColor;  // HDR, what was drawn before the water
[[vk::binding(3)]] Sampler2D sceneDepth;
[[vk::binding(4)]] Sampler2D mirrorColor; // what the water reflects; alpha 0 where only sky
[[vk::binding(5)]] Sampler2DArray surfaceAlbedo;  // photographs of surfaces, one layer per kind
[[vk::binding(6)]] Sampler2DArray surfaceNormal;  // their bumps, OpenGL convention

// A lamp (RFC 0021 §4). Its illuminance on a surface at distance d, θ off its axis, is
// E = I·f(θ)·cos(incidence)/d², in renderer units: `position.w` is I times the units per lux.
struct Lamp
{
    float4 position; // xyz relative to the eye; w: axis intensity, renderer units · m²
    float4 axis;     // xyz unit, where the beam points; w: 1 for a point lamp (all round)
    float4 cone;     // x: half the beam angle (50 %), y: half the field angle (10 %), rad;
                     // z: the near limit of d, m
    float4 color;    // linear RGB of luminance 1
};
[[vk::binding(7)]] StructuredBuffer<Lamp> lamps;
// The moving surface's cells in row order, two each: η (m over the still level), ∂η/∂x, ∂η/∂y,
// whitecap and foam coverage (0-1); then the ripples' slope variance the fetch has grown
// (water_optics::fetchLimitedRipples), the way the air's stress runs (x, y), 0.
[[vk::binding(8)]] StructuredBuffer<float4> surfaceCells;
// The spray's water per cell (kg/m³), x fastest, then y, then z.
[[vk::binding(9)]] StructuredBuffer<float> sprayCells;
// The mirror pass's depth: how far what the water reflects stands from it.
[[vk::binding(10)]] Sampler2D mirrorDepth;
// The cut plants' tops (m over frame.cutLevel.x), x fastest; 1e30 where nothing is cut.
[[vk::binding(11)]] StructuredBuffer<float> cutTops;
// The trees' cut crown cells (vsTree, treeCellCut): an open-addressed table of kTreeCutSlots keys,
// 0xffffffff where empty.
[[vk::binding(12)]] StructuredBuffer<uint> treeCuts;
// Flying leaves (lagrange/world/Leaves.hpp), two each: the centre relative to the eye and the
// blade's length (m); its yaw and pitch (rad), a tone 0..1, 0.
[[vk::binding(13)]] StructuredBuffer<float4> flyingLeaves;

struct Push
{
    float4 model[3];  // rows; translation relative to the camera
    float4 baseColor; // linear RGB, alpha
    float4 surface;   // metallic, roughness, detail (1: stone-like, 0: smooth), pattern
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

// The season: 0 summer, 1 fall, 2 winter, 3 the rainy season.
int seasonNow()
{
    return int(frame.mode.z + 0.5);
}
// How grey the sky is, 0 to 1.
float overcast()
{
    int s = seasonNow();
    float season = s == 3 ? 1.0 : s == 2 ? 0.7 : s == 1 ? 0.25 : 0.0;
    // Rain and snow come from cloud: a few mm/h is overcast.
    return max(season, saturate(frame.weather.x / 4.0 + frame.weather.y * 0.7));
}

// Clouds on a plane far overhead, drifting.
float cloudAmount(float3 dir)
{
    if (dir.z <= 0.02)
        return 0.0;
    float2 at = dir.xy / (dir.z + 0.12) * 1.3 + frame.worldWrap.w * float2(0.004, 0.002);
    float shape = fbm(float3(at, 0.0), 5);
    float low = lerp(0.52, 0.2, overcast());  // an overcast day's clouds meet
    return smoothstep(low, low + 0.22, shape) * smoothstep(0.03, 0.2, dir.z);
}

// What comes from direction `dir`; `sharp` 1 with sun and clouds in detail, 0 for reflections off
// rough surfaces, where only the broad colour matters.
float3 skyByDay(float3 dir, float sharp);
float3 sky(float3 dir, float sharp)
{
    return skyByDay(dir, sharp) * frame.light.x;
}
float3 skyByDay(float3 dir, float sharp)
{
    float3 horizonGround = float3(0.56, 0.68, 0.82) * 0.92;
    float3 clear = clearSky(float3(dir.xy, max(dir.z, 0.02)));
    if (sharp > 0.0)
    {
        float clouds = cloudAmount(dir) * sharp;
        float lit = 0.75 + 0.5 * saturate(dot(dir, frame.sunDirection.xyz));
        float shade = lerp(1.0, 0.5, overcast());  // thick clouds are dark underneath
        clear = lerp(clear, float3(1.0, 1.0, 1.02) * lit * shade, clouds);
    }
    float3 grey = float3(0.5, 0.53, 0.57) * lerp(1.0, 0.72, step(2.5, frame.mode.z));
    if (sharp <= 0.0)
        clear = lerp(clear, grey, overcast());
    else
        clear = lerp(clear, grey, overcast() * 0.8);
    horizonGround = lerp(horizonGround, grey, overcast());
    return lerp(horizonGround, clear, smoothstep(-0.03, 0.02, dir.z));
}

// The spray's water at a point relative to the eye (kg/m³), trilinear between cell centres.
float sprayAt(float3 relative)
{
    uint3 n = uint3(frame.sprayDims.xyz);
    float3 f = clamp((relative - frame.sprayBox.xyz) / frame.sprayBox.w, float3(0, 0, 0),
                     float3(n) - 1.001);
    uint3 c = min(uint3(f), max(n, uint3(2, 2, 2)) - 2u);
    float3 t = f - float3(c);
    uint i = (c.z * n.y + c.y) * n.x + c.x;
    uint dy = n.x;
    uint dz = n.x * n.y;
    float a = lerp(lerp(sprayCells[i], sprayCells[i + 1u], t.x),
                   lerp(sprayCells[i + dy], sprayCells[i + dy + 1u], t.x), t.y);
    float b = lerp(lerp(sprayCells[i + dz], sprayCells[i + dz + 1u], t.x),
                   lerp(sprayCells[i + dz + dy], sprayCells[i + dz + dy + 1u], t.x), t.y);
    return lerp(a, b, t.z);
}

// The spray's optical depth from the eye to `relative`: its extinction (sprayDims.w per kg/m³ of
// water, drops much larger than the wavelength) along the segment's stretch inside its box.
float sprayDepth(float3 relative)
{
    if (any(frame.sprayDims.xyz < 2.0))
        return 0.0;
    float3 lo = frame.sprayBox.xyz - 0.5 * frame.sprayBox.w;
    float3 hi = lo + frame.sprayDims.xyz * frame.sprayBox.w;
    float3 inv = 1.0 / (abs(relative) > 1e-6 ? relative : float3(1e-6, 1e-6, 1e-6));
    float3 t0 = (lo) * inv;
    float3 t1 = (hi) * inv;
    float enter = max(max(min(t0.x, t1.x), min(t0.y, t1.y)), max(min(t0.z, t1.z), 0.0));
    float leave = min(min(max(t0.x, t1.x), max(t0.y, t1.y)), min(max(t0.z, t1.z), 1.0));
    if (leave <= enter)
        return 0.0;
    static const int kSteps = 24;
    float step = (leave - enter) / float(kSteps);
    float water = 0.0;
    for (int k = 0; k < kSteps; ++k)
        water += sprayAt(relative * (enter + (float(k) + 0.5) * step));
    return water * frame.sprayDims.w * step * length(relative);
}

float3 fogged(float3 color, float3 relative)
{
    float distance = length(relative);
    int season = seasonNow();
    float amount = 1.0 - exp(-distance * (season == 3 ? 0.005 : season == 2 ? 0.0028 : 0.0009));
    float3 dir = relative / max(distance, 1e-3);
    float3 far = sky(float3(dir.xy, max(dir.z, 0.03)), 0.0);
    color = lerp(color, far, amount);
    // The spray's mist: what it takes out of the light, scattered back as the daylight's white.
    float tau = sprayDepth(relative);
    if (tau > 0.0)
    {
        float sunUp = saturate(frame.sunDirection.z);
        float3 mist = float3(0.80, 0.83, 0.86) * (0.45 + 0.65 * sunUp) * frame.light.x;
        color = lerp(color, mist, 1.0 - exp(-tau));
    }
    return color;
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

// Light from direction L (unit) by GGX: the reflected radiance per unit of illuminance on a
// surface facing L, times cos(incidence). A Lambertian surface gives albedo/π·NL.
float3 direct(float3 N, float3 V, float3 L, float NV, float3 diffuseColor, float3 F0, float roughness)
{
    float NL = saturate(dot(N, L));
    if (NL <= 0.0)
        return float3(0.0, 0.0, 0.0);
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
    return (diffuseColor / PI + specular) * NL;
}

// A lamp beam's relative intensity θ off its axis: lights::taper (modules/lights/src/Lamp.cpp),
// 1 on the axis, 0.5 at half the beam angle, 0.1 at half the field angle, cosine-squared between,
// 0 beyond.
float taper(Lamp lamp, float cosTheta)
{
    if (lamp.axis.w > 0.5)
        return 1.0;
    float theta = acos(clamp(cosTheta, -1.0, 1.0));
    float beam = lamp.cone.x;
    float field = lamp.cone.y;
    if (theta <= beam)
    {
        float c = cos(0.5 * PI * theta / beam);
        return 0.5 + 0.5 * c * c;
    }
    if (theta <= field)
    {
        float c = cos(0.5 * PI * (theta - beam) / max(field - beam, 1e-6));
        return 0.1 + 0.4 * c * c;
    }
    return 0.0;
}

// The lamps' light on a surface at `relative`, unshadowed (RFC 0021 §4: shadows are phase 2).
float3 lampLight(float3 relative, float3 N, float3 V, float NV, float3 diffuseColor, float3 F0, float roughness)
{
    float3 sum = float3(0.0, 0.0, 0.0);
    int count = int(frame.light.y + 0.5);
    if (count == 0)
        return sum;
    if (frame.mode.x > 0.5)
    {
        relative.z = 2.0 * frame.mode.y - relative.z;  // the mirrored point's real place
        N.z = -N.z;
        V.z = -V.z;
    }
    for (int i = 0; i < count; ++i)
    {
        Lamp lamp = lamps[i];
        float3 toLamp = lamp.position.xyz - relative;
        float d = max(length(toLamp), lamp.cone.z);
        float3 L = toLamp / d;
        float f = taper(lamp, dot(-L, lamp.axis.xyz));
        if (f <= 0.0)
            continue;
        sum += direct(N, V, L, NV, diffuseColor, F0, roughness) * lamp.color.rgb * (lamp.position.w * f / (d * d));
    }
    return sum;
}

float3 shadeSurface(float3 relative, float3 N, float3 albedo, float metallic, float roughness, float3 glow)
{
    float3 V = normalize(-relative);
    float NV = saturate(dot(N, V)) + 1e-3;
    float3 L = frame.sunDirection.xyz;
    float3 F0 = lerp(float3(0.04, 0.04, 0.04), albedo, metallic);
    float3 diffuseColor = albedo * (1.0 - metallic);

    // The sun, by GGX.
    float3 color = float3(0.0, 0.0, 0.0);
    if (frame.sunDirection.w > 0.0 && dot(N, L) > 0.0)
        color = direct(N, V, L, NV, diffuseColor, F0, roughness) * frame.sunColor.rgb * frame.sunDirection.w *
                sunLight(relative, N);
    color += lampLight(relative, N, V, NV, diffuseColor, F0, roughness);

    // The sky: broad light from above and the ground's bounce, and its reflection, blurred with
    // roughness (Lazarov's analytic split-sum).
    float3 irradiance = lerp(float3(0.12, 0.115, 0.105), float3(0.38, 0.46, 0.60), N.z * 0.5 + 0.5) * frame.light.x;
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

// What a kind of surface looks like up close, in place of a photograph: drawn from the world
// position, so it needs no texture coordinates and no asset. `pattern` is SurfacePattern's value.
float hash2(float2 p)
{
    return hash(float3(p, 3.7));
}

// (along a wall or across a floor, up a wall or along a floor), metres.
float2 surfaceCoords(float3 world, float3 N)
{
    float3 a = abs(N);
    if (a.z > 0.7)
        return world.xy;
    return float2(a.x > a.y ? world.y : world.x, world.z);
}

// 0 in a joint of a width in cells, 1 on the face, for a cell coordinate's fraction.
float joint(float2 f, float2 width)
{
    float2 e = smoothstep(float2(0.0), width, f) * smoothstep(float2(0.0), width, 1.0 - f);
    return e.x * e.y;
}

// A photograph tiled every `tile` metres (a power of two, so that it survives the 1024 m wrap of
// `world`), keeping the material's own colour: its average becomes `albedo`.
void imageSurface(float layer, float tile, float bump, float3 world, inout float3 N, inout float3 albedo)
{
    float2 uv = surfaceCoords(world, N) / tile;
    float3 a = abs(N);
    float3 T = a.z > 0.7 ? float3(1, 0, 0) : a.x > a.y ? float3(0, 1, 0) : float3(1, 0, 0);
    float3 B = a.z > 0.7 ? float3(0, 1, 0) : float3(0, 0, 1);
    float3 picture = surfaceAlbedo.Sample(float3(uv, layer)).rgb;
    float3 average = surfaceAlbedo.SampleLevel(float3(0.5, 0.5, layer), 20.0).rgb;
    float3 bumps = surfaceNormal.Sample(float3(uv, layer)).rgb * 2.0 - 1.0;
    // Mostly its light and dark, a little of its hue: the material says what colour it is.
    float3 ratio = picture / max(average, float3(0.02));
    float light = dot(ratio, float3(0.299, 0.587, 0.114));
    albedo *= lerp(float3(light), ratio, 0.2);
    N = normalize(N + (T * bumps.x + B * bumps.y) * bump);
    albedo *= 0.84 + 0.32 * fbm(world * 0.23, 3);  // no two tiles alike from afar
}

// Layer and tile of a kind that has a photograph: 0 when it has none.
float2 imageOf(int kind)
{
    switch (kind)
    {
    case 1: return float2(0.0, 2.0);   // brick
    case 2: return float2(1.0, 4.0);   // asphalt
    case 3: return float2(2.0, 2.0);   // pavement
    case 4: return float2(3.0, 4.0);   // plaster
    case 5: return float2(4.0, 4.0);   // concrete
    case 6: return float2(5.0, 4.0);   // lawn
    case 10: return float2(6.0, 1.0);  // bark
    case 11: return float2(7.0, 4.0);  // roof
    default: return float2(0.0, 0.0);
    }
}


// --- plants cut by propellers ---------------------------------------------------------------
// ground.md "Cutting the plants": the physics' map of cut tops, a world grid. Where a cell is cut,
// grass, reeds and shrubs stand no taller than its top; the cut ends show pale.

static const float kUncut = 1e29;

// The plants' cut top over `relative` (xy relative to the eye), as a height relative to the eye;
// above kUncut where nothing is cut. `wobble` (m) moves the lookup a little, so the cells'
// square edges do not show.
float cutTopAt(float2 relative, float wobble)
{
    uint nx = uint(frame.cutGrid.w);
    uint ny = uint(frame.cutLevel.y);
    if (nx == 0u || ny == 0u)
        return 1e30;
    float2 at = relative;
    if (wobble > 0.0)
    {
        float2 w = (frame.worldWrap.xy + relative) * 23.0;
        at += (float2(noise3(float3(w, 3.0)), noise3(float3(w, 9.0))) - 0.5) * wobble;
    }
    float2 f = (at - frame.cutGrid.xy) / frame.cutGrid.z + 0.5;
    int2 c = int2(floor(f));
    if (c.x < 0 || c.y < 0 || c.x >= int(nx) || c.y >= int(ny))
        return 1e30;
    float top = cutTops[uint(c.y) * nx + uint(c.x)];
    return top > kUncut ? 1e30 : frame.cutLevel.x + top;
}

// --- grass ----------------------------------------------------------------------------------
// Kinds (surfacePatternOf): 6 lawn/grass, 12 tall grass, 13 wet grass, 14 dry grass. Near the eye
// the blades stand up off the ground: the view ray is marched down through a canopy of the kind's
// height (parallax, as in Tatarchuk's parallax occlusion mapping, 2006), only in colour and normal:
// the depth and instance id stay those of the ground, so camera sensors' depth does not change.
// Blades lean with the wind (`frame.weather.zw`, m/s, the way it blows) and gusts run over the
// field as waves at the wind's speed; rain (`frame.weather.x`, mm/h) darkens and wets them and
// bows them a little. The looks are drawn, not measured: TODO(verify) heights and leaning against
// photographs; heights follow the surface presets' canopyHeight (core/world/src/World.cpp).
// TODO(rotor downwash): the realistic renderer has no rotor or `aero/wind_speed` input; flattening
// under a hovering drone needs a per-frame list of rotor discs (position, thrust) in Frame.

int grassOf(int kind)
{
    if (kind == 6) return 1;   // lawn
    if (kind == 12) return 2;  // tall grass
    if (kind == 13) return 3;  // wet grass
    if (kind == 14) return 4;  // dry grass
    return 0;
}

struct GrassLook
{
    float height;   // m, of the tallest blades
    float spacing;  // m, between blades
    float width;    // of a blade at its root, as a share of the spacing
    float stiff;    // how little the wind bends it: 1 stands, 0 lies down
    float wet;      // 0..1 of its own, before the rain
    float3 tint;    // on the material's colour
};

GrassLook grassLook(int grass)
{
    GrassLook g;
    g.height = 0.05; g.spacing = 0.014; g.width = 0.55; g.stiff = 0.8; g.wet = 0.0;
    g.tint = float3(1.0, 1.0, 1.0);
    if (grass == 2) { g.height = 0.4; g.spacing = 0.03; g.width = 0.42; g.stiff = 0.35; g.tint = float3(1.08, 1.05, 0.82); }
    if (grass == 3) { g.wet = 1.0; g.tint = float3(0.9, 0.95, 0.82); }
    if (grass == 4) { g.height = 0.12; g.spacing = 0.02; g.width = 0.45; g.stiff = 0.55; g.tint = float3(1.45, 1.25, 0.72); }
    return g;
}

// The air moving over the ground at `world` (the same coordinates as grassBend's), m/s in
// world xy: the site wind, plus (to come) the outwash of hovering rotors. Grass and vegetation
// read the air only through this, so the downwash reaches all of them in one place.
// Rotor outwash (GroundAir.hpp's outwashProfile, the fit to Tanner et al. 2015 of
// aerodynamics::Downwash): speed at radius rho and height z for a jet arriving at `arriving` m/s
// with top-hat radius `jetRadius`.
float outwashProfile(float arriving, float jetRadius, float rho, float z)
{
    const float calArriving = 1.70710678; // 1 + 1/sqrt(2)
    float r = jetRadius * sqrt(calArriving);
    float peakRadius = 1.7 * r;
    float peak = arriving * 1.9 / calArriving;
    float radial = rho < peakRadius ? pow(rho / peakRadius, 0.8) : pow(peakRadius / rho, 1.1);
    float zPeak = 0.05 * r;
    float layer = 0.15 * r + 0.073 * max(0.0, rho - peakRadius);
    float vertical = z <= zPeak ? pow(max(z, 0.0) / zPeak, 1.0 / 7.0)
                                : exp(-0.693147 * ((z - zPeak) / (layer - zPeak)) * ((z - zPeak) / (layer - zPeak)));
    return peak * radial * vertical;
}

float2 groundAir(float3 world)
{
    float2 air = frame.weather.zw;
    float3 relative = world - frame.worldWrap.xyz;
    float t = frame.worldWrap.w;
    int count = min(int(frame.light.w), kAirJets);
    for (int i = 0; i < count; ++i)
    {
        float4 a = frame.airJets[2 * i];
        float jetRadius = frame.airJets[2 * i + 1].x;
        float2 d = relative.xy - a.xy;
        float rho = length(d);
        float z = relative.z - a.z;
        if (rho < 1e-4 || z < -0.5)  // well below the surface it meets: under a roof
            continue;
        // The blades stand taller than the wall layer: they feel its peak (GroundAir.hpp).
        float r = jetRadius * sqrt(1.70710678);
        float v = outwashProfile(a.w, jetRadius, rho, max(z, 0.05 * r));
        // The outwash is unsteady (Tanner et al. 2015 report fluctuations of the order of the
        // mean): drawn as gusts running outward at about half its speed, rings broken by noise.
        // Their size and strength are a drawing choice. TODO(verify): the measured spectrum.
        float wave = rho / max(jetRadius, 0.05) - t * 0.5 * a.w / max(jetRadius, 0.05);
        float angle = atan2(d.y, d.x);
        float n = noise3(float3(cos(angle) * 3.0 + rho * 1.5, sin(angle) * 3.0, t * 1.7));
        float pulse = 1.0 + 0.45 * sin(wave * 2.2) * (0.5 + n) + 0.25 * (n - 0.5);
        air += d / rho * v * max(pulse, 0.0);
    }
    return air;
}

// How far air of `speed` m/s lays over vegetation of stiffness `stiff` (0 limp .. 1 stiff), 0..1.
// Plants bend by the dynamic pressure q = ½ρv² against their stiffness (the Cauchy number,
// de Langre, "Effects of wind on plants", Annu. Rev. Fluid Mech. 40, 2008, §3): the lean is
// q / (q + q½), with q½ the pressure that lays them half over. q½ = 30·stiff² Pa is a drawing
// gain, TODO(verify) against measured lodging of turf and reeds; ρ is ISA sea level.
float airPush(float speed, float stiff)
{
    float q = 0.5 * 1.225 * speed * speed;
    float half = 30.0 * stiff * stiff + 0.5;
    return q / (q + half);
}

// How far the blades' tips are pushed, m per m of height, in world xy: lean, gusts, sway.
float2 grassBend(float3 world, float stiff, float wetness)
{
    float2 wind = groundAir(world);
    float speed = length(wind);
    float t = frame.worldWrap.w;
    float2 along = speed > 0.05 ? wind / speed : float2(1.0, 0.0);
    float2 across = float2(-along.y, along.x);
    // Gusts: a noise field carried with the wind, so bright and dark bands of bent blades run
    // over the field at its speed.
    float x = dot(world.xy, along) - speed * fmod(t, 600.0);
    float y = dot(world.xy, across);
    float gust = noise3(float3(x * 0.18, y * 0.18, 0.0)) * 0.7 + noise3(float3(x * 0.6, y * 0.6, 3.0)) * 0.3;
    float push = airPush(speed, stiff);
    // Most of the lean steady, the rest in the gusts; near flat (1.4: the tip at the root's
    // height) when the pressure far exceeds the stiffness.
    float lean = 1.4 * push * (0.6 + 0.4 * gust * 1.3);
    // Each tuft sways at its own about 1-2 Hz, more in more wind.
    float phase = hash2(floor(world.xy * 6.0)) * 6.283;
    float sway = sin(t * (6.0 + 5.0 * push) + phase + x * 1.5) * (0.04 + 0.18 * push) * (1.0 - 0.5 * stiff);
    float2 bend = along * min(lean, 1.1) + across * sway;
    // Wet blades bow under their water, all round.
    float2 droop = float2(hash2(floor(world.xy * 40.0)) - 0.5, hash2(floor(world.xy * 40.0) + 7.0) - 0.5);
    return bend + droop * 0.5 * wetness;
}

// How far from the eye grass blades of `kind` are drawn, m (Realistic.cpp grassTiles).
float grassReach(int kind)
{
    if (kind == 12) return 24.0;
    if (kind == 14) return 10.0;
    return 8.0;
}

void grassSurface(int kind, float3 world, float distance, float mid, float near, inout float3 N,
                  inout float3 albedo, inout float roughness)
{
    GrassLook g = grassLook(grassOf(kind));
    float rain = saturate(log2(1.0 + frame.weather.x) / 3.5);  // ~ soaked by 10 mm/h
    float wetness = max(g.wet, rain);
    float2 bend = grassBend(world, g.stiff, wetness);
    float full = g.height * (1.0 - 0.15 * wetness);
    // Cut by a propeller: the blades end at the cut (the drawn ground is the soil, as in the
    // physics), stiff stubble the air hardly bends.
    float3 relative = world - frame.worldWrap.xyz;
    float cutTop = cutTopAt(relative.xy, 0.03) - relative.z;
    float height = clamp(cutTop, 0.004, full);
    float cut = saturate(1.0 - height / max(full, 1e-4));  // 0 uncut .. 1 to the ground
    bend *= 1.0 - 0.8 * cut;
    // Far: the old flat lawn (or its photograph), bands of bent blades show lighter.
    float2 image = imageOf(kind);
    float3 base = albedo * g.tint;
    float3 flat = base;
    float3 Nflat = N;
    if (frame.mode.w > 0.5 && image.y > 0.0)
        imageSurface(image.x, image.y, 1.0, world, Nflat, flat);
    else
    {
        float blades = noise3(float3(world.xy * float2(38.0, 44.0), 1.0));
        float dry = smoothstep(0.45, 0.7, fbm(world * 0.17, 4)) * (grassOf(kind) == 4 ? 0.3 : 1.0);
        flat = lerp(flat, flat * float3(1.75, 1.35, 0.8), dry);
        flat *= 0.55 + 0.75 * mid + 0.55 * blades * near;
        Nflat = normalize(N + float3(blades - 0.5, noise3(world * 31.0) - 0.5, 0.0) * 0.4 * near);
    }
    // Blades bent away show their pale backs and catch the sky.
    // Pressed flat (the outwash, a gale), their backs and the sheen of the laid blades show as a
    // lighter, greyer band that moves with the pulses of the air. Gains: TODO(verify).
    float laid = saturate(length(bend) / 1.2);
    laid = laid * laid;
    flat *= 1.0 + 0.15 * saturate(length(bend)) * (grassOf(kind) == 2 ? 1.0 : 0.5);
    flat = lerp(flat, flat * 1.9 + float3(0.05, 0.06, 0.04), 0.9 * laid);
    roughness = lerp(roughness, 0.45, 0.6 * laid);
    // Cut: the pale cut ends and the clippings over a shorter, sparser green; cut to the ground,
    // the soil between the roots. Gains: a drawing choice, TODO(verify) against photographs.
    if (cut > 0.0)
    {
        float3 stubble = base * float3(1.35, 1.3, 0.78) + float3(0.03, 0.03, 0.015);
        float3 soil = base * 0.22 * float3(1.1, 0.85, 0.6) + float3(0.04, 0.03, 0.02);
        float clippings = smoothstep(0.55, 0.8, noise3(float3(world.xy * 55.0, 5.0)));
        float3 mown = lerp(stubble, base * float3(1.6, 1.45, 0.7), clippings * 0.5);
        flat = lerp(flat, mown, saturate(cut * 1.6));
        flat = lerp(flat, soil, smoothstep(0.75, 1.0, cut));
    }
    float3 outAlbedo = flat;
    float3 outN = Nflat;
    float up = smoothstep(0.6, 0.85, N.z);
    // Under real blades (vsGrassBlade) the ground is what shows between them: the canopy's
    // lower, shaded leaves and thatch, darker where it stands taller; cut, the stubble and soil
    // as they are.
    float reach = grassReach(kind);
    float cover = frame.cutLevel.z * (1.0 - smoothstep(reach * 0.6, reach, distance)) * up;
    if (cover > 0.0)
    {
        float thatch = noise3(float3(world.xy * 9.0, 2.0));
        // (Darker made a lawn's blade zone a dark disc round the eye, seen from a few metres up.)
        float shade = lerp(0.8 - 0.45 * saturate(full / 0.5), 1.0, saturate(cut * 1.4));
        outAlbedo = lerp(flat, flat * shade * (0.8 + 0.4 * thatch), cover);
    }
    // Fade the blades out before one is narrower than a pixel, where they would only shimmer.
    // Under a lawn's blades its fine parallax canopy stays (the many short blades the geometry
    // leaves out); under tall grass the blades are enough, and its coarse canopy only shows dots.
    float close = (1.0 - smoothstep(3.0, 6.0 + 30.0 * height, distance)) * up *
                  (1.0 - cover * saturate(full / 0.2));
    if (close > 0.01)
    {
        // March down from the canopy's top (the drawn ground) to the soil.
        float3 d = normalize(world - frame.worldWrap.xyz);
        float down = max(-d.z, 0.12);
        static const int kSteps = 14;
        float dz = height / float(kSteps);
        float3 hitColor = base * 0.16 * float3(0.9, 0.75, 0.6);  // the soil, between the roots
        float3 hitN = float3(0.0, 0.0, 1.0);
        for (int i = 0; i < kSteps; ++i)
        {
            float z = height - (float(i) + 0.5) * dz;   // above the soil
            float2 q = world.xy + d.xy * ((float(i) + 0.5) * dz / down);
            float f = z / height;                        // 0 root .. 1 top
            // Where this blade grew: undo its bend at this height (it curves, tips most).
            float2 root = q - bend * height * f * f;
            float2 cell = floor(root / g.spacing);
            float2 local = root / g.spacing - cell;
            float2 centre = float2(hash2(cell), hash2(cell + 17.0)) * 0.6 + 0.2;
            float own = 0.55 + 0.45 * hash2(cell + 5.0);  // this blade's height share
            float2 off = local - centre;
            float r = length(off);
            // A cut blade ends at the cut, blunt: its width there is what it had at that height.
            float uncutTop = own * full / height;  // its own top, as a share of the drawn height
            bool cutBlade = uncutTop > 1.0;
            float shape = f / uncutTop;  // 0 root .. 1 its own (uncut) tip
            if (f < min(uncutTop, 1.0) && r < g.width * 0.5 * (1.0 - shape * 0.85))
            {
                float tone = hash2(cell + 9.0);
                float3 c = base * lerp(float3(0.7, 0.78, 0.6), float3(1.25, 1.15, 0.85), tone);
                if (grassOf(kind) == 2 || grassOf(kind) == 4)
                    c = lerp(c, base * float3(1.6, 1.35, 0.7), smoothstep(0.75, 1.0, tone) * 0.7);  // seed heads, straw
                // Dark down in the canopy, where little light gets in.
                c *= lerp(0.28, 1.15, pow(saturate(shape), 0.8));
                if (cutBlade && f > 0.85)
                    c = base * float3(1.5, 1.4, 0.8) + float3(0.04, 0.04, 0.02);  // the cut end
                hitColor = c;
                float2 side = r > 1e-4 ? off / r : float2(0.0);
                hitN = normalize(float3(side * 0.35 + bend * 0.3, 1.0));
                break;
            }
        }
        // Filter by the pixel's footprint: where a pixel covers several blades, show their
        // average (the flat look) instead of one blade picked at random, which sparkles.
        float footprint = distance * 2.0 * frame.cameraUp.w * frame.screen.w / down;  // m per pixel
        float resolved = 1.0 - smoothstep(0.25, 0.9, footprint / (g.spacing * g.width));
        // (The average keeps the field's own colour: pulling it towards the bare material's
        // colour desaturated the tall and wet strips.)
        hitColor = lerp(flat * 0.85, hitColor, resolved);
        hitN = normalize(lerp(Nflat, hitN, resolved));
        outAlbedo = lerp(outAlbedo, hitColor, close);
        outN = normalize(lerp(Nflat, hitN, close));
    }
    albedo = outAlbedo;
    N = outN;
    // Wet: darker (water fills the gaps of the surface), glossier, a sheen on the blades.
    albedo *= lerp(1.0, 0.62, wetness);
    // Blades are matte; water on them glosses them only so far (a sheen, not a mirror).
    // At 0.55 the wet strip mirrored the sky as a pale bluish-white sheen; the canopy hides most
    // of the sky from its own drops, so the sheen stays broad.
    roughness = max(lerp(roughness, 0.75, wetness), lerp(0.85, 0.75, wetness));
}

void surfacePattern(float pattern, float3 world, float distance, inout float3 N, inout float3 albedo,
                    inout float metallic, inout float roughness)
{
    float2 uv = surfaceCoords(world, N);
    // Fine lines give way to their average with distance, before they shimmer.
    float near = 1.0 - smoothstep(25.0, 70.0, distance);
    float wide = fbm(world * 0.23, 4);
    float mid = fbm(world * 1.9, 3);
    float wall = 1.0 - smoothstep(0.3, 0.7, abs(N.z));
    int kind = int(pattern + 0.5);
    float2 image = imageOf(kind);
    if (grassOf(kind) > 0)
        grassSurface(kind, world, distance, mid, near, N, albedo, roughness);
    else if (frame.mode.w > 0.5 && image.y > 0.0)
        imageSurface(image.x, image.y, 0.7, world, N, albedo);
    else if (kind == 15 || kind == 16)  // under reeds and shrubs: their dark litter and shade
        albedo *= (0.35 + 0.35 * mid) * lerp(float3(0.8, 0.75, 0.6), float3(1.0), wide);
    else if (kind == 1)  // brick: running bond, 225 x 75 mm with a 10 mm joint
    {
        float2 size = float2(0.235, 0.085);
        float row = floor(uv.y / size.y);
        float2 cell = float2(uv.x / size.x + 0.5 * fmod(abs(row), 2.0), uv.y / size.y);
        float2 id = floor(cell);
        float face = joint(frac(cell), float2(0.03, 0.08));
        float tint = hash2(id);
        float3 brick = albedo * (0.72 + 0.56 * tint) * (0.85 + 0.3 * mid);
        float3 mortar = float3(0.36, 0.34, 0.31);
        float3 close = lerp(mortar, brick, face);
        float3 far = lerp(mortar, albedo, 0.86) * (0.8 + 0.4 * wide);
        albedo = lerp(far, close, near);
        roughness = lerp(0.95, roughness, face);
    }
    else if (kind == 2)  // asphalt: stones in tar, patches, and pale wear
    {
        float stones = noise3(world * 42.0) * noise3(world * 17.0);
        float patch = smoothstep(0.6, 0.62, fbm(world * 0.09 + 7.0, 3));
        float crack = (1.0 - smoothstep(0.0, 0.0035, abs(fbm(world * 0.33, 4) - 0.5))) * smoothstep(0.52, 0.62, fbm(world * 0.07 + 3.0, 2));
        albedo *= 0.82 + 0.3 * wide + 0.5 * stones * near;
        albedo *= lerp(1.0, 0.8, patch);
        albedo *= lerp(1.0, 0.6, crack * near);
        roughness = saturate(roughness - 0.18 * patch);
    }
    else if (kind == 3)  // paving slabs: 600 mm, offset rows
    {
        float2 size = float2(0.6, 0.6);
        float row = floor(uv.y / size.y);
        float2 cell = float2(uv.x / size.x + 0.5 * fmod(abs(row), 2.0), uv.y / size.y);
        float2 id = floor(cell);
        float face = lerp(0.8, joint(frac(cell), float2(0.016)), near);
        float gum = smoothstep(0.72, 0.76, noise3(world * 2.8));
        albedo *= (0.86 + 0.2 * hash2(id) * near + 0.14 * wide) * lerp(0.5, 1.0, face);
        albedo *= lerp(1.0, 0.72, gum * near);
    }
    else if (kind == 4)  // plaster: stucco, stains running down from what is above
    {
        float streak = fbm(float3(uv.x * 1.3, uv.x * 0.7, uv.y * 0.07), 4);
        albedo *= 0.92 + 0.14 * (mid - 0.5) + 0.42 * (streak - 0.5) + 0.2 * (wide - 0.5);
    }
    else if (kind == 5)  // cast concrete: panels, tie holes, weather
    {
        float2 cell = uv / float2(3.0, 1.65);
        float face = lerp(0.9, joint(frac(cell), float2(0.006, 0.01)), near);
        float2 ties = frac(uv / float2(0.75, 0.55));
        float tie = 1.0 - smoothstep(0.02, 0.035, length(ties - 0.5) * 0.75);
        albedo *= (0.76 + 0.3 * hash2(floor(cell)) * 0.5 + 0.34 * wide) * lerp(0.45, 1.0, face);
        albedo *= lerp(1.0, 0.55, tie * near);
    }
    else if (kind == 7)  // leaves: clumps, light through the gaps
    {
        float clump = fbm(world * 3.1, 3);
        float fine = fbm(world * 23.0, 2);
        albedo *= 0.3 + 1.1 * clump + 0.8 * fine;
        albedo = lerp(albedo, albedo * float3(1.45, 1.2, 0.55), smoothstep(0.55, 0.8, clump));
        N = normalize(N + (float3(fbm(world * 9.0, 2), fbm(world * 9.0 + 9.0, 2), fbm(world * 9.0 + 17.0, 2)) - 0.5) * 1.6);
    }
    else if (kind == 8)  // road paint: worn through where wheels go
    {
        float wear = smoothstep(0.55, 0.72, fbm(world * 4.5, 4));
        albedo *= lerp(1.0, 0.55, wear);
        roughness = lerp(roughness, 0.95, wear);
    }
    else if (kind == 9)  // window glass: reflects the street, a dim room behind, some lit
    {
        float2 pane = floor(uv / float2(1.6, 3.3));
        float room = hash2(pane);
        float lit = step(0.93, room);
        albedo = lerp(float3(0.015, 0.02, 0.028), float3(0.75, 0.58, 0.34), lit * 0.09);
        metallic = 0.55;
        // Old float glass is not flat: the reflection wobbles from pane to pane.
        roughness = 0.03 + 0.05 * room;
        float3 wobble = float3(noise3(float3(uv * 0.9, room * 9.0)) - 0.5, noise3(float3(uv * 0.9 + 5.0, room * 9.0)) - 0.5, 0.0);
        N = normalize(N + wobble * 0.05);
    }
    else if (kind == 10)  // bark: furrows up the trunk
    {
        float furrow = noise3(float3(world.xy * 13.0, world.z * 1.3));
        albedo *= 0.45 + 1.1 * furrow;
    }
    else if (kind == 11)  // roofing: gravel and tar seams
    {
        float2 cell = uv / float2(1.0, 4.0);
        float face = lerp(0.9, joint(frac(cell), float2(0.008, 0.002)), near);
        albedo *= (0.65 + 0.7 * noise3(world * 24.0) * near + 0.35 * wide) * lerp(0.5, 1.0, face);
    }
    // Dirt where walls meet the street, rain marks under what sticks out.
    if (kind != 9 && grassOf(kind) == 0 && kind != 7 && wall > 0.5)
    {
        float foot = 1.0 - smoothstep(0.0, 0.9 + 0.5 * mid, world.z);
        albedo *= lerp(1.0, 0.62, foot);
    }
}

// What the season does to a surface, after its own pattern. Moving things (no pattern, usually)
// get only what falls on them evenly, so that nothing crawls over a passing car.
void seasonOf(float pattern, float3 world, inout float3 N, inout float3 albedo, inout float metallic,
              inout float roughness, float glowing)
{
    int season = seasonNow();
    int kind = int(pattern + 0.5);
    float up = smoothstep(0.55, 0.85, N.z);
    if (glowing > 0.01 || kind == 9)
    {
        if (season == 3)
            roughness = min(roughness, 0.1);
        return;
    }
    if (season == 1)
    {
        if (kind == 7)  // leaves: every tree its own turn of red, orange and gold
        {
            float tree = hash2(floor(world.xy / 9.0)) * 0.7 + 0.3 * fbm(world * 0.8, 2);
            float3 turn = lerp(float3(0.5, 0.06, 0.03), float3(0.78, 0.28, 0.03), smoothstep(0.15, 0.42, tree));
            turn = lerp(turn, float3(0.72, 0.5, 0.06), smoothstep(0.45, 0.72, tree));
            albedo = turn * (dot(albedo, float3(0.33)) * 3.2 + 0.12);
        }
        else if (grassOf(kind) > 0)  // tired grass, leaves lying on it
        {
            albedo = lerp(albedo, albedo.ggg * float3(1.15, 0.85, 0.42), 0.65);
            float fallen = smoothstep(0.62, 0.66, noise3(world * 4.5));
            albedo = lerp(albedo, float3(0.6, 0.27, 0.05), fallen);
        }
        else if ((kind == 3 || kind == 2) && up > 0.5)  // leaves swept to the sides
        {
            float fallen = smoothstep(0.72, 0.75, noise3(world * 3.3)) * smoothstep(0.42, 0.6, fbm(world * 0.22, 2));
            albedo = lerp(albedo, float3(0.58, 0.28, 0.06), fallen);
        }
    }
    else if (season == 2)
    {
        const float3 snow = float3(0.86, 0.88, 0.92);
        if (kind == 7)  // dark crowns under snow, thickest on top
        {
            float clump = fbm(world * 2.3, 3);
            float laden = smoothstep(-0.25, 0.55, N.z + (clump - 0.5) * 1.1);
            albedo = lerp(float3(0.045, 0.06, 0.045) * (0.6 + clump), snow, laden);
            roughness = 0.85;
            return;
        }
        float lies = up;
        if (kind == 2)  // the road: slush, ploughed and driven through
            lies *= 0.55 + 0.45 * smoothstep(0.42, 0.6, fbm(world * 0.7, 3));
        if (kind == 8)
            lies *= 0.35;
        if (lies > 0.0)
        {
            float3 cover = kind == 2 ? float3(0.42, 0.43, 0.45) * (0.7 + 0.6 * fbm(world * 2.5, 3)) : snow;
            albedo = lerp(albedo, cover, lies);
            roughness = lerp(roughness, kind == 2 ? 0.4 : 0.75, lies);
            metallic = lerp(metallic, 0.0, lies);
            if (kind != 2)
                N = normalize(N + (float3(noise3(world * 2.7), noise3(world * 2.7 + 5.0), 0.5) - 0.5) * 0.18 * lies);
        }
    }
    else if (season == 3)
    {
        // Everything soaked: darker, and shining where it is level.
        float porous = grassOf(kind) > 0 || kind == 7 ? 0.75 : 0.55;
        albedo *= lerp(1.0, porous, 0.6 + 0.4 * up);
        roughness *= lerp(0.72, 0.32, up);
        if (up > 0.5 && (kind == 2 || kind == 3 || kind == 8))
        {
            float puddle = smoothstep(0.5, 0.56, fbm(world * 0.28 + 11.0, 4));
            if (puddle > 0.0)
            {
                albedo *= lerp(1.0, 0.5, puddle);
                roughness = lerp(roughness, 0.02, puddle);
                // Rings where drops land.
                float2 cell = floor(world.xy * 1.6);
                float2 f = frac(world.xy * 1.6) - 0.5;
                float born = hash2(cell);
                float age = frac(frame.worldWrap.w * 0.8 + born * 7.0);
                float ring = sin((length(f) - age * 0.55) * 60.0) * smoothstep(0.1, 0.0, abs(length(f) - age * 0.55)) * (1.0 - age);
                float2 away = length(f) > 1e-3 ? f / length(f) : float2(0.0);
                N = normalize(lerp(float3(0.0, 0.0, 1.0), N, 0.15 * (1.0 - puddle)) + float3(away * ring * 0.12, 0.0));
            }
        }
    }
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
    float metallic = push.surface.x;
    if (push.surface.w > 0.5)
        surfacePattern(push.surface.w, world, length(input.relative), N, albedo, metallic, roughness);
    // Grass carries its own detail (blades, tufts): the stone-like grain on top of it only greys
    // it and tilts its normals towards the sky's bluish reflection, which made the tall and wet
    // strips look pale bluish-white and grainy near the eye.
    if (grassOf(int(push.surface.w + 0.5)) == 0)
        surfaceDetail(world, N, albedo, roughness);
    seasonOf(push.surface.w, world, N, albedo, metallic, roughness, dot(push.emissive.rgb, float3(1.0)));
    if (push.baseColor.a < 1.0)
        albedo *= push.baseColor.a;  // glass shows little of its own colour, mostly reflections
    float3 color = shadeSurface(input.relative, N, albedo, metallic, roughness, push.emissive.rgb);
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
    float seen = (1.0 - overcast()) * frame.light.x;
    color += frame.sunColor.rgb * disc * 60.0 * smoothstep(0.0, 0.03, dir.z) * (1.0 - cloudAmount(dir)) * seen;
    color += frame.sunColor.rgb * pow(saturate(cosSun), 900.0) * 0.5 * seen;  // the glow round it
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

// What falls between the eye and everything: drops of rain and flakes of snow in the world, on a
// lattice of cells that moves with them (the wind across, their fall speed down), one candidate
// per cell, found by walking the eye's ray through the cells (Amanatides and Woo, 1987) up to
// what the pixel sees. A drop is a short streak along its motion (what a camera's exposure
// smears it into), a flake a soft dot that sways. How many are drawn grows with the rate; that
// and their sizes are drawing choices, the real number of drops (thousands per m³) is not.
// The season's look alone, with no weather, still brings some (rainy: rain, winter: snow).
float fallingKind(float3 dir, float sceneM, float rate, float snowy)
{
    // `snowy` 1 for snow, 0 for rain: a float, since a bool parameter's selects read as 0 here.
    bool snow = snowy > 0.5;
    float t = frame.worldWrap.w;
    float cellM = lerp(0.6, 0.5, snowy);
    float farM = lerp(22.0, 18.0, snowy);
    // Rain's mass-weighted fall speed at the rate (Atlas over Marshall and Palmer drops, as the
    // aerodynamics domain has it); snow about 1 m/s.
    float slope = 4.1 * pow(max(rate, 0.01), -0.21);
    float fall = lerp(max(0.5, 9.65 - 10.3 * pow(slope / (slope + 0.6), 4.0)), 1.0, snowy);
    float3 velocity = float3(frame.weather.z, frame.weather.w, -fall);
    float present = saturate(0.1 + 0.16 * log2(1.0 + rate));
    float pixel = 2.0 * frame.cameraUp.w * frame.screen.w;  // rad per pixel
    float exposure = lerp(0.033, 0.0, snowy);  // s a streak shows: a 30 fps frame
    float radius = lerp(0.004, 0.009, snowy);  // m, drawn
    // The eye in the falling lattice's frame; the lattice repeats every 64 cells.
    float period = 64.0 * cellM;
    float3 o = fmod(frame.worldWrap.xyz - velocity * fmod(t, 600.0), period) + period;
    float3 p = o / cellM;
    float3 cell = floor(p);
    float3 stepDir = sign(dir);
    float3 inv = 1.0 / max(abs(dir), 1e-5);
    float3 delta = inv * cellM;
    float3 next = (stepDir * (cell - p) + max(stepDir, 0.0)) * delta;
    float sum = 0.0;
    float stopM = min(sceneM, farM);
    float3 along = normalize(velocity);
    float streak = length(velocity) * exposure;
    for (int i = 0; i < 64; ++i)
    {
        float3 id = fmod(cell, 64.0);
        if (hash(id + 0.37) < present)
        {
            float3 jitter = float3(hash(id + 1.7), hash(id + 5.3), hash(id + 9.1)) * 0.8 + 0.1;
            float3 centre = (cell + jitter) * cellM - o;  // relative to the eye
            if (snow)
            {
                float phase = hash(id + 13.0) * 6.283;
                centre.xy += 0.12 * cellM * float2(sin(t * 1.3 + phase), cos(t * 1.1 + phase));
            }
            // The streak's nearest point to the ray: the segment centre ± along·streak/2.
            float3 a = centre - along * (streak * 0.5);
            float3 ab = along * streak;
            float3 w = a;
            float b = dot(dir, ab);
            float c = dot(ab, ab);
            float d = dot(dir, w);
            float e = dot(ab, w);
            float denom = c - b * b;
            float s = c > 1e-8 && denom > 1e-8 ? saturate((d * b - e) / denom) : 0.0;
            float3 q = a + ab * s;
            float tr = dot(q, dir);
            if (tr > 0.05 && tr < stopM)
            {
                float miss = length(q - dir * tr);
                float width = max(radius, tr * pixel);
                // Smaller than a pixel: fainter, not wider.
                float core = saturate(1.0 - miss / width);
                float fade = saturate((stopM - tr) / (0.4 * stopM));
                sum += core * core * pow(min(1.0, radius / (tr * pixel)), 0.6) * fade;
            }
        }
        float nearest = min(next.x, min(next.y, next.z));
        if (nearest > stopM)
            break;
        if (next.x <= nearest)
        {
            cell.x += stepDir.x;
            next.x += delta.x;
        }
        else if (next.y <= nearest)
        {
            cell.y += stepDir.y;
            next.y += delta.y;
        }
        else
        {
            cell.z += stepDir.z;
            next.z += delta.z;
        }
    }
    return sum;
}

float3 falling(float2 uv, float depth)
{
    float rain = frame.weather.x;
    float snow = frame.weather.y;
    int season = seasonNow();
    if (rain <= 0.0 && snow <= 0.0)
    {
        rain = season == 3 ? 4.0 : 0.0;
        snow = season == 2 ? 1.0 : 0.0;
    }
    if (rain <= 0.0 && snow <= 0.0)
        return float3(0.0);
    float2 ndc = float2(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0);
    float3 dir = normalize(frame.cameraForward.xyz + ndc.x * frame.cameraRight.w * frame.cameraRight.xyz +
                           ndc.y * frame.cameraUp.w * frame.cameraUp.xyz);
    // Reversed depth with no far plane: the view depth is near / depth; the sky is endless.
    float limit = depth > 0.0 ? frame.cameraForward.w / depth / max(dot(dir, frame.cameraForward.xyz), 1e-3)
                              : 1e9;
    float3 color = float3(0.0);
    if (rain > 0.0)
        color += float3(0.62, 0.67, 0.74) * min(fallingKind(dir, limit, rain, 0.0), 1.0);
    if (snow > 0.0)
        color += float3(0.95, 0.96, 1.0) * min(fallingKind(dir, limit, snow, 1.0), 1.0);
    return color;
}

// FXAA in the manner of Lottes (2009): smooths edges by their local contrast.
[shader("fragment")]
float4 fsComposite(ScreenOutput input) : SV_Target
{
    float2 uv = input.uv;
    if (frame.light.z > 0.5)
        return float4(sceneColor.SampleLevel(uv, 0.0).rgb, 1.0);  // linear, as calibrated
    float2 px = frame.screen.zw;
    float depth = sceneDepth.SampleLevel(uv, 0.0).r;
    float ao = occlusion(uv, depth);
    float3 glow = bloom(uv);  // once: the edge path below adds it again
    float3 here = sceneColor.SampleLevel(uv, 0.0).rgb * ao + glow;
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
        return float4(center + falling(uv, depth), 1.0);
    float2 dir = float2(lumaS - lumaN, lumaW - lumaE);
    float reduce = max(range * 0.25, 1.0 / 128.0);
    dir = clamp(dir / (min(abs(dir.x), abs(dir.y)) + reduce), -8.0, 8.0) * px;
    float3 a = 0.5 * (sceneColor.SampleLevel(uv + dir * (1.0 / 3.0 - 0.5), 0.0).rgb +
                      sceneColor.SampleLevel(uv + dir * (2.0 / 3.0 - 0.5), 0.0).rgb);
    float3 b = 0.5 * a + 0.25 * (sceneColor.SampleLevel(uv - dir * 0.5, 0.0).rgb +
                                 sceneColor.SampleLevel(uv + dir * 0.5, 0.0).rgb);
    float lumaB = luminance(display(b));
    float3 smoothed = (lumaB < low || lumaB > high) ? a : b;
    return float4(display(smoothed * ao + glow) + falling(uv, depth), 1.0);
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
    return 0.0065 * min(wind * wind, 200.0);  // still without wind: only the physics moves it
}

// 0 in still air, 1 from a light breeze (3 m/s) up: how much of the made-up motion is drawn.
float windShare()
{
    return saturate(frame.water.y / 3.0);
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

// The physics' moving surface at a point (relative to the eye, x and y), bilinear between cell
// centres: η, its slope, the foam; and in `rough`, the ripples' slope variance there and the way
// they run (water_optics, from the air's friction on the cell and the wind's). `share` is how
// much of it is there: 0 outside the patch, rising to 1 over its outer cells, where the patch's
// sponge has already calmed it.
float4 movingSurface(float2 relativeXY, out float share, out float4 rough)
{
    share = 0.0;
    rough = float4(frame.surfaceLook.y, frame.surfaceLook.zw, 0.0);
    uint n = uint(frame.surface.w);
    if (n < 2u)
        return float4(0, 0, 0, 0);
    float2 f = (relativeXY - frame.surface.xy) / frame.surface.z;
    float last = float(n - 1u);
    if (any(f < 0.0) || any(f > last))
        return float4(0, 0, 0, 0);
    uint2 c = min(uint2(f), uint2(n - 2u, n - 2u));
    float2 t = f - float2(c);
    uint i = c.y * n + c.x;
    float4 v = lerp(lerp(surfaceCells[2u * i], surfaceCells[2u * (i + 1u)], t.x),
                    lerp(surfaceCells[2u * (i + n)], surfaceCells[2u * (i + n + 1u)], t.x), t.y);
    float4 r = lerp(lerp(surfaceCells[2u * i + 1u], surfaceCells[2u * (i + 1u) + 1u], t.x),
                    lerp(surfaceCells[2u * (i + n) + 1u], surfaceCells[2u * (i + n + 1u) + 1u], t.x), t.y);
    float edge = min(min(f.x, f.y), min(last - f.x, last - f.y));
    share = saturate(edge / 4.0);
    // The air's ripples fade to the wind's round the patch's middle, over its outer third: the
    // outwash goes on past the patch, which only bounds where it is known, and falls with the
    // distance from the wake (a circle, not the patch's square, so that no edge is drawn).
    float away = length(f - 0.5 * last) / (0.5 * last);
    rough = lerp(rough, r, saturate((1.0 - away) / 0.35));
    return v;
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
    // The physics' surface on top of the drawn swell: linear waves add.
    float share;
    float4 rough;
    float lift = movingSurface(relativeXY, share, rough).x * share;  // its true height; relief: slope
    displacement.z += lift;
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

// A dancing web of light on what lies under the water.
float caustics(float2 p, float t)
{
    float2 a = p * 2.1 + float2(t * 0.31, t * 0.23);
    float2 b = p * 2.7 - float2(t * 0.27, -t * 0.21);
    float n = abs(noise3(float3(a, t * 0.2)) - 0.5) + abs(noise3(float3(b, t * 0.25)) - 0.5);
    return pow(saturate(1.0 - n * 2.4), 5.0);
}

// --- the water's optics (docs/physics/water.md "How the water looks") -------------------------
//
// The surface is three scales of slope: the drawn swell (Gerstner) and the physics' moving
// surface η, which the mesh and its cells hold; the capillary ripples the air raises below the
// grid (λ 4.8 to 0.8 cm), whose slope variance σ² comes from the air's friction velocity by Cox
// and Munk (water_optics, computed per cell on the host); and of those ripples, the ones a pixel
// resolves are drawn as waves, the rest are the facets' spread in a Beckmann distribution of the
// same σ² (Beckmann and Spizzichino 1963; Walter et al. 2007): the total slope variance is kept
// whatever the distance. Reflection is the exact Fresnel of water, n = 1.333, per facet; the
// light under the surface is pure water's absorption (Pope and Fry 1997) and backscatter (Morel
// 1974), and the bottom seen by Snell's law.

static const float kWaterIndex = 1.333;
static const float kRippleLongest = 0.048;  // m: the longest ripple below the 3 cm grid's 6 cm
static const float kRippleRatio = 0.56;      // between octaves: 4 octaves span 4.8 to 0.8 cm
// The sun's disc, 0.53° across (radius 4.65 mrad): the half-vector turns by half the
// direction, a disc of radius 2.33 mrad, whose slope variance (both axes, a²/2) is added, rad².
static const float kSunDiscVariance = 2.7e-6;
// The share of the ripples' slope variance along the air's stress: Cox and Munk's short waves,
// 2.38e-3·U upwind to 1.08e-3·U across (water_optics::rippleSlopeVariances).
static const float kUpwindShare = 2.38 / 3.46;
// The ripples' packets as streaks along the air (cat's paws): the wall layer's streaks are
// elongated in the flow, several times longer than they are wide (Kline et al., J. Fluid Mech.
// 30(4), 1967, near the wall; TODO(verify) the aspect of the streaks in wind ripples, e.g.
// Caulliez, Ricci and Dupont, Phys. Fluids 10(4), 1998). Length over width of a packet.
static const float kStreakAspect = 4.0;
// Pure water's absorption, 1/m, at 620, 550 and 465 nm (Pope and Fry, Appl. Opt. 36(33), 1997).
static const float3 kWaterAbsorption = float3(0.27, 0.0565, 0.0102);
// Pure water's backscatter, 1/m: half its scattering b_w(λ) = 0.00222·(500/λ)^4.32 (Morel,
// "Optical properties of pure water and pure sea water", 1974; TODO(verify) the 0.00222).
static const float3 kWaterBackscatter = float3(0.00044, 0.00073, 0.00152);

// Unpolarized Fresnel reflectance from air into water at an incidence of cosine `c`
// (water_optics::fresnel): 0.020 at normal incidence, 1 at grazing.
float fresnelWater(float c)
{
    c = saturate(c);
    float st2 = (1.0 - c * c) / (kWaterIndex * kWaterIndex);
    float ct = sqrt(max(1.0 - st2, 0.0));
    float rs = (c - kWaterIndex * ct) / (c + kWaterIndex * ct);
    float rp = (kWaterIndex * c - ct) / (kWaterIndex * c + ct);
    return 0.5 * (rs * rs + rp * rp);
}

// ω of a capillary-gravity wave on deep water, rad/s (Lamb 1932, §267; σ/ρ = 7.275e-5 m³/s²).
float rippleOmega(float k)
{
    return sqrt(9.81 * k + 7.275e-5 * k * k * k);
}

// The ripples a pixel of `footprint` m resolves, as the slope they add (x, y), and the share of
// `variance` it does not resolve, returned: four octaves of eight waves each (their wavelengths
// spread ±20 % round the octave's, so that they do not beat into fringes), running within ±63°
// of `runs`, each octave a quarter of the variance (the saturation range: as much slope per
// octave), its waves in packets streaked along `runs` (kStreakAspect). Spread evenly over ±63°
// (1.1 rad) their slope is <cos²>/<sin²> = 2.16 times steeper along `runs` than across, Cox and
// Munk's 2.2 (kUpwindShare): the drawn ripples are as anisotropic as the facets.
float ripples(float2 world, float t, float variance, float2 runs, float footprint, out float2 slope)
{
    slope = float2(0.0, 0.0);
    float unresolved = 0.0;
    if (variance <= 0.0)
        return 0.0;
    float heading = atan2(runs.y, runs.x);
    // Packets in the frame of the stress: long along it, narrow across.
    float2 streak = float2(dot(world, runs) / kStreakAspect, dot(world, float2(-runs.y, runs.x)));
    float lambda = kRippleLongest;
    for (int j = 0; j < 4; ++j)
    {
        float share = variance * 0.25;
        float seen = smoothstep(1.0, 2.0, lambda / (2.0 * footprint));
        unresolved += share * (1.0 - seen);
        if (seen > 0.0)
        {
            float amplitude = sqrt(share * 0.25) * seen;  // each wave's slope: 8·s²/2 = share
            for (int i = 0; i < 8; ++i)
            {
                float h = hash(float3(float(j), float(i), 0.37));
                float h2 = hash(float3(float(i), float(j), 1.91));
                float h3 = hash(float3(float(i) + 3.1, float(j) * 1.7, 5.3));
                float wavelength = lambda * (0.8 + 0.4 * h3);
                float omega = rippleOmega(2.0 * PI / wavelength);
                // Packets: an envelope of mean square ~1 (value noise's is ~0.29), drifting.
                float envelope = 1.85 * noise3(float3(streak / (wavelength * 2.5) + h2 * 17.0, t * 0.7 + float(i)));
                float angle = heading + (h - 0.5) * 2.2;
                float2 d = float2(cos(angle), sin(angle));
                float phase = 2.0 * PI * frac(dot(d, world) / wavelength - frac(omega * t / (2.0 * PI)) + h2);
                slope += amplitude * envelope * d * cos(phase);
            }
        }
        lambda *= kRippleRatio;
    }
    return unresolved;
}

// What the water reflects along R from P (relative to the eye) for something about `reach` m
// away: the scene turned over the surface at that point, else the sky. `h` is the still level.
// A facet that sends the ray down: it meets the water again close by, which reflects the sky
// along the ray turned up by Fresnel at that grazing incidence and lets through the water's own
// light (`own`): a second bounce off a level surface (multiple scattering between facets).
float3 reflectedDown(float3 R, float3 own, float sharpSky)
{
    float3 up = float3(R.xy, -R.z);
    float F = fresnelWater(-R.z);
    return F * sky(normalize(up + float3(0, 0, 0.001)), sharpSky) + (1.0 - F) * own;
}

float3 reflectedAlong(float3 P, float3 R, float reach, float h, float sharpSky)
{
    float3 X = P + R * reach;
    X.z = 2.0 * h - X.z;  // its image under the surface, which the mirror pass drew
    float4 c = project(X);
    float3 skyColor = sky(R, sharpSky);
    if (c.w <= 1e-4)
        return skyColor;
    float2 at = c.xy / c.w * 0.5 + 0.5;
    if (any(at < 0.0) || any(at > 1.0))
        return skyColor;
    float4 m = mirrorColor.SampleLevel(at, 0.0);
    return lerp(skyColor, m.rgb, m.a);
}

[shader("fragment")]
float4 fsWater(WaterOutput input) : SV_Target
{
    float distance = length(input.relative);
    float3 V = -input.relative / max(distance, 1e-3);
    float2 uv = input.position.xy * frame.screen.zw;
    float t = frame.worldWrap.w;
    float h = frame.water.x;   // the still level, relative to the eye
    float3 P = input.relative;

    float above = h < 0.0 ? 1.0 : -1.0;  // the eye above the surface?
    float3 N = normalize(input.normal);
    float4 moving = float4(0, 0, 0, 0);
    float movingShare = 0.0;
    float facets = 0.0;  // the facets' slope variance below the pixel
    float2 runs = frame.surfaceLook.zw;  // the way the air's stress runs there
    if (input.side < 0.5)
    {
        // The swell's slope per pixel: the mesh only holds its shape.
        float3 lifted;
        gerstner(input.world, t, lifted, N);
        // The physics' η where it moves the surface, and the air's ripples everywhere.
        float share;
        float4 rough;
        moving = movingSurface(input.world - frame.worldWrap.xy, share, rough);
        movingShare = share;
        float pixel = 2.0 * frame.cameraUp.w * frame.screen.w;  // rad
        float footprint = distance * pixel / sqrt(max(abs(V.z), 0.05));
        float2 capillary;
        runs = normalize(rough.yz + float2(1e-6, 0.0));
        facets = ripples(input.world, t, rough.x, runs, footprint, capillary);
        float2 slope = moving.yz * frame.surfaceLook.x * share * N.z + capillary;
        N = normalize(float3(N.xy - slope, N.z));
        N *= above;
        // A resolved ripple's back, turned from the eye, is hidden behind the crest in front
        // (masking): the pixel sees that crest's face instead, so the normal is turned back
        // until it faces the eye.
        float facing = dot(N, V);
        if (facing < 0.1)
            N = normalize(N + (0.1 - facing) * V);
    }
    else if (dot(N, V) < 0.0)
    {
        // The far sides, seen from over or in the water: hidden behind its surface. The water
        // pass writes no depth, so it would draw over the surface; it is not drawn.
        discard;
    }
    float NV = saturate(dot(N, V)) + 1e-4;
    // The water's own light as it leaves the surface, for what a second bounce sees (the deep
    // column's: see the refraction below).
    float sunUp0 = saturate(frame.sunDirection.z);
    float3 absorption0 = kWaterAbsorption + frame.waterOptics.xyz;
    float3 own = 0.33 * kWaterBackscatter / (absorption0 + kWaterBackscatter) *
                 (frame.sunColor.rgb * frame.sunDirection.w * sunUp0 / PI + float3(0.38, 0.46, 0.60) * frame.light.x) /
                 (kWaterIndex * kWaterIndex);

    // --- reflection: the facets' Fresnel-weighted view of the scene and the sky --------------
    // How far what is reflected stands: from the mirror pass's depth (its image's view depth).
    float near = frame.cameraForward.w;
    float mirrorDepthHere = mirrorDepth.SampleLevel(uv, 0.0).r;
    float reach = 40.0;
    if (mirrorDepthHere > 0.0)
        reach = clamp(length(viewRay(uv) * (near / mirrorDepthHere) - P), 0.02, 200.0);
    float3 reflection = float3(0.0, 0.0, 0.0);
    float reflectance = 0.0;
    if (facets < 1e-4 || input.side > 0.5)
    {
        float3 R = reflect(-V, N);
        reflectance = fresnelWater(NV);
        reflection = (R.z > 0.0 ? reflectedAlong(P, normalize(R), reach, h, 1.0) : reflectedDown(R, own, 1.0)) *
                     reflectance;
    }
    else
    {
        // Eight facet normals drawn from Cox and Munk's Gaussian of slopes, σ_u² along the air's
        // stress and σ_c² across (kUpwindShare of `facets` and the rest; Box–Muller), each
        // weighted by how much of it the eye sees (V·H / cos θ).
        float3 T1 = normalize(float3(runs, 0.0) - N * dot(N, float3(runs, 0.0)) + float3(0, 0, 1e-6));
        float3 T2 = cross(N, T1);
        float sigmaU = sqrt(facets * kUpwindShare);
        float sigmaC = sqrt(facets * (1.0 - kUpwindShare));
        float spin = hash(float3(input.position.xy, frac(t * 0.37) * 61.0));
        float sumW = 0.0;
        float sharpSky = saturate(1.0 - sqrt(facets) * 6.0);
        for (int i = 0; i < 8; ++i)
        {
            float u1 = frac(spin + float(i) * 0.618034);
            float u2 = (float(i) + 0.5) / 8.0 + spin * 0.125;
            float radius = sqrt(-2.0 * log(1.0 - 0.98 * u1));
            float phi = 2.0 * PI * u2;
            float2 z = radius * float2(sigmaU * cos(phi), sigmaC * sin(phi));  // slopes
            float cosT = rsqrt(1.0 + dot(z, z));
            float3 H = normalize(-T1 * z.x - T2 * z.y + N);
            float VH = dot(V, H);
            if (VH <= 0.0)
                continue;
            float w = VH / cosT;
            float3 R = reflect(-V, H);
            float F = fresnelWater(VH);
            reflection += w * F * (R.z > 0.0 ? reflectedAlong(P, normalize(R), reach, h, sharpSky)
                                             : reflectedDown(R, own, sharpSky));
            reflectance += w * F;
            sumW += w;
        }
        reflection /= max(sumW, 1e-6);
        reflectance /= max(sumW, 1e-6);
    }

    // --- refraction: the bottom through the surface by Snell's law, and the water's own light --
    float sunUp = saturate(frame.sunDirection.z);
    float3 Tr = refract(-V, N, 1.0 / kWaterIndex);
    float waterDepth = input.position.z;
    float2 bentUv = uv;
    float3 bottomAt = P;
    bool seenBottom = false;
    float bottomLevel = frame.water.w;
    if (above > 0.0 && input.side < 0.5 && Tr.z < -1e-3)
    {
        float3 Q = P + Tr * ((bottomLevel - P.z) / Tr.z);
        float4 c = project(Q);
        if (c.w > 1e-4)
        {
            float2 at = c.xy / c.w * 0.5 + 0.5;
            if (all(at >= 0.0) && all(at <= 1.0) && sceneDepth.SampleLevel(at, 0.0).r <= waterDepth)
                bentUv = at;
        }
        bottomAt = Q;
    }
    float depthThere = sceneDepth.SampleLevel(bentUv, 0.0).r;
    if (depthThere > waterDepth)
    {
        // That sample is in front of the water: look straight through instead.
        bentUv = uv;
        depthThere = sceneDepth.SampleLevel(uv, 0.0).r;
    }
    float3 below = float3(0.0, 0.0, 0.0);
    float3 point = bottomAt;
    if (depthThere > 0.0)
    {
        below = sceneColor.SampleLevel(bentUv, 0.0).rgb;
        point = viewRay(bentUv) * (near / depthThere);
        seenBottom = true;
    }
    else if (input.side > 0.5 || above < 0.0)
    {
        point = P + normalize(input.relative) * 0.8;
    }
    float pathView = max(length(point - P), input.side > 0.5 ? 0.8 : 0.0);
    if (!seenBottom && input.side < 0.5)
        pathView = 1e3;  // nothing under it: deep
    float submerged = max(h - point.z, 0.0);
    // The sun's light down to the bottom, along its refracted ray.
    float cosSunWater = sqrt(max(1.0 - (1.0 - sunUp * sunUp) / (kWaterIndex * kWaterIndex), 0.0));
    float3 absorption = kWaterAbsorption + frame.waterOptics.xyz;  // and its dissolved matter
    float3 down = exp(-absorption * submerged / max(cosSunWater, 0.1));
    float3 alongView = exp(-absorption * pathView);
    float3 lightOnBottom = caustics(point.xy + frame.worldWrap.xy, t) * windShare() * exp(-submerged * 0.6) *
                           saturate(submerged * 4.0) * frame.sunDirection.z * 1.3;
    float sunIn = 1.0 - fresnelWater(sunUp);
    below *= (1.0 + lightOnBottom) * down * sunIn;
    // The water's own light: a deep column's irradiance reflectance R∞ = 0.33·b_b/(a + b_b)
    // (Gordon, Brown and Jacobs 1975; Morel and Prieur 1977), lit by the sun and the sky that
    // went in, seen through what is left of the column.
    float3 rInf = 0.33 * kWaterBackscatter / (absorption + kWaterBackscatter);
    float visibility = sunLight(P, float3(0, 0, 1));
    float3 downwelling = (frame.sunColor.rgb * frame.sunDirection.w * sunUp * visibility / PI +
                          float3(0.38, 0.46, 0.60) * frame.light.x) * sunIn;
    float3 upwelling = rInf * downwelling * (1.0 - alongView * down);
    // Out through the surface: what is not reflected, its radiance divided by n².
    float3 water = (below * alongView + upwelling) * (1.0 - reflectance) / (kWaterIndex * kWaterIndex);

    float3 color = water + reflection;

    // The sun's glitter: Beckmann facets of the unresolved ripples and the sun's disc, on the
    // resolved surface: F·D/(4·N·V) times the sun's illuminance (G ≈ 1 for these slopes).
    float3 L = frame.sunDirection.xyz;
    if (frame.sunDirection.w > 0.0 && L.z > 0.0 && input.side < 0.5)
    {
        // Cox and Munk's slope density, anisotropic along the air's stress: p(z_u, z_c) =
        // exp(−½(z_u²/σ_u² + z_c²/σ_c²))/(2π·σ_u·σ_c), D = p/cos⁴θ; with the sun's disc in both.
        float3 H = normalize(V + L);
        float NH = max(dot(N, H), 1e-3);
        float3 Tu = normalize(float3(runs, 0.0) - N * dot(N, float3(runs, 0.0)) + float3(0, 0, 1e-6));
        float3 Tc = cross(N, Tu);
        float zu = dot(H, Tu) / NH;
        float zc = dot(H, Tc) / NH;
        float su2 = facets * kUpwindShare + 0.5 * kSunDiscVariance;
        float sc2 = facets * (1.0 - kUpwindShare) + 0.5 * kSunDiscVariance;
        float D = exp(-0.5 * (zu * zu / su2 + zc * zc / sc2)) /
                  (2.0 * PI * sqrt(su2 * sc2) * NH * NH * NH * NH);
        float glint = min(D * fresnelWater(saturate(dot(V, H))) / (4.0 * NV), 3000.0);
        color += glint * frame.sunColor.rgb * frame.sunDirection.w * visibility;
    }

    // Foam: where the water meets something, and on the crests in a strong wind.
    float crest = saturate((input.relative.z - frame.water.x) / max(waveScale(), 0.01) * 0.8);
    float shore = 1.0 - saturate(pathView * max(dot(-V, frame.cameraForward.xyz), 0.05) / 0.22);
    float tearing = saturate((crest - 0.72) * 3.0) * saturate((frame.water.y - 6.0) / 8.0);
    float tf = t * windShare();  // foam drifts only in a wind
    float cell = fbm(float3(input.world * 3.3 + float2(0.3, 0.2) * tf, tf * 0.25), 3);
    float streak = fbm(float3(input.world * 11.0 - float2(0.25, 0.1) * tf, tf * 0.4), 2);
    float foam = saturate(smoothstep(0.42, 0.62, cell * (0.6 + streak)) * saturate(shore * 1.5 + tearing));
    // Whitecaps (water_optics::whitecapOf): a coverage W of the area white, as patches where a
    // noise passes its (1 − W) quantile, streaked along the air. The noise is near normal of mean
    // 0.5 and sd 0.12 (fbm of 3 octaves; TODO(verify) measured), its quantile by the logistic
    // approximation Φ⁻¹(p) ≈ ln(p/(1−p))/1.702.
    float coverage = saturate(moving.w * movingShare);
    if (coverage > 1e-4)
    {
        float2 along = float2(dot(input.world, runs) / kStreakAspect, dot(input.world, float2(-runs.y, runs.x)));
        // Patches a few centimetres across (spume streaks; their size is not from the law).
        float patches = fbm(float3(along * 28.0, t * 0.6), 3);
        float p = clamp(1.0 - coverage, 1e-3, 1.0 - 1e-3);
        float q = 0.5 + 0.12 * log(p / (1.0 - p)) / 1.702;
        foam = max(foam, smoothstep(q - 0.05, q + 0.05, patches));
    }
    if (input.side < 0.5)
        color = lerp(color, float3(0.78, 0.82, 0.84) * (0.45 + 0.65 * visibility * sunUp) * frame.light.x, foam * 0.85);

    return float4(fogged(color, input.relative), 1.0);
}

// --- vegetation: reeds and shrubs as real geometry ------------------------------------------
// On a surface whose material names `reeds` or `shrub` (kinds 15, 16 of surfacePatternOf), the
// renderer draws a clump in every cell of a world grid over its top (RealisticRenderer::record),
// built here from the vertex and instance index alone: no buffers, the same clump in the same
// cell every frame and on every machine. Reeds are 12 long, thin, curved, tapered blades, two of
// them stems with seed heads; a shrub 60 leaves round a hidden stem. Each blade bends with the air
// at its root (groundAir), tips most (the bend grows with the square of the height along it, as
// a cantilever's under an even load does), each clump at its own stiffness and sway. Clumps fade
// out (shrink to nothing) before `push.model[2].z` metres. The looks are drawn, not measured:
// TODO(verify) clump spacing, blade counts and leaf sizes against photographs of reed beds.
// Push (96 bytes, Push's layout): model[0] = first cell's corner relative to the eye (xyz),
// spacing; model[1] = first cell index (i, j), cells (nx, ny); model[2] = kind (15, 16), height
// (m), fade distance (m), mirror plane (relative z; 1e9 when not mirrored); baseColor = the
// material's colour; surface = the patch's rectangle relative to the eye (x0, y0, x1, y1).

static const uint kVegVertices = 360;  // per clump: reeds 12 blades x 5 segments x 6, shrubs 60 x 6

uint vegHash(uint x)
{
    x ^= x >> 16; x *= 0x7feb352du; x ^= x >> 15; x *= 0x846ca68bu; x ^= x >> 16;
    return x;
}
float vegRandom(inout uint state)
{
    state = vegHash(state + 0x9e3779b9u);
    return float(state & 0xffffffu) / 16777216.0;
}

struct VegOutput
{
    float4 position : SV_Position;
    float3 relative;
    float3 normal;
    float3 color;
    float rough;
};

VegOutput vegNothing()
{
    VegOutput o;
    o.position = float4(0.0, 0.0, -1.0, 1.0);  // outside the clip volume: clipped away
    o.relative = float3(0.0);
    o.normal = float3(0.0, 0.0, 1.0);
    o.color = float3(0.0);
    o.rough = 1.0;
    return o;
}

// --- grass blades: the lawn near the eye as real geometry ----------------------------------
// On grass (kinds 6, 12, 13, 14) the renderer draws a tile of kGrassBlades blades in every cell of
// a world grid near the eye, over the ground's own parallax canopy (grassSurface), so the field has
// silhouettes, gaps and blades lit one by one. Each blade is a tapered strip of kBladeSegments
// segments and a pointed tip, standing at its own place, facing and height, curved outward, bent
// by the air at its root like the reeds (groundAir: wind and rotor outwash, the tip most), bowed
// by rain, and ending at the cut where a propeller cut it (cutTopAt). Far off the blades thin out
// and sink into the ground, where the parallax canopy carries on. Sizes, counts and colours are
// drawing choices: TODO(verify) against photographs of turf and meadow.
// Push as vsVeg's; model[0].w the tile size, model[2].y unused (the height is grassLook's).

static const uint kGrassBlades = 48;     // per tile
static const uint kBladeSegments = 3;
static const uint kBladeVerts = kBladeSegments * 6u + 3u;  // a quad per segment and the tip
// kGrassBlades * kBladeVerts = 1008 vertices per tile: Realistic.cpp kGrassTileVertices.

VegOutput vsGrassBlade(uint vertex, uint instance)
{
    uint nx = uint(push.model[1].z);
    int2 cell = int2(int(push.model[1].x) + int(instance % nx), int(push.model[1].y) + int(instance / nx));
    float tile = push.model[0].w;
    int kind = int(push.model[2].x + 0.5);
    float reach = push.model[2].z;
    GrassLook g = grassLook(grassOf(kind));
    float2 lo = float2(instance % nx, instance / nx);
    float3 corner = float3(push.model[0].xy + lo * tile, push.model[0].z);
    // The whole tile off screen, or too far: nothing (every vertex of it decides the same).
    float3 centre = corner + float3(0.5 * tile, 0.5 * tile, 0.5 * g.height);
    float distance = length(centre);
    if (distance > reach)
        return vegNothing();
    float4 c = project(centre);
    float r = 0.75 * tile + g.height;
    if (c.w < -r || abs(c.x) - 3.0 * r > c.w || abs(c.y) - 3.0 * r > c.w)
        return vegNothing();
    uint b = vertex / kBladeVerts;
    uint v = vertex % kBladeVerts;
    // Fewer blades further off: all of them within a third of the reach, a quarter at its end.
    float keep = lerp(1.0, 0.25, smoothstep(reach * 0.33, reach, distance));
    if (float(b) >= float(kGrassBlades) * keep)
        return vegNothing();
    uint seed = vegHash(uint(cell.x) * 73856093u ^ uint(cell.y) * 19349663u ^ uint(kind) * 83492791u);
    uint s = vegHash(seed + b * 2654435761u);
    float3 root = corner + float3(float2(vegRandom(s), vegRandom(s)) * tile, 0.0);
    if (root.x < push.surface.x || root.y < push.surface.y || root.x > push.surface.z || root.y > push.surface.w)
        return vegNothing();
    float3 world = root + frame.worldWrap.xyz;
    float rain = saturate(log2(1.0 + frame.weather.x) / 3.5);
    float wetness = max(g.wet, rain);
    // Sinking into the ground towards the reach, where the parallax canopy takes over.
    float fade = 1.0 - smoothstep(reach * 0.6, reach, length(root));
    float h = g.height * (0.45 + 0.65 * vegRandom(s)) * (1.0 - 0.15 * wetness) * fade;
    float cap = cutTopAt(root.xy, 0.0) - root.z;
    float kept = saturate(cap / max(h, 1e-4));
    if (h * kept < 0.004)
        return vegNothing();
    float azimuth = vegRandom(s) * 6.283;
    float2 out2 = float2(cos(azimuth), sin(azimuth));
    float2 widthDir = float2(-out2.y, out2.x);
    // Blades are wider than real ones (2-4 mm): fewer of them have to fill the view.
    float width = g.spacing * g.width * (0.35 + 0.25 * vegRandom(s)) * 1.4;
    float curl = 0.1 + 0.5 * vegRandom(s);  // how far it arcs out, m per m of height
    // The air at the root bends it (grassBend: lean, gusts, sway, the rain's droop); a cut stub
    // stands stiff.
    float2 bend = grassBend(world, g.stiff, wetness) * (1.0 - 0.8 * (1.0 - kept));
    float2 tipOff = out2 * curl + bend;  // tip displacement per m of height
    // Strip: segment k, corner; then the tip triangle.
    static const uint level[6] = {0u, 0u, 1u, 1u, 0u, 1u};
    static const float side[6] = {-1.0, 1.0, -1.0, -1.0, 1.0, 1.0};
    float f;
    float across;
    if (v < kBladeSegments * 6u)
    {
        f = float(v / 6u + level[v % 6u]) / float(kBladeSegments + 1u);
        across = side[v % 6u];
    }
    else
    {
        uint t = v - kBladeSegments * 6u;
        f = t == 2u ? 1.0 : float(kBladeSegments) / float(kBladeSegments + 1u);
        across = t == 0u ? -1.0 : (t == 1u ? 1.0 : 0.0);
    }
    bool cutBlade = kept < 1.0;
    f *= kept;
    // A cantilever's curve (the tip most), the height it loses by leaning kept so the blade keeps
    // about its length.
    float2 offset = tipOff * h * f * f;
    float leanAmount = min(length(tipOff), 1.4);
    float lift = h * f * (1.0 - 0.38 * leanAmount * leanAmount * f);
    float w = width * (cutBlade ? 1.0 - 0.6 * f : 1.0 - f * f);
    float3 p = root + float3(offset + widthDir * w * across * 0.5, max(lift, 0.0));
    float3 tangent = normalize(float3(tipOff * 2.0 * f, 1.0));
    float3 n = normalize(cross(tangent, float3(widthDir, 0.0)));
    VegOutput o;
    // Rounded toward the sky, as a blade's fold and the canopy's light make it look.
    o.normal = normalize(n + float3(0.0, 0.0, 0.7));
    o.relative = p;
    // Colour: the ground's (grassSurface's flat lawn), each blade a little its own, dark at the
    // root where the canopy shades it, drier at the tip.
    float3 base = push.baseColor.rgb * g.tint;
    float dry = smoothstep(0.45, 0.7, fbm(world * 0.17, 4)) * (grassOf(kind) == 4 ? 0.3 : 1.0);
    base = lerp(base, base * float3(1.75, 1.35, 0.8), dry);
    float mid = fbm(world * 1.9, 3);
    float tone = vegRandom(s);
    float3 col = base * (0.7 + 0.6 * mid) * lerp(float3(0.8, 0.9, 0.7), float3(1.25, 1.2, 0.9), tone);
    float rootShade = f / max(kept, 1e-3);
    col *= lerp(0.45, 1.05, saturate(rootShade * 1.3));
    float near = 1.0 - saturate(length(root) / reach);
    col = lerp(col, base * float3(1.6, 1.4, 0.8), smoothstep(0.7, 1.0, rootShade) * 0.35 * tone * near * saturate(g.height / 0.3));
    // Far, the blades take the flat lawn's colour they sink into (grassSurface's far look).
    col = lerp(col, base * (0.55 + 0.75 * mid), (1.0 - fade) * 0.8);
    // Laid flat by the air, their pale backs show (as grassSurface's band).
    float laid = saturate(length(bend) / 1.2);
    col = lerp(col, col * 1.45 + float3(0.03, 0.04, 0.02), 0.4 * laid * laid);
    if (cutBlade && f > kept - 0.08)
        col = base * float3(1.45, 1.4, 0.8) + float3(0.03, 0.03, 0.015);  // the pale cut end
    col *= lerp(1.0, 0.62, wetness);
    o.color = col;
    o.rough = lerp(0.8, 0.6, wetness);
    if (push.model[2].w < 1e8)
    {
        o.relative.z = 2.0 * push.model[2].w - o.relative.z;
        o.normal.z = -o.normal.z;
    }
    o.position = project(o.relative);
    return o;
}

// --- trees: TreeAPI_1 prims drawn as trees ------------------------------------------------
// docs/physics/ground.md "Trees". A tree is made here from its parameters and a seed alone (no
// buffers: the same tree every frame and on every machine): a tapered trunk with a root flare
// from its foot to its crown base and on up into the crown, kTreeBranches main branches out to
// the crown's main clumps of leaves, a twig to each further clump, and a crown of leaf cards in
// kTreeClumps clumps inside the crown's ellipsoid (Trees.hpp: semi-axes R, R and half the crown's
// height over the crown base). The crown sways by tree/<n>/sway (the crown centre's displacement):
// a point at height z over the foot moves by sway·(z / centre height)^1.5, so the trunk bends and
// the top moves most; each leaf flutters in the air at it (groundAir: wind and rotor outwash).
// Leaves whose cell (kTreeCutCell, the crown's own unswayed frame) a propeller cut are not drawn.
// Sizes, counts and colours are drawing choices: TODO(verify) against photographs of broadleaf
// trees. The renderer draws a tree in two draws (wood, then leaves; Realistic.cpp), both also into
// the sun's cascades so trees cast their shadows, leaf by leaf.
// Push (96 bytes): model[0] = the foot relative to the eye (xyz), height (m); model[1] = crown
// radius, crown base, trunk diameter (m), leaf cards drawn; model[2] = part (100 wood, 101 leaves,
// 102 flying leaves), the tree's index in the cut table, its seed, mirror plane; baseColor = sway
// (world m), leaf card length (m); surface = its orientation (x, y, z, w); emissive.w = 0 for the
// view, 1 and 2 for the sun's near and far cascades.

static const uint kTreeBranches = 7;
static const uint kTreeClumps = 28;
static const uint kTrunkSides = 10;
static const uint kTrunkSegments = 8;
static const uint kBranchSides = 6;
static const uint kBranchSegments = 4;
static const uint kTwigSides = 4;
static const uint kTwigSegments = 2;
static const uint kTrunkVerts = kTrunkSides * kTrunkSegments * 6u;     // 480
static const uint kBranchVerts = kBranchSides * kBranchSegments * 6u;  // 144
static const uint kTwigVerts = kTwigSides * kTwigSegments * 6u;        // 48
// kTrunkVerts + kTreeBranches * kBranchVerts + (kTreeClumps - kTreeBranches) * kTwigVerts = 2496
// vertices of wood: Realistic.cpp kTreeWoodVertices.
static const uint kTreeCutSlots = 32768;  // Realistic.cpp kTreeCutSlots
static const float kTreeCell = 0.1;       // m, Trees.hpp kTreeCutCell

struct TreeShape
{
    float height;
    float radius;      // R
    float base;        // crown base over the foot
    float half;        // c: half the crown's height
    float trunk;       // trunk radius at the foot
    float trunkTop;    // where the trunk ends, over the foot
    float2 lean;       // the trunk top's offset, m
    uint seed;
};

TreeShape treeShape()
{
    TreeShape t;
    t.height = push.model[0].w;
    t.radius = push.model[1].x;
    t.base = push.model[1].y;
    t.half = 0.5 * (t.height - t.base);
    t.trunk = 0.5 * max(push.model[1].z, 0.02);
    t.trunkTop = t.base + 1.3 * t.half;
    t.seed = uint(push.model[2].z);
    uint s = vegHash(t.seed ^ 0x51ed27u);
    t.lean = (float2(vegRandom(s), vegRandom(s)) - 0.5) * 0.12 * t.radius;
    return t;
}

float3 treeCentre(TreeShape t) { return float3(0.0, 0.0, t.base + t.half); }

// v turned by the tree's orientation (push.surface, a unit quaternion x, y, z, w).
float3 treeTurn(float3 v)
{
    float3 q = push.surface.xyz;
    float3 u = cross(q, v) + push.surface.w * v;
    return v + 2.0 * cross(q, u);
}

// A point of the tree (its own frame over the foot) relative to the eye, swayed.
float3 treePlace(TreeShape t, float3 local)
{
    float bend = pow(max(local.z, 0.0) / (t.base + t.half), 1.5);
    return push.model[0].xyz + treeTurn(local) + push.baseColor.xyz * bend;
}

float4 treeClip(float3 relative)
{
    float cascade = push.emissive.w;
    if (cascade < 0.5)
        return project(relative);
    // A cascade of the sun (Realistic.cpp cascade()): its atlas rows undone to its clip space.
    float4 p = float4(relative, 1.0);
    bool nearOne = cascade < 1.5;
    float u = dot(nearOne ? frame.shadow0[0] : frame.shadow1[0], p);
    float v = dot(nearOne ? frame.shadow0[1] : frame.shadow1[1], p);
    float d = dot(nearOne ? frame.shadow0[2] : frame.shadow1[2], p);
    return float4((u - (nearOne ? 0.25 : 0.75)) / 0.25, (v - 0.5) / 0.5, d, 1.0);
}

// The trunk's axis at height z over the foot.
float3 trunkAxis(TreeShape t, float z)
{
    float f = saturate(z / t.trunkTop);
    return float3(t.lean * f * f, z);
}

float trunkRadius(TreeShape t, float z)
{
    float f = saturate(z / t.trunkTop);
    float flare = 1.0 + 0.45 * exp(-z / max(0.35 * t.trunk * 4.0, 0.05));
    return t.trunk * (1.0 - 0.72 * f) * flare;
}

// Clump k's centre (the crown's frame, from its centre) and radius.
float4 treeClump(TreeShape t, uint k)
{
    uint s = vegHash(t.seed * 2654435761u + k * 0x9e3779b9u + 17u);
    float az;
    float up;
    float reach;
    if (k < kTreeBranches)
    {
        az = (float(k) + 0.35 * vegRandom(s)) * 2.39996;  // the golden angle round the trunk
        up = -0.35 + 1.05 * (float(k) + vegRandom(s)) / float(kTreeBranches);
        reach = 0.6 + 0.15 * vegRandom(s);
    }
    else
    {
        float i = float(k - kTreeBranches);
        float n = float(kTreeClumps - kTreeBranches);
        az = i * 2.39996 + vegRandom(s) * 0.8;
        up = lerp(-0.55, 0.92, (i + vegRandom(s)) / n);
        reach = 0.55 + 0.3 * vegRandom(s);
    }
    float horizontal = sqrt(max(1.0 - up * up, 0.0));
    float3 dir = float3(horizontal * cos(az), horizontal * sin(az), up);
    float3 centre = dir * reach * float3(t.radius, t.radius, t.half);
    float r = (0.27 + 0.2 * vegRandom(s)) * min(t.radius, t.half * 1.3);
    return float4(centre, r);
}

// A point of a quadratic Bézier and its tangent.
float3 bezier(float3 a, float3 b, float3 c, float f) { return lerp(lerp(a, b, f), lerp(b, c, f), f); }
float3 bezierTangent(float3 a, float3 b, float3 c, float f) { return 2.0 * lerp(b - a, c - b, f); }

// Main branch b: its start on the trunk, control point and end (the tree's frame).
void treeBranch(TreeShape t, uint b, out float3 a, out float3 m, out float3 e, out float r)
{
    float4 clump = treeClump(t, b);
    e = treeCentre(t) + clump.xyz;
    uint s = vegHash(t.seed * 747796405u + b * 2891336453u);
    float z = t.base * (0.82 + 0.1 * vegRandom(s)) + t.half * (0.15 + 0.85 * float(b) / float(kTreeBranches));
    z = min(z, t.trunkTop * 0.92);
    a = trunkAxis(t, z);
    m = float3(lerp(a.xy, e.xy, 0.6), lerp(a.z, e.z, 0.25));
    r = trunkRadius(t, z) * (0.5 + 0.12 * vegRandom(s));
}

// A tapered tube along a Bézier: vertex v of `sides` x `segments` quads.
void treeTube(uint v, uint sides, uint segments, float3 a, float3 m, float3 e, float r0, float r1,
              out float3 p, out float3 n, out float along)
{
    static const uint level[6] = {0u, 0u, 1u, 1u, 0u, 1u};
    static const uint turn[6] = {0u, 1u, 0u, 0u, 1u, 1u};
    uint q = v / 6u;
    uint c = v % 6u;
    float f = float(q / sides + level[c]) / float(segments);
    float angle = float(q % sides + turn[c]) / float(sides) * 6.2831853;
    float3 centre = bezier(a, m, e, f);
    float3 T = normalize(bezierTangent(a, m, e, f) + float3(0.0, 0.0, 1e-4));
    float3 side = normalize(cross(T, abs(T.z) < 0.95 ? float3(0.0, 0.0, 1.0) : float3(1.0, 0.0, 0.0)));
    float3 other = cross(T, side);
    n = side * cos(angle) + other * sin(angle);
    p = centre + n * lerp(r0, r1, f);
    along = f;
}

// Whether tree `tree`'s crown cell `cell` (kTreeCell, the crown's unswayed frame) was cut.
bool treeCellCut(uint tree, int3 cell)
{
    if (frame.trees.x < 0.5 || tree > 254u || any(cell < int3(-128)) || any(cell > int3(127)))
        return false;
    uint key = (tree << 24) | (uint(cell.x + 128) << 16) | (uint(cell.y + 128) << 8) | uint(cell.z + 128);
    uint slot = vegHash(key) & (kTreeCutSlots - 1u);
    for (uint probe = 0u; probe < 32u; ++probe)
    {
        uint k = treeCuts[slot];
        if (k == key)
            return true;
        if (k == 0xffffffffu)
            return false;
        slot = (slot + 1u) & (kTreeCutSlots - 1u);
    }
    return false;
}

VegOutput treeFinish(VegOutput o)
{
    float mirror = push.model[2].w;
    if (mirror < 1e8)
    {
        o.relative.z = 2.0 * mirror - o.relative.z;  // the water's mirror pass
        o.normal.z = -o.normal.z;
    }
    o.position = treeClip(o.relative);
    return o;
}

// The bark: grey-brown, darker in its furrows, lighter on the branches.
float3 barkColor(float3 local, float3 n, float along, bool branch)
{
    float3 world = local * float3(1.0, 1.0, 0.25);
    float furrow = noise3(float3(atan2(n.y, n.x) * 6.0, local.z * 9.0, 2.0));
    float3 bark = float3(0.095, 0.075, 0.058) * (0.6 + 0.7 * furrow);
    bark *= 0.85 + 0.3 * noise3(world * 3.0);
    if (branch)
        bark = lerp(bark, float3(0.12, 0.1, 0.075), 0.4 + 0.3 * along);
    // Green algae on the side away from the sun, near the foot.
    float moss = saturate(1.0 - local.z / 1.5) * saturate(-n.x * 0.7 + 0.3) * noise3(local * 4.0);
    return lerp(bark, float3(0.05, 0.075, 0.03), 0.6 * moss);
}

VegOutput vsTreeWood(uint vertex)
{
    TreeShape t = treeShape();
    float3 p;
    float3 n;
    float along;
    bool branch = true;
    if (vertex < kTrunkVerts)
    {
        // The trunk: a Bézier from the foot to its top, so its axis matches trunkAxis.
        float3 a = float3(0.0, 0.0, -0.05);
        float3 e = trunkAxis(t, t.trunkTop);
        float3 m = float3(0.0, 0.0, 0.5 * (a.z + e.z));
        uint q = vertex / 6u;
        uint c = vertex % 6u;
        static const uint level[6] = {0u, 0u, 1u, 1u, 0u, 1u};
        static const uint turn[6] = {0u, 1u, 0u, 0u, 1u, 1u};
        float f = float(q / kTrunkSides + level[c]) / float(kTrunkSegments);
        // Segments closer together at the foot, where the flare is.
        f = f * f * (1.6 - 0.6 * f) / 1.0;
        float z = lerp(a.z, e.z, f);
        float angle = float(q % kTrunkSides + turn[c]) / float(kTrunkSides) * 6.2831853;
        float3 centre = trunkAxis(t, z);
        n = float3(cos(angle), sin(angle), 0.0);
        // Its bark ridged: the radius a little uneven round it.
        float ridge = 1.0 + 0.06 * sin(angle * 5.0 + float(t.seed & 7u)) * saturate(1.0 - z / t.trunkTop);
        p = centre + n * trunkRadius(t, z) * ridge;
        along = z / t.trunkTop;
        branch = false;
    }
    else if (vertex < kTrunkVerts + kTreeBranches * kBranchVerts)
    {
        uint v = vertex - kTrunkVerts;
        uint b = v / kBranchVerts;
        float3 a;
        float3 m;
        float3 e;
        float r;
        treeBranch(t, b, a, m, e, r);
        treeTube(v % kBranchVerts, kBranchSides, kBranchSegments, a, m, e, r, r * 0.25, p, n, along);
    }
    else
    {
        uint v = vertex - kTrunkVerts - kTreeBranches * kBranchVerts;
        uint k = kTreeBranches + v / kTwigVerts;
        if (k >= kTreeClumps)
            return vegNothing();
        uint parent = k % kTreeBranches;
        float3 a;
        float3 m;
        float3 e;
        float r;
        treeBranch(t, parent, a, m, e, r);
        uint s = vegHash(t.seed * 1664525u + k * 1013904223u);
        float f = 0.35 + 0.35 * vegRandom(s);
        float3 start = bezier(a, m, e, f);
        float rs = lerp(r, r * 0.25, f) * 0.6;
        float4 clump = treeClump(t, k);
        float3 end = lerp(start, treeCentre(t) + clump.xyz, 0.8);
        float3 mid = lerp(start, end, 0.5) + float3(0.0, 0.0, 0.1 * distance(start, end));
        treeTube(v % kTwigVerts, kTwigSides, kTwigSegments, start, mid, end, rs, rs * 0.3, p, n, along);
    }
    VegOutput o;
    o.relative = treePlace(t, p);
    o.normal = treeTurn(n);
    o.color = barkColor(p, n, along, branch);
    o.rough = 0.9;
    return treeFinish(o);
}

// A leaf's green: the clump's own tone, each leaf a little its own, dark deep in the crown and
// under it; the season's colours (0 summer, 1 fall, 2 winter, 3 rainy).
float3 leafColor(float clumpTone, float tone, float depth, float below)
{
    float3 green = float3(0.055, 0.12, 0.028);
    green *= lerp(float3(0.8, 0.95, 0.85), float3(1.2, 1.12, 0.7), clumpTone);
    green *= 0.75 + 0.5 * tone;
    int season = seasonNow();
    if (season == 1)
    {
        float3 autumn = tone < 0.33 ? float3(0.36, 0.16, 0.02) : tone < 0.66 ? float3(0.3, 0.06, 0.015)
                                                                         : float3(0.32, 0.26, 0.03);
        green = lerp(green, autumn, 0.35 + 0.6 * clumpTone);
    }
    else if (season == 2)
        green = float3(0.14, 0.09, 0.05) * (0.7 + 0.5 * tone);
    // Deep in the crown and on its underside, the leaves are in the others' shade.
    return green * lerp(0.3, 1.0, depth) * lerp(0.6, 1.0, below);
}

// A leaf card is a spray of kSprayLeaves leaves round a twig's tip (kSprayVerts vertices).
static const uint kSprayLeaves = 3;
static const uint kSprayVerts = kSprayLeaves * 6u;  // Realistic.cpp kTreeCardVertices

VegOutput vsTreeLeaves(uint vertex)
{
    TreeShape t = treeShape();
    uint i = vertex / kSprayVerts;
    uint leaf = (vertex % kSprayVerts) / 6u;
    uint c6 = vertex % 6u;
    uint k = i % kTreeClumps;
    uint s = vegHash(t.seed * 2246822519u + i * 2654435761u + 7u);
    int season = seasonNow();
    if (season == 2 && vegRandom(s) < 0.88)
        return vegNothing();  // winter: a few dead leaves left
    float4 clump = treeClump(t, k);
    // A point in the clump, more of them near its outside.
    float zz = vegRandom(s) * 2.0 - 1.0;
    float az = vegRandom(s) * 6.2831853;
    float rr = pow(vegRandom(s), 0.55);
    float3 dir = float3(sqrt(max(1.0 - zz * zz, 0.0)) * float2(cos(az), sin(az)), zz);
    float3 q = clump.xyz + dir * clump.w * rr * float3(1.0, 1.0, 0.8);
    float3 axes = float3(t.radius, t.radius, t.half);
    float e = length(q / axes);
    if (e > 1.0)
        q *= 1.0 / e;  // kept in the crown's ellipsoid, where the physics has its leaves
    // Its blade faces out of its clump and the crown, and a little up, each its own way.
    float3 outward = normalize(q / (axes * axes) + 1e-5);
    float3 nrm = normalize(dir * 0.8 + outward * 0.6 +
                           (float3(vegRandom(s), vegRandom(s), vegRandom(s)) - 0.5) * 1.6 + float3(0.0, 0.0, 0.35));
    float3 tu = normalize(cross(nrm, abs(nrm.z) < 0.9 ? float3(0.0, 0.0, 1.0) : float3(1.0, 0.0, 0.0)));
    float3 tv = cross(nrm, tu);
    float spin = vegRandom(s) * 6.2831853 + float(leaf) * 2.1 + 0.6 * sin(float(i) + float(leaf) * 1.3);
    float3 along = tu * cos(spin) + tv * sin(spin);
    float3 across = cross(nrm, along);
    float size = push.baseColor.w * (0.7 + 0.6 * vegRandom(s));
    // This leaf of the spray: out from the twig's tip along its own way, tipped a little.
    uint ls = vegHash(s + leaf * 0x632be5abu);
    float3 tilt = (float3(vegRandom(ls), vegRandom(ls), vegRandom(ls)) - 0.5) * 0.7;
    q += along * size * 0.32;
    nrm = normalize(nrm + tilt);
    along = normalize(along - nrm * dot(along, nrm));
    across = cross(nrm, along);
    size *= 0.62;
    // Cut away by a propeller: no leaf whose middle is in a cut cell.
    float3 middle = q + along * size * 0.25;
    uint tree = uint(push.model[2].y);
    int3 at = int3(floor(middle / kTreeCell));
    if (treeCellCut(tree, at))
        return vegNothing();
    // Beside a cut: torn leaves, pale, so the hole shows.
    bool torn = frame.trees.x > 0.5 &&
                (treeCellCut(tree, at + int3(1, 0, 0)) || treeCellCut(tree, at - int3(1, 0, 0)) ||
                 treeCellCut(tree, at + int3(0, 1, 0)) || treeCellCut(tree, at - int3(0, 1, 0)) ||
                 treeCellCut(tree, at + int3(0, 0, 1)) || treeCellCut(tree, at - int3(0, 0, 1)));
    static const float2 kite[6] = {float2(-1, 0), float2(-0.2, -0.42), float2(-0.2, 0.42),
                                   float2(-0.2, 0.42), float2(-0.2, -0.42), float2(1, 0)};
    float2 corner = kite[c6];
    float3 local = treeCentre(t) + q;
    float3 relative = treePlace(t, local);
    // Flutter: each leaf turns about its stalk in the air at it, more in more air.
    float3 world = relative + frame.worldWrap.xyz;
    float air = length(groundAir(world)) + 4.0 * length(push.baseColor.xyz);
    float push_ = airPush(air, 0.45);
    float phase = vegRandom(s) * 6.2831853;
    float flutter = sin(frame.worldWrap.w * (7.0 + 9.0 * push_) + phase) * (0.06 + 0.5 * push_);
    float3 p = q + (along * corner.x + across * corner.y) * (0.5 * size);
    p += nrm * flutter * size * 0.5 * (corner.x + 1.0) * 0.5;
    VegOutput o;
    o.relative = treePlace(t, treeCentre(t) + p);
    o.normal = treeTurn(normalize(nrm - along * flutter * 0.8));
    uint ks = t.seed + k * 977u;
    float clumpTone = vegRandom(ks);
    float depth = smoothstep(0.35, 0.95, e) * smoothstep(-0.2, 0.9, dot(dir, normalize(clump.xyz + 1e-4)) * 0.5 + 0.5 + rr * 0.3);
    float below = saturate(q.z / t.half * 0.6 + 0.7);
    o.color = leafColor(clumpTone, vegRandom(s), depth, below);
    o.rough = season == 3 ? 0.35 : 0.55;
    if (torn)
        o.color = lerp(o.color, float3(0.2, 0.17, 0.07), 0.55);  // bruised and torn at the cut
    return treeFinish(o);
}

// Flying leaves (Leaves.hpp): cut from crowns or stripped by the air, drawn as single leaves at
// the orientation the physics gives them.
VegOutput vsFlyingLeaf(uint vertex)
{
    uint i = vertex / 6u;
    uint c6 = vertex % 6u;
    if (float(i) >= frame.trees.y)
        return vegNothing();
    float4 a = flyingLeaves[2u * i];
    float4 b = flyingLeaves[2u * i + 1u];
    float yaw = b.x;
    float pitch = b.y;
    float3 nrm = float3(cos(pitch) * cos(yaw), cos(pitch) * sin(yaw), sin(pitch));
    float3 along = normalize(cross(nrm, abs(nrm.z) < 0.9 ? float3(0.0, 0.0, 1.0) : float3(1.0, 0.0, 0.0)));
    float3 across = cross(nrm, along);
    static const float2 kite[6] = {float2(-1, 0), float2(-0.2, -0.42), float2(-0.2, 0.42),
                                   float2(-0.2, 0.42), float2(-0.2, -0.42), float2(1, 0)};
    float2 corner = kite[c6];
    // Drawn twice their blade's length (a leaf of 5 cm is less than a pixel a few metres off).
    float size = max(a.w, 0.02) * 1.6;
    VegOutput o;
    o.relative = a.xyz + (along * corner.x + across * corner.y) * (0.5 * size);
    // Lying ones (Leaves.hpp: flat once settled) a few millimetres over what they lie on, or the
    // ground's depth hides them.
    if (abs(nrm.z) > 0.999)
        o.relative.z += 0.004;
    o.normal = nrm;
    o.color = leafColor(b.z, frac(b.z * 7.13), 1.0, 1.0) * lerp(float3(1.0), float3(1.6, 1.3, 0.8), b.z * 0.6);
    o.rough = 0.55;
    return treeFinish(o);
}

VegOutput vsTree(uint vertex)
{
    float part = push.model[2].x;
    if (part < 100.5)
        return vsTreeWood(vertex);
    if (part < 101.5)
        return vsTreeLeaves(vertex);
    return vsFlyingLeaf(vertex);
}

[shader("vertex")]
VegOutput vsVeg(uint vertex : SV_VertexID, uint instance : SV_InstanceID)
{
    if (push.model[2].x > 99.5)
        return vsTree(vertex);  // trees (TreeAPI_1) and flying leaves
    if (grassOf(int(push.model[2].x + 0.5)) > 0)
        return vsGrassBlade(vertex, instance);
    uint nx = uint(push.model[1].z);
    int2 cell = int2(int(push.model[1].x) + int(instance % nx), int(push.model[1].y) + int(instance / nx));
    float spacing = push.model[0].w;
    int kind = int(push.model[2].x + 0.5);
    float H = push.model[2].y;
    uint seed = vegHash(uint(cell.x) * 73856093u ^ uint(cell.y) * 19349663u ^ uint(kind) * 83492791u);
    uint s = seed;
    // The clump's root, jittered in its cell; relative to the eye.
    float2 lo = float2(instance % nx, instance / nx);
    float2 jitter = float2(vegRandom(s), vegRandom(s)) * 0.8 + 0.1;
    float3 root = float3(push.model[0].xy + (lo + jitter) * spacing, push.model[0].z);
    if (root.x < push.surface.x || root.y < push.surface.y || root.x > push.surface.z || root.y > push.surface.w)
        return vegNothing();
    float distance = length(root);
    float fade = 1.0 - smoothstep(push.model[2].z * 0.6, push.model[2].z, distance);
    // Bare patches: about one cell in six has nothing.
    if (fade <= 0.0 || vegRandom(s) < 0.15)
        return vegNothing();
    float3 world = root + frame.worldWrap.xyz;
    float clumpH = H * (0.7 + 0.3 * vegRandom(s)) * (0.4 + 0.6 * fade);
    // Cut by a propeller (cutTopAt): nothing stands above the cut; cut to the ground, nothing.
    float cap = cutTopAt(root.xy, 0.0) - root.z;
    if (cap < 0.03)
        return vegNothing();
    float stiff = kind == 16 ? 0.75 + 0.2 * vegRandom(s) : 0.25 + 0.3 * vegRandom(s);
    // The air at the clump: lean with it, gusts carried with it, a sway of the clump's own.
    float2 wind = groundAir(world);
    float speed = length(wind);
    float t = frame.worldWrap.w;
    float2 along = speed > 0.05 ? wind / speed : float2(1.0, 0.0);
    float2 across = float2(-along.y, along.x);
    float gx = dot(world.xy, along) - speed * fmod(t, 600.0);
    float gust = noise3(float3(gx * 0.15, dot(world.xy, across) * 0.15, 0.0));
    float pushAir = airPush(speed, stiff);
    float lean = 1.3 * pushAir * (0.55 + 0.6 * gust);
    float phase = vegRandom(s) * 6.283;
    float freq = (kind == 16 ? 2.2 : 1.4) + 1.5 * vegRandom(s);  // Hz-ish: TODO(verify)
    // Whipping in strong air: the sway grows and quickens with the push.
    float sway = sin(t * freq * 6.283 / 3.0 * (1.0 + 1.5 * pushAir) + phase + gx * 0.8) * (0.02 + 0.35 * pushAir) * (1.0 - 0.6 * stiff);
    float2 bend = along * min(lean, 1.0) + across * sway;

    uint index = vertex;
    VegOutput o;
    float3 base = push.baseColor.rgb;
    if (kind == 15)
    {
        // Reeds: blade b, segment g, corner c.
        uint b = index / 30u;
        uint r = index % 30u;
        uint g = r / 6u;
        static const uint level[6] = {0u, 0u, 1u, 1u, 0u, 1u};
        static const float side[6] = {-1.0, 1.0, -1.0, -1.0, 1.0, 1.0};
        uint bs = vegHash(seed + b * 2654435761u);
        float azimuth = vegRandom(bs) * 6.283;
        float2 out2 = float2(cos(azimuth), sin(azimuth));
        // A seed head on about one clump in three, on one stem.
        bool stem = b == 0u && (seed & 3u) == 0u;
        if (distance > push.model[2].z * 0.35 && b >= 8u)
            return vegNothing();  // far: fewer blades
        float h = clumpH * (stem ? 1.0 + 0.1 * vegRandom(bs) : 0.55 + 0.45 * vegRandom(bs));
        float curl = stem ? 0.05 : 0.15 + 0.35 * vegRandom(bs);  // how far the blade arcs out
        float width = stem ? 0.006 : 0.010 + 0.006 * vegRandom(bs);  // m at the root
        float2 at = out2 * vegRandom(bs) * 0.07;
        float f = float(g + level[r % 6u]) / 5.0;  // 0 root .. 1 tip
        // A cut blade ends at the cut, blunt (its width there kept), without its seed head.
        float kept = saturate(cap / max(h, 1e-3));
        bool cutBlade = kept < 1.0;
        f *= kept;
        float2 offset = at + out2 * curl * h * f * f + bend * h * f * f;
        float lift = h * f * (1.0 - 0.5 * f * dot(offset - at, offset - at) / max(h * h * f * f, 1e-4) * f);
        float2 widthDir = float2(-out2.y, out2.x);
        float w = width * (1.0 - 0.9 * f);
        bool head = stem && f > 0.84;
        if (head)
            w = 0.008;  // the seed head: a narrow plume at the top of the stem
        float3 p = root + float3(offset + widthDir * w * side[r % 6u], lift);
        float3 tangent = normalize(float3(out2 * (2.0 * curl * f) + bend * 2.0 * f, 1.0));
        o.normal = normalize(cross(tangent, float3(widthDir, 0.0)));
        o.relative = p;
        float tone = vegRandom(bs);
        float3 green = base * lerp(float3(0.75, 0.85, 0.6), float3(1.25, 1.2, 0.85), tone);
        // Dark at the roots, paler and drier towards the tips.
        float3 c = green * lerp(0.35, 1.0, saturate(f * 1.6));
        c = lerp(c, base * float3(1.7, 1.45, 0.75), smoothstep(0.65, 1.0, f) * 0.45 * tone);
        if (head)
            c = float3(0.32, 0.22, 0.13);  // brown plume
        if (cutBlade && f > kept - 0.06)
            c = base * float3(1.55, 1.45, 0.8) + float3(0.04, 0.04, 0.02);  // the pale cut end
        o.color = c;
        o.rough = 0.65;
    }
    else
    {
        // A shrub: leaf l, a kite of two triangles round its centre.
        uint l = index / 6u;
        uint c6 = index % 6u;
        uint ls = vegHash(seed + l * 2246822519u);
        if (distance > push.model[2].z * 0.4 && l >= 36u)
            return vegNothing();
        float radius = clumpH * 0.65;
        // A point in the crown, a dome over the ground.
        float u = vegRandom(ls), v = vegRandom(ls), q = vegRandom(ls);
        float az = u * 6.283;
        float el = asin(v * 0.95);  // 0 at the rim .. up
        float rr = radius * (0.35 + 0.65 * sqrt(q));
        float3 dir = float3(cos(el) * cos(az), cos(el) * sin(az), sin(el));
        float3 centre = float3(dir.xy * rr, clumpH * 0.15 + dir.z * clumpH * 0.85 * (0.55 + 0.45 * sqrt(q)));
        float3 nrm = normalize(dir + (float3(vegRandom(ls), vegRandom(ls), vegRandom(ls)) - 0.5) * 0.8);
        if (centre.z > cap)
            return vegNothing();  // cut off
        bool atCut = cap < clumpH && centre.z > cap - 0.06;  // just under the cut: torn twigs
        float3 tu = normalize(cross(nrm, abs(nrm.z) < 0.9 ? float3(0, 0, 1) : float3(1, 0, 0)));
        float3 tv = cross(nrm, tu);
        float size = clumpH * (0.16 + 0.10 * vegRandom(ls));
        static const float2 kite[6] = {float2(-1, 0), float2(0, -0.45), float2(0, 0.45),
                                       float2(0, 0.45), float2(0, -0.45), float2(1, 0)};
        float2 k = kite[c6];
        float hf = saturate(centre.z / max(clumpH, 0.01));
        float flutter = sin(t * 9.0 + float(l) * 1.7 + phase) * 0.15 * pushAir;
        float3 p = centre + (tu * k.x + tv * k.y * (1.0 + flutter)) * size;
        p.xy += bend * clumpH * hf * hf * 0.6;
        o.relative = root + p;
        o.normal = nrm;
        float tone = vegRandom(ls);
        // Outer leaves lit, inner ones in the crown's shade.
        float3 c = base * lerp(float3(0.65, 0.8, 0.55), float3(1.3, 1.2, 0.8), tone);
        c *= lerp(0.45, 1.05, sqrt(q)) * lerp(0.7, 1.0, hf);
        if (atCut)
            c = lerp(c, base * float3(1.4, 1.15, 0.7), 0.6);  // pale torn leaves and twigs
        o.color = c;
        o.rough = 0.5;
    }
    if (push.model[2].w < 1e8)
    {
        o.relative.z = 2.0 * push.model[2].w - o.relative.z;  // the mirror pass
        o.normal.z = -o.normal.z;
    }
    o.position = project(o.relative);
    return o;
}

[shader("fragment")]
float4 fsVeg(VegOutput input) : SV_Target
{
    float3 N = normalize(input.normal);
    float3 V = normalize(-input.relative);
    // Leaves and blades have two sides: seen from behind, the far side's normal, turned round
    // its upright axis so that a blade lit from the sky stays lit from the sky (turning the whole
    // normal pointed it at the ground, and blades seen from behind went black).
    if (dot(N, V) < 0.0)
        N = normalize(float3(-N.xy, N.z));
    if (dot(N, V) < 0.0)
        N = -N;
    if (frame.mode.x > 0.5 && input.relative.z > frame.mode.y)
        discard;
    float3 albedo = input.color;
    float3 color = shadeSurface(input.relative, N, albedo, 0.0, input.rough, float3(0.0));
    // Sun through the leaf from behind: thin leaves pass on part of what falls on their far side
    // (TODO(verify) share; about 10-20 % for green leaves, Jacquemoud & Ustin 2019, ch. 2).
    float3 L = frame.sunDirection.xyz;
    float back = saturate(dot(-N, L)) * saturate(dot(-V, L) * 0.5 + 0.5);
    if (frame.sunDirection.w > 0.0)
        color += albedo * float3(0.9, 1.0, 0.45) * 0.15 * back * frame.sunColor.rgb * frame.sunDirection.w *
                 sunLight(input.relative, -N);
    if (frame.mode.x < 0.5)
        color = fogged(color, input.relative);
    return float4(color, 1.0);
}
