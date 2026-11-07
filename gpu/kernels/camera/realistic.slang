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
    float4 mode;              // x: 1 in the mirror pass (y: the plane's relative height there);
                              // z: the season; w: 1 with surfaceAlbedo's photographs
    float4 weather;           // x: rain, mm/h; y: snow, mm/h of water; zw: the wind, m/s, world
};

[[vk::binding(0)]] ConstantBuffer<Frame> frame;
[[vk::binding(1)]] Sampler2D shadowMap;   // the sun's depth, two cascades side by side
[[vk::binding(2)]] Sampler2D sceneColor;  // HDR, what was drawn before the water
[[vk::binding(3)]] Sampler2D sceneDepth;
[[vk::binding(4)]] Sampler2D mirrorColor; // what the water reflects; alpha 0 where only sky
[[vk::binding(5)]] Sampler2DArray surfaceAlbedo;  // photographs of surfaces, one layer per kind
[[vk::binding(6)]] Sampler2DArray surfaceNormal;  // their bumps, OpenGL convention

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
float3 sky(float3 dir, float sharp)
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

float3 fogged(float3 color, float3 relative)
{
    float distance = length(relative);
    int season = seasonNow();
    float amount = 1.0 - exp(-distance * (season == 3 ? 0.005 : season == 2 ? 0.0028 : 0.0009));
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
    if (frame.mode.w > 0.5 && image.y > 0.0)
        imageSurface(image.x, image.y, kind == 6 ? 1.0 : 0.7, world, N, albedo);
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
    else if (kind == 6)  // grass: blades of many greens, with dry patches
    {
        float blades = noise3(float3(world.xy * float2(38.0, 44.0), 1.0));
        float dry = smoothstep(0.45, 0.7, fbm(world * 0.17, 4));
        albedo = lerp(albedo, albedo * float3(1.75, 1.35, 0.8), dry);
        albedo *= 0.55 + 0.75 * mid + 0.55 * blades * near;
        N = normalize(N + float3(blades - 0.5, noise3(world * 31.0) - 0.5, 0.0) * 0.8 * near);
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
    if (kind != 9 && kind != 6 && kind != 7 && wall > 0.5)
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
        else if (kind == 6)  // tired grass, leaves lying on it
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
        float porous = kind == 6 || kind == 7 ? 0.75 : 0.55;
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
    float seen = 1.0 - overcast();
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
    return float4(display(smoothed * ao + bloom(uv)) + falling(uv, depth), 1.0);
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
