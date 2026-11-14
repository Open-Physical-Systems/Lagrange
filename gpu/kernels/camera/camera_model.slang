// The camera sensor model on the GPU (docs/design/domains/sensors.md §2): the same per-pixel work
// as modules/sensors CameraModel::apply, the CPU reference, on the images the raster pass has just
// drawn and copied, so only the output's size travels back to the host. One thread per output
// pixel: its ray (lens distortion undone, at z = 1), turned back by the rolling shutter's lag,
// projected into the render, sampled bilinearly in linear light (colour) and at the nearest pixel
// (depth along the axis, instance id, alpha), dimmed by vignetting, then with an exposure the
// sensor's electrons, shot and read noise and the converter's code.
//
// The noise draws the reference's random numbers: Philox4x32-10 (Salmon et al. 2011, as
// core/random), keyed by the module seed, counter (step, step >> 32, entity + row, block); every
// output pixel takes six blocks, two normals per channel, one block per normal, so pixel u of a
// row uses blocks 6u .. 6u + 5 as the CPU's per-row stream does. The bits are the same; Box-Muller
// is in float here (double in the reference), so a code can differ by one where the electrons sit
// on a rounding edge.

struct Params {
    float4 omega;         // xyz: angular velocity in the optical frame (rad/s); w: rolling shutter (s)
    uint width;           // output
    uint height;
    uint renderWidth;
    uint renderHeight;
    float4 intrinsics;    // render fx, fy, cx, cy (pixels)
    uint exposure;        // 1: electrons, noise and the converter; 0: display encoding only
    float electronsPerUnit;  // e- where the render's linear value is 1, this frame
    float gain;
    float fullWell;       // e-
    float readNoise;      // e-, standard deviation
    float fullScale;      // 2^bits - 1
    uint key0;            // module seed, low and high words
    uint key1;
    uint step0;           // step index, low and high words
    uint step1;
    uint entity;          // the first row's stream; row v uses entity + v
    uint pad;
};

// The render, as RasterCamera reads it back: RGBA8 words, then depth (float bits), then instance
// ids, each renderWidth * renderHeight long.
[[vk::binding(0, 0)]] StructuredBuffer<uint> rendered;
// Per output pixel: the ray's x and y at z = 1 before distortion (NaN where the lens maps no ray),
// the vignetting factor, unused.
[[vk::binding(1, 0)]] StructuredBuffer<float4> rays;
// decode[256] (display byte -> linear, float bits), then encoded[fullScale + 1] (converter code
// -> display byte), when there is an exposure.
[[vk::binding(2, 0)]] StructuredBuffer<uint> tables;
// Output: RGBA8 words, depth (float bits), instance ids, then the signal per pixel (float bits):
// the sum over the channels of min(e * gain / fullWell, 1), negative where the render was not seen.
[[vk::binding(3, 0)]] RWStructuredBuffer<uint> outputs;
[[vk::push_constant]] ConstantBuffer<Params> params;

static const uint kNoInstance = 0xFFFFFFFF;
static const float kPi = 3.14159265358979323846;
static const float kDisplayGamma = 2.2;  // the raster shader's display encoding (mesh.slang)

// --- Philox4x32-10 (core/random/src/Philox.cpp) -----------------------------------------------

static const uint kMultiplier0 = 0xD2511F53;
static const uint kMultiplier1 = 0xCD9E8D57;
static const uint kWeyl0 = 0x9E3779B9;
static const uint kWeyl1 = 0xBB67AE85;

// The high and low words of a * b, exactly, in 32-bit arithmetic (no shaderInt64).
void mulhilo(uint a, uint b, out uint hi, out uint lo) {
    const uint a0 = a & 0xFFFF;
    const uint a1 = a >> 16;
    const uint b0 = b & 0xFFFF;
    const uint b1 = b >> 16;
    const uint p00 = a0 * b0;
    const uint p01 = a0 * b1;
    const uint p10 = a1 * b0;
    const uint p11 = a1 * b1;
    const uint middle = (p00 >> 16) + (p01 & 0xFFFF) + (p10 & 0xFFFF);
    lo = (middle << 16) | (p00 & 0xFFFF);
    hi = p11 + (p01 >> 16) + (p10 >> 16) + (middle >> 16);
}

uint4 philox(uint4 counter, uint2 key) {
    for (int round = 0; round < 10; ++round) {
        if (round > 0) {
            key.x += kWeyl0;
            key.y += kWeyl1;
        }
        uint hi0, lo0, hi1, lo1;
        mulhilo(kMultiplier0, counter.x, hi0, lo0);
        mulhilo(kMultiplier1, counter.z, hi1, lo1);
        counter = uint4(hi1 ^ counter.y ^ key.x, lo1, hi0 ^ counter.w ^ key.y, lo0);
    }
    return counter;
}

// Stream::normal() on one fresh block: the first uniform (27 + 26 bits) gives the radius, from
// 1 - u computed exactly from the bits (never 0), the second the angle.
float normalOf(uint4 block) {
    const uint high = block.x >> 5;
    const uint low = block.y >> 6;
    const float oneMinus =
        float(0x7FFFFFF - high) * (1.0 / 134217728.0) + float(0x4000000 - low) * (1.0 / 9007199254740992.0);
    const float u2 = float(block.z >> 5) * (1.0 / 134217728.0) + float(block.w >> 6) * (1.0 / 9007199254740992.0);
    return sqrt(-2.0 * log(oneMinus)) * cos(2.0 * kPi * u2);
}

// --- the model ----------------------------------------------------------------------------------

float decode(uint byte) {
    return asfloat(tables[byte]);
}

uint channel(uint rgba, uint c) {
    return (rgba >> (8 * c)) & 0xFF;
}

uint encodeDisplay(float linear) {
    return uint(floor(255.0 * pow(clamp(linear, 0.0, 1.0), 1.0 / kDisplayGamma) + 0.5));
}

// Rodrigues: the rotation by the rotation vector a, by rows.
float3x3 rotationOf(float3 a) {
    const float angle = length(a);
    if (angle < 1e-12) {
        return float3x3(1, 0, 0, 0, 1, 0, 0, 0, 1);
    }
    const float3 n = a / angle;
    const float c = cos(angle);
    const float s = sin(angle);
    const float t = 1.0 - c;
    return float3x3(t * n.x * n.x + c, t * n.x * n.y - s * n.z, t * n.x * n.z + s * n.y,
                    t * n.x * n.y + s * n.z, t * n.y * n.y + c, t * n.y * n.z - s * n.x,
                    t * n.x * n.z - s * n.y, t * n.y * n.z + s * n.x, t * n.z * n.z + c);
}

[numthreads(64, 1, 1)]
void applyModel(uint3 id: SV_DispatchThreadID) {
    const uint w = params.width;
    const uint h = params.height;
    const uint pixels = w * h;
    const uint i = id.x;
    if (i >= pixels) {
        return;
    }
    const uint u = i % w;
    const uint v = i / w;
    const uint rw = params.renderWidth;
    const uint rh = params.renderHeight;
    const uint renderPixels = rw * rh;

    const float shutter = params.omega.w;
    const float tau = shutter > 0.0 && h > 1 ? shutter * float(h - 1 - v) / float(h - 1) : 0.0;
    const float3x3 r = rotationOf(-params.omega.xyz * tau);

    const float4 ray = rays[i];
    float3 linear = float3(0.0, 0.0, 0.0);
    bool seen = false;
    float depth = 1.0 / 0.0;
    uint instance = kNoInstance;
    uint alpha = 255;
    if (!isnan(ray.x)) {
        const float3 d = mul(r, float3(ray.x, ray.y, 1.0));
        float su = d.z > 1e-9 ? params.intrinsics.x * d.x / d.z + params.intrinsics.z : -1.0;
        float sv = d.z > 1e-9 ? params.intrinsics.y * d.y / d.z + params.intrinsics.w : -1.0;
        if (su >= -0.5 && sv >= -0.5 && su <= float(rw) - 0.5 && sv <= float(rh) - 0.5) {
            seen = true;
            su = clamp(su, 0.0, float(rw) - 1.0);
            sv = clamp(sv, 0.0, float(rh) - 1.0);
            const uint u0 = min(uint(su), rw > 1 ? rw - 2 : rw);
            const uint v0 = min(uint(sv), rh > 1 ? rh - 2 : rh);
            const float fu = su - float(u0);
            const float fv = sv - float(v0);
            const uint u1 = min(u0 + 1, rw - 1);
            const uint v1 = min(v0 + 1, rh - 1);
            const uint c00 = rendered[min(v0, rh - 1) * rw + min(u0, rw - 1)];
            const uint c10 = rendered[min(v0, rh - 1) * rw + u1];
            const uint c01 = rendered[v1 * rw + min(u0, rw - 1)];
            const uint c11 = rendered[v1 * rw + u1];
            for (uint c = 0; c < 3; ++c) {
                linear[c] = (1 - fu) * (1 - fv) * decode(channel(c00, c)) +
                            fu * (1 - fv) * decode(channel(c10, c)) +
                            (1 - fu) * fv * decode(channel(c01, c)) + fu * fv * decode(channel(c11, c));
            }
            const uint nearest = uint(floor(sv + 0.5)) * rw + uint(floor(su + 0.5));
            depth = asfloat(rendered[renderPixels + nearest]) / d.z;
            instance = rendered[2 * renderPixels + nearest];
            alpha = channel(rendered[nearest], 3);
        }
    }

    uint rgba = alpha << 24;
    float signal = seen ? 0.0 : -1.0;
    for (uint c = 0; c < 3; ++c) {
        const float l = linear[c] * ray.z;  // vignetting (0 where the lens maps no ray)
        uint byte;
        if (params.exposure != 0) {
            const float electrons = l * params.electronsPerUnit;
            if (seen) {
                signal += min(electrons * params.gain / params.fullWell, 1.0);
            }
            // digitise(): Poisson shot noise as a Gaussian, read noise, the full well, the gain.
            const uint4 counter0 = uint4(params.step0, params.step1, params.entity + v, 6 * u + 2 * c);
            const uint2 key = uint2(params.key0, params.key1);
            const float shot = normalOf(philox(counter0, key));
            const float read = normalOf(philox(counter0 + uint4(0, 0, 0, 1), key));
            const float mean = max(electrons, 0.0);
            float e = mean + sqrt(mean) * shot + params.readNoise * read;
            e = clamp(e, 0.0, params.fullWell);
            const float level = min(e * params.gain / params.fullWell, 1.0);
            const uint code = uint(floor(level * params.fullScale + 0.5));
            byte = tables[256 + code];
        } else {
            byte = encodeDisplay(l);
        }
        rgba |= byte << (8 * c);
    }
    outputs[i] = rgba;
    outputs[pixels + i] = asuint(depth);
    outputs[2 * pixels + i] = instance;
    outputs[3 * pixels + i] = asuint(signal);
}
