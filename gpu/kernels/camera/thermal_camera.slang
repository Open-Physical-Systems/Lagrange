// The thermal camera's image on the GPU (docs/design/domains/sensors.md §2): the same per-pixel
// work as modules/sensors' Camera::stepThermal, the CPU reference, on the instance ids the raster
// pass has just drawn and copied, so only the images travel back to the host. Two passes:
//
//   thermalTemperature: each pixel's apparent temperature (its surface's, from a table by scene
//     prim; the clear sky where nothing was drawn) plus the detector's noise, and the frame's
//     lowest and highest temperature (atomics on the float bits: temperatures are positive);
//   thermalPalette: the display range (fixed, or the frame's widened to at least a span) and the
//     palette's colour.
//
// The noise draws the reference's random numbers: Philox4x32-10 (Salmon et al. 2011, as
// core/random), keyed by the module seed, counter (step, step >> 32, entity + row, u): one block
// per normal, so pixel u of a row takes block u of the CPU's per-row stream. The bits are the same;
// the sum is in float here (double in the reference), a difference of about 10^-5 K, and a colour
// can differ by one where it sits on a rounding edge.

struct Params {
    uint width;
    uint height;
    uint prims;           // entries of the apparent-temperature table
    uint palette;         // 0 white hot, 1 black hot, 2 ironbow (ThermalPalette)
    float sky;            // K, where nothing was drawn
    float netd;           // K, the noise's standard deviation
    float displayMin;     // K; displayMax > displayMin: a fixed display range
    float displayMax;
    float span;           // K, the least range of a frame's own display range
    uint key0;            // module seed, low and high words
    uint key1;
    uint step0;           // step index, low and high words
    uint step1;
    uint entity;          // the first row's stream; row v uses entity + v
    uint pad0;
    uint pad1;
};

// The render, as RasterCamera reads it back: RGBA8 words (not drawn), then depth (float bits),
// then instance ids, each width * height long.
[[vk::binding(0, 0)]] StructuredBuffer<uint> rendered;
// K, the apparent temperature of each scene prim this frame.
[[vk::binding(1, 0)]] StructuredBuffer<float> apparent;
// The frame's lowest and highest temperature (float bits), set to (+inf, 0) before the first pass.
[[vk::binding(2, 0)]] RWStructuredBuffer<uint> range;
// Output: RGBA8 words (the palette), depth (float bits), instance ids, temperature (float bits, K).
[[vk::binding(3, 0)]] RWStructuredBuffer<uint> outputs;
[[vk::push_constant]] ConstantBuffer<Params> params;

static const float kPi = 3.14159265358979323846;

// --- Philox4x32-10 (core/random/src/Philox.cpp), as camera_model.slang ---------------------------

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

// --- the image ----------------------------------------------------------------------------------

static const uint kNoInstance = 0xFFFFFFFF;

[numthreads(64, 1, 1)]
void thermalTemperature(uint3 id: SV_DispatchThreadID) {
    const uint w = params.width;
    const uint pixels = w * params.height;
    const uint i = id.x;
    if (i >= pixels) {
        return;
    }
    const uint u = i % w;
    const uint v = i / w;
    const uint instance = rendered[2 * pixels + i];
    const float t = instance < params.prims ? apparent[instance] : params.sky;
    const uint4 counter = uint4(params.step0, params.step1, params.entity + v, u);
    const float kelvin = t + params.netd * normalOf(philox(counter, uint2(params.key0, params.key1)));
    outputs[pixels + i] = rendered[pixels + i];
    outputs[2 * pixels + i] = instance;
    outputs[3 * pixels + i] = asuint(kelvin);
    // Positive floats order as their bits do.
    InterlockedMin(range[0], asuint(kelvin));
    InterlockedMax(range[1], asuint(kelvin));
}

uint byteOf(float v) {
    return uint(floor(clamp(v, 0.0, 1.0) * 255.0 + 0.5));
}

// paletteColour (modules/sensors ThermalCamera.cpp): x in [0, 1], cold to hot.
uint colourOf(uint palette, float x) {
    x = isnan(x) ? 0.0 : clamp(x, 0.0, 1.0);
    if (palette == 0) {
        const uint b = byteOf(x);
        return b | (b << 8) | (b << 16) | (255u << 24);
    }
    if (palette == 1) {
        const uint b = byteOf(1.0 - x);
        return b | (b << 8) | (b << 16) | (255u << 24);
    }
    // Ironbow's stops: black, indigo, purple-red, orange, yellow, white.
    const float4 stops[6] = { float4(0.0, 0.0, 0.0, 0.0), float4(0.2, 0.15, 0.0, 0.45),
                              float4(0.45, 0.65, 0.0, 0.55), float4(0.7, 0.95, 0.4, 0.0),
                              float4(0.88, 1.0, 0.8, 0.1), float4(1.0, 1.0, 1.0, 0.95) };
    uint k = 1;
    while (k + 1 < 6 && x > stops[k].x) {
        ++k;
    }
    const float4 a = stops[k - 1];
    const float4 b = stops[k];
    const float f = (x - a.x) / (b.x - a.x);
    return byteOf(a.y + f * (b.y - a.y)) | (byteOf(a.z + f * (b.z - a.z)) << 8) |
           (byteOf(a.w + f * (b.w - a.w)) << 16) | (255u << 24);
}

[numthreads(64, 1, 1)]
void thermalPalette(uint3 id: SV_DispatchThreadID) {
    const uint pixels = params.width * params.height;
    const uint i = id.x;
    if (i >= pixels) {
        return;
    }
    float lo = params.displayMin;
    float hi = params.displayMax;
    if (!(hi > lo)) {
        // The frame's own range, never under the span (a scene at one temperature is one colour).
        lo = asfloat(range[0]);
        hi = asfloat(range[1]);
        if (hi - lo < params.span) {
            const float mid = (lo + hi) / 2.0;
            lo = mid - params.span / 2.0;
            hi = mid + params.span / 2.0;
        }
    }
    const float kelvin = asfloat(outputs[3 * pixels + i]);
    outputs[i] = colourOf(params.palette, (kelvin - lo) / (hi - lo));
}
