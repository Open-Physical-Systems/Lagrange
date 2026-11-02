// One step of the air box round a multicopter: the lattice Boltzmann update of the CPU
// reference, modules/aerodynamics/lbm/src/LbmBox.cpp, in float. See that file for its sources.
// Not deterministic across devices (invariant 7: the CPU path is the reference).
//
// Unlike the tunnel (lbm.slang) the populations are stored one direction after another
// (direction i of cell c at i * cells + c), so neighbouring threads read neighbouring words, and
// in half precision, as the population less its rest weight (FP16S of Lehmann et al., Phys. Rev.
// E 106, 015308, 2022; TODO(verify) the volume), two cells to a 32-bit word: the step is bound by
// memory traffic, and this halves it. Each thread steps the two cells of a word (x even and odd),
// so no two threads write one word; the box is an even number of cells across. Rotors push the
// air as actuator discs through Guo's forcing; the box's sides are open at ambient pressure, and
// its floor, when it has one, is a no-slip ground.

static const int kQ = 19;
static const int kCx[19] = {0, 1, -1, 0, 0, 0, 0, 1, -1, 1, -1, 1, -1, 1, -1, 0, 0, 0, 0};
static const int kCy[19] = {0, 0, 0, 1, -1, 0, 0, 1, -1, -1, 1, 0, 0, 0, 0, 1, -1, 1, -1};
static const int kCz[19] = {0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 1, -1, -1, 1, 1, -1, -1, 1};
static const int kOpposite[19] = {0, 2, 1, 4, 3, 6, 5, 8, 7, 10, 9, 12, 11, 14, 13, 16, 15, 18, 17};
static const float kWeight[19] = {
    1.0 / 3.0,  1.0 / 18.0, 1.0 / 18.0, 1.0 / 18.0, 1.0 / 18.0, 1.0 / 18.0, 1.0 / 18.0,
    1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0,
    1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0, 1.0 / 36.0};
static const uint kFluid = 0xFFFF;
static const int kMaxDiscs = 16;

struct Params {
    int nx;
    int ny;
    int nz;
    int floor;  // 1: the box's bottom is ground
    float tau;
    float smagorinsky2;  // the constant squared
    int discs;
    int solids;  // 0: no solid cell in the box, so none is looked up
};

// A rotor: its centre (cells, from the box's corner) and radius (cells); the push on the air per
// unit of weight (lattice force) as a vector; the hat weights are worked out as on the CPU.
struct Disc {
    float4 centreRadius;
    float4 axisUnused;  // the thrust axis, unit
    float4 pushUnused;  // lattice force on the air per unit weight
};

[[vk::binding(0, 0)]] StructuredBuffer<uint> f;          // half pairs, see above
[[vk::binding(1, 0)]] RWStructuredBuffer<uint> next;
[[vk::binding(2, 0)]] StructuredBuffer<uint> solid;
[[vk::binding(3, 0)]] RWStructuredBuffer<float> velocity;  // 3 per cell, one component after another
[[vk::binding(4, 0)]] StructuredBuffer<Disc> discs;
[[vk::push_constant]] ConstantBuffer<Params> params;

float equilibrium(int i, float rho, float3 u) {
    const float cu = float(kCx[i]) * u.x + float(kCy[i]) * u.y + float(kCz[i]) * u.z;
    return kWeight[i] * rho * (1.0 + 3.0 * cu + 4.5 * cu * cu - 1.5 * dot(u, u));
}

// Direction i of a cell, from the half pairs.
float load(int i, int cell, int cells) {
    const uint word = f[uint(i * cells + cell) >> 1];
    const uint half = (cell & 1) != 0 ? (word >> 16) : (word & 0xFFFFu);
    return f16tof32(half) + kWeight[i];
}

uint packed(int i, float value) { return f32tof16(value - kWeight[i]) & 0xFFFFu; }

// The weight of a disc at a cell centre: a hat one cell thick along the axis, and the share of the
// cell inside the radius across it.
float discWeight(Disc disc, float3 at) {
    const float3 d = at - disc.centreRadius.xyz;
    const float3 axis = disc.axisUnused.xyz;
    const float along = dot(d, axis);
    const float wa = max(0.0, 1.0 - abs(along));
    if (wa <= 0.0) {
        return 0.0;
    }
    const float across = length(d - along * axis);
    const float wr = clamp(disc.centreRadius.w - across + 0.5, 0.0, 1.0);
    return wa * wr;
}

// Streams and collides one cell; its new populations, packed, go to `out`.
void stepCell(int x, int y, int z, int cell, out uint out[19]) {
    const int nx = params.nx;
    const int ny = params.ny;
    const int nz = params.nz;
    const int cells = nx * ny * nz;
    if (params.solids != 0 && solid[cell] != kFluid) {
        for (int i = 0; i < kQ; ++i) {
            out[i] = packed(i, load(i, cell, cells));
        }
        return;
    }
    const float3 own = float3(velocity[cell], velocity[cells + cell], velocity[2 * cells + cell]);
    float in[19];
    for (int i = 0; i < kQ; ++i) {
        const int sx = x - kCx[i];
        const int sy = y - kCy[i];
        const int sz = z - kCz[i];
        if (sz < 0 && params.floor != 0) {
            in[i] = load(kOpposite[i], cell, cells);  // the ground: half-way bounce-back
        } else if (sx < 0 || sy < 0 || sz < 0 || sx >= nx || sy >= ny || sz >= nz) {
            // Open: ambient pressure, the flow carried on outwards; air drawn in comes by the
            // pressure alone, since carrying an inward speed in feeds on itself.
            float3 o = own;
            if ((sx < 0 && o.x > 0.0) || (sx >= nx && o.x < 0.0)) o.x = 0.0;
            if ((sy < 0 && o.y > 0.0) || (sy >= ny && o.y < 0.0)) o.y = 0.0;
            if ((sz < 0 && o.z > 0.0) || (sz >= nz && o.z < 0.0)) o.z = 0.0;
            in[i] = equilibrium(i, 1.0, o);
        } else {
            const int source = sx + nx * (sy + ny * sz);
            if (params.solids != 0 && solid[source] != kFluid) {
                in[i] = load(kOpposite[i], cell, cells);
            } else {
                in[i] = load(i, source, cells);
            }
        }
    }
    float3 force = float3(0.0, 0.0, 0.0);
    const float3 centre = float3(float(x) + 0.5, float(y) + 0.5, float(z) + 0.5);
    for (int d = 0; d < min(params.discs, kMaxDiscs); ++d) {
        const Disc disc = discs[d];
        force += disc.pushUnused.xyz * discWeight(disc, centre);
    }
    float rho = 0.0;
    float3 u = float3(0.0, 0.0, 0.0);
    for (int i = 0; i < kQ; ++i) {
        rho += in[i];
        u += float3(float(kCx[i]), float(kCy[i]), float(kCz[i])) * in[i];
    }
    u = (u + 0.5 * force) / rho;
    float xx = 0.0, yy = 0.0, zz = 0.0, xy = 0.0, xz = 0.0, yz = 0.0;
    float eq[19];
    for (int i = 0; i < kQ; ++i) {
        eq[i] = equilibrium(i, rho, u);
        const float off = in[i] - eq[i];
        const float cx = float(kCx[i]);
        const float cy = float(kCy[i]);
        const float cz = float(kCz[i]);
        xx += cx * cx * off;
        yy += cy * cy * off;
        zz += cz * cz * off;
        xy += cx * cy * off;
        xz += cx * cz * off;
        yz += cy * cz * off;
    }
    float relax = params.tau;
    if (params.smagorinsky2 > 0.0) {
        const float q = sqrt(xx * xx + yy * yy + zz * zz + 2.0 * (xy * xy + xz * xz + yz * yz));
        relax = 0.5 * (params.tau +
                       sqrt(params.tau * params.tau + 18.0 * 1.41421356 * params.smagorinsky2 * q / rho));
    }
    const float omega = 1.0 / relax;
    for (int i = 0; i < kQ; ++i) {
        const float3 c = float3(float(kCx[i]), float(kCy[i]), float(kCz[i]));
        const float source =
            (1.0 - 0.5 * omega) * kWeight[i] * dot(3.0 * (c - u) + 9.0 * dot(c, u) * c, force);
        out[i] = packed(i, in[i] + omega * (eq[i] - in[i]) + source);
    }
    velocity[cell] = u.x;
    velocity[cells + cell] = u.y;
    velocity[2 * cells + cell] = u.z;
}

[shader("compute")]
[numthreads(256, 1, 1)]
void stepCells(uint3 id : SV_DispatchThreadID) {
    const int nx = params.nx;
    const int ny = params.ny;
    const int nz = params.nz;
    const int cells = nx * ny * nz;
    const int first = 2 * int(id.x);  // even: x even, nx even
    if (first >= cells) {
        return;
    }
    const int z = first / (nx * ny);
    const int y = (first - z * nx * ny) / nx;
    const int x = first - nx * (y + ny * z);
    uint low[19];
    uint high[19];
    stepCell(x, y, z, first, low);
    stepCell(x + 1, y, z, first + 1, high);
    for (int i = 0; i < kQ; ++i) {
        next[uint(i * cells + first) >> 1] = low[i] | (high[i] << 16);
    }
}
