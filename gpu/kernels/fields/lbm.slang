// One step of the wind tunnel: the same lattice Boltzmann update as the CPU reference,
// modules/aerodynamics/lbm/src/Lbm.cpp, in float. See that file for its sources. Not
// deterministic across devices (invariant 7: the CPU path is the reference).

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
static const uint kRollingFloor = 0xFFFE;

struct Params {
    int nx;
    int ny;
    int nz;
    int periodic;
    float tau;
    float inletU;
    float smagorinsky2;  // the constant squared
    float pad;
};

[[vk::binding(0, 0)]] StructuredBuffer<float> f;
[[vk::binding(1, 0)]] RWStructuredBuffer<float> next;
[[vk::binding(2, 0)]] StructuredBuffer<uint> solid;
[[vk::push_constant]] ConstantBuffer<Params> params;

float equilibrium(int i, float rho, float3 u) {
    const float cu = float(kCx[i]) * u.x + float(kCy[i]) * u.y + float(kCz[i]) * u.z;
    return kWeight[i] * rho * (1.0 + 3.0 * cu + 4.5 * cu * cu - 1.5 * dot(u, u));
}

[shader("compute")]
[numthreads(256, 1, 1)]
void stepCells(uint3 id : SV_DispatchThreadID) {
    const int nx = params.nx;
    const int ny = params.ny;
    const int nz = params.nz;
    const int cell = int(id.x);
    if (cell >= nx * ny * nz || solid[cell] != kFluid) {
        return;
    }
    const int z = cell / (nx * ny);
    const int y = (cell - z * nx * ny) / nx;
    const int x = cell - nx * (y + ny * z);
    float in[19];
    for (int i = 0; i < kQ; ++i) {
        int sx = x - kCx[i];
        const int sy = (y - kCy[i] + ny) % ny;
        const int sz = (z - kCz[i] + nz) % nz;
        if (params.periodic != 0) {
            sx = (sx + nx) % nx;
        } else if (sx < 0) {
            in[i] = equilibrium(i, 1.0, float3(params.inletU, 0.0, 0.0));
            continue;
        } else if (sx >= nx) {
            sx = nx - 1;
        }
        const int source = sx + nx * (sy + ny * sz);
        const uint kind = solid[source];
        if (kind == kRollingFloor) {
            in[i] = f[cell * kQ + kOpposite[i]] + 6.0 * kWeight[i] * float(kCx[i]) * params.inletU;
        } else if (kind != kFluid) {
            in[i] = f[cell * kQ + kOpposite[i]];
        } else {
            in[i] = f[source * kQ + i];
        }
    }
    float rho = 0.0;
    float3 u = float3(0.0, 0.0, 0.0);
    for (int i = 0; i < kQ; ++i) {
        rho += in[i];
        u += float3(float(kCx[i]), float(kCy[i]), float(kCz[i])) * in[i];
    }
    u /= rho;
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
        next[cell * kQ + i] = in[i] + omega * (eq[i] - in[i]);
    }
}
