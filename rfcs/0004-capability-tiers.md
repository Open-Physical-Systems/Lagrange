# RFC 0004: Capability tiers

| | |
|---|---|
| Status | Draft |
| Created | 2026-10-13 |
| Affects | `gpu/capability/`, `gpu/hal/`, `gpu/kernels/`, every module with a GPU path, CI, the profile format |
| Detail | `docs/design/architecture.md` §9 |

## Summary

Define three hardware capability tiers on Ubuntu 26.04, how a running process determines its
tier, how a user forces a lower tier, and the rule that every feature states its behaviour on
each tier.

## Motivation

The simulator must run in a GPU-less CI container, on a laptop with an integrated GPU, and on an
RTX workstation (invariant 9). Without a small, fixed set of tiers, each feature would grow its
own hardware checks and fallbacks, and "works on my machine" bugs would follow. Tiers also give
the deterministic CPU reference (invariant 7) a clear home: Tier 0.

The cross-platform meaning of tiers in the original design (different operating systems and
graphics APIs) is deferred. Tiers here are hardware tiers on one OS.

## Design

### Definitions

| Tier | Name | Requirements | Typical hardware |
|---|---|---|---|
| 0 | CPU | Nothing beyond the CPU. A software Vulkan device (Mesa lavapipe) may be present and is used only for raster camera sensors and viewer tests | CI runners, servers, containers |
| 1 | Raster GPU | A hardware Vulkan 1.3 device with compute shaders, `shaderFloat64` not required, at least 2 GB of device memory, and the features in the list below | Intel and AMD integrated GPUs, older discrete GPUs |
| 2 | Full | Tier 1 plus `VK_KHR_acceleration_structure` and `VK_KHR_ray_query`, at least 6 GB of device memory. CUDA is optional and detected separately | NVIDIA RTX 20-series and newer; AMD RX 6000 and newer (best-effort) |

Tier 1 required features (proposal, to be confirmed on real devices in issue 0.1-D3):
`VK_KHR_dynamic_rendering`, `VK_KHR_synchronization2`, descriptor indexing (runtime arrays,
partially bound), `VK_EXT_scalar_block_layout`, timeline semaphores, 16-bit and 8-bit storage,
`maxComputeWorkGroupInvocations ≥ 1024`. These are core in Vulkan 1.3 or widely available, but
the claim that all Tier 1 target devices expose them is unverified.

CUDA is an orthogonal flag, not a tier: `cuda = true` when a CUDA runtime and a compatible device
are present. It enables optional fast paths (PhysX GPU, `rosidl::Buffer` CUDA backend) behind the
same HAL interfaces (invariant 8).

### Detection

```cpp
// gpu/capability/Capability.hpp
namespace lagrange::gpu {

enum class CapabilityTier : std::uint8_t { Tier0 = 0, Tier1 = 1, Tier2 = 2 };

struct DeviceInfo {
    std::string name;                  // "NVIDIA GeForce RTX 4060 Laptop GPU"
    std::string driver;                // "NVIDIA 595.91.07" / "Mesa 26.0.8 (lavapipe)"
    std::uint32_t vendorId, deviceId;
    bool isSoftware;                   // VK_PHYSICAL_DEVICE_TYPE_CPU
    std::uint64_t deviceMemoryBytes;
    bool rayQuery, accelerationStructure, rayTracingPipeline;
    std::uint32_t apiVersionMajor, apiVersionMinor;
};

struct Capability {
    CapabilityTier detected;           // what the hardware supports
    CapabilityTier effective;          // after profile / environment override (never above detected)
    bool cuda;
    std::optional<DeviceInfo> device;  // empty when no Vulkan device at all
    std::vector<std::string> reasons;  // why not a higher tier, human-readable
};

Capability detect(std::optional<CapabilityTier> requested);   // requested from profile or LAGRANGE_TIER

}  // namespace lagrange::gpu
```

Rules:

1. Enumerate Vulkan physical devices. Prefer discrete, then integrated; never choose a software
   device when a hardware one exists. `LAGRANGE_GPU=<index|name>` selects explicitly.
2. A software device alone gives Tier 0.
3. The tier is the highest whose requirements the chosen device meets.
4. `tier:` in the profile or `LAGRANGE_TIER` lowers the effective tier. Requesting a tier above
   the detected one is an error (exit code 3), not a silent downgrade.
5. `lagrange_gpu_info` prints the `Capability` as text and JSON; bug reports ask for it.
6. `reasons` lists what is missing ("no VK_KHR_ray_query", "4 GB device memory < 6 GB").

### The per-feature rule

Every feature that touches the GPU or has a CPU fallback declares, in its design doc and in code:

```cpp
struct TierSupport {
    enum class Level : std::uint8_t { Unsupported, Degraded, Full };
    Level tier0, tier1, tier2;
    std::string_view degradedNote;     // what is different when Degraded
};
```

- `ModuleInfo::minimumTier` lets the scheduler refuse a module on a lower tier at wiring time
  with `WiringError::TierUnsupported`, naming the feature.
- "Unsupported" must produce a clear startup error or a disabled UI control with a reason. It
  must never crash or produce silently wrong output.
- The feature table in `architecture.md` §9 is the single list; a PR that adds a GPU feature
  adds a row (PR template checkbox).

### Equivalence between tiers

Where the same physical quantity is computed on several tiers (lidar ranges, field samples, dose
rays), a tier-agreement test compares outputs on the same scene within a stated tolerance.
Tier 0 is the reference. Rendering-based outputs (camera images) are compared statistically
(mean, noise, geometry), not bit-wise.

### Determinism

Deterministic mode forces modules that are not `deterministic` onto their Tier 0 equivalents,
regardless of the hardware tier. GPU paths may declare themselves deterministic only with a test
that proves bit-identical output across runs on the same device and driver.

### CI mapping

| Tier | Where tested | When |
|---|---|---|
| 0 | GitHub-hosted runners in `ros:lyrical` containers, lavapipe for raster | every PR |
| 1 | Self-hosted runner with `LAGRANGE_TIER=1` (same RTX machine, RT features ignored); real integrated-GPU hardware in the manual release matrix | merge queue, nightly |
| 2 | Self-hosted RTX runner | merge queue, nightly |

Running Tier 1 code paths on Tier 2 hardware does not test Tier 1 drivers. That gap is covered
only by the manual matrix until an integrated-GPU runner exists.

## Alternatives considered

| Alternative | Why not |
|---|---|
| Per-feature capability flags only, no tiers | Combinatorial test space; users cannot tell what to expect from their machine |
| Two tiers (CPU, GPU) | Hardware ray tracing changes sensor quality and speed enough to be its own class, and many laptops lack it |
| Four or more tiers (splitting by VRAM or by vendor) | More rows per feature for little benefit before 1.0; VRAM appears as a budget inside Tier 2 instead |
| CUDA as Tier 3 | CUDA availability is independent of ray tracing support; treating it as a flag avoids a false ordering |
| Require a GPU always | Breaks CI on hosted runners and the deterministic reference |
| OpenGL fallback for Tier 1 | A second graphics API in the HAL; Vulkan 1.3 is available on current Mesa drivers for the integrated GPUs that Ubuntu 26.04 runs on [Medium, general knowledge of Mesa driver coverage; confirm on test hardware] |
| OptiX for Tier 2 ray tracing | NVIDIA-only and proprietary licensing; Vulkan ray query is vendor-neutral. OptiX could be an optional fast path later behind the HAL |

## Unresolved questions

1. **Exact Tier 1 feature list** and minimum memory, after testing on real Intel and AMD
   integrated GPUs on Ubuntu 26.04.
2. **Tier 2 memory threshold** (6 GB proposed): is an RTX 3050 with 4 GB Tier 2 with reduced
   budgets, or Tier 1?
3. **Multi-GPU**: one device for the viewer and another for sensors on the same machine; device
   selection per process is possible now, per subsystem is future work.
4. **Viewer and server on one GPU**: sharing rendered sensor images without a CPU round trip
   needs Vulkan external memory between processes. Out of scope here; noted in RFC 0003.
5. **Ray-tracing pipeline versus ray query**: the plan uses ray query from compute shaders only.
   Is `VK_KHR_ray_tracing_pipeline` needed for any sensor? Proposed: no.
6. **Lavapipe for Tier 0 cameras**: acceptable performance for small images only. Should Tier 0
   camera simulation be documented as "tests and low resolution only"? Proposed: yes.
7. **Slang CUDA target** in Ubuntu's package is unverified; affects which fast paths can share
   kernel source.
8. **A Tier 1 hardware runner** in CI: needed before 1.0?
