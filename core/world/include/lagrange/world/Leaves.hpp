#pragma once

// Flying leaves (docs/physics/ground.md "Trees"): leaves and twigs a propeller cuts from a crown,
// or the wind and the rotors' wake strip from it, fly as particles until they settle on the
// ground. The vegetation domain moves them and publishes them on the bus as one ScalarArray signal;
// the viewer and the camera sensors draw them, the lidars may see them. The layout is shared here
// so the modules agree without including each other (architecture invariant 1):
//
//     kLeavesSignal: [version, count, (x, y, z, yaw, pitch, size) x count, unused...]
//
// version grows every step the particles move, count is the particles in use (the oldest are
// dropped past kLeavesCapacity), (x, y, z) a particle's centre (world_local, m), yaw and pitch its
// leaf's orientation (rad: the normal of the leaf blade is (cos pitch cos yaw, cos pitch sin yaw,
// sin pitch)), size its blade's length (m). A particle at rest on the ground stays listed until it
// is dropped for a newer one.

#include <cstdint>
#include <string_view>

namespace lagrange {

inline constexpr std::string_view kLeavesSignal = "vegetation/leaves";
inline constexpr std::uint32_t kLeavesCapacity = 2048;
inline constexpr std::uint32_t kLeavesHeader = 2;
inline constexpr std::uint32_t kLeavesFields = 6;
inline constexpr std::uint32_t kLeavesLength = kLeavesHeader + (kLeavesFields * kLeavesCapacity);

// The air at each listed leaf, from the aerodynamics domain, which reads kLeavesSignal a step late
// (docs/physics/ground.md "Leaves lifted again"):
//
//     kLeavesAirSignal: [version, count, (x, y, z, ax, ay, az, gx, gy, gz) x count, unused...]
//
// version and count are those of the kLeavesSignal it read, (x, y, z) that leaf's centre as it
// read it (m, the key the vegetation domain matches its particles by), (ax, ay, az) the air at the
// centre (m/s: the mean wind at its height, the gust, every robot's rotors with their outwash) for
// a leaf in flight, and (gx, gy, gz) the air along the ground under it (m/s: the mean wind at the
// leaf's blade length above the ground plus the rotors' outwash at the wall jet's peak) for a leaf
// lying there.
inline constexpr std::string_view kLeavesAirSignal = "vegetation/leaves_air";
inline constexpr std::uint32_t kLeavesAirFields = 9;
inline constexpr std::uint32_t kLeavesAirLength =
    kLeavesHeader + (kLeavesAirFields * kLeavesCapacity);

}  // namespace lagrange
