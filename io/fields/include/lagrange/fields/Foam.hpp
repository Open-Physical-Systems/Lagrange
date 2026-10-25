#pragma once

// Offline CFD import (roadmap §6, "OpenFOAM result → VDB wind field"): an OpenFOAM case's cell
// velocities onto a NanoVDB vector grid that the weather domain reads as its wind field. We
// read OpenFOAM's files; OpenFOAM itself is not a dependency (CLAUDE.md invariant 12).

#include "lagrange/fields/Field.hpp"
#include "lagrange/module/Expected.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace lagrange::fields {

/// The internalField of an ASCII OpenFOAM volVectorField file (a time directory's U, or the C
/// that `postProcess -func writeCellCentres` writes): one vector per cell. Reads
/// `nonuniform List<vector> N ( (x y z) ... )`, the compact `N{(x y z)}` and `uniform (x y z)`
/// (returned as one vector). Binary files are refused (write the case with
/// `writeFormat ascii`). TODO(verify) against files from a current OpenFOAM release: the
/// format here follows the files' documented layout, and no OpenFOAM install was at hand.
[[nodiscard]] Expected<std::vector<Vec3>, std::string> readFoamVectorField(const std::string& path);

/// What foamToWindField made.
struct FoamImport {
    std::size_t cells{0};   // cells read
    std::size_t voxels{0};  // voxels that got cells
    Vec3 min{}, max{};      // bounds of the voxel centres, region-local
};

/// Bins cells into voxels of `voxelSize` (m) and writes the mean velocity of each voxel's cells
/// as a vector grid named `name` at `path`. Voxel k covers [k·h, (k + 1)·h) on each axis of the
/// region-local frame (the case's coordinates plus `offset`), so the cells of a mesh of size h
/// aligned with the origin land one per voxel, centre on centre; voxels without cells are
/// left out (the field's background, zero: the weather's mean wind holds outside the grid's
/// bounds). A uniform U (one vector) fills every cell.
[[nodiscard]] Expected<FoamImport, std::string> foamToWindField(
    const std::vector<Vec3>& centres, const std::vector<Vec3>& velocity, const std::string& path,
    const std::string& name, double voxelSize, const Vec3& offset = {});

}  // namespace lagrange::fields
