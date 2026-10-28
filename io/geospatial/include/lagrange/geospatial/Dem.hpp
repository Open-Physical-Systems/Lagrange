#pragma once

#include "lagrange/module/Expected.hpp"

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace lagrange::geospatial {

/// A digital elevation model (huge-worlds.md §4): one or more rasters GDAL reads (GeoTIFF and
/// others), first band the height in metres, in any coordinate reference system PROJ knows.
/// Where they overlap, the first raster listed wins.
///
/// Heights in elevation models are usually orthometric (above the geoid: Copernicus DEM, SRTM),
/// while the world's frame needs heights above the WGS84 ellipsoid. The datum is never guessed:
/// give a geoid undulation raster (EGM96 is /usr/share/proj/egm96_15.gtx with proj-data), or
/// say the heights are ellipsoidal already.
class Dem {
public:
    struct Options {
        std::vector<std::string> paths;  ///< The height rasters. Required.
        std::string geoidPath;           ///< Geoid undulation N (m) on WGS84 lat/lon: h = H + N.
        bool assumeEllipsoidal{false};   ///< The heights are above the ellipsoid already.
    };

    [[nodiscard]] static Expected<Dem, std::string> open(const Options& options);

    /// Height above the WGS84 ellipsoid (m) at a geodetic latitude and longitude (radians),
    /// bilinear between pixel centres; empty outside every raster or on a no-data pixel.
    [[nodiscard]] std::optional<double> ellipsoidalHeight(double latitude, double longitude) const;

    /// What the heights came from, for the world's attribution file.
    [[nodiscard]] const std::vector<std::string>& sources() const;

    Dem(const Dem&) = default;
    Dem& operator=(const Dem&) = default;
    Dem(Dem&&) noexcept = default;
    Dem& operator=(Dem&&) noexcept = default;
    ~Dem();

private:
    Dem();
    struct Impl;
    std::shared_ptr<const Impl> impl_;
};

}  // namespace lagrange::geospatial
