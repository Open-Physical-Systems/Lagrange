// Elevation models through GDAL and PROJ (Dem.hpp).
#include "lagrange/geospatial/Dem.hpp"

#include <array>
#include <cmath>
#include <gdal_priv.h>
#include <mutex>
#include <numbers>
#include <ogr_spatialref.h>

namespace lagrange::geospatial {

namespace {

constexpr double kDegree = 180.0 / std::numbers::pi;

/// One raster's first band in memory, with the way from WGS84 longitude/latitude to its pixels.
struct Raster {
    std::string path;
    int width{0};
    int height{0};
    std::array<double, 6> toPixel{};  // inverse geotransform: CRS x, y -> pixel column, row
    std::optional<double> noData;
    std::vector<float> values;
    std::shared_ptr<OGRCoordinateTransformation> fromWgs84;  // null: the raster is WGS84 lon/lat
    bool geographic{false};  // longitudes may need wrapping by 360 degrees

    [[nodiscard]] std::optional<double> pixel(int column, int row) const {
        if (column < 0 || row < 0 || column >= width || row >= height) {
            return std::nullopt;
        }
        const double v = values[static_cast<std::size_t>(row) * static_cast<std::size_t>(width) +
                                static_cast<std::size_t>(column)];
        if (!std::isfinite(v) || (noData && v == *noData)) {
            return std::nullopt;
        }
        return v;
    }

    [[nodiscard]] std::optional<double> sample(double lonDeg, double latDeg) const {
        double x = lonDeg;
        double y = latDeg;
        if (fromWgs84 && !fromWgs84->Transform(1, &x, &y)) {
            return std::nullopt;
        }
        for (const double wrap : {0.0, 360.0, -360.0}) {
            if (wrap != 0.0 && !geographic) {
                break;
            }
            const double px = toPixel[0] + toPixel[1] * (x + wrap) + toPixel[2] * y - 0.5;
            const double py = toPixel[3] + toPixel[4] * (x + wrap) + toPixel[5] * y - 0.5;
            const int c = static_cast<int>(std::floor(px));
            const int r = static_cast<int>(std::floor(py));
            const double fx = px - c;
            const double fy = py - r;
            const auto a = pixel(c, r);
            const auto b = pixel(c + 1, r);
            const auto d = pixel(c, r + 1);
            const auto e = pixel(c + 1, r + 1);
            if (a && b && d && e) {
                return (*a * (1 - fx) + *b * fx) * (1 - fy) + (*d * (1 - fx) + *e * fx) * fy;
            }
            // At the raster's edge: the nearest pixel.
            if (const auto nearest =
                    pixel(static_cast<int>(std::lround(px)), static_cast<int>(std::lround(py)))) {
                return nearest;
            }
        }
        return std::nullopt;
    }
};

Expected<Raster, std::string> readRaster(const std::string& path) {
    static std::once_flag once;
    std::call_once(once, [] { GDALAllRegister(); });
    std::unique_ptr<GDALDataset> dataset(
        static_cast<GDALDataset*>(GDALOpen(path.c_str(), GA_ReadOnly)));
    if (!dataset) {
        return Unexpected{"cannot open " + path + " as a raster (GDAL)"};
    }
    Raster out;
    out.path = path;
    out.width = dataset->GetRasterXSize();
    out.height = dataset->GetRasterYSize();
    std::array<double, 6> geo{};
    if (dataset->GetGeoTransform(geo.data()) != CE_None ||
        GDALInvGeoTransform(geo.data(), out.toPixel.data()) == 0) {
        return Unexpected{path + ": the raster has no usable geotransform"};
    }
    GDALRasterBand* band = dataset->GetRasterBand(1);
    if (band == nullptr) {
        return Unexpected{path + ": the raster has no band"};
    }
    int hasNoData = 0;
    const double noData = band->GetNoDataValue(&hasNoData);
    if (hasNoData != 0) {
        out.noData = static_cast<float>(noData);
    }
    out.values.resize(static_cast<std::size_t>(out.width) * static_cast<std::size_t>(out.height));
    if (band->RasterIO(GF_Read, 0, 0, out.width, out.height, out.values.data(), out.width,
                       out.height, GDT_Float32, 0, 0) != CE_None) {
        return Unexpected{path + ": cannot read the raster's first band"};
    }
    const OGRSpatialReference* crs = dataset->GetSpatialRef();
    OGRSpatialReference wgs84;
    wgs84.SetWellKnownGeogCS("WGS84");
    wgs84.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
    if (crs == nullptr) {
        return Unexpected{path + ": the raster has no coordinate reference system"};
    }
    OGRSpatialReference target(*crs);
    target.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
    out.geographic = target.IsGeographic() != 0;
    if (!(out.geographic && target.IsSameGeogCS(&wgs84) != 0)) {
        out.fromWgs84.reset(OGRCreateCoordinateTransformation(&wgs84, &target));
        if (!out.fromWgs84) {
            return Unexpected{path + ": no transformation from WGS84 to the raster's CRS (PROJ)"};
        }
    }
    return out;
}

}  // namespace

struct Dem::Impl {
    std::vector<Raster> heights;
    std::optional<Raster> geoid;
    bool assumeEllipsoidal{false};
    std::vector<std::string> sources;
};

Dem::Dem() = default;
Dem::~Dem() = default;

Expected<Dem, std::string> Dem::open(const Options& options) {
    if (options.paths.empty()) {
        return Unexpected{std::string("an elevation model needs at least one raster")};
    }
    if (options.geoidPath.empty() && !options.assumeEllipsoidal) {
        return Unexpected{std::string(
            "elevation models usually give heights above the geoid: name a geoid undulation "
            "raster (--geoid, e.g. /usr/share/proj/egm96_15.gtx) or say the heights are "
            "ellipsoidal (--assume-ellipsoidal)")};
    }
    auto impl = std::make_shared<Impl>();
    impl->assumeEllipsoidal = options.assumeEllipsoidal;
    for (const std::string& path : options.paths) {
        auto raster = readRaster(path);
        if (!raster) {
            return Unexpected{raster.error()};
        }
        impl->heights.push_back(std::move(*raster));
        impl->sources.push_back(path);
    }
    if (!options.geoidPath.empty()) {
        auto geoid = readRaster(options.geoidPath);
        if (!geoid) {
            return Unexpected{geoid.error()};
        }
        impl->geoid = std::move(*geoid);
        impl->sources.push_back(options.geoidPath + " (geoid undulation)");
    }
    Dem dem;
    dem.impl_ = std::move(impl);
    return dem;
}

std::optional<double> Dem::ellipsoidalHeight(double latitude, double longitude) const {
    const double lat = latitude * kDegree;
    const double lon = longitude * kDegree;
    for (const Raster& raster : impl_->heights) {
        if (const auto h = raster.sample(lon, lat)) {
            if (impl_->assumeEllipsoidal || !impl_->geoid) {
                return h;
            }
            const auto n = impl_->geoid->sample(lon, lat);
            if (!n) {
                return std::nullopt;
            }
            return *h + *n;
        }
    }
    return std::nullopt;
}

const std::vector<std::string>& Dem::sources() const {
    return impl_->sources;
}

}  // namespace lagrange::geospatial
