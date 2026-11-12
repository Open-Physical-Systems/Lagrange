// Kind "imaging_radar" (sensors.md §2 "4D imaging radar"): the radar's rays and echoes
// (Radar.cpp), resolved in range, Doppler, azimuth and elevation by the virtual array's cells.
#include "KindModule.hpp"
#include "lagrange/sensors/Radar.hpp"

namespace lagrange::sensors::kinds {

std::unique_ptr<IModule> makeImagingRadar(const World& world, std::uint32_t sensor,
                                          bool groundTruth, const KindContext& context) {
    return createImagingRadar(world, sensor, groundTruth, context.rayContext);
}

}  // namespace lagrange::sensors::kinds
