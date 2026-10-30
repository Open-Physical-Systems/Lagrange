// A C++ test plugin on the SDK (lagrange/plugin_sdk/World.hpp): it reads the world in declare()
// and writes sdk/bodies, sdk/joints, sdk/colliders, sdk/sensors (counts) and sdk/mass (kg).
#include "lagrange/plugin_sdk/World.hpp"
#include "lagrange_plugin.h"

#include <array>
#include <cstring>
#include <new>

namespace {

struct Probe {
    const lagrange_host_api* host{nullptr};
    std::array<std::uint32_t, 5> out{};
    std::array<double, 5> value{};
};

void* create(const lagrange_host_api* host) {
    auto* self = new (std::nothrow) Probe;
    if (self != nullptr) {
        self->host = host;
    }
    return self;
}

void destroy(void* self) {
    delete static_cast<Probe*>(self);
}

lagrange_status declare(void* p, const lagrange_world* world, const lagrange_config*,
                        lagrange_declarer* declarer) {
    auto* self = static_cast<Probe*>(p);
    const auto read = lagrange::plugin_sdk::World::read(*self->host, world);
    if (!read) {
        return LAGRANGE_UNSUPPORTED;
    }
    double mass = 0.0;
    for (const auto& body : read->bodies) {
        mass += body.mass;
    }
    self->value = {static_cast<double>(read->bodies.size()),
                   static_cast<double>(read->joints.size()),
                   static_cast<double>(read->colliders.size()),
                   static_cast<double>(read->sensors.size()), mass};
    constexpr std::array<const char*, 5> kNames{"sdk/bodies", "sdk/joints", "sdk/colliders",
                                                "sdk/sensors", "sdk/mass"};
    for (std::size_t i = 0; i < kNames.size(); ++i) {
        lagrange_signal_spec spec{};
        spec.struct_size = sizeof spec;
        spec.name = kNames.at(i);
        spec.type = LAGRANGE_SCALAR;
        spec.unit.scale = 1.0;
        if (i == 4) {
            spec.unit.mass = 1;
        }
        self->out.at(i) = self->host->declare_output(declarer, &spec);
    }
    return LAGRANGE_OK;
}

lagrange_status init(void* p, const lagrange_world*, const lagrange_config*, lagrange_bus* bus) {
    auto* self = static_cast<Probe*>(p);
    for (std::size_t i = 0; i < self->out.size(); ++i) {
        self->host->bus_write(bus, self->out.at(i), 0, self->value.at(i));
    }
    return LAGRANGE_OK;
}

lagrange_status step(void* p, const lagrange_step_context*, lagrange_bus* bus) {
    return init(p, nullptr, nullptr, bus);
}

const lagrange_module_api kModule = {sizeof(lagrange_module_api),
                                     "sdk_probe",
                                     "probe",
                                     1,
                                     0,
                                     0,
                                     1,
                                     0,
                                     100'000'000,
                                     create,
                                     destroy,
                                     declare,
                                     init,
                                     step,
                                     nullptr,
                                     nullptr,
                                     nullptr,
                                     nullptr,
                                     nullptr};

}  // namespace

extern "C" LAGRANGE_EXPORT const lagrange_module_api* lagrange_plugin_get_module(
    std::uint32_t major, std::uint32_t, std::uint32_t index) {
    return major == LAGRANGE_PLUGIN_ABI_MAJOR && index == 0 ? &kModule : nullptr;
}
