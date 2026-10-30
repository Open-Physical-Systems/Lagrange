/* A test plugin with a domain of its own (RFC 0008): acme_probe/probe reads the robot's base
 * pose and writes its x plus the profile value "offset" as acme/probe/x (m). */
#include "lagrange_plugin.h"

#include <stdlib.h>
#include <string.h>

typedef struct probe {
    const lagrange_host_api* host;
    uint32_t pose;
    uint32_t x;
    double offset;
} probe;

static const lagrange_unit kMetre = {1, 0, 0, 0, 0, 0, 0, 0, 1.0, 14 /* Position */};

static void* create(const lagrange_host_api* host) {
    probe* self = (probe*)calloc(1, sizeof(probe));
    if (self != NULL) {
        self->host = host;
    }
    return self;
}

static void destroy(void* self) {
    free(self);
}

static lagrange_status declare(void* p, const lagrange_world* world, const lagrange_config* config,
                               lagrange_declarer* declarer) {
    probe* self = (probe*)p;
    lagrange_signal_spec spec;
    (void)world;
    self->offset = 0.0;
    (void)self->host->config_get_real(config, "offset", &self->offset);
    memset(&spec, 0, sizeof spec);
    spec.struct_size = sizeof spec;
    spec.name = "body/base_link/pose";
    spec.type = LAGRANGE_POSE;
    spec.unit = kMetre;
    spec.frame = "world_local";
    self->pose = self->host->declare_input(declarer, &spec);
    spec.name = "acme/probe/x";
    spec.type = LAGRANGE_SCALAR;
    spec.frame = NULL;
    self->x = self->host->declare_output(declarer, &spec);
    return self->pose == LAGRANGE_INVALID_SIGNAL || self->x == LAGRANGE_INVALID_SIGNAL
               ? LAGRANGE_INVALID_CONFIG
               : LAGRANGE_OK;
}

static lagrange_status step(void* p, const lagrange_step_context* context, lagrange_bus* bus) {
    probe* self = (probe*)p;
    uint32_t count = 0;
    const double* pose = self->host->bus_read_n(bus, self->pose, 0, &count);
    (void)context;
    self->host->bus_write(bus, self->x, 0, (count == 7 ? pose[0] : 0.0) + self->offset);
    return LAGRANGE_OK;
}

static lagrange_status init(void* p, const lagrange_world* world, const lagrange_config* config,
                            lagrange_bus* bus) {
    probe* self = (probe*)p;
    (void)world;
    (void)config;
    self->host->bus_write(bus, self->x, 0, self->offset);
    return LAGRANGE_OK;
}

static lagrange_status save_state(const void* p, uint8_t* buffer, size_t capacity,
                                  size_t* written) {
    (void)p;
    (void)buffer;
    (void)capacity;
    *written = 0; /* no state of its own */
    return LAGRANGE_OK;
}

static lagrange_status restore_state(void* p, const uint8_t* buffer, size_t size) {
    (void)p;
    (void)buffer;
    return size == 0 ? LAGRANGE_OK : LAGRANGE_INTERNAL;
}

static const lagrange_module_api kModule = {
    sizeof(lagrange_module_api),
    "acme_probe",
    "probe",
    1,
    0,
    0,        /* not a substitute */
    1,        /* deterministic */
    0,        /* Tier 0 */
    10000000, /* 10 ms */
    create,
    destroy,
    declare,
    init,
    step,
    NULL,
    save_state,
    restore_state,
    NULL,
    NULL,
};

LAGRANGE_EXPORT const lagrange_module_api* lagrange_plugin_get_module(uint32_t host_abi_major,
                                                                      uint32_t host_abi_minor,
                                                                      uint32_t index) {
    (void)host_abi_minor;
    return host_abi_major == LAGRANGE_PLUGIN_ABI_MAJOR && index == 0 ? &kModule : NULL;
}
