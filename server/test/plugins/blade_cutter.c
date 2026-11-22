/* A test plugin with a domain of its own (RFC 0008): blade_cutter/cutter writes
 * rotor/rotor_1_joint/damage (rotor_aero.md §10), zero until the profile value "at" (s, default
 * 3) and then "damage" (default 0.1), as a branch strike would leave it. */
#include "lagrange_plugin.h"

#include <stdlib.h>
#include <string.h>

typedef struct cutter {
    const lagrange_host_api* host;
    uint32_t damage;
    double value;
    double at; /* s */
} cutter;

static const lagrange_unit kOne = {0, 0, 0, 0, 0, 0, 0, 0, 1.0, 24 /* Ratio */};

static void* create(const lagrange_host_api* host) {
    cutter* self = (cutter*)calloc(1, sizeof(cutter));
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
    cutter* self = (cutter*)p;
    lagrange_signal_spec spec;
    (void)world;
    self->value = 0.1;
    self->at = 3.0;
    (void)self->host->config_get_real(config, "damage", &self->value);
    (void)self->host->config_get_real(config, "at", &self->at);
    memset(&spec, 0, sizeof spec);
    spec.struct_size = sizeof spec;
    spec.name = "rotor/rotor_1_joint/damage";
    spec.type = LAGRANGE_SCALAR;
    spec.unit = kOne;
    self->damage = self->host->declare_output(declarer, &spec);
    return self->damage == LAGRANGE_INVALID_SIGNAL ? LAGRANGE_INVALID_CONFIG : LAGRANGE_OK;
}

static lagrange_status step(void* p, const lagrange_step_context* context, lagrange_bus* bus) {
    cutter* self = (cutter*)p;
    const double t = (double)context->t_ns * 1e-9;
    self->host->bus_write(bus, self->damage, 0, t >= self->at ? self->value : 0.0);
    return LAGRANGE_OK;
}

static lagrange_status init(void* p, const lagrange_world* world, const lagrange_config* config,
                            lagrange_bus* bus) {
    cutter* self = (cutter*)p;
    (void)world;
    (void)config;
    self->host->bus_write(bus, self->damage, 0, 0.0);
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
    "blade_cutter",
    "cutter",
    1,
    0,
    0,       /* not a substitute */
    1,       /* deterministic */
    0,       /* Tier 0 */
    1000000, /* 1 ms */
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
