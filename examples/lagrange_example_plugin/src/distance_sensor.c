/* A Lagrange plugin module (RFC 0008), the starting point of your own: it has a domain of its
 * own, reads the pose of the robot's base, and writes how far the base is from the world
 * origin, my_sensor/distance (m), at 100 Hz. Its profile setting "offset" (m) is added.
 *
 * The lifecycle (docs/guide/writing-a-plugin.md):
 *   create -> declare (inputs and outputs, with units) -> [the host wires and checks units]
 *   -> init (write the first outputs) -> step x N -> shutdown -> destroy
 * step() must not allocate, block or do I/O; it may read the scene (scene_ray, ABI 1.1). */
#include "lagrange_plugin.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct my_sensor {
    const lagrange_host_api* host;
    uint32_t pose;     /* input: body/base_link/pose */
    uint32_t distance; /* output: my_sensor/distance */
    double offset;     /* from the profile: plugins[].config.offset */
    char error[128];
} my_sensor;

/* Units are SI dimensions, a scale and a quantity kind (lagrange::QuantityKind): 14 Position. */
static const lagrange_unit kMetre = {1, 0, 0, 0, 0, 0, 0, 0, 1.0, 14};

static void* create(const lagrange_host_api* host) {
    my_sensor* self = (my_sensor*)calloc(1, sizeof(my_sensor));
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
    my_sensor* self = (my_sensor*)p;
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

    spec.name = "my_sensor/distance";
    spec.type = LAGRANGE_SCALAR;
    spec.frame = NULL;
    self->distance = self->host->declare_output(declarer, &spec);
    if (self->pose == LAGRANGE_INVALID_SIGNAL || self->distance == LAGRANGE_INVALID_SIGNAL) {
        snprintf(self->error, sizeof self->error, "invalid signal declaration");
        return LAGRANGE_INVALID_CONFIG;
    }
    return LAGRANGE_OK;
}

static lagrange_status init(void* p, const lagrange_world* world, const lagrange_config* config,
                            lagrange_bus* bus) {
    my_sensor* self = (my_sensor*)p;
    (void)world;
    (void)config;
    self->host->bus_write(bus, self->distance, 0, self->offset);
    return LAGRANGE_OK;
}

static lagrange_status step(void* p, const lagrange_step_context* context, lagrange_bus* bus) {
    my_sensor* self = (my_sensor*)p;
    uint32_t count = 0;
    const double* pose = self->host->bus_read_n(bus, self->pose, 0, &count);
    (void)context; /* context->seed and step_index give deterministic noise */
    if (count == 7) {
        self->host->bus_write(bus, self->distance, 0,
                              sqrt(pose[0] * pose[0] + pose[1] * pose[1]) + self->offset);
    }
    return LAGRANGE_OK;
}

static lagrange_status save_state(const void* p, uint8_t* buffer, size_t capacity,
                                  size_t* written) {
    (void)p;
    (void)buffer;
    (void)capacity;
    *written = 0; /* no state between steps; a module with state writes it all here */
    return LAGRANGE_OK;
}

static lagrange_status restore_state(void* p, const uint8_t* buffer, size_t size) {
    (void)p;
    (void)buffer;
    return size == 0 ? LAGRANGE_OK : LAGRANGE_INTERNAL;
}

static const char* last_error(const void* p) {
    const my_sensor* self = (const my_sensor*)p;
    return self->error[0] != '\0' ? self->error : NULL;
}

static const lagrange_module_api kModule = {
    sizeof(lagrange_module_api),
    "my_sensor", /* its domain: --domain my_sensor=off turns it off */
    "my_sensor", /* its name */
    0,
    1,
    0,        /* not a substitute */
    1,        /* deterministic */
    0,        /* runs on Tier 0 (CPU) */
    10000000, /* 10 ms */
    create,
    destroy,
    declare,
    init,
    step,
    NULL, /* shutdown */
    save_state,
    restore_state,
    last_error,
    NULL, /* shift_origin: it keeps no positions */
};

LAGRANGE_EXPORT const lagrange_module_api* lagrange_plugin_get_module(uint32_t host_abi_major,
                                                                      uint32_t host_abi_minor,
                                                                      uint32_t index) {
    (void)host_abi_minor;
    return host_abi_major == LAGRANGE_PLUGIN_ABI_MAJOR && index == 0 ? &kModule : NULL;
}
