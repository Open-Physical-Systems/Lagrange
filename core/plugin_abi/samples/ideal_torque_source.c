/* A plugin in C99 against lagrange_plugin.h: the ideal torque source of CLAUDE.md §11.5, the
 * electrical domain's substitute. Each joint's commanded torque reaches the joint unchanged:
 *
 *     joint/<j>/torque_command (N·m)  ->  joint/<j>/torque (N·m)
 *
 * The joints are the profile value "joint" (one name), or every joint of the world. Built with
 * -std=c99 -pedantic -Werror (core/CMakeLists.txt) to keep the header plain C. */
#include "lagrange_plugin.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_JOINTS 64

/* The ABI major this build accepts; the tests build it once more for another major. */
#ifndef SAMPLE_ABI_MAJOR
#define SAMPLE_ABI_MAJOR LAGRANGE_PLUGIN_ABI_MAJOR
#endif

typedef struct source {
    const lagrange_host_api* host;
    uint32_t joints;
    uint32_t command[MAX_JOINTS];
    uint32_t torque[MAX_JOINTS];
    uint64_t steps; /* the whole dynamic state: saved and restored */
    char error[128];
} source;

static const lagrange_unit kNewtonMetre = {2, 1, -2, 0, 0, 0, 0, 0, 1.0, 1 /* Torque */};

static void* create(const lagrange_host_api* host) {
    source* self = (source*)calloc(1, sizeof(source));
    if (self != NULL) {
        self->host = host;
    }
    return self;
}

static void destroy(void* self) {
    free(self);
}

static lagrange_status add_joint(source* self, const char* joint, lagrange_declarer* declarer) {
    char name[160];
    lagrange_signal_spec spec;
    if (self->joints == MAX_JOINTS) {
        snprintf(self->error, sizeof self->error, "more than %d joints", MAX_JOINTS);
        return LAGRANGE_INVALID_CONFIG;
    }
    memset(&spec, 0, sizeof spec);
    spec.struct_size = sizeof spec;
    spec.type = LAGRANGE_SCALAR;
    spec.unit = kNewtonMetre;
    spec.name = name;
    snprintf(name, sizeof name, "joint/%s/torque_command", joint);
    self->command[self->joints] = self->host->declare_input(declarer, &spec);
    snprintf(name, sizeof name, "joint/%s/torque", joint);
    self->torque[self->joints] = self->host->declare_output(declarer, &spec);
    if (self->command[self->joints] == LAGRANGE_INVALID_SIGNAL ||
        self->torque[self->joints] == LAGRANGE_INVALID_SIGNAL) {
        snprintf(self->error, sizeof self->error, "invalid joint name '%s'", joint);
        return LAGRANGE_INVALID_CONFIG;
    }
    ++self->joints;
    return LAGRANGE_OK;
}

static lagrange_status declare(void* p, const lagrange_world* world, const lagrange_config* config,
                               lagrange_declarer* declarer) {
    source* self = (source*)p;
    const char* joint = NULL;
    uint32_t j;
    self->joints = 0;
    if (self->host->config_get_string(config, "joint", &joint)) {
        return add_joint(self, joint, declarer);
    }
    for (j = 0; j < self->host->world_joint_count(world); ++j) {
        lagrange_status status = add_joint(self, self->host->world_joint_name(world, j), declarer);
        if (status != LAGRANGE_OK) {
            return status;
        }
    }
    return LAGRANGE_OK;
}

static lagrange_status init(void* p, const lagrange_world* world, const lagrange_config* config,
                            lagrange_bus* bus) {
    source* self = (source*)p;
    uint32_t j;
    (void)world;
    (void)config;
    for (j = 0; j < self->joints; ++j) {
        self->host->bus_write(bus, self->torque[j], 0, 0.0);
    }
    self->steps = 0;
    return LAGRANGE_OK;
}

static lagrange_status step(void* p, const lagrange_step_context* context, lagrange_bus* bus) {
    source* self = (source*)p;
    uint32_t j;
    (void)context;
    for (j = 0; j < self->joints; ++j) {
        self->host->bus_write(bus, self->torque[j], 0,
                              self->host->bus_read(bus, self->command[j], 0));
    }
    ++self->steps;
    return LAGRANGE_OK;
}

static lagrange_status save_state(const void* p, uint8_t* buffer, size_t capacity,
                                  size_t* written) {
    const source* self = (const source*)p;
    *written = sizeof self->steps;
    if (buffer == NULL) {
        return LAGRANGE_OK; /* size query */
    }
    if (capacity < sizeof self->steps) {
        return LAGRANGE_INTERNAL;
    }
    memcpy(buffer, &self->steps, sizeof self->steps);
    return LAGRANGE_OK;
}

static lagrange_status restore_state(void* p, const uint8_t* buffer, size_t size) {
    source* self = (source*)p;
    if (size != sizeof self->steps) {
        snprintf(self->error, sizeof self->error, "state of %u bytes, expected %u", (unsigned)size,
                 (unsigned)sizeof self->steps);
        return LAGRANGE_INTERNAL;
    }
    memcpy(&self->steps, buffer, sizeof self->steps);
    return LAGRANGE_OK;
}

static const char* last_error(const void* p) {
    const source* self = (const source*)p;
    return self->error[0] != '\0' ? self->error : NULL;
}

static const lagrange_module_api kModule = {
    sizeof(lagrange_module_api),
    "electrical",
    "ideal_torque_source",
    0,
    1,
    1,       /* a substitute */
    1,       /* deterministic */
    0,       /* Tier 0 */
    1000000, /* 1 ms */
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

/* A second module: one output per prim carrying TestMotorAPI, named after the prim and set
 * to its elec:torqueConstant. It exercises the schema queries of the host API. */
typedef struct probe {
    const lagrange_host_api* host;
    uint32_t count;
    uint32_t out[MAX_JOINTS];
    double value[MAX_JOINTS];
} probe;

static void* probe_create(const lagrange_host_api* host) {
    probe* self = (probe*)calloc(1, sizeof(probe));
    if (self != NULL) {
        self->host = host;
    }
    return self;
}

static lagrange_status probe_declare(void* p, const lagrange_world* world,
                                     const lagrange_config* config, lagrange_declarer* declarer) {
    probe* self = (probe*)p;
    const char* schema = "TestMotorAPI";
    uint32_t i;
    (void)config;
    self->count = self->host->world_schema_instance_count(world, schema);
    if (self->count > MAX_JOINTS) {
        return LAGRANGE_INVALID_CONFIG;
    }
    for (i = 0; i < self->count; ++i) {
        char name[160];
        lagrange_signal_spec spec;
        memset(&spec, 0, sizeof spec);
        spec.struct_size = sizeof spec;
        spec.type = LAGRANGE_SCALAR;
        spec.unit.scale = 1.0;
        spec.name = name;
        snprintf(name, sizeof name, "test/%s/torque_constant",
                 self->host->world_schema_instance_name(world, schema, i));
        self->out[i] = self->host->declare_output(declarer, &spec);
        if (!self->host->world_schema_real(world, schema, i, "elec:torqueConstant",
                                           &self->value[i])) {
            self->value[i] = -1.0;
        }
    }
    return LAGRANGE_OK;
}

static lagrange_status probe_init(void* p, const lagrange_world* world,
                                  const lagrange_config* config, lagrange_bus* bus) {
    probe* self = (probe*)p;
    uint32_t i;
    (void)world;
    (void)config;
    for (i = 0; i < self->count; ++i) {
        self->host->bus_write(bus, self->out[i], 0, self->value[i]);
    }
    return LAGRANGE_OK;
}

static lagrange_status probe_step(void* p, const lagrange_step_context* context,
                                  lagrange_bus* bus) {
    (void)context;
    return probe_init(p, NULL, NULL, bus);
}

static const lagrange_module_api kProbe = {
    sizeof(lagrange_module_api),
    "test",
    "schema_probe",
    0,
    1,
    0,
    1,
    0,
    1000000,
    probe_create,
    destroy,
    probe_declare,
    probe_init,
    probe_step,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
};

LAGRANGE_EXPORT const lagrange_module_api* lagrange_plugin_get_module(uint32_t host_abi_major,
                                                                      uint32_t host_abi_minor,
                                                                      uint32_t index) {
    (void)host_abi_minor;
    if (host_abi_major != SAMPLE_ABI_MAJOR || index > 1) {
        return NULL;
    }
    return index == 0 ? &kModule : &kProbe;
}
