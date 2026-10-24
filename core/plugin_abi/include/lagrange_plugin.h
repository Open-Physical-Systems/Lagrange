/* core/plugin_abi/include/lagrange_plugin.h
 * SPDX-License-Identifier: Apache-2.0
 *
 * The C ABI for simulation modules built outside the project (invariant 10, RFC 0001 "C ABI",
 * docs/design/architecture.md §13). C99. No exceptions and no C++ types cross this boundary.
 *
 * A plugin is a shared object that exports lagrange_plugin_get_module(). The host
 * (lagrange::PluginLibrary) wraps each module it returns in an IModule, so the scheduler wires,
 * checks units and steps it like any built-in module.
 *
 * Versioning: a host loads only plugins built for the same MAJOR. MINOR grows when fields are
 * appended to structs or functions to tables; every struct starts with struct_size so that
 * either side can tell an older, shorter struct. The ABI is MAJOR 0, unstable, until product
 * v1.0. */
#ifndef LAGRANGE_PLUGIN_H
#define LAGRANGE_PLUGIN_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LAGRANGE_PLUGIN_ABI_MAJOR 0u
#define LAGRANGE_PLUGIN_ABI_MINOR 1u

#define LAGRANGE_EXPORT __attribute__((visibility("default")))

typedef enum lagrange_status {
    LAGRANGE_OK = 0,
    LAGRANGE_INVALID_CONFIG = 1,
    LAGRANGE_UNIT_MISMATCH = 2,
    LAGRANGE_MISSING_INPUT = 3,
    LAGRANGE_DIVERGED = 4,
    LAGRANGE_UNSUPPORTED = 5,
    LAGRANGE_INTERNAL = 6
} lagrange_status;

/* Same values as lagrange::SignalType. */
typedef enum lagrange_signal_type {
    LAGRANGE_SCALAR = 0,
    LAGRANGE_VEC3 = 1,
    LAGRANGE_QUAT = 2,
    LAGRANGE_POSE = 3,
    LAGRANGE_WRENCH = 4,
    LAGRANGE_SCALAR_ARRAY = 5,
    LAGRANGE_BOOL = 6,
    LAGRANGE_INT = 7,
    LAGRANGE_BYTES = 8,
    LAGRANGE_FIELD3D = 9,
    LAGRANGE_IMAGE = 10,
    LAGRANGE_POINT_CLOUD = 11
} lagrange_signal_type;

/* A unit: SI base dimensions, a scale to the coherent SI unit (1.0 for SI) and the quantity
 * kind (a lagrange::QuantityKind value, 0 unspecified), which tells torque from energy. */
typedef struct lagrange_unit {
    int8_t length, mass, time, current, temperature, amount, luminous, angle;
    double scale;
    uint16_t kind;
} lagrange_unit;

typedef struct lagrange_signal_spec {
    uint32_t struct_size;
    const char* name; /* "joint/left_wheel/torque"; UTF-8; copied by the host */
    lagrange_signal_type type;
    lagrange_unit unit;
    uint32_t array_length; /* for LAGRANGE_SCALAR_ARRAY; 0 (a zero-filled spec) means 1 */
    const char* frame;     /* "world_local", "body:<name>" or NULL */
    uint8_t hold;          /* inputs: 0 zero-order, 1 linear extrapolate, 2 window average */
    uint8_t optional;      /* inputs: may stay unconnected; reads give default_value */
    uint8_t feedback;      /* inputs: read the writer's previous step (breaks a loop) */
    double default_value;
} lagrange_signal_spec;

typedef struct lagrange_step_context {
    uint32_t struct_size;
    int64_t t_ns;        /* time at the start of this step */
    int64_t dt_ns;       /* this module's step */
    uint64_t step_index; /* 0, 1, 2 ... */
    uint64_t seed;       /* per (run, module) seed for deterministic noise */
} lagrange_step_context;

/* Opaque host objects, valid only during the call they are passed to. */
typedef struct lagrange_world lagrange_world;
typedef struct lagrange_config lagrange_config;
typedef struct lagrange_bus lagrange_bus;
typedef struct lagrange_declarer lagrange_declarer;

/* Returned by declare_input/declare_output for an invalid declaration. */
#define LAGRANGE_INVALID_SIGNAL 0xFFFFFFFFu

/* Functions the host gives to the plugin (in create()). */
typedef struct lagrange_host_api {
    uint32_t struct_size;
    uint32_t abi_major, abi_minor;

    /* declare() only. The id is what step() passes to bus_read and bus_write. */
    uint32_t (*declare_input)(lagrange_declarer*, const lagrange_signal_spec*);
    uint32_t (*declare_output)(lagrange_declarer*, const lagrange_signal_spec*);

    /* init() and step(). env is 0 unless the run is batched. */
    double (*bus_read)(const lagrange_bus*, uint32_t id, uint32_t env);
    void (*bus_write)(lagrange_bus*, uint32_t id, uint32_t env, double value);
    const double* (*bus_read_n)(const lagrange_bus*, uint32_t id, uint32_t env, uint32_t* count);
    void (*bus_write_n)(lagrange_bus*, uint32_t id, uint32_t env, const double* values,
                        uint32_t count);

    /* declare() and init(). */
    uint32_t (*world_joint_count)(const lagrange_world*);
    const char* (*world_joint_name)(const lagrange_world*, uint32_t joint);
    /* Custom schemas applied in the stage ("ElectricalMotorAPI_1"): how many prims carry one,
     * a real attribute of the n-th ("elec:torqueConstant"), and that prim's name. */
    uint32_t (*world_schema_instance_count)(const lagrange_world*, const char* schema);
    int (*world_schema_real)(const lagrange_world*, const char* schema, uint32_t instance,
                             const char* attr, double* out); /* 1 if present */
    const char* (*world_schema_instance_name)(const lagrange_world*, const char* schema,
                                              uint32_t instance);

    /* The domain's profile values; 1 if present with that type. */
    int (*config_get_real)(const lagrange_config*, const char* key, double* out);
    int (*config_get_string)(const lagrange_config*, const char* key, const char** out);

    void (*log)(int level, const char* message); /* 0 debug, 1 info, 2 warning, 3 error */
} lagrange_host_api;

/* Functions the plugin gives to the host. `self` is what create() returned. */
typedef struct lagrange_module_api {
    uint32_t struct_size;
    const char* domain; /* "electrical": the key in the profile's domains */
    const char* name;   /* "acme_bldc_drive", or for a substitute "ideal_torque_source" */
    uint32_t version_major, version_minor;
    uint8_t is_substitute;
    uint8_t deterministic; /* 1: bit-reproducible; save_state and restore_state required */
    uint8_t minimum_tier;  /* 0 CPU, 1 raster GPU, 2 full */
    int64_t preferred_step_ns;

    void* (*create)(const lagrange_host_api*);
    void (*destroy)(void* self);
    lagrange_status (*declare)(void* self, const lagrange_world*, const lagrange_config*,
                               lagrange_declarer*);
    lagrange_status (*init)(void* self, const lagrange_world*, const lagrange_config*,
                            lagrange_bus*);
    lagrange_status (*step)(void* self, const lagrange_step_context*, lagrange_bus*);
    void (*shutdown)(void* self);
    /* Optional (NULL when unsupported). Size query: call with buffer == NULL. */
    lagrange_status (*save_state)(const void* self, uint8_t* buffer, size_t capacity,
                                  size_t* written);
    lagrange_status (*restore_state)(void* self, const uint8_t* buffer, size_t size);
    /* Optional: the reason for the last non-OK status, or NULL. */
    const char* (*last_error)(const void* self);
} lagrange_module_api;

/* The single symbol a plugin exports. Returns NULL when it does not support host_abi_major, and
 * for the index after its last module: index 0, 1, ... enumerates the modules it holds. */
LAGRANGE_EXPORT const lagrange_module_api* lagrange_plugin_get_module(uint32_t host_abi_major,
                                                                      uint32_t host_abi_minor,
                                                                      uint32_t index);

#ifdef __cplusplus
}
#endif
#endif /* LAGRANGE_PLUGIN_H */
