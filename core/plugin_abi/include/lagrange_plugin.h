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
 * either side can tell an older, shorter struct. MAJOR 1 is frozen for the 1.x releases
 * (RFC 0007): within it the ABI changes only by appending; core/plugin_abi/test/
 * test_abi_layout.c pins every size and offset of 1.0.
 *
 * 1.1 (RFC 0008) appends to lagrange_host_api what a plugin needs to see the scene: the world's
 * bodies, joints, colliders and sensors, and ray and overlap queries. A 1.1 plugin checks the
 * host's struct_size (or abi_minor) before calling them, so it can still run on a 1.0 host. */
#ifndef LAGRANGE_PLUGIN_H
#define LAGRANGE_PLUGIN_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LAGRANGE_PLUGIN_ABI_MAJOR 1u
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

/* ABI 1.1 (RFC 0008): the world, read with the world_* functions in declare() and init(). Each
 * getter fills the struct the plugin passes as far as both sides know it: the plugin sets
 * struct_size, the host fills that many bytes at most. Poses are x, y, z, then the unit
 * quaternion w, x, y, z; lengths in m, masses in kg, angles in rad, in the region-local frame.
 * Strings and arrays stay valid until the run ends. */
#define LAGRANGE_WORLD_BODY 0xFFFFFFFFu /* "no body": the world itself, fixed scenery */

typedef enum lagrange_joint_type {
    LAGRANGE_JOINT_FIXED = 0,
    LAGRANGE_JOINT_REVOLUTE = 1,
    LAGRANGE_JOINT_PRISMATIC = 2,
    LAGRANGE_JOINT_SPHERICAL = 3,
    LAGRANGE_JOINT_FREE = 4,
    LAGRANGE_JOINT_CONTINUOUS = 5
} lagrange_joint_type;

typedef enum lagrange_shape_type {
    LAGRANGE_SHAPE_PLANE = 0,       /* size[0] 0: infinite; else half-extents x, y */
    LAGRANGE_SHAPE_SPHERE = 1,      /* size[0] radius */
    LAGRANGE_SHAPE_CAPSULE = 2,     /* size[0] radius, size[1] half-length (along z) */
    LAGRANGE_SHAPE_CYLINDER = 3,    /* size[0] radius, size[1] half-length (along z) */
    LAGRANGE_SHAPE_BOX = 4,         /* half-extents */
    LAGRANGE_SHAPE_CONVEX_HULL = 5, /* points and triangles */
    LAGRANGE_SHAPE_HEIGHT_FIELD = 6 /* columns x rows heights on a cell grid */
} lagrange_shape_type;

typedef struct lagrange_body_info {
    uint32_t struct_size;
    const char* name;
    uint32_t parent; /* LAGRANGE_WORLD_BODY for a root */
    uint32_t robot;  /* which robot (or scenery) it belongs to */
    double mass;
    double com[3];            /* in the body frame */
    double inertia[6];        /* about the centre of mass, body axes: xx, yy, zz, xy, xz, yz */
    double pose_in_parent[7]; /* with every joint at zero */
    double rest_pose[7];      /* in the world, with every joint at zero */
    uint8_t kinematic;        /* moved by body/<b>/pose_command, not by forces */
} lagrange_body_info;

#define LAGRANGE_NO_MIMIC 0xFFFFFFFFu

typedef struct lagrange_joint_info {
    uint32_t struct_size;
    const char* name;
    lagrange_joint_type type;
    uint32_t parent, child; /* bodies; parent LAGRANGE_WORLD_BODY for a root */
    double axis[3];         /* in the child body's frame */
    double lower, upper;    /* equal: unlimited */
    double effort_limit, velocity_limit, armature, initial_position;
    uint8_t actuated;
    /* Appended after ABI 1.1's first release: check struct_size. */
    double damping, friction; /* N·m·s/rad or N·s/m; N·m or N */
    uint32_t mimic;           /* the joint this one copies; LAGRANGE_NO_MIMIC for none */
    double mimic_multiplier, mimic_offset; /* position = multiplier × mimic's + offset */
} lagrange_joint_info;

typedef struct lagrange_collider_info {
    uint32_t struct_size;
    uint32_t body; /* LAGRANGE_WORLD_BODY for fixed scenery */
    lagrange_shape_type shape;
    double size[3];
    double pose[7]; /* in the body (in the world for fixed scenery) */
    double friction;
    uint32_t point_count, triangle_count; /* a convex hull: x, y, z per point */
    const double* points;
    const uint32_t* triangles; /* three point indices each, outward */
    uint32_t columns, rows;    /* a height field: heights[row * columns + column] */
    double cell[2];
    const double* heights;
} lagrange_collider_info;

typedef struct lagrange_sensor_info {
    uint32_t struct_size;
    const char* name;
    const char* kind; /* "imu", "lidar", "camera", "gnss", "magnetometer", "barometer",
                         "radar", or a custom sensor's kind */
    uint32_t body;
    double pose_in_body[7];
    double rate; /* Hz */
} lagrange_sensor_info;

typedef struct lagrange_site_info {
    uint32_t struct_size;
    double gravity[3];       /* m/s², region-local frame */
    double region_origin[3]; /* world ENU of the region-local frame, m */
} lagrange_site_info;

/* ABI 1.1: what a ray met (scene_ray, scene_rays). */
typedef struct lagrange_ray_hit {
    uint32_t struct_size;
    double distance;   /* m along the unit direction; +inf when nothing is hit */
    uint32_t collider; /* as world_collider; LAGRANGE_INVALID_SIGNAL when nothing is hit */
    uint32_t body;     /* the collider's body, or LAGRANGE_WORLD_BODY */
} lagrange_ray_hit;

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

    /* ABI 1.1 (RFC 0008). The world, in declare() and init(): counts, and getters that return 1
     * and fill `out` for a valid index (0 otherwise). */
    uint32_t (*world_body_count)(const lagrange_world*);
    int (*world_body)(const lagrange_world*, uint32_t body, lagrange_body_info* out);
    int (*world_joint)(const lagrange_world*, uint32_t joint, lagrange_joint_info* out);
    uint32_t (*world_collider_count)(const lagrange_world*);
    int (*world_collider)(const lagrange_world*, uint32_t collider, lagrange_collider_info* out);
    uint32_t (*world_sensor_count)(const lagrange_world*);
    int (*world_sensor)(const lagrange_world*, uint32_t sensor, lagrange_sensor_info* out);
    /* A sensor's parameters by name ("max_range", "range/max"); 1 if present with that type. */
    int (*world_sensor_param_real)(const lagrange_world*, uint32_t sensor, const char* key,
                                   double* out);
    int (*world_sensor_param_string)(const lagrange_world*, uint32_t sensor, const char* key,
                                     const char** out);
    int (*world_site)(const lagrange_world*, lagrange_site_info* out);

    /* The scene, in step(): every collider at the body poses committed on the bus when the call
     * is made (a module that needs moving bodies where physics just put them declares their
     * body/<b>/pose inputs, which orders it after physics). Exact and deterministic: ties go to
     * the lower collider index; safe from several threads during one step. `dir` is a unit
     * vector, region-local frame. */
    int (*scene_ray)(const lagrange_bus*, const double origin[3], const double dir[3],
                     double max_distance, lagrange_ray_hit* out); /* 1 if something is hit */
    /* `count` rays: origins and dirs hold 3 doubles per ray; out holds `count` hits, each with
     * the plugin's struct_size in out[0].struct_size. Returns how many hit. */
    uint32_t (*scene_rays)(const lagrange_bus*, uint32_t count, const double* origins,
                           const double* dirs, double max_distance, lagrange_ray_hit* out);
    /* Colliders within `radius` of `center` (touching counts), in ascending order: writes at
     * most `capacity`, returns how many there are. */
    uint32_t (*scene_overlap_sphere)(const lagrange_bus*, const double center[3], double radius,
                                     uint32_t* colliders, uint32_t capacity);
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
    /* Optional: the floating origin moved by `shift` (m, world ENU, whole metres) between two
     * steps (docs/design/huge-worlds.md section 1.4): positions the module keeps in its state in
     * the region frame move by -shift. Pose signals on the bus are moved by the host. */
    void (*shift_origin)(void* self, const double shift[3]);
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
