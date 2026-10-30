/* A test plugin for ABI 1.1 (RFC 0008 §2): it counts what the world getters report, and casts
 * rays at the scene around the robot's base:
 *   probe/world/{bodies,colliders,sensors,lidars,actuated}  counts (1)
 *   probe/site/gravity_z                                     m/s²
 *   probe/ray/ground       distance down from 5 m above a point 3 m beside the base (m)
 *   probe/ray/robot        distance from 5 m ahead of the base back at it (m)
 *   probe/ray/robot_body   the body that ray hit (index, 1)
 *   probe/rays/hits        hits of a batch of four rays (down, back at the robot, up, sideways)
 *   probe/overlap          colliders within 5 cm of the base's origin (1) */
#include "lagrange_plugin.h"

#include <stdlib.h>
#include <string.h>

enum {
    BODIES,
    COLLIDERS,
    SENSORS,
    LIDARS,
    ACTUATED,
    GRAVITY,
    GROUND,
    ROBOT,
    ROBOT_BODY,
    RAYS,
    OVERLAP,
    OUTPUTS
};
static const char* const kNames[OUTPUTS] = {
    "probe/world/bodies",   "probe/world/colliders", "probe/world/sensors", "probe/world/lidars",
    "probe/world/actuated", "probe/site/gravity_z",  "probe/ray/ground",    "probe/ray/robot",
    "probe/ray/robot_body", "probe/rays/hits",       "probe/overlap"};

typedef struct probe {
    const lagrange_host_api* host;
    uint32_t pose;
    uint32_t out[OUTPUTS];
    double counts[GRAVITY + 1];
} probe;

static const lagrange_unit kOne = {0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0};
static const lagrange_unit kMetre = {1, 0, 0, 0, 0, 0, 0, 0, 1.0, 14 /* Position */};
static const lagrange_unit kAcceleration = {1, 0, -2, 0, 0, 0, 0, 0, 1.0, 0};

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
    const lagrange_host_api* h = self->host;
    lagrange_signal_spec spec;
    lagrange_sensor_info sensor;
    lagrange_joint_info joint;
    lagrange_site_info site;
    uint32_t i;
    (void)config;
    if (h->struct_size < offsetof(lagrange_host_api, scene_overlap_sphere) + sizeof(void*)) {
        return LAGRANGE_UNSUPPORTED; /* a 1.0 host */
    }
    self->counts[BODIES] = h->world_body_count(world);
    self->counts[COLLIDERS] = h->world_collider_count(world);
    self->counts[SENSORS] = h->world_sensor_count(world);
    for (i = 0; i < h->world_sensor_count(world); ++i) {
        memset(&sensor, 0, sizeof sensor);
        sensor.struct_size = sizeof sensor;
        if (h->world_sensor(world, i, &sensor) && strcmp(sensor.kind, "lidar") == 0) {
            self->counts[LIDARS] += 1.0;
        }
    }
    for (i = 0; i < h->world_joint_count(world); ++i) {
        memset(&joint, 0, sizeof joint);
        joint.struct_size = sizeof joint;
        if (h->world_joint(world, i, &joint) && joint.actuated) {
            self->counts[ACTUATED] += 1.0;
        }
    }
    memset(&site, 0, sizeof site);
    site.struct_size = sizeof site;
    (void)h->world_site(world, &site);
    self->counts[GRAVITY] = site.gravity[2];

    memset(&spec, 0, sizeof spec);
    spec.struct_size = sizeof spec;
    spec.name = "body/base_link/pose";
    spec.type = LAGRANGE_POSE;
    spec.unit = kMetre;
    spec.frame = "world_local";
    self->pose = h->declare_input(declarer, &spec);
    spec.type = LAGRANGE_SCALAR;
    spec.frame = NULL;
    for (i = 0; i < OUTPUTS; ++i) {
        spec.name = kNames[i];
        spec.unit = i == GRAVITY ? kAcceleration : (i == GROUND || i == ROBOT) ? kMetre : kOne;
        self->out[i] = h->declare_output(declarer, &spec);
    }
    return LAGRANGE_OK;
}

static lagrange_status init(void* p, const lagrange_world* world, const lagrange_config* config,
                            lagrange_bus* bus) {
    probe* self = (probe*)p;
    uint32_t i;
    (void)world;
    (void)config;
    for (i = 0; i < OUTPUTS; ++i) {
        self->host->bus_write(bus, self->out[i], 0, i <= GRAVITY ? self->counts[i] : 0.0);
    }
    return LAGRANGE_OK;
}

static lagrange_status step(void* p, const lagrange_step_context* context, lagrange_bus* bus) {
    probe* self = (probe*)p;
    const lagrange_host_api* h = self->host;
    uint32_t count = 0;
    const double* pose = h->bus_read_n(bus, self->pose, 0, &count);
    lagrange_ray_hit hit;
    lagrange_ray_hit batch[4];
    uint32_t colliders[16];
    double origin[3];
    double origins[12];
    double dirs[12];
    const double down[3] = {0.0, 0.0, -1.0};
    const double back[3] = {-1.0, 0.0, 0.0};
    (void)context;
    if (count != 7) {
        return LAGRANGE_OK;
    }
    origin[0] = pose[0] + 3.0;
    origin[1] = pose[1] + 3.0;
    origin[2] = 5.0;
    memset(&hit, 0, sizeof hit);
    hit.struct_size = sizeof hit;
    (void)h->scene_ray(bus, origin, down, 100.0, &hit);
    h->bus_write(bus, self->out[GROUND], 0, hit.distance);
    origin[0] = pose[0] + 5.0;
    origin[1] = pose[1];
    origin[2] = pose[2];
    (void)h->scene_ray(bus, origin, back, 100.0, &hit);
    h->bus_write(bus, self->out[ROBOT], 0, hit.distance);
    h->bus_write(bus, self->out[ROBOT_BODY], 0,
                 hit.body == LAGRANGE_WORLD_BODY ? -1.0 : (double)hit.body);

    memcpy(origins + 0, (double[3]){pose[0] + 3.0, pose[1] + 3.0, 5.0}, sizeof(double[3]));
    memcpy(dirs + 0, down, sizeof down);
    memcpy(origins + 3, origin, sizeof origin);
    memcpy(dirs + 3, back, sizeof back);
    memcpy(origins + 6, (double[3]){pose[0] + 3.0, pose[1] + 3.0, 5.0}, sizeof(double[3]));
    memcpy(dirs + 6, (double[3]){0.0, 0.0, 1.0}, sizeof(double[3])); /* up: nothing */
    memcpy(origins + 9, (double[3]){pose[0] + 3.0, pose[1] + 3.0, 5.0}, sizeof(double[3]));
    memcpy(dirs + 9, (double[3]){0.0, 1.0, 0.0}, sizeof(double[3]));
    batch[0].struct_size = sizeof batch[0];
    h->bus_write(bus, self->out[RAYS], 0,
                 (double)h->scene_rays(bus, 4, origins, dirs, 100.0, batch));

    h->bus_write(bus, self->out[OVERLAP], 0,
                 (double)h->scene_overlap_sphere(bus, pose, 0.05, colliders, 16));
    return LAGRANGE_OK;
}

static const lagrange_module_api kModule = {
    sizeof(lagrange_module_api),
    "scene_probe",
    "probe",
    1,
    1,
    0,
    1,
    0,
    10000000,
    create,
    destroy,
    declare,
    init,
    step,
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
    return host_abi_major == LAGRANGE_PLUGIN_ABI_MAJOR && index == 0 ? &kModule : NULL;
}
