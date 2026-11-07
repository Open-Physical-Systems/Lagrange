/* lagrange_tof: a time-of-flight range sensor as a plugin in C99 (RFC 0008, the worked example of
 * docs/guide/writing-a-plugin.md). It also implements kind "ultrasonic": a wide cone (fov 0.5 rad
 * unless given) and a dead zone (range/min 0.2 m unless given), inside which it reads nothing, as a
 * sonar's own ringing hides what is close. It implements every custom sensor of kind "tof", as a
 * URDF gives one:
 *
 *     <gazebo reference="base_link">
 *       <sensor name="front_tof" type="tof">
 *         <pose>0.25 0 0.1 0 0.3 0</pose>
 *         <update_rate>50</update_rate>
 *         <tof><range><min>0.02</min><max>4</max></range><fov>0.44</fov>
 *              <noise><stddev>0.005</stddev></noise></tof>
 *       </sensor>
 *     </gazebo>
 *
 * Each sample casts rays along the sensor's +x (one ray, or a 5 x 5 grid over a square field of
 * view `fov`, rad) against every collider (ABI 1.1 scene_ray), skips its own body, and writes the
 * nearest distance as sensor/<name>/range (m): +inf when nothing is within range/max, Gaussian
 * noise of noise/stddev (m) drawn from the step's seed, so the run stays deterministic.
 * sensor/<name>/valid is 1 when something is in range. */
#include "lagrange_plugin.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_SENSORS 32
#define GRID 5

typedef struct tof {
    uint32_t body;   /* its body's index */
    double mount[7]; /* in the body */
    double rest[7];  /* the body's rest pose, for a body that never moves */
    double min_range, max_range, fov, noise;
    uint32_t every;        /* sample every `every` steps of the module */
    uint32_t pose;         /* input body/<b>/pose */
    uint32_t range, valid; /* outputs */
    double last_range;
} tof;

typedef struct plugin {
    const lagrange_host_api* host;
    uint32_t count;
    tof sensors[MAX_SENSORS];
    char error[160];
} plugin;

static const lagrange_unit kMetre = {1, 0, 0, 0, 0, 0, 0, 0, 1.0, 14 /* Position */};
static const lagrange_unit kOne = {0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0};
#define TOF_STEP_NS 5000000 /* 5 ms: rates up to 200 Hz */

static void* create(const lagrange_host_api* host) {
    plugin* self = (plugin*)calloc(1, sizeof(plugin));
    if (self != NULL) {
        self->host = host;
    }
    return self;
}

static void destroy(void* self) {
    free(self);
}

/* q * v for a unit quaternion (w, x, y, z). */
static void rotate(const double q[4], const double v[3], double out[3]) {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    const double tx = 2.0 * (y * v[2] - z * v[1]);
    const double ty = 2.0 * (z * v[0] - x * v[2]);
    const double tz = 2.0 * (x * v[1] - y * v[0]);
    out[0] = v[0] + w * tx + (y * tz - z * ty);
    out[1] = v[1] + w * ty + (z * tx - x * tz);
    out[2] = v[2] + w * tz + (x * ty - y * tx);
}

/* a then b: pose b given in a's frame, as a pose in a's parent frame. */
static void compose(const double a[7], const double b[7], double out[7]) {
    double moved[3];
    const double* p = a + 3;
    const double* q = b + 3;
    rotate(p, b, moved);
    out[0] = a[0] + moved[0];
    out[1] = a[1] + moved[1];
    out[2] = a[2] + moved[2];
    out[3] = p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3];
    out[4] = p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2];
    out[5] = p[0] * q[2] - p[1] * q[3] + p[2] * q[0] + p[3] * q[1];
    out[6] = p[0] * q[3] + p[1] * q[2] - p[2] * q[1] + p[3] * q[0];
}

/* A uniform number in (0, 1) from a counter (SplitMix64): the same for the same inputs. */
static double uniform(uint64_t key) {
    uint64_t z = key + 0x9E3779B97F4A7C15ULL;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    z ^= z >> 31;
    return ((double)(z >> 11) + 0.5) / 9007199254740992.0; /* 2^53 */
}

static double param(const lagrange_host_api* h, const lagrange_world* w, uint32_t s,
                    const char* key, double fallback) {
    double value = fallback;
    return h->world_sensor_param_real(w, s, key, &value) ? value : fallback;
}

static lagrange_status declare(void* p, const lagrange_world* world, const lagrange_config* config,
                               lagrange_declarer* declarer) {
    plugin* self = (plugin*)p;
    const lagrange_host_api* h = self->host;
    uint32_t s;
    (void)config;
    self->count = 0;
    if (h->struct_size < offsetof(lagrange_host_api, scene_overlap_sphere) + sizeof(void*)) {
        snprintf(self->error, sizeof self->error, "needs a host with plugin ABI 1.1");
        return LAGRANGE_UNSUPPORTED;
    }
    for (s = 0; s < h->world_sensor_count(world); ++s) {
        lagrange_sensor_info info;
        lagrange_body_info body;
        lagrange_signal_spec spec;
        char name[200];
        tof* t;
        memset(&info, 0, sizeof info);
        info.struct_size = sizeof info;
        int ultrasonic;
        if (!h->world_sensor(world, s, &info)) {
            continue;
        }
        ultrasonic = strcmp(info.kind, "ultrasonic") == 0;
        if (!ultrasonic && strcmp(info.kind, "tof") != 0) {
            continue;
        }
        if (self->count == MAX_SENSORS) {
            snprintf(self->error, sizeof self->error, "more than %d tof sensors", MAX_SENSORS);
            return LAGRANGE_INVALID_CONFIG;
        }
        t = &self->sensors[self->count++];
        memset(&body, 0, sizeof body);
        body.struct_size = sizeof body;
        (void)h->world_body(world, info.body, &body);
        t->body = info.body;
        memcpy(t->mount, info.pose_in_body, sizeof t->mount);
        memcpy(t->rest, body.rest_pose, sizeof t->rest);
        t->min_range = param(h, world, s, "range/min", ultrasonic ? 0.2 : 0.02);
        t->max_range = param(h, world, s, "range/max", 4.0);
        t->fov = param(h, world, s, "fov", ultrasonic ? 0.5 : 0.0);
        t->noise = param(h, world, s, "noise/stddev", 0.0);
        t->every =
            info.rate > 0.0 ? (uint32_t)floor(1e9 / (info.rate * (double)TOF_STEP_NS) + 0.5) : 1;
        if (t->every == 0) {
            t->every = 1;
        }
        memset(&spec, 0, sizeof spec);
        spec.struct_size = sizeof spec;
        spec.name = name;
        /* Where its body is; a body that never moves has no pose signal: its rest pose. */
        snprintf(name, sizeof name, "body/%s/pose", body.name);
        spec.type = LAGRANGE_POSE;
        spec.unit = kMetre;
        spec.frame = "world_local";
        spec.optional = 1;
        t->pose = h->declare_input(declarer, &spec);
        spec.type = LAGRANGE_SCALAR;
        spec.frame = NULL;
        spec.optional = 0;
        snprintf(name, sizeof name, "sensor/%s/range", info.name);
        t->range = h->declare_output(declarer, &spec);
        spec.unit = kOne;
        snprintf(name, sizeof name, "sensor/%s/valid", info.name);
        t->valid = h->declare_output(declarer, &spec);
        if (t->pose == LAGRANGE_INVALID_SIGNAL || t->range == LAGRANGE_INVALID_SIGNAL ||
            t->valid == LAGRANGE_INVALID_SIGNAL) {
            snprintf(self->error, sizeof self->error, "tof %s: invalid signal names", info.name);
            return LAGRANGE_INVALID_CONFIG;
        }
    }
    return LAGRANGE_OK;
}

static lagrange_status init(void* p, const lagrange_world* world, const lagrange_config* config,
                            lagrange_bus* bus) {
    plugin* self = (plugin*)p;
    uint32_t i;
    (void)world;
    (void)config;
    for (i = 0; i < self->count; ++i) {
        self->sensors[i].last_range = INFINITY;
        self->host->bus_write(bus, self->sensors[i].range, 0, INFINITY);
        self->host->bus_write(bus, self->sensors[i].valid, 0, 0.0);
    }
    return LAGRANGE_OK;
}

/* Distance along `dir` from `origin` to the first collider not on `own` body, or +inf. */
static double cast(const lagrange_host_api* h, const lagrange_bus* bus, const double origin[3],
                   const double dir[3], double max_range, uint32_t own) {
    double travelled = 0.0;
    double from[3];
    int hops;
    memcpy(from, origin, sizeof from);
    for (hops = 0; hops < 8; ++hops) {
        lagrange_ray_hit hit;
        memset(&hit, 0, sizeof hit);
        hit.struct_size = sizeof hit;
        if (!h->scene_ray(bus, from, dir, max_range - travelled, &hit)) {
            return INFINITY;
        }
        travelled += hit.distance;
        if (hit.body != own) {
            return travelled;
        }
        /* Its own body (the sensor sits on or in it): go on from just past where it left it. */
        travelled += 1e-6;
        from[0] = origin[0] + dir[0] * travelled;
        from[1] = origin[1] + dir[1] * travelled;
        from[2] = origin[2] + dir[2] * travelled;
    }
    return INFINITY;
}

static lagrange_status step(void* p, const lagrange_step_context* context, lagrange_bus* bus) {
    plugin* self = (plugin*)p;
    const lagrange_host_api* h = self->host;
    uint32_t i;
    for (i = 0; i < self->count; ++i) {
        tof* t = &self->sensors[i];
        uint32_t n = 0;
        const double* moving = h->bus_read_n(bus, t->pose, 0, &n);
        double body[7];
        double sensor[7];
        double nearest = INFINITY;
        int a, b;
        const int grid = t->fov > 0.0 ? GRID : 1;
        if (context->step_index % t->every != 0) {
            continue;
        }
        if (n == 7 && moving[3] * moving[3] + moving[4] * moving[4] + moving[5] * moving[5] +
                              moving[6] * moving[6] >
                          0.5) {
            memcpy(body, moving, sizeof body);
        } else {
            memcpy(body, t->rest, sizeof body); /* a body that never moves */
        }
        compose(body, t->mount, sensor);
        for (a = 0; a < grid; ++a) {
            for (b = 0; b < grid; ++b) {
                /* Square field of view: yaw and pitch from -fov/2 to fov/2. */
                const double u = grid == 1 ? 0.0 : t->fov * ((double)a / (grid - 1) - 0.5);
                const double v = grid == 1 ? 0.0 : t->fov * ((double)b / (grid - 1) - 0.5);
                double local[3];
                double dir[3];
                double length;
                double d;
                local[0] = 1.0;
                local[1] = tan(u);
                local[2] = tan(v);
                length = sqrt(local[0] * local[0] + local[1] * local[1] + local[2] * local[2]);
                local[0] /= length;
                local[1] /= length;
                local[2] /= length;
                rotate(sensor + 3, local, dir);
                d = cast(h, bus, sensor, dir, t->max_range, t->body);
                nearest = d < nearest ? d : nearest;
            }
        }
        if (nearest < t->min_range || nearest > t->max_range) {
            nearest = INFINITY;
        } else if (t->noise > 0.0) {
            /* Box-Muller from two counter-based uniforms: (seed, step, sensor). */
            const uint64_t key =
                context->seed ^ (context->step_index * 0x2545F4914F6CDD1DULL) ^ ((uint64_t)i << 48);
            const double u1 = uniform(key);
            const double u2 = uniform(key ^ 0xD1B54A32D192ED03ULL);
            nearest += t->noise * sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2);
            nearest = nearest < t->min_range ? t->min_range : nearest;
        }
        t->last_range = nearest;
        h->bus_write(bus, t->range, 0, nearest);
        h->bus_write(bus, t->valid, 0, isinf(nearest) ? 0.0 : 1.0);
    }
    return LAGRANGE_OK;
}

static lagrange_status save_state(const void* p, uint8_t* buffer, size_t capacity,
                                  size_t* written) {
    (void)p;
    (void)buffer;
    (void)capacity;
    *written = 0; /* each sample depends only on the scene and the step: no state */
    return LAGRANGE_OK;
}

static lagrange_status restore_state(void* p, const uint8_t* buffer, size_t size) {
    (void)p;
    (void)buffer;
    return size == 0 ? LAGRANGE_OK : LAGRANGE_INTERNAL;
}

static const char* last_error(const void* p) {
    const plugin* self = (const plugin*)p;
    return self->error[0] != '\0' ? self->error : NULL;
}

static const lagrange_module_api kModule = {
    sizeof(lagrange_module_api),
    "tof",
    "lagrange_tof",
    1,
    0,
    0, /* not a substitute: its own domain */
    1, /* deterministic */
    0, /* Tier 0 */
    TOF_STEP_NS,
    create,
    destroy,
    declare,
    init,
    step,
    NULL,
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
