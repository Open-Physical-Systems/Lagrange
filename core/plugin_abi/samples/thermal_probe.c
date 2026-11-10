/* lagrange_thermal_probe: temperature sensors as a plugin in C99 (RFC 0008), reading the thermal
 * domain (the lumped nodes, or the conduction fidelity of RFC 0012). Two custom sensor kinds, as a
 * URDF gives them:
 *
 *     <gazebo reference="led_board">
 *       <sensor name="board_ntc" type="thermistor">
 *         <update_rate>10</update_rate>
 *         <thermistor><target>led_left</target><time_constant>2.0</time_constant>
 *           <noise><stddev>0.05</stddev></noise><resolution>0.0625</resolution></thermistor>
 *       </sensor>
 *       <sensor name="sink_ir" type="ir_thermometer">
 *         <pose>0 0 0.05 0 1.5708 0</pose>
 *         <ir_thermometer><fov>0.6</fov><range>1.0</range><emissivity>0.95</emissivity>
 *           <ambient>298.15</ambient><noise><stddev>0.1</stddev></noise></ir_thermometer>
 *       </sensor>
 *     </gazebo>
 *
 * thermistor: a contact sensor. It reads thermal/<target>/temperature (target: a thermal node, a
 * link with a material, or a heat source; default its own link) through a first-order lag of
 * `time_constant` s (0: none; a bead thermistor in still air is a few seconds), Gaussian noise of
 * `noise/stddev` K and quantisation to `resolution` K (0: none). Output sensor/<name>/temperature
 * (K).
 *
 * ir_thermometer: a spot pyrometer looking along its +x. A 5 x 5 grid of rays over a square field
 * of view `fov` (rad; 0: one ray) up to `range` m; each ray that meets a body with a temperature
 * (thermal/<body>/temperature) sees it, with the target's `emissivity` (the sensor's setting, as
 * on a real pyrometer), the rest of the radiance reflected from the surroundings at the ambient
 * temperature; a ray that meets something with no temperature, or nothing, sees the ambient. The
 * apparent temperature is the one whose grey-body radiance is the rays' mean, (mean(ε T⁴ +
 * (1 − ε) T_amb⁴))^(1/4), broadband (Stefan–Boltzmann) rather than in the sensor's band. The
 * ambient is env/air_temperature when the weather writes it, else the `ambient` parameter (K, no
 * default: a site parameter); with neither the sensor is not valid. Outputs
 * sensor/<name>/temperature (K) and sensor/<name>/valid. */
#include "lagrange_plugin.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_SENSORS 32
#define MAX_BODIES 256
#define GRID 5
#define PROBE_STEP_NS 10000000 /* 10 ms: rates up to 100 Hz */

typedef struct probe {
    int infrared;
    uint32_t body;
    double mount[7], rest[7];
    double time_constant, noise, resolution;
    double fov, range, emissivity, ambient; /* ambient < 0: not given */
    uint32_t every;
    uint32_t target, pose, air;  /* inputs */
    uint32_t temperature, valid; /* outputs */
    double state; /* the thermistor's lagged temperature, K; < 0 before the first sample */
} probe;

typedef struct plugin {
    const lagrange_host_api* host;
    uint32_t count;
    probe sensors[MAX_SENSORS];
    uint32_t bodies;                       /* how many bodies have a temperature input */
    uint32_t body_temperature[MAX_BODIES]; /* thermal/<body>/temperature, per body index */
    char error[200];
} plugin;

static const lagrange_unit kKelvin = {0, 0, 0, 0, 1, 0, 0, 0, 1.0, 9 /* Temperature */};
static const lagrange_unit kMetre = {1, 0, 0, 0, 0, 0, 0, 0, 1.0, 14 /* Position */};
static const lagrange_unit kOne = {0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0};

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

static void rotate(const double q[4], const double v[3], double out[3]) {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    const double tx = 2.0 * (y * v[2] - z * v[1]);
    const double ty = 2.0 * (z * v[0] - x * v[2]);
    const double tz = 2.0 * (x * v[1] - y * v[0]);
    out[0] = v[0] + w * tx + (y * tz - z * ty);
    out[1] = v[1] + w * ty + (z * tx - x * tz);
    out[2] = v[2] + w * tz + (x * ty - y * tx);
}

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

/* A uniform number in (0, 1) from a counter (SplitMix64): the run stays deterministic. */
static double uniform(uint64_t key) {
    uint64_t z = key + 0x9E3779B97F4A7C15ULL;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    z ^= z >> 31;
    return ((double)(z >> 11) + 0.5) / 9007199254740992.0;
}

static double gaussian(uint64_t seed, uint64_t step, uint32_t i) {
    const uint64_t key = seed ^ (step * 0x2545F4914F6CDD1DULL) ^ ((uint64_t)i << 48);
    const double u1 = uniform(key);
    const double u2 = uniform(key ^ 0xD1B54A32D192ED03ULL);
    return sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2);
}

static double param(const lagrange_host_api* h, const lagrange_world* w, uint32_t s,
                    const char* key, double fallback) {
    double value = fallback;
    return h->world_sensor_param_real(w, s, key, &value) ? value : fallback;
}

static uint32_t input(const lagrange_host_api* h, lagrange_declarer* d, const char* name,
                      lagrange_unit unit, lagrange_signal_type type, int optional,
                      double fallback) {
    lagrange_signal_spec spec;
    memset(&spec, 0, sizeof spec);
    spec.struct_size = sizeof spec;
    spec.name = name;
    spec.type = type;
    spec.unit = unit;
    spec.frame = type == LAGRANGE_POSE ? "world_local" : NULL;
    spec.optional = (uint8_t)optional;
    spec.feedback = 1; /* the thermal domain steps slower: last value is fine, and no loop */
    spec.default_value = fallback;
    return h->declare_input(d, &spec);
}

static uint32_t output(const lagrange_host_api* h, lagrange_declarer* d, const char* name,
                       lagrange_unit unit) {
    lagrange_signal_spec spec;
    memset(&spec, 0, sizeof spec);
    spec.struct_size = sizeof spec;
    spec.name = name;
    spec.type = LAGRANGE_SCALAR;
    spec.unit = unit;
    return h->declare_output(d, &spec);
}

static lagrange_status declare(void* p, const lagrange_world* world, const lagrange_config* config,
                               lagrange_declarer* declarer) {
    plugin* self = (plugin*)p;
    const lagrange_host_api* h = self->host;
    uint32_t s, b;
    int anyInfrared = 0;
    char name[200];
    (void)config;
    self->count = 0;
    self->bodies = 0;
    if (h->struct_size < offsetof(lagrange_host_api, scene_overlap_sphere) + sizeof(void*)) {
        snprintf(self->error, sizeof self->error, "needs a host with plugin ABI 1.1");
        return LAGRANGE_UNSUPPORTED;
    }
    for (s = 0; s < h->world_sensor_count(world); ++s) {
        lagrange_sensor_info info;
        lagrange_body_info body;
        const char* target = NULL;
        probe* t;
        memset(&info, 0, sizeof info);
        info.struct_size = sizeof info;
        if (!h->world_sensor(world, s, &info)) {
            continue;
        }
        if (strcmp(info.kind, "thermistor") != 0 && strcmp(info.kind, "ir_thermometer") != 0) {
            continue;
        }
        if (self->count == MAX_SENSORS) {
            snprintf(self->error, sizeof self->error, "more than %d thermal probes", MAX_SENSORS);
            return LAGRANGE_INVALID_CONFIG;
        }
        t = &self->sensors[self->count++];
        memset(t, 0, sizeof *t);
        memset(&body, 0, sizeof body);
        body.struct_size = sizeof body;
        (void)h->world_body(world, info.body, &body);
        t->infrared = strcmp(info.kind, "ir_thermometer") == 0;
        t->body = info.body;
        memcpy(t->mount, info.pose_in_body, sizeof t->mount);
        memcpy(t->rest, body.rest_pose, sizeof t->rest);
        t->time_constant = param(h, world, s, "time_constant", 0.0);
        t->noise = param(h, world, s, "noise/stddev", 0.0);
        t->resolution = param(h, world, s, "resolution", 0.0);
        t->fov = param(h, world, s, "fov", 0.0);
        t->range = param(h, world, s, "range", 1.0);
        t->emissivity = param(h, world, s, "emissivity", 0.95);
        t->ambient = param(h, world, s, "ambient", -1.0);
        if (t->time_constant < 0.0 || t->noise < 0.0 || t->resolution < 0.0 || t->fov < 0.0 ||
            !(t->range > 0.0) || !(t->emissivity > 0.0) || t->emissivity > 1.0) {
            snprintf(self->error, sizeof self->error,
                     "%s: time_constant, noise, resolution and fov must not be negative, range "
                     "over 0 and emissivity within (0, 1]",
                     info.name);
            return LAGRANGE_INVALID_CONFIG;
        }
        t->every =
            info.rate > 0.0 ? (uint32_t)floor(1e9 / (info.rate * (double)PROBE_STEP_NS) + 0.5) : 1;
        if (t->every == 0) {
            t->every = 1;
        }
        t->state = -1.0;
        if (t->infrared) {
            anyInfrared = 1;
            snprintf(name, sizeof name, "body/%s/pose", body.name);
            t->pose = input(h, declarer, name, kMetre, LAGRANGE_POSE, 1, 0.0);
            t->air = input(h, declarer, "env/air_temperature", kKelvin, LAGRANGE_SCALAR, 1, -1.0);
            snprintf(name, sizeof name, "sensor/%s/valid", info.name);
            t->valid = output(h, declarer, name, kOne);
        } else {
            if (!h->world_sensor_param_string(world, s, "target", &target) || target == NULL ||
                target[0] == '\0') {
                target = body.name;
            }
            snprintf(name, sizeof name, "thermal/%s/temperature", target);
            t->target = input(h, declarer, name, kKelvin, LAGRANGE_SCALAR, 0, 0.0);
        }
        snprintf(name, sizeof name, "sensor/%s/temperature", info.name);
        t->temperature = output(h, declarer, name, kKelvin);
        if (t->temperature == LAGRANGE_INVALID_SIGNAL ||
            (!t->infrared && t->target == LAGRANGE_INVALID_SIGNAL)) {
            snprintf(self->error, sizeof self->error, "%s: invalid signal names", info.name);
            return LAGRANGE_INVALID_CONFIG;
        }
    }
    /* What an infrared probe can see: every body's temperature, when it has one. */
    if (anyInfrared) {
        self->bodies = h->world_body_count(world);
        if (self->bodies > MAX_BODIES) {
            self->bodies = MAX_BODIES;
        }
        for (b = 0; b < self->bodies; ++b) {
            lagrange_body_info body;
            memset(&body, 0, sizeof body);
            body.struct_size = sizeof body;
            (void)h->world_body(world, b, &body);
            snprintf(name, sizeof name, "thermal/%s/temperature", body.name);
            self->body_temperature[b] = input(h, declarer, name, kKelvin, LAGRANGE_SCALAR, 1, -1.0);
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
        probe* t = &self->sensors[i];
        t->state = -1.0;
        self->host->bus_write(bus, t->temperature, 0, NAN);
        if (t->infrared) {
            self->host->bus_write(bus, t->valid, 0, 0.0);
        }
    }
    return LAGRANGE_OK;
}

/* The first body other than `own` along a ray, or LAGRANGE_WORLD_BODY-like "nothing" (returns 0).
 */
static int first_body(const lagrange_host_api* h, const lagrange_bus* bus, const double origin[3],
                      const double dir[3], double range, uint32_t own, uint32_t* out) {
    double travelled = 0.0;
    double from[3];
    int hops;
    memcpy(from, origin, sizeof from);
    for (hops = 0; hops < 8; ++hops) {
        lagrange_ray_hit hit;
        memset(&hit, 0, sizeof hit);
        hit.struct_size = sizeof hit;
        if (!h->scene_ray(bus, from, dir, range - travelled, &hit)) {
            return 0;
        }
        travelled += hit.distance;
        if (hit.body != own) {
            *out = hit.body;
            return 1;
        }
        travelled += 1e-6;
        from[0] = origin[0] + dir[0] * travelled;
        from[1] = origin[1] + dir[1] * travelled;
        from[2] = origin[2] + dir[2] * travelled;
    }
    return 0;
}

static lagrange_status step(void* p, const lagrange_step_context* context, lagrange_bus* bus) {
    plugin* self = (plugin*)p;
    const lagrange_host_api* h = self->host;
    uint32_t i;
    for (i = 0; i < self->count; ++i) {
        probe* t = &self->sensors[i];
        double reading;
        if (context->step_index % t->every != 0) {
            continue;
        }
        if (!t->infrared) {
            const double truth = h->bus_read(bus, t->target, 0);
            const double dt = (double)t->every * (double)context->dt_ns * 1e-9;
            if (t->state < 0.0 || t->time_constant <= 0.0) {
                t->state = truth;
            } else {
                /* First order, exact for a held input over the sample period. */
                t->state += (truth - t->state) * (1.0 - exp(-dt / t->time_constant));
            }
            reading = t->state;
        } else {
            uint32_t n = 0;
            const double* moving = h->bus_read_n(bus, t->pose, 0, &n);
            double body[7], sensor[7];
            const double air = h->bus_read(bus, t->air, 0);
            const double ambient = air > 0.0 ? air : t->ambient;
            const int grid = t->fov > 0.0 ? GRID : 1;
            double radiance = 0.0;
            int a, b;
            if (ambient <= 0.0) {
                h->bus_write(bus, t->temperature, 0, NAN);
                h->bus_write(bus, t->valid, 0, 0.0);
                continue;
            }
            if (n == 7 && moving[3] * moving[3] + moving[4] * moving[4] + moving[5] * moving[5] +
                                  moving[6] * moving[6] >
                              0.5) {
                memcpy(body, moving, sizeof body);
            } else {
                memcpy(body, t->rest, sizeof body);
            }
            compose(body, t->mount, sensor);
            for (a = 0; a < grid; ++a) {
                for (b = 0; b < grid; ++b) {
                    const double u = grid == 1 ? 0.0 : t->fov * ((double)a / (grid - 1) - 0.5);
                    const double v = grid == 1 ? 0.0 : t->fov * ((double)b / (grid - 1) - 0.5);
                    double local[3], dir[3], length, seen = ambient;
                    uint32_t hit = 0;
                    local[0] = 1.0;
                    local[1] = tan(u);
                    local[2] = tan(v);
                    length = sqrt(local[0] * local[0] + local[1] * local[1] + local[2] * local[2]);
                    local[0] /= length;
                    local[1] /= length;
                    local[2] /= length;
                    rotate(sensor + 3, local, dir);
                    if (first_body(h, bus, sensor, dir, t->range, t->body, &hit) &&
                        hit < self->bodies) {
                        const double surface = h->bus_read(bus, self->body_temperature[hit], 0);
                        if (surface > 0.0) {
                            seen = surface;
                        }
                    }
                    /* Grey body: emitted at its temperature, the rest reflected from the
                     * surroundings at the ambient. */
                    radiance +=
                        t->emissivity * pow(seen, 4.0) + (1.0 - t->emissivity) * pow(ambient, 4.0);
                }
            }
            reading = pow(radiance / (double)(grid * grid), 0.25);
            h->bus_write(bus, t->valid, 0, 1.0);
        }
        if (t->noise > 0.0) {
            reading += t->noise * gaussian(context->seed, context->step_index, i);
        }
        if (t->resolution > 0.0) {
            reading = floor(reading / t->resolution + 0.5) * t->resolution;
        }
        h->bus_write(bus, t->temperature, 0, reading);
    }
    return LAGRANGE_OK;
}

/* The thermistors' lagged temperatures. */
static lagrange_status save_state(const void* p, uint8_t* buffer, size_t capacity,
                                  size_t* written) {
    const plugin* self = (const plugin*)p;
    const size_t size = self->count * sizeof(double);
    uint32_t i;
    *written = size;
    if (buffer == NULL || capacity < size) {
        return buffer == NULL ? LAGRANGE_OK : LAGRANGE_INTERNAL;
    }
    for (i = 0; i < self->count; ++i) {
        memcpy(buffer + i * sizeof(double), &self->sensors[i].state, sizeof(double));
    }
    return LAGRANGE_OK;
}

static lagrange_status restore_state(void* p, const uint8_t* buffer, size_t size) {
    plugin* self = (plugin*)p;
    uint32_t i;
    if (size != self->count * sizeof(double)) {
        return LAGRANGE_INTERNAL;
    }
    for (i = 0; i < self->count; ++i) {
        memcpy(&self->sensors[i].state, buffer + i * sizeof(double), sizeof(double));
    }
    return LAGRANGE_OK;
}

static const char* last_error(const void* p) {
    const plugin* self = (const plugin*)p;
    return self->error[0] != '\0' ? self->error : NULL;
}

static const lagrange_module_api kModule = {
    sizeof(lagrange_module_api),
    "thermal_probe",
    "lagrange_thermal_probe",
    1,
    0,
    0, /* not a substitute: its own domain */
    1, /* deterministic */
    0, /* Tier 0 */
    PROBE_STEP_NS,
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
