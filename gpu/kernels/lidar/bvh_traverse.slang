// Tier 1 lidar (docs/design/domains/sensors.md §6): one thread per beam, a stack traversal of a
// bounding-volume hierarchy over the collision shapes, and the exact ray-shape tests of the CPU
// reference (modules/sensors raycast) in float. Everything is in a frame centred on the sensor,
// computed in double on the CPU and converted last, so float only ever holds sensor-relative
// coordinates (docs/design/huge-worlds.md §1, render-local).

// The hierarchy is in world axes centred on the sensor, so traversal uses the beam turned into
// world axes; the exact tests use the beam in each shape's frame.

// Shape types, in the order of lagrange::ShapeType.
static const uint kPlane = 0;
static const uint kSphere = 1;
static const uint kCapsule = 2;
static const uint kCylinder = 3;
static const uint kBox = 4;

static const uint kNone = 0xFFFFFFFF;
static const int kStackSize = 64;

// Both members computed in double on the CPU, so a ray in the shape's frame takes one float
// rotation of the beam direction.
struct Shape {
    float3 origin;  // the sensor in the shape's frame
    uint type;
    float3 size;  // lagrange::ShapeType conventions
    uint pad;
    float4 row0;  // sensor-to-shape rotation, by rows (xyz)
    float4 row1;
    float4 row2;
};

// Leaf: left = -1 - shape index. Internal: indices of both children.
struct Node {
    float3 lo;
    int left;
    float3 hi;
    int right;
};

struct Hit {
    float range;  // m; +inf for no hit
    uint shape;   // index into shapes, kNone for no hit
};

struct Params {
    float4 rot0;  // sensor-to-world rotation, by rows (xyz)
    float4 rot1;
    float4 rot2;
    uint rays;
    uint planes;  // shapes[0, planes) are unbounded planes, tested outside the hierarchy
    uint nodes;
    uint pad;
};

[[vk::binding(0, 0)]] StructuredBuffer<Shape> shapes;
[[vk::binding(1, 0)]] StructuredBuffer<Node> nodes;
[[vk::binding(2, 0)]] StructuredBuffer<float4> directions;  // per beam, sensor frame
[[vk::binding(3, 0)]] RWStructuredBuffer<Hit> hits;
[[vk::push_constant]] ConstantBuffer<Params> params;

static const float kNoHit = 1.0 / 0.0;

// Smallest positive root of a t^2 + 2 b t + c = 0, given its reduced discriminant b^2 - a c.
float firstRoot(float a, float b, float c, float discriminant) {
    if (a <= 0.0 || discriminant < 0.0) {
        return kNoHit;
    }
    // The two roots without cancellation: q = -(b + sign(b) root); roots q / a and c / q.
    const float root = sqrt(discriminant);
    const float q = b >= 0.0 ? -(b + root) : -(b - root);
    float near = q / a;
    float far = q != 0.0 ? c / q : near;
    if (near > far) {
        const float swap = near;
        near = far;
        far = swap;
    }
    if (near > 0.0) {
        return near;
    }
    return far > 0.0 ? far : kNoHit;
}

// b^2 - a c for |o + t d|^2 = r^2, written as a (r^2 - |o - (b / a) d|^2): the distance from
// the centre to the line, which keeps its precision for a ray grazing a far shape (Haines et al.,
// "Precision Improvements for Ray/Sphere Intersection", Ray Tracing Gems, 2019, ch. 7).
float sphere(float3 o, float3 d, float radius) {
    const float a = dot(d, d);
    const float b = dot(o, d);
    const float3 l = o - (b / a) * d;
    return firstRoot(a, b, dot(o, o) - radius * radius, a * (radius * radius - dot(l, l)));
}

// The same for the side of a cylinder along z, in the xy plane; |z| <= half.
float cylinderSide(float3 o, float3 d, float radius, float half) {
    const float a = d.x * d.x + d.y * d.y;
    if (a <= 0.0) {
        return kNoHit;
    }
    const float b = o.x * d.x + o.y * d.y;
    const float c = o.x * o.x + o.y * o.y - radius * radius;
    const float2 l = o.xy - (b / a) * d.xy;
    const float discriminant = a * (radius * radius - dot(l, l));
    if (discriminant < 0.0) {
        return kNoHit;
    }
    const float root = sqrt(discriminant);
    const float q = b >= 0.0 ? -(b + root) : -(b - root);
    float t0 = q / a;
    float t1 = q != 0.0 ? c / q : t0;
    if (t0 > t1) {
        const float swap = t0;
        t0 = t1;
        t1 = swap;
    }
    if (t0 > 0.0 && abs(o.z + t0 * d.z) <= half) {
        return t0;
    }
    if (t1 > 0.0 && abs(o.z + t1 * d.z) <= half) {
        return t1;
    }
    return kNoHit;
}

float cap(float3 o, float3 d, float radius, float z) {
    if (d.z == 0.0) {
        return kNoHit;
    }
    const float t = (z - o.z) / d.z;
    const float x = o.x + t * d.x;
    const float y = o.y + t * d.y;
    return t > 0.0 && x * x + y * y <= radius * radius ? t : kNoHit;
}

float box(float3 o, float3 d, float3 half) {
    float enter = -kNoHit;
    float exit = kNoHit;
    for (int i = 0; i < 3; ++i) {
        if (d[i] == 0.0) {
            if (abs(o[i]) > half[i]) {
                return kNoHit;
            }
            continue;
        }
        float t0 = (-half[i] - o[i]) / d[i];
        float t1 = (half[i] - o[i]) / d[i];
        if (t0 > t1) {
            const float swap = t0;
            t0 = t1;
            t1 = swap;
        }
        enter = max(enter, t0);
        exit = min(exit, t1);
    }
    if (enter > exit) {
        return kNoHit;
    }
    if (enter > 0.0) {
        return enter;
    }
    return exit > 0.0 ? exit : kNoHit;
}

// The ray leaves the sensor along `local` (the beam in the sensor frame).
float intersect(Shape s, float3 local) {
    const float3 o = s.origin;
    const float3 d =
        float3(dot(s.row0.xyz, local), dot(s.row1.xyz, local), dot(s.row2.xyz, local));
    switch (s.type) {
        case kPlane: {
            if (d.z == 0.0) {
                return kNoHit;
            }
            const float t = -o.z / d.z;
            return t > 0.0 ? t : kNoHit;
        }
        case kSphere:
            return sphere(o, d, s.size.x);
        case kBox:
            return box(o, d, s.size);
        case kCylinder:
            return min(cylinderSide(o, d, s.size.x, s.size.y),
                       min(cap(o, d, s.size.x, s.size.y), cap(o, d, s.size.x, -s.size.y)));
        case kCapsule:
            return min(cylinderSide(o, d, s.size.x, s.size.y),
                       min(sphere(float3(o.x, o.y, o.z - s.size.y), d, s.size.x),
                           sphere(float3(o.x, o.y, o.z + s.size.y), d, s.size.x)));
        default:
            return kNoHit;
    }
}

// Entry distance of the ray into a node's box, or kNoHit if it misses or starts beyond `limit`.
float enterBox(Node n, float3 inverse, float limit) {
    const float3 t0 = n.lo * inverse;
    const float3 t1 = n.hi * inverse;
    const float3 near = min(t0, t1);
    const float3 far = max(t0, t1);
    const float enter = max(max(near.x, near.y), max(near.z, 0.0));
    const float exit = min(min(far.x, far.y), min(far.z, limit));
    return enter <= exit ? enter : kNoHit;
}

[shader("compute")]
[numthreads(64, 1, 1)]
void castRays(uint3 id: SV_DispatchThreadID) {
    const uint ray = id.x;
    if (ray >= params.rays) {
        return;
    }
    const float3 local = directions[ray].xyz;
    const float3 dir = float3(dot(params.rot0.xyz, local), dot(params.rot1.xyz, local),
                              dot(params.rot2.xyz, local));
    Hit hit;
    hit.range = kNoHit;
    hit.shape = kNone;
    for (uint p = 0; p < params.planes; ++p) {
        const float t = intersect(shapes[p], local);
        if (t < hit.range) {
            hit.range = t;
            hit.shape = p;
        }
    }
    if (params.nodes > 0) {
        // A zero component gives an infinite inverse; 1e30 keeps 0 * it finite at the slabs.
        const float3 inverse = float3(dir.x != 0.0 ? 1.0 / dir.x : 1e30,
                                      dir.y != 0.0 ? 1.0 / dir.y : 1e30,
                                      dir.z != 0.0 ? 1.0 / dir.z : 1e30);
        int stack[kStackSize];
        int top = 0;
        stack[top++] = 0;
        while (top > 0) {
            const Node node = nodes[stack[--top]];
            if (enterBox(node, inverse, hit.range) == kNoHit) {
                continue;
            }
            if (node.left < 0) {
                const uint s = uint(-1 - node.left);
                const float t = intersect(shapes[s], local);
                if (t < hit.range) {
                    hit.range = t;
                    hit.shape = s;
                }
            } else if (top + 2 <= kStackSize) {
                stack[top++] = node.left;
                stack[top++] = node.right;
            }
        }
    }
    hits[ray] = hit;
}
