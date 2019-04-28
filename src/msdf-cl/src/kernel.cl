
#define NEXT_SEGMENT(s) s = (__constant segment *)(((__constant float2 *)(s + 1)) + s->npoints);
#define MSDF_CUBIC_SEARCH_STARTS 4
#define MSDF_CUBIC_SEARCH_STEPS 4

enum color {
    BLACK = 0,
    RED = 1,
    GREEN = 2,
    BLUE = 4,

    YELLOW = RED | GREEN,
    MAGENTA = BLUE | RED,
    CYAN = BLUE | GREEN,
    WHITE = RED | GREEN | BLUE
};

typedef struct segment {
    uint color;
    int npoints;
    float2 points[];
} segment;

typedef struct contour {
    int winding;
    int nsegments;
    struct segment segments[];
} contour;

typedef struct glyph {
    int ncontours;
    struct contour contours[];
} glyph;

typedef struct multi_distance {
    float r;
    float g;
    float b;
} multi_distance;

/* Local data structures */
struct distance_selector {
    float2 min_true, min_negative, min_positive;
    float near_param;
    __constant segment *near_segment;
};

struct segment_selector {
    struct distance_selector r, g, b;
};

struct workspace {
    struct segment_selector shape;
    struct segment_selector inner;
    struct segment_selector outer;

    multi_distance max_inner;
    multi_distance max_outer;
    multi_distance min_abs;
};

void add_segment(__private struct segment_selector *, __constant segment *,
                 __constant segment *, __constant segment *, float2);

void set_contour_segment(__private struct workspace *,
                         __private struct segment_selector *,
                         __constant contour *, float2);

bool point_facing_edge(__constant segment *prev, __constant segment *cur, 
                       __constant segment *next, float2 p, float param);

float2 signed_distance(__constant segment *, float2, float *);
float2 signed_distance_linear(__constant segment *, float2, float *);
float2 signed_distance_quad(__constant segment *, float2, float *);
float2 signed_distance_cubic(__constant segment *, float2, float *);

void add_segment_true_distance(struct distance_selector *psdb, __constant segment *s,
                               float2 d, float param);
void add_segment_pseudo_distance(struct distance_selector *psdb, float2 d);
void distance_to_pseudo_distance(__constant segment *s, float2 *d, float2 p, float param);
float2 segment_direction(__constant segment *e, float param);
float2 segment_point(__constant segment *e, float param);

multi_distance get_distance(struct segment_selector *e, float2 point);
void merge_multi_segment(struct segment_selector *e, struct segment_selector *other);
void merge_segment(struct distance_selector *s, struct distance_selector *other);
float compute_distance(struct distance_selector *b, float2 point);

multi_distance get_pixel_distance(struct workspace *, __constant glyph *, float2);

float length_(float2 v) { return sqrt(v.x * v.x + v.y * v.y); }


inline float4 to_pixel(multi_distance d, float range) {
    float4 _f = {d.r / range + 0.5f, d.g / range + 0.5f, d.b / range + 0.5f, 1.0};
    return _f;
}

inline float cross_(float2 a, float2 b) { return a.x * b.y - a.y * b.x; }

inline float dot_(float2 a, float2 b) { return a.x * b.x + a.y * b.y; }

inline float2 orthonormal(float2 v, bool polarity) {
    float len = fast_length(v);
    float2 a = {-v.y / len, v.x / len};
    float2 b = {v.y / len, -v.x / len};
    return polarity ? a : b;
}

bool less(float2 a, float2 b) {
    return fabs(a.x) < fabs(b.x) || (fabs(a.x) == fabs(b.x) && a.y < b.y);
}

inline float median(float a, float b, float c) {
    return max(min(a, b), min(max(a, b), c));
}

inline float resolve_multi_distance(multi_distance d) {
    return median(d.r, d.g, d.b);
}

__kernel void msdf(const __constant glyph *_glyph, __write_only image2d_t output,
                   float2 scale, float2 translate, float range) {
    const __constant glyph *g = _glyph;
    
    /* printf("color!: %.2f\n", g->contours[0].segments[0].points[0].y); */
        
    int2 coords = {get_global_id(0), get_global_id(1)};
    float2 p = {(coords.x + 0.5f) / scale.x - translate.x,
                (coords.y + 0.5f) / scale.y - translate.y};

    struct workspace ws = {
        .shape = {
            .r = {.min_true     = {-INFINITY, 1.0},
                  .min_negative = {-INFINITY, 1.0},
                  .min_positive = {-INFINITY, 1.0},
                  .near_param   = 0.0,
                  .near_segment = NULL},
            .g = {.min_true     = {-INFINITY, 1.0},
                  .min_negative = {-INFINITY, 1.0},
                  .min_positive = {-INFINITY, 1.0},
                  .near_param   = 0.0,
                  .near_segment = NULL},
            .b = {.min_true     = {-INFINITY, 1.0},
                  .min_negative = {-INFINITY, 1.0},
                  .min_positive = {-INFINITY, 1.0},
                  .near_param   = 0.0,
                  .near_segment = NULL}
        },
        .inner = {
            .r = {.min_true     = {-INFINITY, 1.0},
                  .min_negative = {-INFINITY, 1.0},
                  .min_positive = {-INFINITY, 1.0},
                  .near_param   = 0.0,
                  .near_segment = NULL},
            .g = {.min_true     = {-INFINITY, 1.0},
                  .min_negative = {-INFINITY, 1.0},
                  .min_positive = {-INFINITY, 1.0},
                  .near_param   = 0.0,
                  .near_segment = NULL},
            .b = {.min_true     = {-INFINITY, 1.0},
                  .min_negative = {-INFINITY, 1.0},
                  .min_positive = {-INFINITY, 1.0},
                  .near_param   = 0.0,
                  .near_segment = NULL}
        },
        .outer = {
            .r = {.min_true     = {-INFINITY, 1.0},
                  .min_negative = {-INFINITY, 1.0},
                  .min_positive = {-INFINITY, 1.0},
                  .near_param   = 0.0,
                  .near_segment = NULL},
            .g = {.min_true     = {-INFINITY, 1.0},
                  .min_negative = {-INFINITY, 1.0},
                  .min_positive = {-INFINITY, 1.0},
                  .near_param   = 0.0,
                  .near_segment = NULL},
            .b = {.min_true     = {-INFINITY, 1.0},
                  .min_negative = {-INFINITY, 1.0},
                  .min_positive = {-INFINITY, 1.0},
                  .near_param   = 0.0,
                  .near_segment = NULL}
        },
        .max_inner = {-INFINITY, -INFINITY, -INFINITY},
        .max_outer = {-INFINITY, -INFINITY, -INFINITY},
        .min_abs   = {-INFINITY, -INFINITY, -INFINITY}
    };

    __constant contour *c = &(g->contours[0]);
    for (int _i = 0; _i < g->ncontours; ++_i) {
        if (!c->nsegments) {
            c += 1;
            continue;
        }

        struct segment_selector e = {
            .r = {.min_true     = {-INFINITY, 1.0},
                  .min_negative = {-INFINITY, 1.0},
                  .min_positive = {-INFINITY, 1.0},
                  .near_param   = 0.0,
                  .near_segment = NULL},
            .g = {.min_true     = {-INFINITY, 1.0},
                  .min_negative = {-INFINITY, 1.0},
                  .min_positive = {-INFINITY, 1.0},
                  .near_param   = 0.0,
                  .near_segment = NULL},
            .b = {.min_true     = {-INFINITY, 1.0},
                  .min_negative = {-INFINITY, 1.0},
                  .min_positive = {-INFINITY, 1.0},
                  .near_param   = 0.0,
                  .near_segment = NULL}
        };

        __constant segment *s = c->segments;

        /* Initialize cur to the last segment in the list */
        __constant segment *cur = s;
        for (int _i = 0; _i < c->nsegments - 1; ++_i)
            NEXT_SEGMENT(cur);

        /*
         * Initialize prev to the second last segment in the list, or the first
         * one if there are less than two segments.
         */
        __constant segment *prev = s;
        for (int _i = 0; _i < c->nsegments - 2 && c->nsegments >= 2; ++_i)
            NEXT_SEGMENT(prev);

        for (int _i = 0; _i < c->nsegments; ++_i) {
            add_segment(&e, prev, cur, s, p);
            prev = cur;
            cur = s;
            NEXT_SEGMENT(s);
        }
        set_contour_segment(&ws, &e, c, p);
        c = (__constant contour *)s;
    }
    multi_distance d = get_pixel_distance(&ws, g, p);
    /* printf("pixel_distance: %.2f %.2f %.2f\n", d.r, d.g, d.b); */
    float4 color = to_pixel(d, range);
    /* printf("pixel_distance: %.2f %.2f %.2f\n", colo.r, color.g, color.b, color.a); */
    printf("pixel_distance: %.2f %.2f %.2f %.2f\n", color.r, color.g, color.b, color.a);
    write_imagef(output, coords, color);
}

void add_segment(__private struct segment_selector *e, __constant segment *prev,
                 __constant segment *cur, __constant segment *next, float2 point) {
    float param;
    /* printf("color: %d, points: %d\n", cur->color, cur->npoints); */
    float2 d = signed_distance(cur, point, &param);
    int2 coords = {get_global_id(0), get_global_id(1)};
    /* printf("distance: %.2f %.2f\n", d.x, d.y); */
    /* printf("color: %d\n", cur->color); */

    if (cur->color & RED)
        add_segment_true_distance(&e->r, cur, d, param);
    if (cur->color & GREEN)
        add_segment_true_distance(&e->g, cur, d, param);
    if (cur->color & BLUE)
        add_segment_true_distance(&e->b, cur, d, param);

    if (point_facing_edge(prev, cur, next, point, param)) {

        distance_to_pseudo_distance(cur, &d, point, param);
        if (cur->color & RED)
            add_segment_pseudo_distance(&e->r, d);
        if (cur->color & GREEN)
            add_segment_pseudo_distance(&e->g, d);
        if (cur->color & BLUE)
            add_segment_pseudo_distance(&e->b, d);
    }
    /* printf("distance: %.2f, %.2f\n", d.x, d.y); */
}

void set_contour_segment(__private struct workspace *ws,
                         __private struct segment_selector *e,
                         __constant contour *c, float2 point) {

    multi_distance d = get_distance(e, point);
    /* printf("multi distance: %.2f, %.2f, %.2f\n", */
    /*        d.r, d.g, d.b); */

    merge_multi_segment(&ws->shape, e);
    if (c->winding > 0 && resolve_multi_distance(d) >= 0)
        merge_multi_segment(&ws->inner, e);
    if (c->winding < 0 && resolve_multi_distance(d) <= 0)
        merge_multi_segment(&ws->outer, e);

    multi_distance *inner_target = &(ws->max_inner);
    multi_distance *outer_target = &(ws->max_outer);
    multi_distance *target = c->winding < 0 ? inner_target : outer_target;
    target = outer_target;
    if (c->winding > 0) {
        if (resolve_multi_distance(d) > resolve_multi_distance(ws->max_inner))
            ws->max_inner = d;
    } else {
        if (resolve_multi_distance(d) > resolve_multi_distance(ws->max_outer))
            ws->max_outer = d;
    }

    if (fabs((float)resolve_multi_distance(d)) < fabs((float)resolve_multi_distance(ws->min_abs)))
        ws->min_abs = d;
}

float compute_distance(struct distance_selector *b, float2 point) {
    float min_distance = b->min_true.x < 0 ? b->min_negative.x : b->min_positive.x;
    float2 d = b->min_true;
    distance_to_pseudo_distance(b->near_segment, &d, point, b->near_param);
    if (fabs(d.x) < fabs(min_distance))
        min_distance = d.x;
    return min_distance;
}

bool point_facing_edge(__constant segment *prev, __constant segment *cur, 
                       __constant segment *next, float2 p, float param) {
    if (param >= 0 && param <= 1)
        return true;
    float2 prev_edge_dir = -normalize(segment_direction(prev, 1));
    float2 edge_dir = normalize(segment_direction(cur, param < 0 ? 0 : 1)) * (param < 0 ? 1 : -1);
    float2 next_edge_dir = normalize(segment_direction(next, 0));
    float2 point_dir = p - segment_point(cur, param < 0 ? 0 : 1);
    return dot_(point_dir, edge_dir) >= dot_(point_dir, param < 0 ? prev_edge_dir : next_edge_dir);
}

void add_segment_true_distance(struct distance_selector *psdb, __constant segment *s,
                               float2 d, float param) {
    if (less(d, psdb->min_true)) {
        psdb->min_true = d;
        psdb->near_segment = s;
        psdb->near_param = param;
    }
    /* printf("min_true: %.2f, %.2f (comparing %.2f, %.2f)\n", psdb->min_true.x, psdb->min_true.y, */
    /*        d.x, d.y); */
}

void add_segment_pseudo_distance(struct distance_selector *psdb, float2 d) {
    float2 *min_pseudo = d.x < 0 ? &(psdb->min_negative) : &(psdb->min_positive);
    if (less(d, *min_pseudo)) {
        *min_pseudo = d;
    }
    /* printf("min_pseudo: %.2f, %.2f\n", min_pseudo->x, min_pseudo->y); */

}

void distance_to_pseudo_distance(__constant segment *s, float2 *d, float2 p, float param) {
    if (param >= 0 && param <= 1)
        return;

    float2 dir = normalize(segment_direction(s, param < 0 ? 0 : 1));
    float2 aq = p - segment_point(s, param < 0 ? 0 : 1);
    float ts = dot_(aq, dir);
    if (param < 0 ? ts < 0 : ts > 0) {
        float pseudo_distance = cross_(aq, dir);
        if (fabs(pseudo_distance) <= fabs(d->x)) {
            d->x = pseudo_distance;
            d->y = 0;
        }
    }
}

float2 segment_direction(__constant segment *e, float param) {
    if (e->npoints == 2)
        return e->points[1] - e->points[0];
    if (e->npoints == 3) {
        return mix(e->points[1] - e->points[0], e->points[2] - e->points[1], param);
    }

    float2 tangent =
        mix(mix(e->points[1] - e->points[0], e->points[2] - e->points[1], param),
            mix(e->points[2] - e->points[1], e->points[3] - e->points[2], param), param);

    if (!tangent.x && !tangent.y) {
        if (param == 0)
            return e->points[2] - e->points[0];
        if (param == 1)
            return e->points[3] - e->points[1];
    }
    return tangent;
}

float2 segment_point(__constant segment *e, float param) {
    if (e->npoints == 2)
        return mix(e->points[0], e->points[1], param);
    if (e->npoints == 3) {
        return mix(mix(e->points[0], e->points[1], param),
                   mix(e->points[1], e->points[2], param), param);
    }
    float2 p12 = mix(e->points[1], e->points[2], param);
    return mix(mix(mix(e->points[0], e->points[1], param), p12, param),
               mix(p12, mix(e->points[2], e->points[3], param), param), param);
}

void merge_segment(struct distance_selector *s,
                   struct distance_selector *other) {
    if (less(other->min_true, s->min_true)) {
        s->min_true = other->min_true;
        s->near_segment = other->near_segment;
        s->near_param = other->near_param;
    }
    if (less(other->min_negative, s->min_negative))
        s->min_negative = other->min_negative;
    if (less(other->min_positive, s->min_positive)) {
        s->min_positive = other->min_positive;
    }
}

void merge_multi_segment(struct segment_selector *e, struct segment_selector *other) {
    merge_segment(&e->r, &other->r);
    merge_segment(&e->g, &other->g);
    merge_segment(&e->b, &other->b);
}

multi_distance get_distance(struct segment_selector *e, float2 point) {
    multi_distance d;
    d.r = compute_distance(&e->r, point);
    d.g = compute_distance(&e->g, point);
    d.b = compute_distance(&e->b, point);
    return d;
}
multi_distance get_pixel_distance(struct workspace *ws, __constant glyph *g, float2 point) {
    
    multi_distance shape_distance = get_distance(&ws->shape, point);
    multi_distance inner_distance = get_distance(&ws->inner, point);
    multi_distance outer_distance = get_distance(&ws->outer, point);
    float inner_d = resolve_multi_distance(inner_distance);
    float outer_d = resolve_multi_distance(outer_distance);
    
    /* printf("shape_distance: %.2f %.2f %.2f\n", shape_distance.r, shape_distance.g, shape_distance.b); */
    /* printf("inner_d: %.2f\n", inner_d); */
    /* printf("outer_d: %.2f\n", outer_d); */

    bool inner = inner_d >= 0 && fabs(inner_d) <= fabs(outer_d);
    bool outer = outer_d <= 0 && fabs(outer_d) < fabs(inner_d);
    if (!inner && !outer)
        return shape_distance;

    multi_distance d = inner ? inner_distance : outer_distance;
    multi_distance contour_distance = inner ? ws->max_inner : ws->max_outer;

    float contour_d = resolve_multi_distance(contour_distance);
    if (fabs(contour_d) < fabs(outer_d) && contour_d > resolve_multi_distance(d))
        d = contour_distance;

    contour_distance = ws->min_abs;
    contour_d = resolve_multi_distance(contour_distance);
    float d_d = resolve_multi_distance(d);

    if (fabs(contour_d) < fabs(d_d))
        d = contour_distance;

    if (resolve_multi_distance(d) == resolve_multi_distance(shape_distance))
        d = shape_distance;

    return d;
    
}

int solve_quadratic(float x[2], float a, float b, float c) {
    if (fabs(a) < 1e-14) {
        if (fabs(b) < 1e-14) {
            if (c == 0)
                return -1;
            return 0;
        }
        x[0] = -c / b;
        return 1;
    }
    float dscr = b * b - 4 * a * c;
    if (dscr > 0) {
        dscr = sqrt((float)dscr);
        x[0] = (-b + dscr) / (2 * a);
        x[1] = (-b - dscr) / (2 * a);
        return 2;
    } else if (dscr == 0) {
        x[0] = -b / (2 * a);
        return 1;
    } else
        return 0;
}

static int solve_cubic_normed(float x[3], float a, float b, float c) {
    float a2 = a * a;
    float q = (a2 - 3 * b) / 9;
    float r = (a * (2 * a2 - 9 * b) + 27 * c) / 54;
    float r2 = r * r;
    float q3 = q * q * q;
    float A, B;
    if (r2 < q3) {
        float t = r / sqrt((float)q3);
        if (t < -1)
            t = -1;
        if (t > 1)
            t = 1;
        t = acos(t);
        a /= 3;
        q = -2 * sqrt((float)q);
        x[0] = q * cos(t / 3) - a;
        x[1] = q * cos((t + 2 * M_PI_F) / 3) - a;
        x[2] = q * cos((t - 2 * M_PI_F) / 3) - a;
        return 3;
    } else {
        A = -pow(fabs(r) + sqrt((float)(r2 - q3)), 1 / 3.);
        if (r < 0)
            A = -A;
        B = A == 0 ? 0 : q / A;
        a /= 3;
        x[0] = (A + B) - a;
        x[1] = -0.5 * (A + B) - a;
        x[2] = 0.5 * sqrt(3.0f) * (A - B);
        if (fabs(x[2]) < 1e-14)
            return 2;
        return 1;
    }
}

int solve_cubic(float x[3], float a, float b, float c, float d) {
    if (fabs(a) < 1e-14)
        return solve_quadratic(x, b, c, d);
    return solve_cubic_normed(x, b / a, c / a, d / a);
}

float2 signed_distance(__constant segment *s, float2 p, float *param) {
    if (s->npoints == 2)
        return signed_distance_linear(s, p, param);
    if (s->npoints == 3)
        return signed_distance_quad(s, p, param);
    return signed_distance_cubic(s, p, param);
}

float2 signed_distance_linear(__constant segment *s, float2 origin, float *param) {
    float2 aq = origin - s->points[0];
    float2 ab = s->points[1] - s->points[0];
    *param = dot(aq, ab) / dot(ab, ab);
    float2 eq = s->points[*param > .5] - origin;
    float endpointDistance = fast_length(eq);
    if (*param > 0 && *param < 1) {
        float orthoDistance = dot(orthonormal(ab, false), aq);
        if (fabs(orthoDistance) < endpointDistance) {
            float2 _f = {orthoDistance, 0};
            return _f;
        }
    }
    float2 _f = {sign((float)cross_(aq, ab)) * endpointDistance,
                 fabs(dot(normalize(ab), normalize(eq)))};
    return _f;
}

float2 signed_distance_quad(__constant segment *s, float2 origin, float *param) {
    float2 qa = s->points[0] - origin;
    float2 ab = s->points[1] - s->points[0];
    float2 br = s->points[2] - s->points[1] - ab;
    float a = dot(br, br);
    float b = 3 * dot(ab, br);
    float c = 2 * dot(ab, ab) + dot(qa, br);
    float d = dot(qa, ab);
    float t[3];
    int solutions = solve_cubic(t, a, b, c, d);

    float minDistance = sign((float)cross_(ab, qa)) * fast_length(qa); // distance from A
    *param = -dot(qa, ab) / dot(ab, ab);
    {
        float distance = sign((float)cross_(s->points[2] - s->points[1],
                                     s->points[2] - origin)) *
                         fast_length(s->points[2] - origin); // distance from B
        if (fabs(distance) < fabs(minDistance)) {
            minDistance = distance;
            *param = dot(origin - s->points[1], s->points[2] - s->points[1]) /
                     dot(s->points[2] - s->points[1], s->points[2] - s->points[1]);
        }
    }
    for (int i = 0; i < solutions; ++i) {
        if (t[i] > 0 && t[i] < 1) {
            float2 endpoint = s->points[0] + 2 * t[i] * ab + t[i] * t[i] * br;
            float distance = sign((float)cross_(s->points[2] - s->points[0],
                                         endpoint - origin)) * fast_length(endpoint - origin);
            if (fabs(distance) <= fabs(minDistance)) {
                minDistance = distance;
                *param = t[i];
            }
        }
    }

    if (*param >= 0 && *param <= 1) {
        float2 out = {minDistance, 0};
        return out;
    }
    if (*param < .5) {
        float2 out = {minDistance, fabs(dot(normalize(ab), normalize(qa)))};
        return out;
    } else {
        float2 out = {minDistance, fabs(dot(normalize(s->points[2] - s->points[1]),
                                            normalize(s->points[2] - origin)))};
        return out;
    }
}

float2 signed_distance_cubic(__constant segment *e, float2 origin, float *param) {
    float2 qa = e->points[0] - origin;
    float2 ab = e->points[1] - e->points[0];
    float2 br = e->points[2] - e->points[1] - ab;
    float2 as = (e->points[3] - e->points[2]) - (e->points[2] - e->points[1]) - br;

    float2 epDir = segment_direction(e, 0);
    float minDistance = sign((float)cross_(epDir, qa)) * fast_length(qa); // distance from A
    *param = -dot(qa, epDir) / dot(epDir, epDir);
    {
        epDir = segment_direction(e, 1);
        float distance = sign((float)cross_(epDir, e->points[3] - origin)) *
                         fast_length(e->points[3] - origin); // distance from B
        if (fabs(distance) < fabs(minDistance)) {
            minDistance = distance;
            *param = dot(origin + epDir - e->points[3], epDir) / dot(epDir, epDir);
        }
    }
    // Iterative minimum distance search
    for (int i = 0; i <= MSDF_CUBIC_SEARCH_STARTS; ++i) {
        float t = (float)i / MSDF_CUBIC_SEARCH_STARTS;
        for (int step = 0;; ++step) {
            float2 qpt = segment_point(e, t) - origin;
            float distance = sign((float)cross_(segment_direction(e, t), qpt)) * fast_length(qpt);
            if (fabs(distance) < fabs(minDistance)) {
                minDistance = distance;
                *param = t;
            }
            if (step == MSDF_CUBIC_SEARCH_STEPS)
                break;

            // Improve t
            float2 d1 = 3 * as * t * t + 6 * br * t + 3 * ab;
            float2 d2 = 6 * as * t + 6 * br;
            t -= dot(qpt, d1) / (dot(d1, d1) + dot(qpt, d2));
            if (t < 0 || t > 1)
                break;
        }
    }

    if (*param >= 0 && *param <= 1) {
        float2 _f = {minDistance, 0};
        return _f;
    }
    if (*param < .5) {
        float2 _f = {minDistance, fabs(dot(normalize(segment_direction(e, 0)), normalize(qa)))};
        return _f;
    } else {
        float2 _f = {minDistance, fabs(dot(normalize(segment_direction(e, 1)),
                                           normalize(e->points[3] - origin)))};
        return _f;
    }
}
