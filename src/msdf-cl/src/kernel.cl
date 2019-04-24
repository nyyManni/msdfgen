
#define NEXT_SEGMENT(s) s = (__constant segment *)(((__constant float2 *)(s + 1)) + s->npoints);

typedef struct segment {
    int color;
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
    float2 r;
    float2 g;
    float2 b;
} multi_distance;

/* Local data structures */
struct distance_selector {
    float2 min_true, min_negative, min_positive;
    float near_param;
    segment *near_segment;
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

__kernel void msdf(__constant void *_glyph, __write_only image2d_t output,
                   float2 scale, float2 translate, float range) {

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


    int2 coords = {get_global_id(0), get_global_id(1)};
    float2 p = {(coords.x + 0.5f) / scale.x - translate.x,
                (coords.y + 0.5f) / scale.y - translate.y};

    __constant glyph *g = (__constant glyph *)_glyph;

    __constant contour *c = g->contours;
    for (int _i = 0; _i < g->ncontours; ++_i) {
        if (!c->segments) {
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
    float4 color = {1.0, 1.0, 1.0, 1.0};
    write_imagef(output, coords, color);
}

void add_segment(__private struct segment_selector *e, __constant segment *prev, 
                 __constant segment *cur, __constant segment *next, float2 point) {
    
}

void set_contour_segment(__private struct workspace *ws,
                         __private struct segment_selector *e,
                         __constant contour *c, float2 point) {
    
}
