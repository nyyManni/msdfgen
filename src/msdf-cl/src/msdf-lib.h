#ifndef MSDF_LIB_H
#define MSDF_LIB_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

enum color {
    BLACK = 0,
    RED = 1,
    GREEN = 2,
    YELLOW = 3,
    BLUE = 4,
    MAGENTA = 5,
    CYAN = 6,
    WHITE = 7
};



typedef struct _vec2 {
    float x;
    float y;

    _vec2() : x(0), y(0) {}
    _vec2(float x, float y) : x(x), y(y) {}

    struct _vec2 operator-() {
        return _vec2(-x, -y);
    }

    struct _vec2 operator+(struct _vec2 other) {
        return _vec2(x + other.x, y + other.y);
    }

    struct _vec2 operator-(struct _vec2 other) {
        return _vec2(x - other.x, y - other.y);
    }

    struct _vec2 operator*(float value) {
        return _vec2(x * value, y * value);
    }


} vec2;
static inline struct _vec2 operator*(float value, struct _vec2 vector) {
    return _vec2(value * vector.x, value * vector.y);
}

typedef struct _vec3 {
    _vec3() : x(0), y(0), z(0) {}
    _vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    union {
        float x;
        float r;
    };
    union {
        float y;
        float g;
    };
    union {
        float z;
        float b;
    };
} vec3;

typedef struct _segment {
    int32_t color;
    int32_t npoints;
    vec2 points[];
} segment;

/* Iterating over segments has to be done manually because the size of the
   struct is not ant. */
#define NEXT_SEGMENT(s) s = (segment *)(((vec2 *)(s + 1)) + s->npoints);

typedef struct _contour {
    int32_t winding;
    int32_t nsegments;
    segment segments[];
} contour;

struct shape {
    int32_t ncontours;
    contour contours[];
};

typedef vec2 distance_t;

typedef struct segment_distance {
    distance_t d;
    float param;
} segment_distance;

struct multi_distance {
    float r;
    float g;
    float b;
};

static inline float min(float a, float b) { return a < b ? a : b; }
static inline float max(float a, float b) { return a > b ? a : b; }

static inline float median(float a, float b, float c) {
    return max(min(a, b), min(max(a, b), c));
}

static inline float resolve_multi_distance(multi_distance d) {
    return median(d.r, d.g, d.b);
}

/* distance_t signed_distance(segment *s, vec2 p, float *param); */
segment_distance signed_distance(segment *s, vec2 p);

static inline float dot(vec2 a, vec2 b) { return a.x * b.x + a.y * b.y; }

static inline float cross_(vec2 a, vec2 b) { return a.x * b.y - a.y * b.x; }

static inline float length(vec2 v) { return sqrt(v.x * v.x + v.y * v.y); }

static inline int sign(float n) { return 2 * (n > float(0)) - 1; }

static inline vec2 normalize(vec2 v) {
    float len = length(v);
    return vec2(v.x / len, v.y / len);
}

static inline vec2 orthonormal(vec2 v, bool polarity /* , bool allowZero = false */) {
    float len = length(v);
    return polarity ? vec2(-v.y / len, v.x / len) : vec2(v.y / len, -v.x / len);
}

static inline vec2 mix(vec2 a, vec2 b, float weight) {
    return vec2((float(1) - weight) * a + weight * b);
}
static inline vec2 segment_direction(segment *e, float param) {
    return mix(e->points[1] - e->points[0],
               e->points[e->npoints - 1] - e->points[e->npoints - 2],
               param);
}

static inline vec2 segment_point(segment *e, float param) {
    return mix(mix(e->points[0], e->points[1], param),
               mix(e->points[e->npoints - 2], e->points[e->npoints - 1], param),
               param);
}

#endif /* MSDF_LIB_H */
