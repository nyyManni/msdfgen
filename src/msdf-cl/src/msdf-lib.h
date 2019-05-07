#ifndef MSDF_LIB_H
#define MSDF_LIB_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

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


extern vec2 *point_data;

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


/* segment_distance signed_distance_linear(segment *s, vec2 origin); */
/* segment_distance signed_distance_quad(segment *s, vec2 origin); */

segment_distance signed_distance_linear(vec2 p1, vec2 p2, vec2 origin);
segment_distance signed_distance_quad(vec2 p1, vec2 p2, vec2 p3, vec2 origin);
/* segment_distance signed_distance(segment *s, int, vec2 p); */
/* segment_distance signed_distance2(segment *s, int, int, vec2 p); */
/* segment_distance signed_distance_linear2(vec2 p1, vec2 p2, vec2 origin); */
/* segment_distance signed_distance_quad2(vec2 p1, vec2 p2, vec2 p3, vec2 origin); */

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
    return vec2(a * (float(1) - weight) + b * weight);
}
static inline vec2 segment_direction(vec2 *points, int npoints, float param) {
    return mix(points[1] - points[0],
               points[npoints - 1] - points[npoints - 2],
               param);
}

static inline vec2 segment_point(vec2 *points, int npoints, float param) {
    return mix(mix(points[0], points[1], param),
               mix(points[npoints - 2], points[npoints - 1], param),
               param);
}

#endif /* MSDF_LIB_H */
