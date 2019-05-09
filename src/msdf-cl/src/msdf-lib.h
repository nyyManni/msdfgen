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


typedef struct vec2 {
    float x;
    float y;

    vec2() : x(0), y(0) {}
    vec2(float x, float y) : x(x), y(y) {}

    struct vec2 operator-() {
        return vec2(-x, -y);
    }

    struct vec2 operator+(struct vec2 other) {
        return vec2(x + other.x, y + other.y);
    }

    struct vec2 operator-(struct vec2 other) {
        return vec2(x - other.x, y - other.y);
    }

    struct vec2 operator*(float value) {
        return vec2(x * value, y * value);
    }
} vec2;


extern vec2 *point_data;

typedef struct vec3 {
    float r;
    float g;
    float b;

    vec3() : r(0), g(0), b(0) {}
    vec3(float x, float y, float z) : r(x), g(y), b(z) {}
} vec3;


typedef struct segment_distance {
    vec2 d;
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

static inline vec2 segment_direction(int points, int npoints, float param) {
    return mix(point_data[points + 1] - point_data[points],
               point_data[points + npoints - 1] - point_data[points + npoints - 2],
               param);
}

static inline vec2 segment_point(int points, int npoints, float param) {
    return mix(mix(point_data[points], point_data[points + 1], param),
               mix(point_data[points + npoints - 2], point_data[points + npoints - 1], param),
               param);
}

#endif /* MSDF_LIB_H */
