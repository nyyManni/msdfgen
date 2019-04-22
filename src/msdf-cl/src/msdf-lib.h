#ifndef MSDF_LIB_H
#define MSDF_LIB_H

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

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

    bool operator!() {
        return !x && !y;
    }
    bool operator==( struct _vec2 other)  {
        return x == other.x && y == other.y;
    }

    bool operator!=( struct _vec2 other)  {
        return x != other.x || y != other.y;
    }

    struct _vec2 operator+()  {
        return *this;
    }

    struct _vec2 operator-()  {
        return _vec2(-x, -y);
    }

    struct _vec2 operator+( struct _vec2 other)  {
        return _vec2(x + other.x, y + other.y);
    }

    struct _vec2 operator-( struct _vec2 other)  {
        return _vec2(x - other.x, y - other.y);
    }

    struct _vec2 operator*( struct _vec2 other)  {
        return _vec2(x * other.x, y * other.y);
    }

    struct _vec2 operator/( struct _vec2 other)  {
        return _vec2(x / other.x, y / other.y);
    }

    struct _vec2 operator*(float value)  {
        return _vec2(x * value, y * value);
    }

    struct _vec2 operator/(float value)  {
        return _vec2(x / value, y / value);
    }

    struct _vec2 operator+=( struct _vec2 other) {
        x += other.x, y += other.y;
        return *this;
    }

    struct _vec2 operator-=( struct _vec2 other) {
        x -= other.x, y -= other.y;
        return *this;
    }

    struct _vec2 operator*=( struct _vec2 other) {
        x *= other.x, y *= other.y;
        return *this;
    }

    struct _vec2 operator/=( struct _vec2 other) {
        x /= other.x, y /= other.y;
        return *this;
    }

    struct _vec2 operator*=(float value) {
        x *= value, y *= value;
        return *this;
    }

    struct _vec2 operator/=(float value) {
        x /= value, y /= value;
        return *this;
    }

} vec2;
static inline struct _vec2 operator*(float value,  struct _vec2 vector) {
    return _vec2(value*vector.x, value*vector.y);
}
static inline struct _vec2 operator/(float value,  struct _vec2 vector) {
    return _vec2(value/vector.x, value/vector.y);
}

typedef struct _vec3 {
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

/* typedef struct _distance { */
/*     float distance; */
/*     float dot; */
/* } distance_t; */
typedef vec2 distance_t;

struct multi_distance {
    float r;
    float g;
    float b;
};

static inline float min(float a, float b) {
    return a < b ? a : b;
}
static inline float max(float a, float b) {
    return a > b ? a : b;
}

static inline float median(float a, float b, float c) {
    return max(min(a, b), min(max(a, b), c));
}

static inline float resolve_multi_distance(multi_distance d) {
    return median(d.r, d.g, d.b);
}

distance_t signed_distance(segment *s, vec2 p, float *param);
vec2 segment_direction(segment *e, float param);
vec2 segment_point(segment *e, float param);
void dump_segment(segment *s);
static inline void dump_vec2(vec2 *vec) {
    printf("vec2: %.2f, %.2f\n", vec->x, vec->y);
}

static inline float dotProduct( vec2 a,  vec2 b) {
    return a.x*b.x+a.y*b.y;
}

static inline float crossProduct( vec2 a,  vec2 b) {
    return a.x*b.y-a.y*b.x;
}

static inline float length( vec2 v) {
    return sqrt(v.x*v.x+v.y*v.y);
}

static inline int nonZeroSign(float n) {
    return 2*(n > float(0))-1;
}

static inline vec2 normalize(vec2 v, bool allowZero = false) {
    float len = length(v);
    if (len == 0)
        return vec2(0, !allowZero);
    return vec2(v.x/len, v.y/len);
}

static inline vec2 getOrthogonal(vec2 v, bool polarity) {
    return polarity ? vec2(-v.y, v.x) : vec2(v.y, -v.x);
}

static inline vec2 getOrthonormal(vec2 v, bool polarity, bool allowZero = false) {
    float len = length(v);
    if (len == 0)
        return polarity ? vec2(0, !allowZero) : vec2(0, -!allowZero);
    return polarity ? vec2(-v.y/len, v.x/len) : vec2(v.y/len, -v.x/len);
}

/* template <typename T, typename S> */
static inline vec2 mix(vec2 a, vec2 b, float weight) {
    return vec2((float(1)-weight)*a+weight*b);
}

#endif /* MSDF_LIB_H */
