#include "math.h"
#include "msdf-lib.h"

segment_distance signed_distance_linear(segment *s, vec2 origin);
segment_distance signed_distance_quad(segment *s, vec2 origin);

static int solve_cubic_normed(float x[3], float a, float b, float c) {
    float a2 = a * a;
    float q = (a2 - 3 * b) / 9;
    float r = (a * (2 * a2 - 9 * b) + 27 * c) / 54;
    float r2 = r * r;
    float q3 = q * q * q;
    float A, B;
    a /= 3;
    if (r2 < q3) {
        float t = r / sqrt(q3);
        t = t < -1 ? -1 : t;
        t = t > 1 ? 1 : t;
        t = acos(t);
        q = -2 * sqrt(q);
        x[0] = q * cos(t / 3) - a;
        x[1] = q * cos((t + 2 * M_PI) / 3) - a;
        x[2] = q * cos((t - 2 * M_PI) / 3) - a;
        return 3;
    } else {
        A = -pow(fabs(r) + sqrt(r2 - q3), 1 / 3.);
        A = r < 0 ? -A : A;
        B = A == 0 ? 0 : q / A;
        x[0] = (A + B) - a;
        x[1] = -0.5 * (A + B) - a;
        x[2] = 0.5 * sqrt(3.) * (A - B);
        return fabs(x[2]) < 1e-14 ? 2 : 1;
    }
}

segment_distance signed_distance(segment *s, int npoints, vec2 p) {
    if (npoints == 2)
        return signed_distance_linear(s, p);
    return signed_distance_quad(s, p);
}

segment_distance signed_distance_linear(segment *s, vec2 origin) {
    vec2 aq = origin - s->points[0];
    vec2 ab = s->points[1] - s->points[0];
    float param = dot(aq, ab) / dot(ab, ab);
    vec2 eq = s->points[param > .5] - origin;
    float endpointDistance = length(eq);
    if (param > 0 && param < 1) {
        float orthoDistance = dot(orthonormal(ab, false), aq);
        if (fabs(orthoDistance) < endpointDistance)
            return {distance_t(orthoDistance, 0), param};
    }
    return {distance_t(sign(cross_(aq, ab)) * endpointDistance,
                       fabs(dot(normalize(ab), normalize(eq)))), param};
}

segment_distance signed_distance_quad(segment *s, vec2 origin) {
    vec2 qa = s->points[0] - origin;
    vec2 ab = s->points[1] - s->points[0];
    vec2 br = s->points[2] - s->points[1] - ab;
    float a = dot(br, br);
    float b = 3 * dot(ab, br);
    float c = 2 * dot(ab, ab) + dot(qa, br);
    float d = dot(qa, ab);
    float t[3];
    int solutions = solve_cubic_normed(t, b / a, c / a, d / a);

    float minDistance = sign(cross_(ab, qa)) * length(qa); // distance from A
    float param = -dot(qa, ab) / dot(ab, ab);
    float distance = sign(cross_(s->points[2] - s->points[1],
                                    s->points[2] - origin)) *
                        length(s->points[2] - origin); // distance from B
    if (fabs(distance) < fabs(minDistance)) {
        minDistance = distance;
        param = dot(origin - s->points[1], s->points[2] - s->points[1]) /
                    dot(s->points[2] - s->points[1], s->points[2] - s->points[1]);
    }
    for (int i = 0; i < solutions; ++i) {
        if (t[i] > 0 && t[i] < 1) {
            vec2 endpoint = s->points[0] + ab * 2 * t[i] + br * t[i] * t[i];
            float distance =
                sign(cross_(s->points[2] - s->points[0], endpoint - origin)) *
                length(endpoint - origin);
            if (fabs(distance) <= fabs(minDistance)) {
                minDistance = distance;
                param = t[i];
            }
        }
    }

    if (param >= 0 && param <= 1)
        return {distance_t(minDistance, 0), param};
    if (param < .5)
        return {distance_t(minDistance, fabs(dot(normalize(ab), normalize(qa)))), param};
    return {distance_t(minDistance, fabs(dot(normalize(s->points[2] - s->points[1]),
                                             normalize(s->points[2] - origin)))), param};
}

