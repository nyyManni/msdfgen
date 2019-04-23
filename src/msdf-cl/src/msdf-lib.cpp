#include <stdio.h>

#include "msdf-lib.h"
#include "math.h"
#define MSDFGEN_CUBIC_SEARCH_STARTS 4
#define MSDFGEN_CUBIC_SEARCH_STEPS 4

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
        dscr = sqrt(dscr);
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
        float t = r / sqrt(q3);
        if (t < -1)
            t = -1;
        if (t > 1)
            t = 1;
        t = acos(t);
        a /= 3;
        q = -2 * sqrt(q);
        x[0] = q * cos(t / 3) - a;
        x[1] = q * cos((t + 2 * M_PI) / 3) - a;
        x[2] = q * cos((t - 2 * M_PI) / 3) - a;
        return 3;
    } else {
        A = -pow(fabs(r) + sqrt(r2 - q3), 1 / 3.);
        if (r < 0)
            A = -A;
        B = A == 0 ? 0 : q / A;
        a /= 3;
        x[0] = (A + B) - a;
        x[1] = -0.5 * (A + B) - a;
        x[2] = 0.5 * sqrt(3.) * (A - B);
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

distance_t signed_distance_linear(segment *s, vec2 origin, float *param);
distance_t signed_distance_quad(segment *s, vec2 origin, float *param);
distance_t signed_distance_cubic(segment *s, vec2 origin, float *param);
distance_t signed_distance(segment *s, vec2 p, float *param) {
    if (s->npoints == 2)
        return signed_distance_linear(s, p, param);
    if (s->npoints == 3)
        return signed_distance_quad(s, p, param);
    return signed_distance_cubic(s, p, param);
}

distance_t signed_distance_linear(segment *s, vec2 origin, float *param) {
    vec2 aq = origin - s->points[0];
    vec2 ab = s->points[1] - s->points[0];
    *param = dotProduct(aq, ab) / dotProduct(ab, ab);
    vec2 eq = s->points[*param > .5] - origin;
    float endpointDistance = length(eq);
    if (*param > 0 && *param < 1) {
        float orthoDistance = dotProduct(getOrthonormal(ab, false), aq);
        if (fabs(orthoDistance) < endpointDistance)
            return distance_t(orthoDistance, 0);
    }
    return distance_t(nonZeroSign(crossProduct(aq, ab)) * endpointDistance,
                      fabs(dotProduct(normalize(ab), normalize(eq))));
}


distance_t signed_distance_quad(segment *s, vec2 origin, float *param) {
    vec2 qa = s->points[0] - origin;
    vec2 ab = s->points[1] - s->points[0];
    vec2 br = s->points[2] - s->points[1] - ab;
    float a = dotProduct(br, br);
    float b = 3 * dotProduct(ab, br);
    float c = 2 * dotProduct(ab, ab) + dotProduct(qa, br);
    float d = dotProduct(qa, ab);
    float t[3];
    int solutions = solve_cubic(t, a, b, c, d);

    float minDistance = nonZeroSign(crossProduct(ab, qa)) * length(qa); // distance from A
    *param = -dotProduct(qa, ab) / dotProduct(ab, ab);
    {
        float distance = nonZeroSign(crossProduct(s->points[2] - s->points[1],
                                                  s->points[2] - origin)) *
                         length(s->points[2] - origin); // distance from B
        if (fabs(distance) < fabs(minDistance)) {
            minDistance = distance;
            *param = dotProduct(origin - s->points[1], s->points[2] - s->points[1]) /
                     dotProduct(s->points[2] - s->points[1], s->points[2] - s->points[1]);
        }
    }
    for (int i = 0; i < solutions; ++i) {
        if (t[i] > 0 && t[i] < 1) {
            vec2 endpoint = s->points[0] + 2 * t[i] * ab + t[i] * t[i] * br;
            float distance = nonZeroSign(crossProduct(s->points[2] - s->points[0],
                                                      endpoint - origin)) *
                             length(endpoint - origin);
            if (fabs(distance) <= fabs(minDistance)) {
                minDistance = distance;
                *param = t[i];
            }
        }
    }

    if (*param >= 0 && *param <= 1)
        return distance_t(minDistance, 0);
    if (*param < .5)
        return distance_t(minDistance, fabs(dotProduct(normalize(ab), normalize(qa))));
    else
        return distance_t(minDistance,
                          fabs(dotProduct(normalize(s->points[2] - s->points[1]),
                                          normalize(s->points[2] - origin))));
}

vec2 segment_direction(segment *e, float param) {
    if (e->npoints == 2) return e->points[1] - e->points[0];
    if (e->npoints == 3) {
        return mix(e->points[1]-e->points[0], e->points[2]-e->points[1], param);
    }
    
    vec2 tangent = mix(mix(e->points[1]-e->points[0], 
                           e->points[2]-e->points[1], param), 
                       mix(e->points[2]-e->points[1], 
                           e->points[3]-e->points[2], param), param);

    if (!tangent) {
        if (param == 0) return e->points[2]-e->points[0];
        if (param == 1) return e->points[3]-e->points[1];
    }
    return tangent;
}
vec2 segment_point(segment *e, float param) {
    if (e->npoints == 2) return mix(e->points[0], e->points[1], param);
    if (e->npoints == 3) {
        return mix(mix(e->points[0], 
                       e->points[1], param), 
                   mix(e->points[1], 
                       e->points[2], param), param);
    }
    vec2 p12 = mix(e->points[1], e->points[2], param);
    return mix(mix(mix(e->points[0], 
                       e->points[1], param), 
                   p12, param), 
               mix(p12, 
                   mix(e->points[2], 
                       e->points[3], param), param), param);
}

distance_t signed_distance_cubic(segment *e, vec2 origin, float *param) {
    vec2 qa = e->points[0] - origin;
    vec2 ab = e->points[1] - e->points[0];
    vec2 br = e->points[2] - e->points[1] - ab;
    vec2 as = (e->points[3] - e->points[2]) - (e->points[2] - e->points[1]) - br;

    vec2 epDir = segment_direction(e, 0);
    float minDistance =
        nonZeroSign(crossProduct(epDir, qa)) * length(qa); // distance from A
    *param = -dotProduct(qa, epDir) / dotProduct(epDir, epDir);
    {
        epDir = segment_direction(e, 1);
        float distance = nonZeroSign(crossProduct(epDir, e->points[3] - origin)) *
                         length(e->points[3] - origin); // distance from B
        if (fabs(distance) < fabs(minDistance)) {
            minDistance = distance;
            *param = dotProduct(origin + epDir - e->points[3], epDir) / dotProduct(epDir, epDir);
        }
    }
    // Iterative minimum distance search
    for (int i = 0; i <= MSDFGEN_CUBIC_SEARCH_STARTS; ++i) {
        float t = (float)i / MSDFGEN_CUBIC_SEARCH_STARTS;
        for (int step = 0;; ++step) {
            vec2 qpt = segment_point(e, t) - origin;
            float distance = nonZeroSign(crossProduct(segment_direction(e, t), qpt)) * length(qpt);
            if (fabs(distance) < fabs(minDistance)) {
                minDistance = distance;
                *param = t;
            }
            if (step == MSDFGEN_CUBIC_SEARCH_STEPS)
                break;
            // Improve t
            vec2 d1 = 3 * as * t * t + 6 * br * t + 3 * ab;
            vec2 d2 = 6 * as * t + 6 * br;
            t -= dotProduct(qpt, d1) / (dotProduct(d1, d1) + dotProduct(qpt, d2));
            if (t < 0 || t > 1)
                break;
        }
    }

    if (*param >= 0 && *param <= 1)
        return distance_t(minDistance, 0);
    if (*param < .5)
        return distance_t(minDistance,
                          fabs(dotProduct(normalize(segment_direction(e, 0)), normalize(qa))));
    else
        return distance_t(minDistance, fabs(dotProduct(normalize(segment_direction(e, 1)),
                                                       normalize(e->points[3] - origin))));
}
