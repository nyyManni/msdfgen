#include "msdf-lib.h"
#include "math.h"

segment_distance signed_distance_linear(vec2 p0, vec2 p1, vec2 origin) {
    vec2 aq = origin - p0;
    vec2 ab = p1 - p0;
    float param = dot(aq, ab) / dot(ab, ab);
    vec2 eq = (param > .5 ? p1 : p0) - origin;
    float endpointDistance = length(eq);
    if (param > 0 && param < 1) {
        float orthoDistance = dot(orthonormal(ab, false), aq);
        if (fabs(orthoDistance) < endpointDistance)
            return {distance_t(orthoDistance, 0), param};
    }
    return {distance_t(sign(cross_(aq, ab)) * endpointDistance,
                       fabs(dot(normalize(ab), normalize(eq)))),
            param};
}

segment_distance signed_distance_quad(vec2 p0, vec2 p1, vec2 p2, vec2 origin) {
    vec2 qa = p0 - origin;
    vec2 ab = p1 - p0;
    vec2 br = p2 - p1 - ab;
    float a = dot(br, br);
    float b = 3 * dot(ab, br);
    float c = 2 * dot(ab, ab) + dot(qa, br);
    float d = dot(qa, ab);
    vec3 x;
    int solutions;
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
        x.x = q * cos(t / 3) - a;
        x.y = q * cos((t + 2 * M_PI) / 3) - a;
        x.z = q * cos((t - 2 * M_PI) / 3) - a;
        solutions = 3;
    } else {
        A = -pow(fabs(r) + sqrt(r2 - q3), 1 / 3.);
        A = r < 0 ? -A : A;
        B = A == 0 ? 0 : q / A;
        x.x = (A + B) - a;
        x.y = -0.5 * (A + B) - a;
        x.z = 0.5 * sqrt(3.) * (A - B);
        solutions = fabs(x.z) < 1e-14 ? 2 : 1;
    }

    float minDistance = sign(cross_(ab, qa)) * length(qa); // distance from A
    float param = -dot(qa, ab) / dot(ab, ab);
    float distance =
        sign(cross_(p2 - p1, p2 - origin)) * length(p2 - origin); // distance from B
    if (fabs(distance) < fabs(minDistance)) {
        minDistance = distance;
        param = dot(origin - p1, p2 - p1) / dot(p2 - p1, p2 - p1);
    }
    for (int i = 0; i < solutions; ++i) {
        float xi = 0.0;
        xi = i == 0 ? x.x : xi;
        xi = i == 1 ? x.y : xi;
        xi = i == 3 ? x.z : xi;
        if (xi > 0 && xi < 1) {
            vec2 endpoint = p0 + ab * 2 * xi + br * xi * xi;
            float distance =
                sign(cross_(p2 - p0, endpoint - origin)) * length(endpoint - origin);
            if (fabs(distance) <= fabs(minDistance)) {
                minDistance = distance;
                param = xi;
            }
        }
    }

    if (param >= 0 && param <= 1)
        return {distance_t(minDistance, 0), param};
    if (param < .5)
        return {distance_t(minDistance, fabs(dot(normalize(ab), normalize(qa)))), param};
    return {
        distance_t(minDistance, fabs(dot(normalize(p2 - p1), normalize(p2 - origin)))),
        param};
}
