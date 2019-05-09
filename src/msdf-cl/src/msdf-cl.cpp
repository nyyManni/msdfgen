#include "msdf-lib.h"
#include "msdf.h"
#include "msdfgen-ext.h"
#include "msdfgen.h"
#include <Contour.h>
#include <Shape.h>
#include <edge-coloring.h>
#include <edge-segments.h>
#include <import-font.h>
#include <msdf.h>
#include <stdlib.h>


#define IDX_CURR  0
#define IDX_SHAPE 1
#define IDX_INNER 2
#define IDX_OUTER 3
#define IDX_RED   0
#define IDX_GREEN 1
#define IDX_BLUE  2

struct workspace {
    struct {
        segment_distance min_true;
        vec2 min_negative, min_positive;
        int nearest_points;
        int nearest_npoints;
    } segments[4 * 3];

    multi_distance max_inner;
    multi_distance max_outer;
    multi_distance min_absolute;
} ws;


vec2 *point_data;
unsigned char *metadata;


static inline float resolve_multi_distance(multi_distance d) {
    return median(d.r, d.g, d.b);
}

static inline vec3 to_pixel(multi_distance d, float range) {
    return vec3(d.r / range + 0.5f, d.g / range + 0.5f, d.b / range + 0.5f);
}

void add_segment(int prev_npoints, int prev_points,
                 int cur_npoints, int cur_points,
                 int next_npoints, int next_points,
                 int color, vec2 point);
void set_contour_edge(int, vec2);

bool less(vec2 a, vec2 b) {
    return fabs(a.x) < fabs(b.x) || (fabs(a.x) == fabs(b.x) && a.y < b.y);
}
void add_segment_true_distance(int segment_index, int npoints, int points, segment_distance d) {
    bool is_less = less(d.d, ws.segments[segment_index].min_true.d);
    ws.segments[segment_index].min_true = is_less ? d : ws.segments[segment_index].min_true;
    
    
    ws.segments[segment_index].nearest_points = is_less ? points :  ws.segments[segment_index].nearest_points;
    ws.segments[segment_index].nearest_npoints = is_less ? npoints :  ws.segments[segment_index].nearest_npoints;
}

segment_distance signed_distance_linear(vec2 p1, vec2 p2, vec2 origin);
segment_distance signed_distance_quad(vec2 p1, vec2 p2, vec2 p3, vec2 origin);

void add_segment_pseudo_distance(int segment_index, vec2 d) {
    vec2 *min_pseudo = d.x < 0 ? &(ws.segments[segment_index].min_negative) : &(ws.segments[segment_index].min_positive);
    *min_pseudo = less(d, *min_pseudo) ? d : *min_pseudo;
}

vec2 distance_to_pseudo_distance(int npoints, int points, segment_distance d, vec2 p) {
    if (d.param >= 0 && d.param <= 1)
        return d.d;

    vec2 dir = normalize(segment_direction(points, npoints, d.param < 0 ? 0 : 1));
    vec2 aq = p - segment_point(points, npoints, d.param < 0 ? 0 : 1);
    float ts = dot(aq, dir);
    if (d.param < 0 ? ts < 0 : ts > 0) {
        float pseudo_distance = cross_(aq, dir);
        if (fabs(pseudo_distance) <= fabs(d.d.x)) {
            d.d.x = pseudo_distance;
            d.d.y = 0;
        }
    }
    return d.d;
}

bool point_facing_edge(int prev_npoints, int prev_points,
                        int cur_npoints, int cur_points,
                        int next_npoints, int next_points, vec2 p, float param) {

    if (param >= 0 && param <= 1)
        return true;

    vec2 prev_edge_dir = -normalize(segment_direction(prev_points, prev_npoints, 1));
    vec2 edge_dir = normalize(segment_direction(cur_points, cur_npoints, param < 0 ? 0 : 1)) * (param < 0 ? 1 : -1);
    vec2 next_edge_dir = normalize(segment_direction(next_points, next_npoints, 0));
    vec2 point_dir = p - segment_point(cur_points, cur_npoints, param < 0 ? 0 : 1);
    return dot(point_dir, edge_dir) >=
           dot(point_dir, param < 0 ? prev_edge_dir : next_edge_dir);
}

multi_distance get_pixel_distance(vec2);

void calculate_pixel(vec3 *, int, int, int, vec2, vec2, float);

static inline vec2 Point2_to_vec2(msdfgen::Point2 p) { return vec2(p.x, p.y); }
int main() {
    fprintf(stderr, "ws size:%lu\n", sizeof(ws));

    msdfgen::Shape shape;
    msdf_font_handle f =
        msdf_load_font("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf");
    msdfgen::FontHandle *font = (msdfgen::FontHandle *)f->__handle;

    // msdfgen::loadGlyph(shape, font, '1');
    msdfgen::loadGlyph(shape, font, 0x00e4);
    // msdfgen::loadGlyph(shape, font, '#');
    // msdfgen::loadGlyph(shape, font, '0');
    // msdfgen::loadGlyph(shape, font, ' ');

    shape.normalize();
    edgeColoringSimple(shape, 3.0);


    size_t point_data_size = 0;
    size_t metadata_size = 1;
    for (msdfgen::Contour &c : shape.contours) {
        metadata_size += 2; /* winding + nsegments */
        for (msdfgen::EdgeHolder &e : c.edges) {
            metadata_size += 2; /* color + npoints */
            if (dynamic_cast<msdfgen::LinearSegment *>(e.edgeSegment)) {
                point_data_size += 2 * sizeof(vec2);
            } else if (dynamic_cast<msdfgen::QuadraticSegment *>(e.edgeSegment)) {
                point_data_size += 3 * sizeof(vec2);
            } else if (dynamic_cast<msdfgen::CubicSegment *>(e.edgeSegment)) {
                return -1;
            }
        }
    }
    point_data = (vec2 *)malloc(point_data_size);
    metadata = (unsigned char *)malloc(metadata_size);
    fprintf(stderr, "point data size: %lu\n", point_data_size);
    fprintf(stderr, "metadata size: %lu\n", metadata_size);
    size_t _p = 0;
    size_t _m = 0;

    {
        metadata[_m++] = shape.contours.size();
        for (msdfgen::Contour &_c : shape.contours) {
            metadata[_m++] = (unsigned char)_c.winding() + 1;
            metadata[_m++] = _c.edges.size();

            _p++;  /* The first segment should also have the first point */

            for (msdfgen::EdgeHolder &_e : _c.edges) {

                _p--; /* Each consecutive segment share one point */

                metadata[_m++] = _e->color;
                if (auto p = dynamic_cast<msdfgen::LinearSegment *>(_e.edgeSegment)) {
                    metadata[_m++] = 2;
                    point_data[_p++] = Point2_to_vec2(p->p[0]);
                    point_data[_p++] = Point2_to_vec2(p->p[1]);
                } else if (auto p = dynamic_cast<msdfgen::QuadraticSegment *>(
                               _e.edgeSegment)) {
                    metadata[_m++] = 3;
                    point_data[_p++] = Point2_to_vec2(p->p[0]);
                    point_data[_p++] = Point2_to_vec2(p->p[1]);
                    point_data[_p++] = Point2_to_vec2(p->p[2]);
                }
            }
        }
    }
    fprintf(stderr, "point data size (real): %lu\n", _p * sizeof(vec2));

    vec2 scale = {1.0, 1.0};
    vec2 translate = {0.0, 0.0};
    float range = 4.0;

    float width = font->face->glyph->metrics.width / 64.0;
    float height = font->face->glyph->metrics.height / 64.0;
    int w = ceil((width + range) * scale.x);
    int h = ceil((height + range) * scale.x);

    msdfgen::Bitmap<msdfgen::FloatRGB> msdf(w, h);
    msdfgen::generateMSDF(msdf, shape, range, 1.0, msdfgen::Vector2(0.0, 0.0), 1.001, true);

    vec3 *output = (vec3 *)malloc(h * w * sizeof(vec3));

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            calculate_pixel(output, x, y, w, scale, translate, range);
        }
    }

    return 0;
}

void calculate_pixel(vec3 *output, int x, int y, int stride,
                     vec2 scale, vec2 translate, float range) {
    vec2 p = vec2((x + 0.5f) / scale.x - translate.x, (y + 0.5f) / scale.y - translate.y);

    ws.max_inner.r = -INFINITY;
    ws.max_inner.g = -INFINITY;
    ws.max_inner.b = -INFINITY;
    ws.max_outer.r = -INFINITY;
    ws.max_outer.g = -INFINITY;
    ws.max_outer.b = -INFINITY;
    ws.min_absolute.r = -INFINITY;
    ws.min_absolute.g = -INFINITY;
    ws.min_absolute.b = -INFINITY;

    for (int _i = 0; _i < (4 * 3); ++_i) {
        ws.segments[_i].min_negative.x = -INFINITY;
        ws.segments[_i].min_negative.y = 1;
        ws.segments[_i].min_positive.x = -INFINITY;
        ws.segments[_i].min_positive.y = 1;
        ws.segments[_i].min_true.d.x = -INFINITY;
        ws.segments[_i].min_true.d.y = 1;
        ws.segments[_i].min_true.param = 0;
    }
    size_t point_index = 0;
    size_t meta_index = 0;

    unsigned char ncontours = metadata[meta_index++];

    for (int _i = 0; _i < ncontours; ++_i) {

        char winding = (char)metadata[meta_index++] - 1;
        unsigned char nsegments = metadata[meta_index++];

        if (!nsegments) continue;
        for (int _i = 0; _i < 3; ++_i) {
            ws.segments[_i].min_negative.x = -INFINITY;
            ws.segments[_i].min_negative.y = 1;
            ws.segments[_i].min_positive.x = -INFINITY;
            ws.segments[_i].min_positive.y = 1;
            ws.segments[_i].min_true.d.x = -INFINITY;
            ws.segments[_i].min_true.d.y = 1;
            ws.segments[_i].min_true.param = 0;
        }

        unsigned char s_color = metadata[meta_index + 0];
        unsigned char s_npoints = metadata[meta_index + 1];

        int cur_points = point_index;
        unsigned char cur_color = metadata[meta_index + 2 * (nsegments - 1) + 0];
        unsigned char cur_npoints = metadata[meta_index + 2 * (nsegments - 1) + 1];
        unsigned char prev_npoints = nsegments >= 2 ? metadata[meta_index + 2 * (nsegments - 2) + 1] : s_npoints;

        int prev_points = point_index;

        for (int _i = 0; _i < nsegments - 1; ++_i) {
            int npoints = metadata[meta_index + 2 * _i + 1];
            cur_points += npoints - 1;
        }

        for (int _i = 0; _i < nsegments - 2 && nsegments >= 2; ++_i) {
            int npoints = metadata[meta_index + 2 * _i + 1];
            prev_points += npoints - 1;
        }

        for (int _i = 0; _i < nsegments; ++_i) {

            add_segment(prev_npoints, prev_points, cur_npoints, cur_points, 
                        s_npoints, point_index, cur_color, p);

            prev_points = cur_points;
            prev_npoints = cur_npoints;
            cur_points = point_index;
            cur_npoints = s_npoints;
            cur_color = s_color;

            s_color = metadata[meta_index++ + 2];
            point_index += s_npoints - 1;
            s_npoints = metadata[meta_index++ + 2];
        }
        point_index += 1;

        set_contour_edge(winding, p);
    }

    multi_distance d = get_pixel_distance(p);
    vec3 pixel = to_pixel(d, range);
    printf("==> PIXEL: %.2f %.2f %.2f\n", pixel.r, pixel.g, pixel.b);
    output[y * stride + x] = pixel;
}

void add_segment(int prev_npoints, int prev_points,
                 int cur_npoints, int cur_points,
                 int next_npoints, int next_points,
                 int color, vec2 point) {

    segment_distance d;
    if (cur_npoints == 2)
        d = signed_distance_linear(point_data[cur_points],
                                   point_data[cur_points + 1], point);
    else
        d = signed_distance_quad(point_data[cur_points],
                                 point_data[cur_points + 1],
                                 point_data[cur_points + 2], point);

    if (color & RED)
        add_segment_true_distance(IDX_CURR * 3 + IDX_RED, cur_npoints, cur_points, d);
    if (color & GREEN)
        add_segment_true_distance(IDX_CURR * 3 + IDX_GREEN, cur_npoints, cur_points, d);
    if (color & BLUE)
        add_segment_true_distance(IDX_CURR * 3 + IDX_BLUE, cur_npoints, cur_points, d);

    if (point_facing_edge(prev_npoints, prev_points,
                           cur_npoints, cur_points,
                           next_npoints, next_points,
                           point, d.param)) {

        vec2 pd = distance_to_pseudo_distance(cur_npoints, cur_points, d, point);
        if (color & RED)
            add_segment_pseudo_distance(IDX_CURR * 3 + IDX_RED, pd);
        if (color & GREEN)
            add_segment_pseudo_distance(IDX_CURR * 3 + IDX_GREEN, pd);
        if (color & BLUE)
            add_segment_pseudo_distance(IDX_CURR * 3 + IDX_BLUE, pd);
    }
}

float compute_distance(int segment_index, vec2 point) {
    float min_distance = ws.segments[segment_index].min_true.d.x < 0 ?
        ws.segments[segment_index].min_negative.x : ws.segments[segment_index].min_positive.x;
    vec2 d = distance_to_pseudo_distance(ws.segments[segment_index].nearest_npoints,
                                                ws.segments[segment_index].nearest_points,
                                               ws.segments[segment_index].min_true, point);
    if (fabs(d.x) < fabs(min_distance))
        min_distance = d.x;
    return min_distance;
}

void merge_segment(int s, int other) {
    if (less(ws.segments[other].min_true.d, ws.segments[s].min_true.d)) {
        ws.segments[s].min_true = ws.segments[other].min_true;
        
        ws.segments[s].nearest_npoints = ws.segments[other].nearest_npoints;
        ws.segments[s].nearest_points = ws.segments[other].nearest_points;
    }
    if (less(ws.segments[other].min_negative, ws.segments[s].min_negative))
        ws.segments[s].min_negative = ws.segments[other].min_negative;
    if (less(ws.segments[other].min_positive, ws.segments[s].min_positive)) {
        ws.segments[s].min_positive = ws.segments[other].min_positive;
    }
}

void merge_multi_segment(int e, int other) {
    merge_segment(e * 3 + IDX_RED, other * 3 + IDX_RED);
    merge_segment(e * 3 + IDX_GREEN, other * 3 + IDX_GREEN);
    merge_segment(e * 3 + IDX_BLUE, other * 3 + IDX_BLUE);
}

multi_distance get_distance(int segment_index, vec2 point) {
    multi_distance d;
    d.r = compute_distance(segment_index * 3 + IDX_RED, point);
    d.g = compute_distance(segment_index * 3 + IDX_GREEN, point);
    d.b = compute_distance(segment_index * 3 + IDX_BLUE, point);
    return d;
}

void set_contour_edge(int winding, vec2 point) {

    multi_distance d = get_distance(IDX_CURR, point);

    merge_multi_segment(IDX_SHAPE, IDX_CURR);
    if (winding > 0 && resolve_multi_distance(d) >= 0)
        merge_multi_segment(IDX_INNER, IDX_CURR);
    if (winding < 0 && resolve_multi_distance(d) <= 0)
        merge_multi_segment(IDX_INNER, IDX_CURR);

    multi_distance *target = winding < 0 ? &ws.max_inner : &ws.max_outer;

    if (resolve_multi_distance(d) > resolve_multi_distance(*target))
        *target = d;

    if (fabs(resolve_multi_distance(d)) < fabs(resolve_multi_distance(ws.min_absolute)))
        ws.min_absolute = d;
}

multi_distance get_pixel_distance(vec2 point) {
    multi_distance shape_distance = get_distance(IDX_SHAPE, point);
    multi_distance inner_distance = get_distance(IDX_INNER, point);
    multi_distance outer_distance = get_distance(IDX_OUTER, point);
    float inner_d = resolve_multi_distance(inner_distance);
    float outer_d = resolve_multi_distance(outer_distance);

    bool inner = inner_d >= 0 && fabs(inner_d) <= fabs(outer_d);
    bool outer = outer_d <= 0 && fabs(outer_d) < fabs(inner_d);
    if (!inner && !outer)
        return shape_distance;

    multi_distance d = inner ? inner_distance : outer_distance;
    multi_distance contour_distance = inner ? ws.max_inner : ws.max_outer;

    float contour_d = resolve_multi_distance(contour_distance);
    if (fabs(contour_d) < fabs(outer_d) && contour_d > resolve_multi_distance(d))
        d = contour_distance;

    contour_distance = ws.min_absolute;
    contour_d = resolve_multi_distance(contour_distance);
    float d_d = resolve_multi_distance(d);

    if (fabs(contour_d) < fabs(d_d))
        d = contour_distance;

    if (resolve_multi_distance(d) == resolve_multi_distance(shape_distance))
        d = shape_distance;

    return d;
}

segment_distance signed_distance_linear(vec2 p0, vec2 p1, vec2 origin) {
    vec2 aq = origin - p0;
    vec2 ab = p1 - p0;
    float param = dot(aq, ab) / dot(ab, ab);
    vec2 eq = (param > .5 ? p1 : p0) - origin;
    float endpointDistance = length(eq);
    if (param > 0 && param < 1) {
        float orthoDistance = dot(orthonormal(ab, false), aq);
        if (fabs(orthoDistance) < endpointDistance)
            return {vec2(orthoDistance, 0), param};
    }
    return {vec2(sign(cross_(aq, ab)) * endpointDistance,
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
    float tttt[3];
    float _a = b / a;
    int solutions;

    float a2 = _a * _a;
    float q = (a2 - 3 * (c / a)) / 9;
    float r = (_a * (2 * a2 - 9 * (c / a)) + 27 * (d / a)) / 54;
    float r2 = r * r;
    float q3 = q * q * q;
    float A, B;
    _a /= 3;
    float t = r / sqrt(q3);
    t = t < -1 ? -1 : t;
    t = t > 1 ? 1 : t;
    t = acos(t);
    A = -pow(fabs(r) + sqrt(r2 - q3), 1 / 3.);
    A = r < 0 ? -A : A;
    B = A == 0 ? 0 : q / A;
    if (r2 < q3) {
        q = -2 * sqrt(q);
        tttt[0] = q * cos(t / 3) - _a;
        tttt[1] = q * cos((t + 2 * M_PI) / 3) - _a;
        tttt[2] = q * cos((t - 2 * M_PI) / 3) - _a;
        solutions =  3;
    } else {
        tttt[0] = (A + B) - _a;
        tttt[1] = -0.5 * (A + B) - _a;
        tttt[2] = 0.5 * sqrt(3.) * (A - B);
        solutions = fabs(tttt[2]) < 1e-14 ? 2 : 1;
    }

    float minDistance = sign(cross_(ab, qa)) * length(qa); // distance from A
    float param = -dot(qa, ab) / dot(ab, ab);
    float distance = sign(cross_(p2 - p1, p2 - origin)) * length(p2 - origin); // distance from B
    if (fabs(distance) < fabs(minDistance)) {
        minDistance = distance;
        param = dot(origin - p1, p2 - p1) / dot(p2 - p1, p2 - p1);
    }
    for (int i = 0; i < solutions; ++i)
        {
        if (tttt[i] > 0 && tttt[i] < 1) {
            vec2 endpoint = p0 + ab * 2 * tttt[i] + br * tttt[i] * tttt[i];
            float distance =
                sign(cross_(p2 - p0, endpoint - origin)) *
                length(endpoint - origin);
            if (fabs(distance) <= fabs(minDistance)) {
                minDistance = distance;
                param = tttt[i];
            }
        }
    }

    if (param >= 0 && param <= 1)
        return {vec2(minDistance, 0), param};
    if (param < .5)
        return {vec2(minDistance, fabs(dot(normalize(ab), normalize(qa)))), param};
    return {vec2(minDistance, fabs(dot(normalize(p2 - p1),
                                             normalize(p2 - origin)))), param};
}
