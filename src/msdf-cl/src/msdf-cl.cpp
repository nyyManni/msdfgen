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
        distance_t min_negative, min_positive;
        segment *nearest_segment;
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

void add_segment(segment *, segment *, segment *, int, int, int, vec2);
void set_contour_edge(int, vec2);

bool less(distance_t a, distance_t b) {
    return fabs(a.x) < fabs(b.x) || (fabs(a.x) == fabs(b.x) && a.y < b.y);
}
void add_segment_true_distance(int segment_index, segment *s, segment_distance d) {
    bool is_less = less(d.d, ws.segments[segment_index].min_true.d);
    ws.segments[segment_index].min_true = is_less ? d : ws.segments[segment_index].min_true;
    ws.segments[segment_index].nearest_segment = is_less ? s : ws.segments[segment_index].nearest_segment;
    // ws.segments[segment_index].nearest_points = is_less ? s->points_idx : ws.segments[segment_index].nearest_points_idx;
    ws.segments[segment_index].nearest_npoints = is_less ? s->npoints : ws.segments[segment_index].nearest_npoints;
}

void add_segment_pseudo_distance(int segment_index, distance_t d) {
    distance_t *min_pseudo = d.x < 0 ? &(ws.segments[segment_index].min_negative) : &(ws.segments[segment_index].min_positive);
    *min_pseudo = less(d, *min_pseudo) ? d : *min_pseudo;
}

distance_t distance_to_pseudo_distance(segment *s, segment_distance d, vec2 p) {
    if (d.param >= 0 && d.param <= 1)
        return d.d;

    vec2 dir = normalize(segment_direction(s->points, s->npoints, d.param < 0 ? 0 : 1));
    vec2 aq = p - segment_point(s->points, s->npoints, d.param < 0 ? 0 : 1);
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

bool point_facing_edge(segment *prev, segment *cur, segment *next, vec2 p, float param) {
    if (param >= 0 && param <= 1)
        return true;

    vec2 prev_edge_dir = -normalize(segment_direction(prev->points, prev->npoints, 1));
    vec2 edge_dir = normalize(segment_direction(cur->points, cur->npoints, param < 0 ? 0 : 1)) * (param < 0 ? 1 : -1);
    vec2 next_edge_dir = normalize(segment_direction(next->points, next->npoints, 0));
    vec2 point_dir = p - segment_point(cur->points, cur->npoints, param < 0 ? 0 : 1);
    return dot(point_dir, edge_dir) >=
           dot(point_dir, param < 0 ? prev_edge_dir : next_edge_dir);
}
// bool point_facing_edge2(segment *prev, segment *cur, segment *next, vec2 p, float param) {
//     if (param >= 0 && param <= 1)
//         return true;

//     vec2 prev_edge_dir = -normalize(segment_direction(prev->points, prev->npoints, 1));
//     vec2 edge_dir = normalize(segment_direction(cur->points, cur->npoints, param < 0 ? 0 : 1)) * (param < 0 ? 1 : -1);
//     vec2 next_edge_dir = normalize(segment_direction(next->points, next->npoints, 0));
//     vec2 point_dir = p - segment_point(cur->points, cur->npoints, param < 0 ? 0 : 1);
//     return dot(point_dir, edge_dir) >=
//            dot(point_dir, param < 0 ? prev_edge_dir : next_edge_dir);
// }

multi_distance get_pixel_distance(vec2);

void calculate_pixel(struct shape *, vec3 *, int, int, int, vec2, vec2, float);

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
    size_t input_size = sizeof(struct shape);
    for (msdfgen::Contour &c : shape.contours) {
        input_size += sizeof(contour);
        metadata_size += 2; /* winding + nsegments */
        for (msdfgen::EdgeHolder &e : c.edges) {
            metadata_size += 2; /* color + npoints */

            input_size += sizeof(segment);
            if (dynamic_cast<msdfgen::LinearSegment *>(e.edgeSegment)) {
                point_data_size += 2 * sizeof(vec2);
                input_size += 2 * sizeof(vec2);
            } else if (dynamic_cast<msdfgen::QuadraticSegment *>(e.edgeSegment)) {
                point_data_size += 3 * sizeof(vec2);
                input_size += 3 * sizeof(vec2);
            } else if (dynamic_cast<msdfgen::CubicSegment *>(e.edgeSegment)) {
                return -1;
                // input_size += 4 * sizeof(vec2);
            }
        }
    }
    void *input_buffer = malloc(input_size);
    point_data = (vec2 *)malloc(point_data_size);
    metadata = (unsigned char *)malloc(metadata_size);
    fprintf(stderr, "point data size: %lu\n", point_data_size);
    fprintf(stderr, "metadata size: %lu\n", metadata_size);
    size_t _p = 0;
    size_t _m = 0;

    struct shape *glyph_data = (struct shape *)input_buffer;
    {
        metadata[_m++] = shape.contours.size();
        glyph_data->ncontours = shape.contours.size();
        contour *c = glyph_data->contours;
        for (msdfgen::Contour &_c : shape.contours) {
            metadata[_m++] = (unsigned char)_c.winding() + 1;
            metadata[_m++] = _c.edges.size();
            c->nsegments = _c.edges.size();
            c->winding = _c.winding();

            _p++;  /* The first segment should also have the first point */

            segment *s = c->segments;
            for (msdfgen::EdgeHolder &_e : _c.edges) {

                _p--; /* Each consecutive segment share one point */

                metadata[_m++] = _e->color;
                s->color = (unsigned char)_e->color;
                if (auto p = dynamic_cast<msdfgen::LinearSegment *>(_e.edgeSegment)) {
                    metadata[_m++] = 2;
                    s->npoints = 2;
                    s->points[0] = Point2_to_vec2(p->p[0]);
                    s->points[1] = Point2_to_vec2(p->p[1]);
                    point_data[_p++] = Point2_to_vec2(p->p[0]);
                    point_data[_p++] = Point2_to_vec2(p->p[1]);
                } else if (auto p = dynamic_cast<msdfgen::QuadraticSegment *>(
                               _e.edgeSegment)) {
                    metadata[_m++] = 3;
                    s->npoints = 3;
                    s->points[0] = Point2_to_vec2(p->p[0]);
                    s->points[1] = Point2_to_vec2(p->p[1]);
                    s->points[2] = Point2_to_vec2(p->p[2]);
                    point_data[_p++] = Point2_to_vec2(p->p[0]);
                    point_data[_p++] = Point2_to_vec2(p->p[1]);
                    point_data[_p++] = Point2_to_vec2(p->p[2]);
                // } else if (auto p =
                //                dynamic_cast<msdfgen::CubicSegment *>(_e.edgeSegment)) {
                //     s->npoints = 4;
                //     s->points[0] = Point2_to_vec2(p->p[0]);
                //     s->points[1] = Point2_to_vec2(p->p[1]);
                //     s->points[2] = Point2_to_vec2(p->p[2]);
                //     s->points[3] = Point2_to_vec2(p->p[3]);
                }
                /* Move s to the beginning of the next segment */
                s = (segment *)(((vec2 *)(s + 1)) + s->npoints);
            }
            /* s already points to the following contour in the list */
            c = (contour *)s;
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
            calculate_pixel(glyph_data, output, x, y, w, scale, translate, range);
        }
    }

    return 0;
}

void calculate_pixel(struct shape *shape, vec3 *output, int x, int y, int stride,
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
        ws.segments[_i].nearest_segment = NULL;
    }
    size_t point_index = 0;
    size_t meta_index = 0;

    unsigned char ncontours = metadata[meta_index++];

    contour *c = shape->contours;
    for (int _i = 0; _i < shape->ncontours; ++_i) {
        
        char winding = (char)metadata[meta_index++] - 1;
        unsigned char nsegments = metadata[meta_index++];

        if (!c->nsegments) {
            c += 1;
            continue;
        }
        for (int _i = 0; _i < 3; ++_i) {
            ws.segments[_i].min_negative.x = -INFINITY;
            ws.segments[_i].min_negative.y = 1;
            ws.segments[_i].min_positive.x = -INFINITY;
            ws.segments[_i].min_positive.y = 1;
            ws.segments[_i].min_true.d.x = -INFINITY;
            ws.segments[_i].min_true.d.y = 1;
            ws.segments[_i].min_true.param = 0;
            ws.segments[_i].nearest_segment = NULL;
        }


        segment *s = c->segments;

        /* Initialize cur to the last segment in the list */
        segment *cur = s;
        for (int _i = 0; _i < c->nsegments - 1; ++_i)
            NEXT_SEGMENT(cur);


        /*
         * Initialize prev to the second last segment in the list, or the first
         * one if there are less than two segments.
         */
        segment *prev = s;
        for (int _i = 0; _i < c->nsegments - 2 && c->nsegments >= 2; ++_i)
            NEXT_SEGMENT(prev);

        // return;
        // int s_points = point_index;
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

        fprintf(stderr, "=============================================\n");
        for (int _i = 0; _i < c->nsegments; ++_i) {

            fprintf(stderr, "=============================================\n");
            if (prev->npoints == 2) {
                fprintf(stderr, "s: %.2f %.2f, %.2f %.2f\n",
                        prev->points[0].x, prev->points[0].y,
                        prev->points[1].x, prev->points[1].y);
            } else {
                fprintf(stderr, "s: %.2f %.2f, %.2f %.2f, %.2f %.2f\n",
                        prev->points[0].x, prev->points[0].y,
                        prev->points[1].x, prev->points[1].y,
                        prev->points[2].x, prev->points[2].y);
            }
            if (prev_npoints == 2) {
                fprintf(stderr, "s: %.2f %.2f, %.2f %.2f\n",
                        point_data[prev_points].x, point_data[prev_points].y,
                        point_data[prev_points + 1].x, point_data[prev_points + 1].y);
            } else {
                fprintf(stderr, "s: %.2f %.2f, %.2f %.2f, %.2f %.2f\n",
                        point_data[prev_points].x, point_data[prev_points].y,
                        point_data[prev_points + 1].x, point_data[prev_points + 1].y,
                        point_data[prev_points + 2].x, point_data[prev_points + 2].y);
            }
            add_segment(prev, cur, s, cur_points, cur->npoints, cur->color, p);
            prev = cur;
            cur = s;
            NEXT_SEGMENT(s);
            
            
            prev_points = cur_points;
            prev_npoints = cur_npoints;
            cur_points = point_index;
            cur_npoints = s_npoints;
            cur_color = s_color;
            
            s_color = metadata[meta_index++ + 2];
            point_index += s_npoints - 1;
            s_npoints = metadata[meta_index++ + 2];
            // s_points += s_npoints;
        }
        point_index += 1;

        set_contour_edge(winding, p);

        /* s now points to the next contour structure (if any) */
        c = (contour *)s;
    }

    multi_distance d = get_pixel_distance(p);
    vec3 pixel = to_pixel(d, range);
    printf("==> PIXEL: %.2f %.2f %.2f\n", pixel.r, pixel.g, pixel.b);
    output[y * stride + x] = pixel;
}

void add_segment(segment *prev, segment *cur, segment *next,
                 int cur_points,
                 int npoints, int color, vec2 point) {

    segment_distance d;
    if (npoints == 2)
        d = signed_distance_linear(cur->points[0], cur->points[1], point);
        // d = signed_distance_linear(point_data[cur_points], point_data[cur_points + 1], point);
    else
        d = signed_distance_quad(cur->points[0], cur->points[1], cur->points[2], point);
        // d = signed_distance_quad(point_data[cur_points], point_data[cur_points + 1], point_data[cur_points + 2], point);


    if (color & RED)
        add_segment_true_distance(IDX_CURR * 3 + IDX_RED, cur, d);
    if (color & GREEN)
        add_segment_true_distance(IDX_CURR * 3 + IDX_GREEN, cur, d);
    if (color & BLUE)
        add_segment_true_distance(IDX_CURR * 3 + IDX_BLUE, cur, d);

    if (point_facing_edge(prev, cur, next, point, d.param)) {

        distance_t pd = distance_to_pseudo_distance(cur, d, point);
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
    distance_t d = distance_to_pseudo_distance(ws.segments[segment_index].nearest_segment,
                                               ws.segments[segment_index].min_true, point);
    if (fabs(d.x) < fabs(min_distance))
        min_distance = d.x;
    return min_distance;
}

void merge_segment(int s, int other) {
    if (less(ws.segments[other].min_true.d, ws.segments[s].min_true.d)) {
        ws.segments[s].min_true = ws.segments[other].min_true;
        ws.segments[s].nearest_segment = ws.segments[other].nearest_segment;
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
