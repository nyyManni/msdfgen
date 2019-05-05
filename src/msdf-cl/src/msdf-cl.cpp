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


struct multi_distance {
    float r;
    float g;
    float b;
};

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
    } segments[4 * 3];
    
    multi_distance max_inner;
    multi_distance max_outer;
    multi_distance min_absolute;
} ws;

static inline float resolve_multi_distance(multi_distance d) {
    return median(d.r, d.g, d.b);
}

static inline vec3 to_pixel(multi_distance d, float range) {
    return vec3(d.r / range + 0.5f, d.g / range + 0.5f, d.b / range + 0.5f);
}

void add_segment(segment *, segment *, segment *, vec2);
void set_contour_edge(contour *, vec2);

bool less(distance_t a, distance_t b) {
    return fabs(a.x) < fabs(b.x) || (fabs(a.x) == fabs(b.x) && a.y < b.y);
}
void add_segment_true_distance(int segment_index, segment *s, segment_distance d) {
    bool is_less = less(d.d, ws.segments[segment_index].min_true.d);
    ws.segments[segment_index].min_true = is_less ? d : ws.segments[segment_index].min_true;
    ws.segments[segment_index].nearest_segment = is_less ? s : ws.segments[segment_index].nearest_segment;
}

void add_segment_pseudo_distance(int segment_index, distance_t d) {
    distance_t *min_pseudo = d.x < 0 ? &(ws.segments[segment_index].min_negative) : &(ws.segments[segment_index].min_positive);
    *min_pseudo = less(d, *min_pseudo) ? d : *min_pseudo;
}

distance_t distance_to_pseudo_distance(segment *s, segment_distance d, vec2 p) {
    if (d.param >= 0 && d.param <= 1)
        return d.d;

    vec2 dir = normalize(segment_direction(s, d.param < 0 ? 0 : 1));
    vec2 aq = p - segment_point(s, d.param < 0 ? 0 : 1);
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

    vec2 prev_edge_dir = -normalize(segment_direction(prev, 1));
    vec2 edge_dir = normalize(segment_direction(cur, param < 0 ? 0 : 1)) * (param < 0 ? 1 : -1);
    vec2 next_edge_dir = normalize(segment_direction(next, 0));
    vec2 point_dir = p - segment_point(cur, param < 0 ? 0 : 1);
    return dot(point_dir, edge_dir) >=
           dot(point_dir, param < 0 ? prev_edge_dir : next_edge_dir);
}

multi_distance get_pixel_distance(struct workspace *, struct shape *, vec2);
multi_distance get_pixel_distance(struct shape *, vec2);

void calculate_pixel(struct shape *, vec3 *, int, int, int, vec2, vec2, float);

static inline vec2 Point2_to_vec2(msdfgen::Point2 p) { return vec2(p.x, p.y); }
int main() {

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

    size_t input_size = sizeof(struct shape);
    for (msdfgen::Contour &c : shape.contours) {
        input_size += sizeof(contour);
        for (msdfgen::EdgeHolder &e : c.edges) {
            input_size += sizeof(segment);
            if (dynamic_cast<msdfgen::LinearSegment *>(e.edgeSegment))
                input_size += 2 * sizeof(vec2);
            if (dynamic_cast<msdfgen::QuadraticSegment *>(e.edgeSegment))
                input_size += 3 * sizeof(vec2);
            if (dynamic_cast<msdfgen::CubicSegment *>(e.edgeSegment))
                input_size += 4 * sizeof(vec2);
        }
    }
    void *input_buffer = malloc(input_size);

    struct shape *glyph_data = (struct shape *)input_buffer;
    {
        glyph_data->ncontours = shape.contours.size();
        contour *c = glyph_data->contours;
        for (msdfgen::Contour &_c : shape.contours) {
            c->nsegments = _c.edges.size();
            c->winding = _c.winding();
            segment *s = c->segments;
            for (msdfgen::EdgeHolder &_e : _c.edges) {
                s->color = _e->color;
                if (auto p = dynamic_cast<msdfgen::LinearSegment *>(_e.edgeSegment)) {
                    s->npoints = 2;
                    s->points[0] = Point2_to_vec2(p->p[0]);
                    s->points[1] = Point2_to_vec2(p->p[1]);
                } else if (auto p = dynamic_cast<msdfgen::QuadraticSegment *>(
                               _e.edgeSegment)) {
                    s->npoints = 3;
                    s->points[0] = Point2_to_vec2(p->p[0]);
                    s->points[1] = Point2_to_vec2(p->p[1]);
                    s->points[2] = Point2_to_vec2(p->p[2]);
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

    vec2 scale = {1.0, 1.0};
    vec2 translate = {0.0, 0.0};
    float range = 4.0;

    float width = font->face->glyph->metrics.width / 64.0;
    float height = font->face->glyph->metrics.height / 64.0;
    int w = ceil((width + range) * scale.x);
    int h = ceil((height + range) * scale.x);

    msdfgen::Bitmap<msdfgen::FloatRGB> msdf(w, h);
    msdfgen::generateMSDF(msdf, shape, range, 1.0, msdfgen::Vector2(0.0, 0.0), 1.001,
                          true);

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

    contour *c = shape->contours;
    for (int _i = 0; _i < shape->ncontours; ++_i) {

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
            NEXT_SEGMENT(cur)

        /*
         * Initialize prev to the second last segment in the list, or the first
         * one if there are less than two segments.
         */
        segment *prev = s;
        for (int _i = 0; _i < c->nsegments - 2 && c->nsegments >= 2; ++_i)
            NEXT_SEGMENT(prev)

        for (int _i = 0; _i < c->nsegments; ++_i) {
            add_segment(prev, cur, s, p);
            prev = cur;
            cur = s;
            NEXT_SEGMENT(s);
        }

        set_contour_edge(c, p);

        /* s now points to the next contour structure (if any) */
        c = (contour *)s;
    }

    multi_distance d = get_pixel_distance(shape, p);
    vec3 pixel = to_pixel(d, range);
    printf("==> PIXEL: %.2f %.2f %.2f\n", pixel.r, pixel.g, pixel.b);
    output[y * stride + x] = pixel;
}

void add_segment(segment *prev, segment *cur, segment *next,
                 vec2 point) {

    segment_distance d = signed_distance(cur, point);

    if (cur->color & RED)
        add_segment_true_distance(IDX_CURR * 3 + IDX_RED, cur, d);
    if (cur->color & GREEN)
        add_segment_true_distance(IDX_CURR * 3 + IDX_GREEN, cur, d);
    if (cur->color & BLUE)
        add_segment_true_distance(IDX_CURR * 3 + IDX_BLUE, cur, d);

    if (point_facing_edge(prev, cur, next, point, d.param)) {

        distance_t pd = distance_to_pseudo_distance(cur, d, point);
        if (cur->color & RED)
            add_segment_pseudo_distance(IDX_CURR * 3 + IDX_RED, pd);
        if (cur->color & GREEN)
            add_segment_pseudo_distance(IDX_CURR * 3 + IDX_GREEN, pd);
        if (cur->color & BLUE)
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

void set_contour_edge(contour *c, vec2 point) {

    multi_distance d = get_distance(IDX_CURR, point);

    merge_multi_segment(IDX_SHAPE, IDX_CURR);
    if (c->winding > 0 && resolve_multi_distance(d) >= 0)
        merge_multi_segment(IDX_INNER, IDX_CURR);
    if (c->winding < 0 && resolve_multi_distance(d) <= 0)
        merge_multi_segment(IDX_INNER, IDX_CURR);

    multi_distance *target = c->winding < 0 ? &ws.max_inner : &ws.max_outer;

    if (resolve_multi_distance(d) > resolve_multi_distance(*target))
        *target = d;

    if (fabs(resolve_multi_distance(d)) < fabs(resolve_multi_distance(ws.min_absolute)))
        ws.min_absolute = d;
}

multi_distance get_pixel_distance(struct shape *shape, vec2 point) {
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
