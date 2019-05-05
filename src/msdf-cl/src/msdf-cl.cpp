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

struct pseudo_distance_selector_base {
    distance_t min_true, min_negative, min_positive;
    float near_edge_param;
    segment *near_edge;
};


struct edge_selector {
    vec2 point;
    pseudo_distance_selector_base r, g, b;
};

struct workspace {
    struct edge_selector shape;
    struct edge_selector inner;
    struct edge_selector outer;

    multi_distance max_inner;
    multi_distance max_outer;
    multi_distance min_absolute;
};

static inline vec3 to_pixel(multi_distance d, float range) {
    return vec3(d.r / range + 0.5f, d.g / range + 0.5f, d.b / range + 0.5f);
}

void add_segment(struct edge_selector *, segment *, segment *, segment *);
void set_contour_edge(struct workspace *, int, struct edge_selector *, contour *);

bool less(distance_t a, distance_t b) {
    return fabs(a.x) < fabs(b.x) || (fabs(a.x) == fabs(b.x) && a.y < b.y);
}

void add_segment_true_distance(struct pseudo_distance_selector_base *psdb, segment *s,
                               distance_t d, float param) {
    if (less(d, psdb->min_true)) {
        psdb->min_true = d;
        psdb->near_edge = s;
        psdb->near_edge_param = param;
    }
}

void add_segment_pseudo_distance(struct pseudo_distance_selector_base *psdb, distance_t d) {
    distance_t *min_pseudo = d.x < 0 ? &(psdb->min_negative) : &(psdb->min_positive);
    if (less(d, *min_pseudo)) {
        *min_pseudo = d;
    }
}

distance_t distance_to_pseudo_distance(segment *s, distance_t d, vec2 p, float param) {
    if (param >= 0 && param <= 1)
        return d;

    vec2 dir = normalize(segment_direction(s, param < 0 ? 0 : 1));
    vec2 aq = p - segment_point(s, param < 0 ? 0 : 1);
    float ts = dot(aq, dir);
    if (param < 0 ? ts < 0 : ts > 0) {
        float pseudo_distance = cross_(aq, dir);
        if (fabs(pseudo_distance) <= fabs(d.x)) {
            d.x = pseudo_distance;
            d.y = 0;
        }
    }
    return d;
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

multi_distance get_pixel_distance(struct workspace *ws, struct shape *shape);

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
                } else if (auto p =
                               dynamic_cast<msdfgen::CubicSegment *>(_e.edgeSegment)) {
                    s->npoints = 4;
                    s->points[0] = Point2_to_vec2(p->p[0]);
                    s->points[1] = Point2_to_vec2(p->p[1]);
                    s->points[2] = Point2_to_vec2(p->p[2]);
                    s->points[3] = Point2_to_vec2(p->p[3]);
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
void init_edge_selector(struct edge_selector *e, vec2 p) {
    e->point.x = p.x;
    e->point.y = p.y;
    e->r.near_edge = NULL;
    e->g.near_edge = NULL;
    e->b.near_edge = NULL;
    e->r.near_edge_param = 0;
    e->g.near_edge_param = 0;
    e->b.near_edge_param = 0;
    e->r.min_true.x = -INFINITY;
    e->r.min_true.y = 1;
    e->g.min_true.x = -INFINITY;
    e->g.min_true.y = 1;
    e->b.min_true.x = -INFINITY;
    e->b.min_true.y = 1;
    e->r.min_negative.x = -INFINITY;
    e->r.min_negative.y = 1;
    e->g.min_negative.x = -INFINITY;
    e->g.min_negative.y = 1;
    e->b.min_negative.x = -INFINITY;
    e->b.min_negative.y = 1;
    e->r.min_positive.x = -INFINITY;
    e->r.min_positive.y = 1;
    e->g.min_positive.x = -INFINITY;
    e->g.min_positive.y = 1;
    e->b.min_positive.x = -INFINITY;
    e->b.min_positive.y = 1;
}

void calculate_pixel(struct shape *shape, vec3 *output, int x, int y, int stride,
                     vec2 scale, vec2 translate, float range) {
    vec2 p = vec2((x + 0.5f) / scale.x - translate.x, (y + 0.5f) / scale.y - translate.y);

    struct workspace ws;
    ws.max_inner.r = -INFINITY;
    ws.max_inner.g = -INFINITY;
    ws.max_inner.b = -INFINITY;
    ws.max_outer.r = -INFINITY;
    ws.max_outer.g = -INFINITY;
    ws.max_outer.b = -INFINITY;
    ws.min_absolute.r = -INFINITY;
    ws.min_absolute.g = -INFINITY;
    ws.min_absolute.b = -INFINITY;

    init_edge_selector(&ws.shape, p);
    init_edge_selector(&ws.inner, p);
    init_edge_selector(&ws.outer, p);

    struct edge_selector e;

    contour *c = shape->contours;
    for (int contour_idx = 0; contour_idx < shape->ncontours; ++contour_idx) {

        if (!c->nsegments) {
            c += 1;
            continue;
        }
        init_edge_selector(&e, p);

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
            add_segment(&e, prev, cur, s);
            prev = cur;
            cur = s;
            NEXT_SEGMENT(s);
        }

        set_contour_edge(&ws, contour_idx, &e, c);

        /* s now points to the next contour structure (if any) */
        c = (contour *)s;
    }

    multi_distance d = get_pixel_distance(&ws, shape);
    vec3 pixel = to_pixel(d, range);
    printf("==> PIXEL: %.2f %.2f %.2f\n", pixel.r, pixel.g, pixel.b);
    output[y * stride + x] = pixel;
}

void add_segment(struct edge_selector *e, segment *prev, segment *cur, segment *next) {

    segment_distance d = signed_distance(cur, e->point);

    if (cur->color & RED)
        add_segment_true_distance(&e->r, cur, d.d, d.param);
    if (cur->color & GREEN)
        add_segment_true_distance(&e->g, cur, d.d, d.param);
    if (cur->color & BLUE)
        add_segment_true_distance(&e->b, cur, d.d, d.param);

    if (point_facing_edge(prev, cur, next, e->point, d.param)) {

        distance_t pd = distance_to_pseudo_distance(cur, d.d, e->point, d.param);
        if (cur->color & RED)
            add_segment_pseudo_distance(&e->r, pd);
        if (cur->color & GREEN)
            add_segment_pseudo_distance(&e->g, pd);
        if (cur->color & BLUE)
            add_segment_pseudo_distance(&e->b, pd);
    }
}
float compute_distance(pseudo_distance_selector_base *b, vec2 point);

void merge_segment(struct pseudo_distance_selector_base *s,
                   struct pseudo_distance_selector_base *other) {
    if (less(other->min_true, s->min_true)) {
        s->min_true = other->min_true;
        s->near_edge = other->near_edge;
        s->near_edge_param = other->near_edge_param;
    }
    if (less(other->min_negative, s->min_negative))
        s->min_negative = other->min_negative;
    if (less(other->min_positive, s->min_positive)) {
        s->min_positive = other->min_positive;
    }
}

void merge_multi_segment(struct edge_selector *e, struct edge_selector *other) {
    merge_segment(&e->r, &other->r);
    merge_segment(&e->g, &other->g);
    merge_segment(&e->b, &other->b);
}

multi_distance get_distance(edge_selector *e) {
    multi_distance d;
    d.r = compute_distance(&e->r, e->point);
    d.g = compute_distance(&e->g, e->point);
    d.b = compute_distance(&e->b, e->point);
    return d;
}

void set_contour_edge(struct workspace *ws, int i, struct edge_selector *e, contour *c) {

    multi_distance d = get_distance(e);

    merge_multi_segment(&ws->shape, e);
    if (c->winding > 0 && resolve_multi_distance(d) >= 0)
        merge_multi_segment(&ws->inner, e);
    if (c->winding < 0 && resolve_multi_distance(d) <= 0)
        merge_multi_segment(&ws->outer, e);

    multi_distance *target = c->winding < 0 ? &ws->max_inner : &ws->max_outer;

    if (resolve_multi_distance(d) > resolve_multi_distance(*target))
        *target = d;

    if (fabs(resolve_multi_distance(d)) < fabs(resolve_multi_distance(ws->min_absolute)))
        ws->min_absolute = d;
}

float compute_distance(pseudo_distance_selector_base *b, vec2 point) {
    float min_distance = b->min_true.x < 0 ? b->min_negative.x : b->min_positive.x;
    distance_t d = distance_to_pseudo_distance(b->near_edge, b->min_true, point, b->near_edge_param);
    if (fabs(d.x) < fabs(min_distance))
        min_distance = d.x;
    return min_distance;
}

multi_distance get_pixel_distance(struct workspace *ws, struct shape *shape) {
    multi_distance shape_distance = get_distance(&ws->shape);
    multi_distance inner_distance = get_distance(&ws->inner);
    multi_distance outer_distance = get_distance(&ws->outer);
    float inner_d = resolve_multi_distance(inner_distance);
    float outer_d = resolve_multi_distance(outer_distance);

    bool inner = inner_d >= 0 && fabs(inner_d) <= fabs(outer_d);
    bool outer = outer_d <= 0 && fabs(outer_d) < fabs(inner_d);
    if (!inner && !outer)
        return shape_distance;

    multi_distance d = inner ? inner_distance : outer_distance;
    multi_distance contour_distance = inner ? ws->max_inner : ws->max_outer;

    float contour_d = resolve_multi_distance(contour_distance);
    if (fabs(contour_d) < fabs(outer_d) && contour_d > resolve_multi_distance(d))
        d = contour_distance;

    contour_distance = ws->min_absolute;
    contour_d = resolve_multi_distance(contour_distance);
    float d_d = resolve_multi_distance(d);

    if (fabs(contour_d) < fabs(d_d))
        d = contour_distance;

    if (resolve_multi_distance(d) == resolve_multi_distance(shape_distance))
        d = shape_distance;

    return d;
}
