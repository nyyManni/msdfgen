#include <stdlib.h>
#include <Shape.h>
#include <msdf.h>
#include "msdf-lib.h"
#include <import-font.h>
#include <edge-coloring.h>
#include <edge-segments.h>
#include "msdf.h"
#include "msdfgen-ext.h"
#include "msdfgen.h"
#include <Contour.h>


struct pseudo_distance_selector_base {
    distance_t min_true, min_negative, min_positive;
    float near_edge_param;
    segment *near_edge;
};

struct edge_selector {
    vec2 point;
    pseudo_distance_selector_base r, g, b;
};

void dump_distance_selector(struct pseudo_distance_selector_base * p) {
    printf("==> distances true: (%.2e, %.2f), negative: (%.2e, %.2f), positive: (%.2e, %.2f)\n",
           p->min_true.x, p->min_true.y, p->min_negative.x, p->min_negative.y,
           p->min_positive.x, p->min_positive.y);
    printf("==> near_edge %p (%.2f)\n", p->near_edge, p->near_edge_param);
}

void dump_selector(struct edge_selector *e) {
    printf("==> point: (%.2f, %.2f)\n", e->point.x, e->point.y);
    dump_distance_selector(&e->r);
    dump_distance_selector(&e->g);
    dump_distance_selector(&e->b);
}

struct workspace {
    struct edge_selector shape;
    struct edge_selector inner;
    struct edge_selector outer;
    int ncontours;
    struct {
        int32_t winding;
        struct edge_selector *edge_selector;
    } edges[2];
};

static inline vec3 to_pixel(multi_distance d, float range) {
    vec3 p;
    p.r = d.r / range + 0.5f;
    p.g = d.g / range + 0.5f;
    p.b = d.b / range + 0.5f;
    return p;
}

void add_segment(struct edge_selector *, segment *, segment *, segment *);
void set_contour_edge(struct workspace *, int, struct edge_selector *);

bool less(distance_t a, distance_t b) {
    return fabs(a.x) < fabs(b.x) || (fabs(a.x) == fabs(b.x) && a.y < b.y);
}

bool greater(distance_t a, distance_t b) {
    return fabs(a.x) > fabs(b.x) || (fabs(a.x) == fabs(b.x) && a.y > b.y);
}

bool lesseq(distance_t a, distance_t b) {
    return fabs(a.x) < fabs(b.x) || (fabs(a.x) == fabs(b.x) && a.y <= b.y);
}

bool greateq(distance_t a, distance_t b) {
    return fabs(a.x) > fabs(b.x) || (fabs(a.x) == fabs(b.x) && a.y >= b.y);
}

void add_segment_true_distance(struct pseudo_distance_selector_base *psdb, 
                               segment *s, distance_t d, float param) {
    printf("==> adding true distance\n");
    dump_distance_selector(psdb);
    dump_segment(s);
    if (less(d, psdb->min_true)) {
        psdb->min_true = d;
        psdb->near_edge = s;
        psdb->near_edge_param = param;
    }
    // static int debug = 20;
    // if (debug > 0) {
        // printf("min true: %.2f * %.2f (%.2f)\n", psdb->min_true.x, psdb->min_true.y, psdb->near_edge_param);
    // }
    // debug--;
}
void add_segment_pseudo_distance(struct pseudo_distance_selector_base *psdb, distance_t d) {
    printf("==> adding pseudo distance\n");
    dump_distance_selector(psdb);
    distance_t *min_pseudo = d.x < 0 ? &(psdb->min_negative) : &(psdb->min_positive);
    printf("==> min pseudo: %.2e, %.2f\n", min_pseudo->x, min_pseudo->y);
    if (less(d, *min_pseudo)) {
        printf("==> was less, changin (%.2e %.2f) to (%.2e, %.2f)\n",
               min_pseudo->x, min_pseudo->y, d.x, d.y);
        min_pseudo->x = d.x;
        min_pseudo->y = d.y;
        
    }
}

void dump_segment(segment *s) {
    if (s->npoints == 2)
        printf("==> linear segment (%i): (%.2f, %.2f), (%.2f, %.2f)\n",
               s->color, s->points[0].x, s->points[0].y, s->points[1].x, s->points[1].y);

    if (s->npoints == 3)
        printf("==> quad   segment (%i): (%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f)\n", 
               s->color, s->points[0].x, s->points[0].y, s->points[1].x, s->points[1].y, s->points[2].x, s->points[2].y);

    if (s->npoints == 4)
        printf("==> cubic  segment (%i): (%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f)\n", 
               s->color, s->points[0].x, s->points[0].y, s->points[1].x, s->points[1].y, s->points[2].x, s->points[2].y, s->points[3].x, s->points[3].y);
}


void distance_to_pseudo_distance(segment *s, distance_t *d, vec2 p, float param) {
    printf("==> p: (%.2f, %.2f)\n", p.x, p.y);
    dump_segment(s);
    if (param < 0) {
        // printf("larger than ")
        vec2 dir = normalize(segment_direction(s, 0));
        vec2 aq = p - segment_point(s, 0);
        float ts = dotProduct(aq, dir);
        if (ts < 0) {
            float pseudo_distance = crossProduct(aq, dir);
            if (fabs(pseudo_distance) <= fabs(d->x)) {
                d->x = pseudo_distance;
                d->y = 0;
            }
        }
    } else if (param > 1) {
        vec2 dir = normalize(segment_direction(s, 1));
        vec2 bq = p - segment_point(s, 1);
        float ts = dotProduct(bq, dir);
        if (ts > 0) {
            float pseudo_distance = crossProduct(bq, dir);
            if (fabs(pseudo_distance) <= fabs(d->x)) {
                d->x = pseudo_distance;
                d->y = 0;
            }
        }
    }
    static int debug = 10;
    if (debug) {
        // printf("    d: %.2f\n", resolve_multi_distance(d));
        // printf("------>  \t%.2f,\t%.2f (%.2f)\n", d->x, d->y, param);
        // printf("    d: %.2f, %.2f, %.2f\n", d.r.x, d.g.x, d.b.x);
        debug--;
    }
    
}

bool point_facing_edge(segment *prev, segment *cur, segment *next, vec2 p, float param) {
    if (param < 0) {
        vec2 prev_edge_dir = -normalize(segment_direction(prev, 1), true);
        vec2 edge_dir = normalize(segment_direction(cur, 0), true);
        vec2 point_dir = p - segment_point(cur, 0);
        return dotProduct(point_dir, edge_dir) >= dotProduct(point_dir, prev_edge_dir);
    }
    if (param > 1) {
        vec2 edge_dir = -normalize(segment_direction(cur, 1), true);
        vec2 next_edge_dir = normalize(segment_direction(next, 0), true);
        vec2 point_dir = p - segment_point(cur, 1);
        return dotProduct(point_dir, edge_dir) >= dotProduct(point_dir, next_edge_dir);
    }
    return true;
}

multi_distance get_pixel_distance(struct workspace *ws);

void calculate_pixel(struct shape *, vec3 *, struct workspace *, int, int, int, vec2, vec2, float);

static inline vec2 Point2_to_vec2(msdfgen::Point2 p) {
    vec2 p_;
    p_.x = p.x;
    p_.y = p.y;
    return p_;
}
int main() {

    msdfgen::Shape shape;
    msdf_font_handle f = msdf_load_font("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf");
    msdfgen::FontHandle *font = (msdfgen::FontHandle *)f->__handle;
    // msdfgen::loadGlyph(shape, font, '1');
    msdfgen::loadGlyph(shape, font, 0x00e4);
    // msdfgen::loadGlyph(shape, font, '#');

    shape.normalize();
    edgeColoringSimple(shape, 3.0);
    
    size_t input_size = sizeof (struct shape);
    for (msdfgen::Contour &c : shape.contours) {
        input_size += sizeof (contour);
        for (msdfgen::EdgeHolder &e : c.edges) {
            input_size += sizeof (segment);
            if (dynamic_cast<msdfgen::LinearSegment *>(e.edgeSegment))
                input_size += 2 * sizeof (vec2);
            if (dynamic_cast<msdfgen::QuadraticSegment *>(e.edgeSegment))
                input_size += 3 * sizeof (vec2);
            if (dynamic_cast<msdfgen::CubicSegment *>(e.edgeSegment))
                input_size += 4 * sizeof (vec2);
        }
    }
    void * input_buffer = malloc(input_size * 4);
    // printf("input size: %lu\n", input_size);
    size_t work_area_size = sizeof (struct workspace)
        + shape.contours.size() * (sizeof (edge_selector) + sizeof (int32_t));
    // printf("work area size: %lu\n", work_area_size);

    struct workspace *ws = (struct workspace *)malloc(work_area_size * 16);


    ws->ncontours = shape.contours.size();
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
                } else if (auto p = dynamic_cast<msdfgen::QuadraticSegment *>(_e.edgeSegment)) {
                    s->npoints = 3;
                    s->points[0] = Point2_to_vec2(p->p[0]);
                    s->points[1] = Point2_to_vec2(p->p[1]);
                    s->points[2] = Point2_to_vec2(p->p[2]);
                } else if (auto p = dynamic_cast<msdfgen::CubicSegment *>(_e.edgeSegment)) {
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
    // return 0;

    vec2 scale = {1.0, 1.0};
    vec2 translate = {0.0, 0.0};
    float range = 4.0;

    float width = font->face->glyph->metrics.width / 64.0;
    float height = font->face->glyph->metrics.height / 64.0;
    int w = ceil((width + range) * scale.x);
    int h = ceil((height + range) * scale.x);

    msdfgen::Bitmap<msdfgen::FloatRGB> msdf(w, h);
    msdfgen::generateMSDF(msdf, shape, range, 1.0,
                          msdfgen::Vector2(0.0, 0.0), 1.001, true);

    vec3 *output = (vec3 *)malloc(h * w * sizeof(vec3));

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            calculate_pixel(glyph_data, output, ws, x, y,
                            w, scale, translate, range);
            // return 0;
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

void calculate_pixel(struct shape *shape, vec3 *output, struct workspace *ws, 
                     int x, int y, int stride, vec2 scale, vec2 translate, float range) {
    vec2 p = vec2((x + 0.5f) / scale.x - translate.x,
                  (y + 0.5f) / scale.y - translate.y);

    init_edge_selector(&ws->shape, p);
    init_edge_selector(&ws->inner, p);
    init_edge_selector(&ws->outer, p);
        

    contour *c = shape->contours;
    for (int contour_idx = 0; contour_idx < shape->ncontours; ++contour_idx) {

        ws->edges[contour_idx].winding = c->winding;
        if (!c->nsegments) {
            c += 1;
            continue;
        }
        printf("==> creating edge selector\n");
        struct edge_selector *e = (struct edge_selector *)malloc(sizeof (struct edge_selector) * 10);
        init_edge_selector(e, p);

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
            // printf("%%%%\n");
            // dump_segment(prev);
            // dump_segment(cur);
            // dump_segment(s);
            // printf("%%%%\n");
            printf("==> adding edge\n");
            add_segment(e, prev, cur, s);
            prev = cur;
            cur = s;
            NEXT_SEGMENT(s);
        }

        printf("==> setting contour edge %d\n", contour_idx);
        set_contour_edge(ws, contour_idx, e);

        /* s now points to the next contour structure (if any) */
        c = (contour *)s;
    }

    multi_distance d = get_pixel_distance(ws);
    vec3 pixel = to_pixel(d, range);
    printf("==> PIXEL: %.2f %.2f %.2f\n", pixel.r, pixel.g, pixel.b);
    output[y * stride + x] = pixel;

}

void add_segment(struct edge_selector *e, segment *prev, segment *cur, segment *next) {
    float param;

    distance_t d = signed_distance(cur, e->point, &param);
    // printf("signed distance: %.2f %.2f\n", d.x, d.y);
    // printf("edge color: %i\n", cur->color);

    if (cur->color & RED)
        add_segment_true_distance(&e->r, cur, d, param);
    if (cur->color & GREEN)
        add_segment_true_distance(&e->g, cur, d, param);
    if (cur->color & BLUE)
        add_segment_true_distance(&e->b, cur, d, param);
    
    static int debug = 20;
    if (point_facing_edge(prev, cur, next, e->point, param)) {
        // if (debug > 0)
        //     // printf("my  facing edge\n");

        printf("==> d before: %.2e %.2f\n", d.x, d.y);
        distance_to_pseudo_distance(cur, &d, e->point, param);
        printf("==> d after:  %.2e %.2f\n", d.x, d.y);
        dump_segment(cur);
        if (cur->color & RED)
            add_segment_pseudo_distance(&e->r, d);
        if (cur->color & GREEN)
            add_segment_pseudo_distance(&e->g, d);
        if (cur->color & BLUE)
            add_segment_pseudo_distance(&e->b, d);
        dump_selector(e);
    }
}
float compute_distance(pseudo_distance_selector_base *b, vec2 point);

void merge_segment(struct pseudo_distance_selector_base *s, struct pseudo_distance_selector_base *other) {
    printf("==> merging segment\n");
    if (less(other->min_true, s->min_true)) {
        s->min_true = other->min_true;
        s->near_edge = other->near_edge;
        s->near_edge_param = other->near_edge_param;
    }
    if (less(other->min_negative, s->min_negative))
        s->min_negative = other->min_negative;
    if (less(other->min_positive, s->min_positive)) {
        // printf("--> (%.2e, %.2f) < (%.2e, %.2f)\n",
        //        other->min_positive.x, other->min_positive.y,
        //        s->min_positive.x, s->min_positive.y);
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

void set_contour_edge(struct workspace *ws, int i, struct edge_selector *e) {

    dump_selector(e);

    multi_distance d = get_distance(e);
    // d.r = compute_distance(&e->r, e->point);
    // d.g = compute_distance(&e->g, e->point);
    // d.b = compute_distance(&e->b, e->point);
    // d = edge_selector_distance(e);
    printf("==> d: %.2f\n", resolve_multi_distance(d));

    ws->edges[i].edge_selector = e;

    merge_multi_segment(&ws->shape, e);
    // printf("==> winding: %d\n", ws->edges[i].winding);
    if (ws->edges[i].winding > 0 && resolve_multi_distance(d) >= 0)
        merge_multi_segment(&ws->inner, e);
    if (ws->edges[i].winding < 0 && resolve_multi_distance(d) <= 0)
        merge_multi_segment(&ws->outer, e);
}

float compute_distance(pseudo_distance_selector_base *b, vec2 point) {
    float min_distance = b->min_true.x < 0 ? b->min_negative.x : b->min_positive.x;
    if (b->near_edge) {
        // printf("min distance: near edge");
        distance_t d = b->min_true;
        distance_to_pseudo_distance(b->near_edge, &d, point, b->near_edge_param);
        if (fabs(d.x) < fabs(min_distance))
            min_distance = d.x;
        // b->near_edge->
    }
    // printf("min distance: compute distance: %.2f\n", min_distance);
    return min_distance;
}

multi_distance get_pixel_distance(struct workspace *ws) {
    multi_distance shape_distance = get_distance(&ws->shape);
    multi_distance inner_distance = get_distance(&ws->inner);
    multi_distance outer_distance = get_distance(&ws->outer);
    float inner_d = resolve_multi_distance(inner_distance);
    float outer_d = resolve_multi_distance(outer_distance);
    printf("==> inner: %.2e, outer: %.2e\n", inner_d, outer_d);
    int contour_count = ws->ncontours;
    printf("==> contour count: %d\n", contour_count);
    
    multi_distance d;
    d.r = -INFINITY;
    d.g = -INFINITY;
    d.b = -INFINITY;
    
    
    int winding = 0;
    
    if (inner_d >= 0 && fabs(inner_d) <= fabs(outer_d)) {
        d = inner_distance;
        winding = 1;
        for (int i = 0; i < contour_count; ++i) {
            if (ws->edges[i].winding > 0) {
                multi_distance contour_distance = get_distance(ws->edges[i].edge_selector);
                if (fabs(resolve_multi_distance(contour_distance)) < fabs(outer_d)
                        && resolve_multi_distance(contour_distance) > resolve_multi_distance(d))
                    d = contour_distance;
            }
        }
    } else if (outer_d <= 0 && fabs(outer_d) < fabs(inner_d)) {
        d = outer_distance;
        winding = -1;
        for (int i = 0; i < contour_count; ++i) {
            if (ws->edges[i].winding < 0) {
                multi_distance contour_distance = get_distance(ws->edges[i].edge_selector);
                if (fabs(resolve_multi_distance(contour_distance)) < fabs(inner_d)
                        && resolve_multi_distance(contour_distance) > resolve_multi_distance(d))
                    d = contour_distance;
            }
        }
    } else {
        return shape_distance;
    }
    
    for (int i = 0; i < contour_count; ++i) {
        if (ws->edges[i].winding != winding) {
            multi_distance contour_distance = get_distance(ws->edges[i].edge_selector);
            if (resolve_multi_distance(contour_distance) * resolve_multi_distance(d) >= 0
                && fabs(resolve_multi_distance(contour_distance)) < fabs(resolve_multi_distance(d)))
                d = contour_distance;
        }
    }
    if (resolve_multi_distance(d) == resolve_multi_distance(shape_distance))
        d = shape_distance;
    return d;
}
