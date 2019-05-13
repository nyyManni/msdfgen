#include "msdf.h"
#include "msdf-lib.h"
#include "msdfgen-ext.h"
#include "msdfgen.h"
#include <Contour.h>
#include <Shape.h>
#include <edge-coloring.h>
#include <edge-segments.h>
#include <import-font.h>
#include <msdf.h>
#include <stdlib.h>

#define IDX_CURR 0
#define IDX_SHAPE 1
#define IDX_INNER 2
#define IDX_OUTER 3
#define IDX_RED 0
#define IDX_GREEN 1
#define IDX_BLUE 2
#define IDX_NEGATIVE 0
#define IDX_POSITIVE 1
#define IDX_MAX_INNER 0
#define IDX_MAX_OUTER 1

struct workspace {
    struct {
        segment_distance min_true;
        vec2 mins[2];
        int nearest_points;
        int nearest_npoints;
    } segments[4 * 3];

    vec3 maximums[2];
    vec3 min_absolute;
} ws;

vec2 *point_data;
vec2 *point_data2;
unsigned char *metadata;
unsigned char *metadata2;
vec3 *output;

void add_segment(int, int, int, int, int, int, int, vec2);
void set_contour_edge(int, vec2);
vec3 get_pixel_distance(vec2);
void calculate_pixel(int, int, int, vec2, vec2, float);
segment_distance signed_distance_linear(vec2, vec2, vec2);
segment_distance signed_distance_quad(vec2, vec2, vec2, vec2);

bool less(vec2 a, vec2 b) {
    return fabs(a.x) < fabs(b.x) || (fabs(a.x) == fabs(b.x) && a.y < b.y);
}

void add_segment_true_distance(int segment_index, int npoints, int points,
                               segment_distance d) {
    bool is_less = less(d.xy, ws.segments[segment_index].min_true.xy);
    ws.segments[segment_index].min_true =
        is_less ? d : ws.segments[segment_index].min_true;

    ws.segments[segment_index].nearest_points =
        is_less ? points : ws.segments[segment_index].nearest_points;
    ws.segments[segment_index].nearest_npoints =
        is_less ? npoints : ws.segments[segment_index].nearest_npoints;
}


void add_segment_pseudo_distance(int segment_index, vec2 d) {
    int i = d.x < 0 ? IDX_NEGATIVE : IDX_POSITIVE;
    vec2 _d = ws.segments[segment_index].mins[i];
    ws.segments[segment_index].mins[i] = less(d, _d) ? d : _d;
}

vec2 distance_to_pseudo_distance(int npoints, int points, segment_distance d, vec2 p) {
    if (d.z >= 0 && d.z <= 1)
        return d.xy;

    vec2 dir = normalize(segment_direction(points, npoints, d.z < 0 ? 0 : 1));
    vec2 aq = p - segment_point(points, npoints, d.z < 0 ? 0 : 1);
    float ts = dot(aq, dir);
    if (d.z < 0 ? ts < 0 : ts > 0) {
        float pseudo_distance = cross_(aq, dir);
        if (fabs(pseudo_distance) <= fabs(d.xy.x)) {
            d.xy.x = pseudo_distance;
            d.xy.y = 0;
        }
    }
    return d.xy;
}

bool point_facing_edge(int prev_npoints, int prev_points, int cur_npoints, int cur_points,
                       int next_npoints, int next_points, vec2 p, float param) {

    if (param >= 0 && param <= 1)
        return true;

    vec2 prev_edge_dir = -normalize(segment_direction(prev_points, prev_npoints, 1));
    vec2 edge_dir =
        normalize(segment_direction(cur_points, cur_npoints, param < 0 ? 0 : 1)) *
        (param < 0 ? 1 : -1);
    vec2 next_edge_dir = normalize(segment_direction(next_points, next_npoints, 0));
    vec2 point_dir = p - segment_point(cur_points, cur_npoints, param < 0 ? 0 : 1);
    return dot(point_dir, edge_dir) >=
           dot(point_dir, param < 0 ? prev_edge_dir : next_edge_dir);
}

// struct __glyph_data_ctx {
//     bool allocated;
//     int meta_size;
//     int data_size;
//     int current_contour_meta;
// };

// static inline float shoelace(const vec2 a, const vec2 b) {
//     return (b.x - a.x) * (a.y + b.y);
// }
// static void update_winding(struct __glyph_data_ctx *ctx, int npoints, int points) {
//     float total = 0;
//     if (metadata2[ctx->current_contour_meta + 1] == 1) {
//         /* First segment, calculate case nsegments == 1 */
//         vec2 a = segment_point(points, npoints, 0.0);
//         vec2 b = segment_point(points, npoints, 1.0 / 3.0);
//         vec2 c = segment_point(points, npoints, 2.0 / 3.0);
//     } else if (metadata2[ctx->current_contour_meta + 1] == 2) {
//         /* Second segment, calculate case nsegments == 2 */
//     } else {
//         /* Third or later segment, update case nsegments == n */
//     }
//     metadata2[ctx->current_contour_meta + 1] = sign(total);
// }

// static int __add_contour_size(const FT_Vector *to, void *user) {
//     struct __glyph_data_ctx *ctx = (struct __glyph_data_ctx *)user;
//     if (ctx->allocated) {
//         point_data2[ctx->data_size].x = to->x / 64.0;
//         point_data2[ctx->data_size].y = to->y / 64.0;
        
//         metadata2[ctx->meta_size] = 0; /* winding */
//         metadata2[ctx->meta_size + 1] = 0;
//         ctx->current_contour_meta = ctx->meta_size;
//     }

//     ctx->data_size += 1;
//     ctx->meta_size += 2;  /* winding + nsegments */
//     return 0;
// }
// static int __add_linear_size(const FT_Vector *to, void *user) {
//     struct __glyph_data_ctx *ctx = (struct __glyph_data_ctx *)user;
//     if (ctx->allocated) {
//         point_data2[ctx->data_size].x = to->x / 64.0;
//         point_data2[ctx->data_size].y = to->y / 64.0;
//         metadata2[ctx->current_contour_meta + 1] += 1;
//         metadata2[ctx->meta_size] = 0;  /* color */
//         metadata2[ctx->meta_size + 1] = 2;  /* npoints */
//     }
//     ctx->data_size += 1;
//     ctx->meta_size += 2;  /* color + npoints */
//     return 0;
// }
// static int __add_quad_size(const FT_Vector *control, const FT_Vector *to, void *user) {
//     struct __glyph_data_ctx *ctx = (struct __glyph_data_ctx *)user;
//     if (ctx->allocated) {
//         point_data2[ctx->data_size].x = to->x / 64.0;
//         point_data2[ctx->data_size].y = to->y / 64.0;
//         point_data2[ctx->data_size + 1].x = to->x / 64.0;
//         point_data2[ctx->data_size + 1].y = to->y / 64.0;

//         metadata2[ctx->current_contour_meta + 1] += 1;
//         metadata2[ctx->meta_size] = 0;  /* color */
//         metadata2[ctx->meta_size + 1] = 3;  /* npoints */
//     }
//     ctx->data_size += 2;
//     ctx->meta_size += 2;  /* color + npoints */
//     return 0;
// }
// static int __add_cubic_size(const FT_Vector *control1, const FT_Vector *control2, 
//                             const FT_Vector *to, void *s) {
//     fprintf(stderr, "Cubic segments not supported\n");
//     return -1;
// }

// void *msdf_load_glyph(FT_Face face, int code) {
//     if (FT_Load_Char(face, code, FT_LOAD_NO_SCALE)) return NULL;

//     FT_Outline_Funcs fns;
//     fns.shift = 0;
//     fns.delta = 0;
//     fns.move_to = &__add_contour_size;
//     fns.line_to = &__add_linear_size;
//     fns.conic_to = &__add_quad_size;
//     fns.cubic_to = &__add_cubic_size;

//     /* We need two rounds of decomposing, the first one will just figure out
//        how much space we need to serialize the glyph, and the second one
//        serializes it and generates colour mapping for the segments. */
//     struct __glyph_data_ctx ctx = {0, 0};
//     if (FT_Outline_Decompose(&face->glyph->outline, &fns, &ctx)) return NULL;

//     point_data2 = (vec2 *)malloc(ctx.data_size * sizeof(vec2));
//     metadata2 = (unsigned char *)malloc(ctx.meta_size);
    
//     ctx.allocated = true;
//     ctx.meta_size = 0;
//     ctx.data_size = 0;

//     /* Second round populates the point data. */
//     if (FT_Outline_Decompose(&face->glyph->outline, &fns, &ctx)) return NULL;

//     /* Calculate windings. */
//     size_t point_index = 0;
//     size_t meta_index = 0;

//     unsigned char ncontours = metadata[meta_index++];

//     for (int _i = 0; _i < ncontours; ++_i) {

//         char winding = (char)metadata[meta_index++] - 1;
//         unsigned char nsegments = metadata[meta_index++];

//         if (!nsegments)
//             continue;
        
//         float total = 0;
//         if (nsegments == 1) {
//             // vec2 a
//         }
        
//     }

//     return NULL;
// }


// static inline vec2 Point2_to_vec2(msdfgen::Point2 p) { return vec2(p.x, p.y); }
int main() {

    msdf_font_handle f =
        msdf_load_font("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf");

    int character = 0x00e4;
    size_t point_data_size;
    size_t metadata_size;
    msdf_glyph_buffer_size(f, character, &metadata_size, &point_data_size);
    
    point_data = (vec2 *)malloc(point_data_size);
    metadata = (unsigned char *)malloc(metadata_size);
    
    int width, height;
    msdf_serialize_glyph(f, character, metadata, point_data, &width, &height);

    vec2 scale = {1.0, 1.0};
    vec2 translate = {0.0, 0.0};
    float range = 4.0;

    int w = ceil((width + range) * scale.x);
    int h = ceil((height + range) * scale.x);

    output = (vec3 *)malloc(h * w * sizeof(vec3));

    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            calculate_pixel(x, y, w, scale, translate, range);

    /* Validate */
    msdfgen::Shape shape;

    msdfgen::FontHandle *font = (msdfgen::FontHandle *)f->__handle;
    msdfgen::loadGlyph(shape, font, 0x00e4);
    shape.normalize();
    edgeColoringSimple(shape, 3.0);
    msdfgen::Bitmap<msdfgen::FloatRGB> msdf(w, h);
    msdfgen::generateMSDF(msdf, shape, range, 1.0, msdfgen::Vector2(0.0, 0.0), 1.001,
                          true);

    return 0;
}

void calculate_pixel(int x, int y, int stride, vec2 scale, vec2 translate, float range) {
    vec2 p = vec2((x + 0.5f) / scale.x - translate.x, (y + 0.5f) / scale.y - translate.y);

    ws.maximums[0].r = -INFINITY;
    ws.maximums[1].r = -INFINITY;
    ws.maximums[0].g = -INFINITY;
    ws.maximums[1].g = -INFINITY;
    ws.maximums[0].b = -INFINITY;
    ws.maximums[1].b = -INFINITY;
    ws.min_absolute.r = -INFINITY;
    ws.min_absolute.g = -INFINITY;
    ws.min_absolute.b = -INFINITY;

    for (int _i = 0; _i < (4 * 3); ++_i) {
        ws.segments[_i].mins[0].x = -INFINITY;
        ws.segments[_i].mins[1].x = -INFINITY;
        ws.segments[_i].min_true.xy.x = -INFINITY;
    }
    size_t point_index = 0;
    size_t meta_index = 0;

    unsigned char ncontours = metadata[meta_index++];

    for (int _i = 0; _i < ncontours; ++_i) {

        char winding = (char)metadata[meta_index++] - 1;
        unsigned char nsegments = metadata[meta_index++];

        if (!nsegments)
            continue;

        unsigned char s_color = metadata[meta_index + 0];
        unsigned char s_npoints = metadata[meta_index + 1];

        int cur_points = point_index;
        unsigned char cur_color = metadata[meta_index + 2 * (nsegments - 1) + 0];
        unsigned char cur_npoints = metadata[meta_index + 2 * (nsegments - 1) + 1];
        unsigned char prev_npoints =
            nsegments >= 2 ? metadata[meta_index + 2 * (nsegments - 2) + 1] : s_npoints;

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

            add_segment(prev_npoints, prev_points, cur_npoints, cur_points, s_npoints,
                        point_index, cur_color, p);

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

    vec3 d = get_pixel_distance(p);
    vec3 pixel = d / range + 0.5;

    printf("==> PIXEL: %.2f %.2f %.2f\n", pixel.r, pixel.g, pixel.b);
    output[y * stride + x] = pixel;
}

void add_segment(int prev_npoints, int prev_points, int cur_npoints, int cur_points,
                 int next_npoints, int next_points, int color, vec2 point) {

    segment_distance d;
    if (cur_npoints == 2)
        d = signed_distance_linear(point_data[cur_points], point_data[cur_points + 1],
                                   point);
    else
        d = signed_distance_quad(point_data[cur_points], point_data[cur_points + 1],
                                 point_data[cur_points + 2], point);

    if (color & RED)
        add_segment_true_distance(IDX_CURR * 3 + IDX_RED, cur_npoints, cur_points, d);
    if (color & GREEN)
        add_segment_true_distance(IDX_CURR * 3 + IDX_GREEN, cur_npoints, cur_points, d);
    if (color & BLUE)
        add_segment_true_distance(IDX_CURR * 3 + IDX_BLUE, cur_npoints, cur_points, d);

    if (point_facing_edge(prev_npoints, prev_points, cur_npoints, cur_points,
                          next_npoints, next_points, point, d.z)) {

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

    int i = ws.segments[segment_index].min_true.xy.x < 0 ? IDX_NEGATIVE : IDX_POSITIVE;
    float min_distance = ws.segments[segment_index].mins[i].x;

    vec2 d = distance_to_pseudo_distance(ws.segments[segment_index].nearest_npoints,
                                         ws.segments[segment_index].nearest_points,
                                         ws.segments[segment_index].min_true, point);
    if (fabs(d.x) < fabs(min_distance))
        min_distance = d.x;
    return min_distance;
}

void merge_segment(int s, int other) {
    if (less(ws.segments[other].min_true.xy, ws.segments[s].min_true.xy)) {
        ws.segments[s].min_true = ws.segments[other].min_true;

        ws.segments[s].nearest_npoints = ws.segments[other].nearest_npoints;
        ws.segments[s].nearest_points = ws.segments[other].nearest_points;
    }
    if (less(ws.segments[other].mins[IDX_NEGATIVE], ws.segments[s].mins[IDX_NEGATIVE]))
        ws.segments[s].mins[IDX_NEGATIVE] = ws.segments[other].mins[IDX_NEGATIVE];
    if (less(ws.segments[other].mins[IDX_POSITIVE], ws.segments[s].mins[IDX_POSITIVE])) {
        ws.segments[s].mins[IDX_POSITIVE] = ws.segments[other].mins[IDX_POSITIVE];
    }
}

void merge_multi_segment(int e, int other) {
    merge_segment(e * 3 + IDX_RED, other * 3 + IDX_RED);
    merge_segment(e * 3 + IDX_GREEN, other * 3 + IDX_GREEN);
    merge_segment(e * 3 + IDX_BLUE, other * 3 + IDX_BLUE);
}

vec3 get_distance(int segment_index, vec2 point) {
    vec3 d;
    d.r = compute_distance(segment_index * 3 + IDX_RED, point);
    d.g = compute_distance(segment_index * 3 + IDX_GREEN, point);
    d.b = compute_distance(segment_index * 3 + IDX_BLUE, point);
    return d;
}

void set_contour_edge(int winding, vec2 point) {

    vec3 d = get_distance(IDX_CURR, point);

    merge_multi_segment(IDX_SHAPE, IDX_CURR);
    if (winding > 0 && median(d) >= 0)
        merge_multi_segment(IDX_INNER, IDX_CURR);
    if (winding < 0 && median(d) <= 0)
        merge_multi_segment(IDX_INNER, IDX_CURR);

    int i = winding < 0 ? IDX_MAX_INNER : IDX_MAX_OUTER;

    if (median(d) > median(ws.maximums[i]))
        ws.maximums[i] = d;

    if (fabs(median(d)) < fabs(median(ws.min_absolute)))
        ws.min_absolute = d;
}

vec3 get_pixel_distance(vec2 point) {
    vec3 shape_distance = get_distance(IDX_SHAPE, point);
    vec3 inner_distance = get_distance(IDX_INNER, point);
    vec3 outer_distance = get_distance(IDX_OUTER, point);
    float inner_d = median(inner_distance);
    float outer_d = median(outer_distance);

    bool inner = inner_d >= 0 && fabs(inner_d) <= fabs(outer_d);
    bool outer = outer_d <= 0 && fabs(outer_d) < fabs(inner_d);
    if (!inner && !outer)
        return shape_distance;

    vec3 d = inner ? inner_distance : outer_distance;
    vec3 contour_distance = ws.maximums[inner ? IDX_MAX_INNER : IDX_MAX_OUTER];

    float contour_d = median(contour_distance);
    d = (fabs(contour_d) < fabs(outer_d) && contour_d > median(d)) ? contour_distance : d;

    contour_distance = ws.min_absolute;
    contour_d = median(contour_distance);
    float d_d = median(d);

    d = fabs(contour_d) < fabs(d_d) ? contour_distance : d;
    d = median(d) == median(shape_distance) ? shape_distance : d;

    return d;
}

segment_distance signed_distance_linear(vec2 p0, vec2 p1, vec2 origin) {
    vec2 aq = origin - p0;
    vec2 ab = p1 - p0;
    float param = dot(aq, ab) / dot(ab, ab);
    vec2 eq = (param > .5 ? p1 : p0) - origin;
    float endpointDistance = length(eq);
    if (param > 0 && param < 1) {
        float ortho_distance = dot(orthonormal(ab), aq);
        if (fabs(ortho_distance) < endpointDistance)
            return {vec2(ortho_distance, 0), param};
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
        solutions = 3;
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
    for (int i = 0; i < solutions; ++i) {
        if (tttt[i] > 0 && tttt[i] < 1) {
            vec2 endpoint = p0 + ab * 2 * tttt[i] + br * tttt[i] * tttt[i];
            float distance = sign(cross_(p2 - p0, endpoint - origin)) * length(endpoint - origin);
            if (fabs(distance) <= fabs(minDistance)) {
                minDistance = distance;
                param = tttt[i];
            }
        }
    }
    vec2 v = vec2(minDistance, 0);
    v.y = param > 1 ? fabs(dot(normalize(p2 - p1), normalize(p2 - origin))) : v.y;
    v.y = param < 0 ? fabs(dot(normalize(ab), normalize(qa))) : v.y;

    return {v, param};
}
