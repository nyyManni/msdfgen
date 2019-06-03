

#include <cstring>
#include <lodepng.h>
#include <math.h>

#include "msdf.h"
#include "msdfgen-ext.h"
#include "msdfgen.h"

using namespace msdfgen;

static FreetypeHandle *ft = nullptr;

msdf_font_handle msdf_load_font(const char *path) {
    if (!ft)
        ft = initializeFreetype();
    FontHandle *font = loadFont(ft, path);
    msdf_font_handle f = (msdf_font_handle)std::malloc(sizeof(struct msdf_font));
    f->face = font->face;
    f->__handle = font;

    FT_Load_Char(f->face, 'x', FT_LOAD_NO_SCALE);
    f->xheight = f->face->glyph->metrics.height / 64.0;
    f->ascender = f->face->ascender / 64.0;
    f->descender = f->face->descender / 64.0;
    f->height = (f->ascender - f->descender);

    f->underline_y = f->face->underline_position / 64.0;
    f->underline_thickness = f->face->underline_thickness / 64.0;

    return f;
}

void msdf_release_font(msdf_font_handle f) {
    delete (FontHandle *)f->__handle;
    std::free(f);
}

msdf_glyph_handle msdf_generate_glyph(msdf_font_handle f, int c, double range,
                                      float scale) {
    Shape shape;
    FontHandle *font = (FontHandle *)f->__handle;
    if (!loadGlyph(shape, font, c))
        return NULL;

    float width = font->face->glyph->metrics.width / 64.0;
    float height = font->face->glyph->metrics.height / 64.0;
    float bearing_x = font->face->glyph->metrics.horiBearingX / 64.0;
    float bearing_y = font->face->glyph->metrics.horiBearingY / 64.0;

    int bitmap_width = ceil((width + range) * scale);
    int bitmap_height = ceil((height + range) * scale);

    msdf_glyph_handle g = new msdf_glyph();

    g->code = c;
    g->index = font->face->glyph->reserved;
    g->bitmap.width = bitmap_width;
    g->bitmap.height = bitmap_height;
    g->bitmap.channels = 3;
    g->bitmap.scale = scale;
    g->bitmap.data = (float *)malloc(sizeof(FloatRGB) * bitmap_width * bitmap_height);

    g->advance = (font->face->glyph->metrics.horiAdvance / 64.0) / f->xheight;
    g->bearing[0] = bearing_x / f->xheight;
    g->bearing[1] = bearing_y / f->xheight;
    g->size[0] = width / f->xheight;
    g->size[1] = height / f->xheight;
    g->padding = (range / 2.0) / f->xheight;

    shape.normalize();
    edgeColoringSimple(shape, 3.0);
    Bitmap<FloatRGB> msdf(bitmap_width, bitmap_height);
    generateMSDF(msdf, shape, range, scale,
                 Vector2(-bearing_x, height - bearing_y) + Vector2(range, range) / 2.0,
                 1.001, true);

    std::memcpy(g->bitmap.data, msdf.content,
                sizeof(FloatRGB) * bitmap_width * bitmap_height);

    return g;
}

void msdf_release_glyph(msdf_glyph_handle g) {
    free(g->bitmap.data);
    delete g;
}

int msdf_dump_glyph(const msdf_glyph_handle g, const char *filename) {
    std::vector<unsigned char> pixels(g->bitmap.channels * g->bitmap.width *
                                      g->bitmap.height);
    std::vector<unsigned char>::iterator it = pixels.begin();
    for (int y = g->bitmap.height - 1; y >= 0; --y)
        for (int x = 0; x < g->bitmap.width; ++x) {
            for (int channel = 0; channel < g->bitmap.channels; ++channel) {
                int i = g->bitmap.channels * (y * g->bitmap.width + x) + channel;
                *it++ = clamp((int)(g->bitmap.data[i] * 0x100), 0xff);
            }
        }
    return !lodepng::encode(filename, pixels, g->bitmap.width, g->bitmap.height, LCT_RGB);
}

typedef struct vec2 {
    float x;
    float y;

    vec2() : x(0), y(0) {}
    vec2(float x, float y) : x(x), y(y) {}

} vec2;

int msdf_glyph_buffer_size(msdf_font_handle f, int c, size_t *meta_size,
                           size_t *buf_size) {
    msdfgen::FontHandle *font = (msdfgen::FontHandle *)f->__handle;
    msdfgen::Shape shape;
    msdfgen::loadGlyph(shape, font, c);

    *buf_size = 0;
    *meta_size = 1;
    for (msdfgen::Contour &c : shape.contours) {
        *meta_size += 2; /* winding + nsegments */

        (*buf_size)++;
        for (msdfgen::EdgeHolder &e : c.edges) {
            *meta_size += 2; /* color + npoints */
            (*buf_size)--;
            if (dynamic_cast<msdfgen::LinearSegment *>(e.edgeSegment)) {
                *buf_size += 2;
            } else if (dynamic_cast<msdfgen::QuadraticSegment *>(e.edgeSegment)) {

                *buf_size += 3;
            } else if (dynamic_cast<msdfgen::CubicSegment *>(e.edgeSegment)) {
                return -1;
            }
        }
    }
    *buf_size *= sizeof(vec2);
    *meta_size *= sizeof(unsigned char);
    
    return 0;
}


static inline vec2 Point2_to_vec2(msdfgen::Point2 p) { return vec2(p.x, p.y); }

int msdf_serialize_glyph(msdf_font_handle f, int c, void *meta, void *buf,
                         float *width, float *height, float *bearing_x, float *bearing_y,
                         float *advance) {
    msdfgen::FontHandle *font = (msdfgen::FontHandle *)f->__handle;
    msdfgen::Shape shape;
    msdfgen::loadGlyph(shape, font, c);
    
    unsigned char *metadata = (unsigned char *)meta;
    vec2 *point_data = (vec2 *)buf;


    size_t _p = 0;
    size_t _m = 0;

    shape.normalize();
    edgeColoringSimple(shape, 3.0);
    {
        metadata[_m++] = shape.contours.size();
        for (msdfgen::Contour &_c : shape.contours) {
            metadata[_m++] = (unsigned char)_c.winding() + 1;
            metadata[_m++] = _c.edges.size();

            _p++; /* The first segment should also have the first point */

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
    *width = (float)font->face->glyph->metrics.width / 64.0;
    *height = (float)font->face->glyph->metrics.height / 64.0;

    *bearing_x = (float)font->face->glyph->metrics.horiBearingX / 64.0;
    *bearing_y = (float)font->face->glyph->metrics.horiBearingY / 64.0;
    
    *advance = (float)font->face->glyph->metrics.horiAdvance / 64.0;
    return 0;
}
