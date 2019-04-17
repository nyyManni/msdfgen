

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
    f->height = (f->ascender - f->descender) / f->xheight;

    f->underline_y = f->face->underline_position / 64.0 / f->xheight;
    f->underline_thickness = f->face->underline_thickness / 64.0 / f->xheight;

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

    shape.normalize();

    edgeColoringSimple(shape, 3.0);

    float width = font->face->glyph->metrics.width / 64.0;
    float height = font->face->glyph->metrics.height / 64.0;
    float bearing_x = font->face->glyph->metrics.horiBearingX / 64.0;
    float bearing_y = font->face->glyph->metrics.horiBearingY / 64.0;

    int bitmap_width = ceil((width + range) * scale);
    int bitmap_height = ceil((height + range) * scale);
    Bitmap<FloatRGB> msdf(bitmap_width, bitmap_height);

    generateMSDF(msdf, shape, range, scale,
                 Vector2(-bearing_x, height - bearing_y) + Vector2(range, range) / 2.0,
                 1.001, true);

    msdf_glyph_handle g = new msdf_glyph();

    g->code = c;
    g->index = font->face->glyph->reserved;
    g->bitmap.width = msdf.width();
    g->bitmap.height = msdf.height();
    g->bitmap.channels = 3;
    g->bitmap.scale = scale;
    g->bitmap.data = (float *)malloc(sizeof(FloatRGB) * msdf.width() * msdf.height());

    g->advance = (font->face->glyph->metrics.horiAdvance / 64.0) / f->xheight;
    g->bearing[0] = bearing_x / f->xheight;
    g->bearing[1] = bearing_y / f->xheight;
    g->size[0] = width / f->xheight;
    g->size[1] = height / f->xheight;
    g->padding = (range / 2.0) / f->xheight;

    std::memcpy(g->bitmap.data, msdf.content,
                sizeof(FloatRGB) * msdf.width() * msdf.height());

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
