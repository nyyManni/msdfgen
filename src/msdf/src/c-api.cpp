

#include <cstring>
#include <lodepng.h>
#include <math.h>

#include "msdf.h"
#include "msdfgen-ext.h"
#include "msdfgen.h"

using namespace msdfgen;

msdf_font_handle msdf_load_font(const char *path) {
    static FreetypeHandle *ft = nullptr;
    if (!ft)
        ft = initializeFreetype();
    FontHandle *font = loadFont(ft, path);
    return (void *)font;
}

struct msdf_glyph *msdf_generate_glyph(msdf_font_handle f, int c, double range,
                                       float scale) {
    Shape shape;
    FontHandle *font = (FontHandle *)f;
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
                 Vector2(-bearing_x, height - bearing_y) +
                     Vector2(range / 2.0, range / 2.0),
                 1.001, true);

    struct msdf_glyph *g = new msdf_glyph();

    g->bitmap.width = msdf.width();
    g->bitmap.height = msdf.height();
    g->bitmap.channels = 3;
    g->bitmap.data = (float *)malloc(sizeof(FloatRGB) * msdf.width() * msdf.height());
    g->advance = (font->face->glyph->metrics.horiAdvance / 64.0) * scale;
    std::memcpy(g->bitmap.data, msdf.content, sizeof(FloatRGB) * msdf.width() * msdf.height());

    return g;
}

void msdf_release_glyph(struct msdf_glyph *g) {
    free(g->bitmap.data);
    delete g;
}

int msdf_dump_glyph(const msdf_glyph_handle g, const char *filename) {
    std::vector<unsigned char> pixels(g->bitmap.channels * g->bitmap.width * g->bitmap.height);
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
