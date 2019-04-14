

#include <cstring>
#include <math.h>

#include "../msdfgen-api.h"
#include "../msdfgen-ext.h"
#include "../msdfgen.h"


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

  g->width = msdf.w;
  g->height = msdf.h;
  g->channels = 3;
  g->bitmap = (float *)malloc(sizeof(FloatRGB) * msdf.w * msdf.h);
  g->advance = (font->face->glyph->metrics.horiAdvance / 64.0) * scale;
  std::memcpy(g->bitmap, msdf.content, sizeof(FloatRGB) * msdf.w * msdf.h);

  return g;
}

void msdf_release_glyph(struct msdf_glyph *g) {
  free(g->bitmap);
  delete g;
}
