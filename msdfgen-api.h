#ifndef MSDFGEN_API_H
#define MSDFGEN_API_H

/*
 * MULTI-CHANNEL SIGNED DISTANCE FIELD GENERATOR v1.6 (2019-04-08)
 * ---------------------------------------------------------------
 * A utility by Viktor Chlumsky, (c) 2014 - 2019
 *
 * The technique used to generate multi-channel distance fields in this code
 * has been developed by Viktor Chlumsky in 2014 for his master's thesis,
 * "Shape Decomposition for Multi-Channel Distance Fields". It provides improved
 * quality of sharp corners in glyphs and other 2D shapes in comparison to monochrome
 * distance fields. To reconstruct an image of the shape, apply the median of three
 * operation on the triplet of sampled distance field values.
 *
 * This file contains a simple C-API for generating glyph bitmaps.
 */

#ifdef __cplusplus
extern "C" {
#endif

  typedef void *msdf_font_handle;

  struct msdf_glyph {
    float *bitmap;
    int width;
    int height;
    int channels;
    float advance;
  };
  typedef struct msdf_glyph *msdf_glyph_handle;

  msdf_font_handle msdf_load_font(const char *path);
  
  msdf_glyph_handle
  msdf_generate_glyph(msdf_font_handle f, int c, double range, float scale);
  
  void msdf_release_glyph(msdf_glyph_handle g);

#ifdef __cplusplus
}
#endif

#endif /* MSDFGEN_API_H */
