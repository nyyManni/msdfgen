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
#include <ft2build.h>
#include FT_FREETYPE_H
#include FT_OUTLINE_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * MSDF glyph representation
 */
struct msdf_glyph {
    /** Unicode character code of this glyph */
    int c;

    /**
     * Bitmap storing the rendered MSDF data
     */
    struct {
        /** Pointer to the stored floating point array */
        float *data;
        /** X-dimension of the data array */
        int width;
        /** Y-dimension of the data array */
        int height;
        /** Z-dimension of the data array */
        int channels;
        /** Scale factor of the generated bitmap */
        float scale;
    } bitmap;

    /** The number of pixels this glyph should move the cursor forwards */
    float advance;
    /** Distance from the glyph origin to the bottom-left corner of the bitmap */
    float bearing[2];
    /** The size of the glyph (unrelated to the size of the bitmap) */
    float size[2];
    /** The padding caused by the thickening support */
    float padding;
};
typedef struct msdf_glyph *msdf_glyph_handle;

typedef void *msdf_font_handle;

/**
 * Load a FreeType font from a file.
 *
 * Returns a handle to an internal font type.
 */
msdf_font_handle msdf_load_font(const char *path);

/**
 * Generate a font handle from already created Freetype Face object.
 */
msdf_font_handle msdf_font_from_face(FT_Library lib, FT_Face face);

/**
 * Render an MSDF bitmap and fill in font metrics
 */
msdf_glyph_handle msdf_generate_glyph(msdf_font_handle f, int c, double range,
                                      float scale);
/**
 * Release memory allocated by the glyph.
 */
void msdf_release_glyph(msdf_glyph_handle g);

/**
 * Write the bitmap into a (.png) file.
 */
int msdf_dump_glyph(msdf_glyph_handle g, const char *filename);

#ifdef __cplusplus
}
#endif

#endif /* MSDFGEN_API_H */
