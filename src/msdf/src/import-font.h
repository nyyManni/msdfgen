
#pragma once

#include <cstdlib>
#include "Shape.h"
#include <ft2build.h>
#include FT_FREETYPE_H
#include FT_OUTLINE_H

namespace msdfgen {

class FreetypeHandle;
class FontHandle {
    friend FontHandle * loadFont(FreetypeHandle *library, const char *filename);
    friend void destroyFont(FontHandle *font);
    friend bool getFontScale(double &output, FontHandle *font);
    friend bool getFontWhitespaceWidth(double &spaceAdvance, double &tabAdvance, FontHandle *font);
    friend bool loadGlyph(Shape &output, FontHandle *font, int unicode, double *advance);
    friend bool getKerning(double &output, FontHandle *font, int unicode1, int unicode2);
public:
    FT_Face face;

};
class FreetypeHandle {
    friend FreetypeHandle * initializeFreetype();
    friend void deinitializeFreetype(FreetypeHandle *library);
    friend FontHandle * loadFont(FreetypeHandle *library, const char *filename);

public:
    FT_Library library;

};

/// Initializes the FreeType library
FreetypeHandle * initializeFreetype();
/// Deinitializes the FreeType library
void deinitializeFreetype(FreetypeHandle *library);
/// Loads a font file and returns its handle
FontHandle * loadFont(FreetypeHandle *library, const char *filename);
/// Unloads a font file
void destroyFont(FontHandle *font);
/// Returns the size of one EM in the font's coordinate system
bool getFontScale(double &output, FontHandle *font);
/// Returns the width of space and tab
bool getFontWhitespaceWidth(double &spaceAdvance, double &tabAdvance, FontHandle *font);
/// Loads the shape prototype of a glyph from font file
bool loadGlyph(Shape &output, FontHandle *font, int unicode, double *advance = NULL);
/// Returns the kerning distance adjustment between two specific glyphs.
bool getKerning(double &output, FontHandle *font, int unicode1, int unicode2);

}
