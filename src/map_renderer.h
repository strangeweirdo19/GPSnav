// map_renderer.h
#ifndef MAP_RENDERER_H
#define MAP_RENDERER_H

#include "common.h" // Include common definitions
#include <Arduino.h> // For basic Arduino types like uint16_t

// Struct to hold icon bitmap properties (re-declared here for map_renderer.cpp)
struct IconBitmap {
    const uint16_t* bitmap;
    int width;
    int height;
};

// Icon drawing helper functions
void drawMonochromeIcon(int centerX, int centerY, uint16_t color, const IconBitmap& iconBitmap);
void drawTrafficSignalIcon(int centerX, int centerY, uint16_t color);
void drawIcon(IconType type, int centerX, int centerY, uint16_t color);

// Geometry drawing functions
int getRoadWidth(float zoomScaleFactor);
void renderRing(const std::vector<std::pair<int, int>, PSRAMAllocator<std::pair<int, int>>>& points, uint16_t color, bool isPolygon, int geomType, bool hasBridge, bool hasTunnel, float zoomScaleFactor);
void drawNavigationArrow(int centerX, int centerY, int size, uint16_t color);
void drawParsedFeature(const ParsedFeature& feature, int layerExtent, const TileKey& tileKey, const RenderParams& params);
void drawTriangleAndCircle(int centerX, int centerY, int size, uint16_t color, bool drawTriangle, bool drawCircle);

// Helper functions for tile coordinates (used by renderTask)
void latlonToTile(double lat, double lon, int z, int &x, int &y, int &ytms);
void latLonToMVTCoords(double lat, double lon, int z, int tileX, int tileY_TMS, int& mvtX, int& mvtY, int extent);

// Helper function to blend colors
uint16_t blendColors(uint16_t background, uint16_t foreground, float alpha);

// Perspective transformation for 3D navigation view
void applyPerspective(float x, float y, float &outX, float &outY, int pivotY);

// Anti-aliasing helper functions
float fpart(float x);
float rfpart(float x);
void drawPixelAlpha(int x, int y, uint16_t color, float alpha);
void drawAntiAliasedLine(int x0, int y0, int x1, int y1, uint16_t color);


// Removed extern declarations for icon dimensions from here, moved to common.h


#endif // MAP_RENDERER_H
