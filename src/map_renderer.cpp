// map_renderer.cpp
#include "map_renderer.h"
#include "common.h"
#include "colors.h" // Include the new colors header
#include <cfloat>   // Required for FLT_MAX and FLT_MIN

// =========================================================
// ICON BITMAPS (Defined locally in map_renderer.cpp)
// =========================================================

// Fuel-16 Icon (Monochrome) - 10x16 pixels, vertically stored
// Each uint16_t represents a full column of 16 pixels (LSB is bottom pixel)
static const uint16_t Fuel_16_bit_map[] = {
    0x0001, // Column 0
    0x03ff, // Column 1
    0x027f, // Column 2
    0x027f, // Column 3
    0x027f, // Column 4
    0x027f, // Column 5
    0x03ff, // Column 6
    0x0005, // Column 7
    0x03fc, // Column 8
    0x0180  // Column 9
};

static const int FUEL_16_WIDTH = 10;  // Width of the fuel icon
static const int FUEL_16_HEIGHT = 16; // Height of the fuel icon

// Bus Stop Icon (Monochrome) - 10x16 pixels, vertically stored
// Each uint16_t represents a full column of 16 pixels (LSB is bottom pixel)
static const uint16_t Bus_bit_map[] = {
    0x013e, // Column 0
    0x017f, // Column 1
    0x014b, // Column 2
    0x014e, // Column 3
    0x014e, // Column 4
    0x014e, // Column 5
    0x014b, // Column 6
    0x017f, // Column 7
    0x013e, // Column 8
    0x0000  // Column 9
};

static const int BUS_WIDTH = 10;  // Width of the bus icon
static const int BUS_HEIGHT = 16; // Height of the bus icon

// GPS Icon (Monochrome) - 14x5 pixels, vertically stored
const uint16_t GPS_16_bit_map[] = {
    0x001f, // Column 0
    0x0011, // Column 1
    0x0015, // Column 2
    0x0017, // Column 3
    0x0000, // Column 4
    0x001f, // Column 5
    0x0014, // Column 6
    0x0014, // Column 7
    0x001c, // Column 8
    0x0000, // Column 9
    0x001d, // Column 10
    0x0015, // Column 11
    0x0015, // Column 12
    0x0017  // Column 13
};

// Removed 'static' keyword as these are declared extern in common.h
const int GPS_16_WIDTH = 14;  // Width of the GPS icon
const int GPS_16_HEIGHT = 5; // Height of the GPS icon

// Connected Icon (Monochrome) - 11x8 pixels, vertically stored
const uint16_t connected_16_bit_map[] = {
    0x0028, // Column 0
    0x002c, // Column 1
    0x002e, // Column 2
    0x002f, // Column 3
    0x0028, // Column 4
    0x0028, // Column 5
    0x0028, // Column 6
    0x01e8, // Column 7
    0x00e8, // Column 8
    0x0068, // Column 9
    0x0028  // Column 10
};

// Removed 'static' keyword as these are declared extern in common.h
const int CONNECTED_16_WIDTH = 11;  // Width of the connected icon
const int CONNECTED_16_HEIGHT = 8; // Height of the connected icon


// =========================================================
// ANTI-ALIASING HELPER FUNCTIONS
// =========================================================

// Helper for fractional part
float fpart(float x)
{
    return x - floor(x);
}

float rfpart(float x)
{
    return 1.0f - fpart(x);
}

// Draws a pixel with a given color and opacity, blending with the existing pixel color
void drawPixelAlpha(int x, int y, uint16_t color, float alpha)
{
    if (x >= 0 && x < screenW && y >= 0 && y < screenH)
    {
        uint16_t existingColor = sprite.readPixel(x, y);
        uint16_t blendedColor = blendColors(existingColor, color, alpha);
        sprite.drawPixel(x, y, blendedColor);
    }
}

// Xiaolin Wu's line algorithm for anti-aliased lines
void drawAntiAliasedLine(int x0, int y0, int x1, int y1, uint16_t color)
{
    bool steep = abs(y1 - y0) > abs(x1 - x0);

    if (steep)
    {
        std::swap(x0, y0);
        std::swap(x1, y1);
    }
    if (x0 > x1)
    {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    float dx = x1 - x0;
    float dy = y1 - y0;
    float gradient = (dx == 0) ? 1.0f : dy / dx;

    // ===== HANDLE FIRST ENDPOINT =====
    float xend0 = round(x0);
    float yend0 = y0 + gradient * (xend0 - x0);
    float xgap0 = rfpart(x0 + 0.5f);
    
    int xpx0 = (int)xend0;
    int ypx0_low = (int)floor(yend0);
    int ypx0_high = ypx0_low + 1;
    
    if (steep)
    {
        drawPixelAlpha(ypx0_low, xpx0, color, rfpart(yend0) * xgap0);
        drawPixelAlpha(ypx0_high, xpx0, color, fpart(yend0) * xgap0);
    }
    else
    {
        drawPixelAlpha(xpx0, ypx0_low, color, rfpart(yend0) * xgap0);
        drawPixelAlpha(xpx0, ypx0_high, color, fpart(yend0) * xgap0);
    }

    float intery = yend0 + gradient;

    // ===== HANDLE SECOND ENDPOINT =====
    float xend1 = round(x1);
    float yend1 = y1 + gradient * (xend1 - x1);
    float xgap1 = fpart(x1 + 0.5f);
    
    int xpx1 = (int)xend1;
    int ypx1_low = (int)floor(yend1);
    int ypx1_high = ypx1_low + 1;
    
    if (steep)
    {
        drawPixelAlpha(ypx1_low, xpx1, color, rfpart(yend1) * xgap1);
        drawPixelAlpha(ypx1_high, xpx1, color, fpart(yend1) * xgap1);
    }
    else
    {
        drawPixelAlpha(xpx1, ypx1_low, color, rfpart(yend1) * xgap1);
        drawPixelAlpha(xpx1, ypx1_high, color, fpart(yend1) * xgap1);
    }

    // ===== MAIN LOOP =====
    int xstart = (int)xend0 + 1;
    int xstop = (int)xend1;
    
    for (int x = xstart; x < xstop; ++x)
    {
        int py_low = (int)floor(intery);
        int py_high = py_low + 1;
        
        if (steep)
        {
            drawPixelAlpha(py_low, x, color, rfpart(intery));
            drawPixelAlpha(py_high, x, color, fpart(intery));
        }
        else
        {
            drawPixelAlpha(x, py_low, color, rfpart(intery));
            drawPixelAlpha(x, py_high, color, fpart(intery));
        }
        intery += gradient;
    }
}

// =========================================================
// ICON DRAWING HELPER FUNCTIONS
// =========================================================

// Function to draw a monochrome icon from its bit map
void drawMonochromeIcon(int centerX, int centerY, uint16_t color, const IconBitmap &iconBitmap)
{
    // Calculate top-left corner to center the icon
    int startX = centerX - (iconBitmap.width / 2);
    int startY = centerY - (iconBitmap.height / 2);

    for (int col = 0; col < iconBitmap.width; ++col)
    {
        // Get the 16-bit data for the current column
        // Each bit represents a pixel in the column, from bottom (LSB) to top (MSB)
        uint16_t column_data = iconBitmap.bitmap[col];

        for (int row = 0; row < iconBitmap.height; ++row)
        {
            // Check if the bit at the current 'row' position (from bottom) is set
            if ((column_data >> row) & 0x0001)
            {
                // To map the bottom-up bitmap data to the screen's top-down Y-coordinate system:
                // The pixel at 'row' (0-15, where 0 is bottom-most)
                // corresponds to screen Y: startY + (iconBitmap.height - 1 - row)
                sprite.drawPixel(startX + col, startY + (iconBitmap.height - 1 - row), color);
            }
        }
    }
}

// New function to draw a traffic signal icon
void drawTrafficSignalIcon(int centerX, int centerY, uint16_t color)
{
    // Traffic signal body - Fixed dimensions as requested (7x16)
    const int bodyWidth = 7;
    const int bodyHeight = 16;
    sprite.fillRect(centerX - bodyWidth / 2, centerY - bodyHeight / 2, bodyWidth, bodyHeight, TRAFFIC_SIGNAL_BODY_COLOR);

    // Define the size of the circles - Fixed to 5x5 pixels (radius 2) as requested
    const int CIRCLE_LIGHT_RADIUS = 2;

    // Calculate positions for the three lights (circles)
    // Positioned to be centered horizontally and distributed vertically within the 16-pixel body.
    // We'll use a spacing that divides the body height into equal segments for the light centers.

    // Red light (top) - moved 1 pixel up
    sprite.fillCircle(centerX, centerY - bodyHeight / 2 + (bodyHeight / 4) - 1, CIRCLE_LIGHT_RADIUS, TRAFFIC_SIGNAL_RED);

    // Yellow light (middle)
    // Positioned at the vertical center of the body
    sprite.fillCircle(centerX, centerY, CIRCLE_LIGHT_RADIUS, TRAFFIC_SIGNAL_YELLOW);

    // Green light (bottom)
    // Positioned at 3/4th of the body height from the top (or 1/4th from the bottom)
    sprite.fillCircle(centerX, centerY + bodyHeight / 2 - (bodyHeight / 4), CIRCLE_LIGHT_RADIUS, TRAFFIC_SIGNAL_GREEN);
}

// Common function to draw icons based on IconType
void drawIcon(IconType type, int centerX, int centerY, uint16_t color)
{
    // Static map to store IconBitmap data, initialized once
    static const std::map<IconType, IconBitmap> iconBitmaps = {
        {IconType::Fuel, {Fuel_16_bit_map, FUEL_16_WIDTH, FUEL_16_HEIGHT}},
        {IconType::BusStop, {Bus_bit_map, BUS_WIDTH, BUS_HEIGHT}},
        {IconType::GPS, {GPS_16_bit_map, GPS_16_WIDTH, GPS_16_HEIGHT}},
        {IconType::Connected, {connected_16_bit_map, CONNECTED_16_WIDTH, CONNECTED_16_HEIGHT}}
    };

    if (type == IconType::TrafficSignal) {
        drawTrafficSignalIcon(centerX, centerY, color);
        return;
    }

    auto it = iconBitmaps.find(type);
    if (it != iconBitmaps.end()) {
        drawMonochromeIcon(centerX, centerY, color, it->second);
    }
    // No action for IconType::None or unknown types not found in the map.
}

// Function to determine road width based on zoom scale factor
// This function now provides specific pixel widths for discrete zoom levels (1x, 2x, 3x, 4x)
// to allow for more precise control over road appearance.
int getRoadWidth(float zoomScaleFactor)
{
    // Round the zoomScaleFactor to the nearest integer to treat discrete zoom levels
    int discreteZoom = static_cast<int>(round(zoomScaleFactor));

    switch (discreteZoom)
    {
    case 1:
        return 1; // Zoom 1x: 1 pixel wide
    case 2:
        return 2; // Zoom 2x: 2 pixels wide
    case 3:
        return 3; // Zoom 3x: 3 pixels wide
    case 4:
        return 4; // Zoom 4x: 4 pixels wide (decreased from 5)
    default:
        return 1; // Default to 1 pixel for any other zoom factors
    }
}

// =========================================================
// GEOMETRY DRAWING (Render Task will use this)
// =========================================================
// This function is now only used for rendering pre-parsed geometry.
// It takes a set of screen-space points and draws/fills them.
void renderRing(const std::vector<std::pair<int, int>, PSRAMAllocator<std::pair<int, int>>> &points, uint16_t color, bool isPolygon, int geomType, bool hasBridge, bool hasTunnel, float zoomScaleFactor)
{
    if (points.empty())
        return;

    if (geomType == 1)
    { // Explicitly handle Point geometry (which includes MultiPoint, where each "ring" is a single point)
        for (const auto &p : points)
        { // Iterate through all points in the vector
            int px = p.first;
            int py = p.second;
            sprite.fillRect(px - POINT_FEATURE_SIZE / 2, py - POINT_FEATURE_SIZE / 2, POINT_FEATURE_SIZE, POINT_FEATURE_SIZE, color);
        }
    }
    else if (geomType == 2)
    { // Explicitly handle LineString geometry (roads, waterways)
        if (points.size() > 1)
        {
            int lineWidth = getRoadWidth(zoomScaleFactor); // Get dynamic line width

            // If it's a bridge (and not a tunnel), draw the black border first
            if (hasBridge && !hasTunnel)
            {
                for (size_t k = 0; k < points.size() - 1; ++k)
                {
                    int x1 = points[k].first;
                    int y1 = points[k].second;
                    int x2 = points[k + 1].first;
                    int y2 = points[k + 1].second;

                    float dx = (float)(x2 - x1);
                    float dy = (float)(y2 - y1);
                    float length = sqrt(dx * dx + dy * dy);

                    if (length > 0)
                    {
                        float perp_dx_norm = dy / length;
                        float perp_dy_norm = -dx / length;

                        // Calculate the border offsets based on lineWidth
                        float halfBorderOffset;
                        if (lineWidth == 1)
                        {
                            halfBorderOffset = 1.0f; // 1px border on each side for 1px road
                        }
                        else if (lineWidth == 2)
                        {
                            halfBorderOffset = 2.0f; // 1px left, 2px right for 2px road (total 3px width)
                        }
                        else
                        {                                                            // For 3, 5 pixels
                            halfBorderOffset = (float)(lineWidth - 1) / 2.0f + 1.0f; // 1px border on each side
                        }

                        // Calculate the 4 corner points of the border rectangle
                        int p1x_b = round(x1 - perp_dx_norm * halfBorderOffset);
                        int p1y_b = round(y1 - perp_dy_norm * halfBorderOffset);
                        int p2x_b = round(x2 - perp_dx_norm * halfBorderOffset);
                        int p2y_b = round(y2 - perp_dy_norm * halfBorderOffset);
                        int p3x_b = round(x2 + perp_dx_norm * halfBorderOffset);
                        int p3y_b = round(y2 + perp_dy_norm * halfBorderOffset);
                        int p4x_b = round(x1 + perp_dx_norm * halfBorderOffset);
                        int p4y_b = round(y1 + perp_dy_norm * halfBorderOffset);

                        // Draw the black border rectangle using two triangles
                        sprite.fillTriangle(p1x_b, p1y_b, p2x_b, p2y_b, p3x_b, p3y_b, BRIDGE_BORDER_COLOR);
                        sprite.fillTriangle(p1x_b, p1y_b, p3x_b, p3y_b, p4x_b, p4y_b, BRIDGE_BORDER_COLOR);

                        // Now anti-alias the *edges* of this filled border
                        drawAntiAliasedLine(p1x_b, p1y_b, p2x_b, p2y_b, BRIDGE_BORDER_COLOR); // Top edge
                        drawAntiAliasedLine(p4x_b, p4y_b, p3x_b, p3y_b, BRIDGE_BORDER_COLOR); // Bottom edge
                    }
                }
            }

            // Now draw the main road on top
            if (lineWidth == 1)
            {
                for (size_t k = 0; k < points.size() - 1; ++k)
                {
                    drawAntiAliasedLine(points[k].first, points[k].second, points[k + 1].first, points[k + 1].second, color);
                }
            }
            else
            { // For lineWidth 2, 3, 5 (or any other > 1)
                float halfWidthOffset = (float)(lineWidth - 1) / 2.0f;
                for (size_t k = 0; k < points.size() - 1; ++k)
                {
                    int x1 = points[k].first;
                    int y1 = points[k].second;
                    int x2 = points[k + 1].first;
                    int y2 = points[k + 1].second;

                    float dx = (float)(x2 - x1);
                    float dy = (float)(y2 - y1);
                    float length = sqrt(dx * dx + dy * dy);

                    if (length > 0)
                    {
                        float perp_dx_norm = dy / length;
                        float perp_dy_norm = -dx / length;

                        int p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y;

                        if (lineWidth == 2)
                        {
                            // For 2-pixel line, define points to cover exactly two pixels.
                            // One side is the original line, the other is offset by 1 pixel.
                            p1x = x1;
                            p1y = y1;
                            p2x = x2;
                            p2y = y2;
                            p3x = round(x2 + perp_dx_norm * 1.0f); // Offset by 1 pixel
                            p3y = round(y2 + perp_dy_norm * 1.0f);
                            p4x = round(x1 + perp_dx_norm * 1.0f); // Offset by 1 pixel
                            p4y = round(y1 + perp_dy_norm * 1.0f);
                        }
                        else
                        {
                            // For other widths (3, 5), use the standard halfWidthOffset calculation
                            p1x = round(x1 - perp_dx_norm * halfWidthOffset);
                            p1y = round(y1 - perp_dy_norm * halfWidthOffset);
                            p2x = round(x2 - perp_dx_norm * halfWidthOffset);
                            p2y = round(y2 - perp_dy_norm * halfWidthOffset);
                            p3x = round(x2 + perp_dx_norm * halfWidthOffset);
                            p3y = round(y2 + perp_dy_norm * halfWidthOffset);
                            p4x = round(x1 + perp_dx_norm * halfWidthOffset);
                            p4y = round(y1 + perp_dy_norm * halfWidthOffset);
                        }

                        // Draw the rectangle using two triangles (fill the body)
                        sprite.fillTriangle(p1x, p1y, p2x, p2y, p3x, p3y, color);
                        sprite.fillTriangle(p1x, p1y, p3x, p3y, p4x, p4y, color);

                        // Now anti-alias the *edges* of this filled line
                        drawAntiAliasedLine(p1x, p1y, p2x, p2y, color); // Top edge
                        drawAntiAliasedLine(p4x, p4y, p3x, p3y, color); // Bottom edge
                    }
                    else
                    { // Handle single point case for line (x1==x2 && y1==y2)
                        sprite.drawPixel(x1, y1, color);
                    }
                }
            }
        }
        else if (points.size() == 1)
        { // A single point in a LineString is still a point
            sprite.drawPixel(points[0].first, points[0].second, color);
        }
    }
    else if (geomType == 3)
    { // Explicitly handle Polygon geometry
        // Fill the polygon first
        int minY = screenH, maxY = 0;
        for (const auto &p : points)
        {
            if (p.second < minY)
                minY = p.second;
            if (p.second > maxY)
                maxY = p.second;
        }

        minY = std::max(0, minY);
        maxY = std::min(screenH - 1, maxY);

        std::vector<int, PSRAMAllocator<int>> intersections_psram{PSRAMAllocator<int>()};
        intersections_psram.reserve(points.size());

        for (int scanY = minY; scanY <= maxY; ++scanY)
        {
            intersections_psram.clear();

            for (size_t k = 0; k < points.size(); ++k)
            {
                int x1 = points[k].first;
                int y1 = points[k].second;
                int x2 = points[(k + 1) % points.size()].first;
                int y2 = points[(k + 1) % points.size()].second;

                if (y1 == y2)
                    continue;
                if ((scanY < std::min(y1, y2)) || (scanY >= std::max(y1, y2)))
                    continue;

                float intersectX = (float)(x2 - x1) * (scanY - y1) / (float)(y2 - y1) + x1;
                intersections_psram.push_back(round(intersectX));
            }
            std::sort(intersections_psram.begin(), intersections_psram.end());

            for (size_t k = 0; k + 1 < intersections_psram.size(); k += 2)
            {
                int startX = intersections_psram[k];
                int endX = intersections_psram[k + 1];
                if (startX > endX)
                    std::swap(startX, endX);

                startX = std::max(0, startX);
                endX = std::min(screenW - 1, endX);

                if (startX <= endX)
                {
                    sprite.drawFastHLine(startX, scanY, endX - startX + 1, color);
                }
            }
        }
        // Draw the anti-aliased outline of the polygon
        if (points.size() > 1)
        {
            for (size_t k = 0; k < points.size() - 1; ++k)
            {
                drawAntiAliasedLine(points[k].first, points[k].second, points[k + 1].first, points[k + 1].second, color);
            }
            // Close the path for polygons with an anti-aliased line
            if (points.front().first != points.back().first || points.front().second != points.back().second)
            {
                drawAntiAliasedLine(points.back().first, points.back().second, points.front().first, points.front().second, color);
            }
        }
        else if (points.size() == 1)
        { // A single point in a Polygon is still a point
            sprite.drawPixel(points[0].first, points[0].second, color);
        }
    }
    else
    { // Fallback for unknown geomType, draw as points
        if (!points.empty())
        {
            sprite.fillRect(points[0].first - POINT_FEATURE_SIZE / 2, points[0].second - POINT_FEATURE_SIZE / 2, POINT_FEATURE_SIZE, POINT_FEATURE_SIZE, color);
        }
    }
}

// Custom function to draw the navigation arrow using two triangles
void drawNavigationArrow(int centerX, int centerY, int size, uint16_t color)
{
    int halfSize = size / 2;

    // Top point of the triangle
    int x0 = centerX;
    int y0 = centerY - size;

    // Bottom left
    int x1 = centerX - halfSize;
    int y1 = centerY + halfSize;

    // Bottom right of the top triangle (mid-point of the arrow's base)
    int x2 = centerX;
    int y2 = centerY + halfSize / 1.5; // Adjusted for a slightly wider base

    // Right point for the bottom triangle
    int x3 = centerX + halfSize;

    // Draw the left triangle of the arrow
    sprite.fillTriangle(x0, y0, x1, y1, x2, y2, color);
    // Draw the right triangle of the arrow
    sprite.fillTriangle(x0, y0, x3, y1, x2, y2, color);
}

// New function to draw a pre-parsed feature. It transforms MVT coordinates
// to screen coordinates and then calls renderRing. It now takes a TileKey
// to correctly offset features from neighboring tiles.
void drawParsedFeature(const ParsedFeature &feature, int layerExtent, const TileKey &tileKey, const RenderParams &params)
{
    float scaleX = (float)screenW * params.zoomScaleFactor / layerExtent;
    float scaleY = (float)screenH * params.zoomScaleFactor / layerExtent;

    // Calculate the pixel offset for this specific tile relative to the central tile (params.centralTileX, params.centralTileY_TMS)
    // A full MVT extent (e.g., 4096 units) maps to `screenW` pixels at scale 1.0.
    // When zoomed, it maps to `screenW * params.zoomScaleFactor` pixels.
    float tileRenderWidth = (float)screenW * params.zoomScaleFactor;
    float tileRenderHeight = (float)screenH * params.zoomScaleFactor;

    // Corrected Y-axis offset calculation for TMS Y (which increases upwards) to screen Y (which increases downwards).
    // If tileKey.y_tms is smaller (tile is "above" in TMS, meaning it's "below" in standard Y/screen Y),
    // then (params.centralTileY_TMS - tileKey.y_tms) will be positive, shifting content downwards.
    // If tileKey.y_tms is larger (tile is "below" in TMS, meaning it's "above" in standard Y/screen Y),
    // then (params.centralTileY_TMS - tileKey.y_tms) will be negative, shifting content upwards.
    float tileRenderOffsetX_float = (float)(tileKey.x - params.centralTileX) * tileRenderWidth;
    float tileRenderOffsetY_float = (float)(params.centralTileY_TMS - tileKey.y_tms) * tileRenderHeight;

    // Define the center of rotation (screen center X, and arrow's tip Y)
    int centerX = screenW / 2;
    int centerY = params.pivotY; // Reverted to use params.pivotY (arrow's tip Y) as rotation center

    // Get the four corner points of the feature's MVT bounding box in screen-space (pre-rotation)
    float screenMinX_float_pre_rot = feature.minX_mvt * scaleX + tileRenderOffsetX_float - params.displayOffsetX;
    float screenMinY_float_pre_rot = feature.minY_mvt * scaleY + tileRenderOffsetY_float - params.displayOffsetY;
    float screenMaxX_float_pre_rot = feature.maxX_mvt * scaleX + tileRenderOffsetX_float - params.displayOffsetX;
    float screenMaxY_float_pre_rot = feature.maxY_mvt * scaleY + tileRenderOffsetY_float - params.displayOffsetY;

    // --- Bounding Box Culling Optimization ---
    // Dynamic culling buffers based on a percentage of the scaled screen dimension
    // This makes the buffer size adapt to zoom level.
    int cullingBufferLeft = round(screenW * params.zoomScaleFactor * params.cullingBufferPercentageLeft);
    int cullingBufferRight = round(screenW * params.zoomScaleFactor * params.cullingBufferPercentageRight);
    int cullingBufferTop = round(screenH * params.zoomScaleFactor * params.cullingBufferPercentageTop);
    int cullingBufferBottom = round(screenH * params.zoomScaleFactor * params.cullingBufferPercentageBottom);

    // Apply minimum buffer to ensure some safety margin even at very low zooms
    cullingBufferLeft = std::max(cullingBufferLeft, MIN_CULLING_BUFFER_PIXELS);
    cullingBufferRight = std::max(cullingBufferRight, MIN_CULLING_BUFFER_PIXELS);
    cullingBufferTop = std::max(cullingBufferTop, MIN_CULLING_BUFFER_PIXELS);
    cullingBufferBottom = std::max(cullingBufferBottom, MIN_CULLING_BUFFER_PIXELS);

    // Perform culling check using the unrotated bounding box and independent buffers
    if (screenMaxX_float_pre_rot < -cullingBufferLeft ||
        screenMinX_float_pre_rot > screenW + cullingBufferRight ||
        screenMaxY_float_pre_rot < -cullingBufferTop ||
        screenMinY_float_pre_rot > screenH + cullingBufferBottom)
    {
        return; // Cull feature if its unrotated bounding box is completely outside the buffered screen area
    }

    // Calculate cosine and sine of the rotation angle (in radians)
    float cosTheta = cos(radians(params.mapRotationDegrees));
    float sinTheta = sin(radians(params.mapRotationDegrees));

    for (const auto &ring : feature.geometryRings)
    {
        // This vector needs to use the PSRAMAllocator to match the renderRing function's signature
        std::vector<std::pair<int, int>, PSRAMAllocator<std::pair<int, int>>> screenPoints{PSRAMAllocator<std::pair<int, int>>()};
        try
        {
            screenPoints.reserve(ring.size()); // Pre-allocate memory

            for (const auto &p : ring)
            {
                // Convert MVT (x,y) to screen (x,y) coordinates
                float screen_x = (float)p.first * scaleX + tileRenderOffsetX_float - params.displayOffsetX;
                float screen_y = (float)p.second * scaleY + tileRenderOffsetY_float - params.displayOffsetY;

                // Translate point so that the center of rotation is at the origin
                float translatedX = screen_x - centerX;
                float translatedY = screen_y - centerY;

                // Apply 2D rotation transformation
                float rotatedX = translatedX * cosTheta - translatedY * sinTheta;
                float rotatedY = translatedX * sinTheta + translatedY * cosTheta;

                // Move back to screen coordinates
                float finalX = rotatedX + centerX;
                float finalY = rotatedY + centerY;
                
                // Apply perspective transformation in screen space AFTER rotation (only at zoom 3+)
                if (params.zoomScaleFactor >= 3.0f) {
                    float perspectiveX, perspectiveY;
                    applyPerspective(finalX, finalY, perspectiveX, perspectiveY, params.pivotY);
                    screenPoints.push_back({round(perspectiveX), round(perspectiveY)});
                } else {
                    // No perspective at lower zoom levels
                    screenPoints.push_back({round(finalX), round(finalY)});
                }
            }

            if (!screenPoints.empty() && feature.geomType == 1)
            { // Apply only to point geometries
                // Define icon names once
                static const PSRAMString PSRAM_TRAFFIC_SIGNALS_ICON("traffic_signals", PSRAMAllocator<char>());
                static const PSRAMString PSRAM_FUEL_ICON("fuel", PSRAMAllocator<char>());
                static const PSRAMString PSRAM_BUS_STOP_ICON("bus_stop", PSRAMAllocator<char>());

                // FILTER: Hide all POI dots at zoom 1 & 2, except traffic signals
                if (params.zoomScaleFactor <= 2.5f && feature.iconName != PSRAM_TRAFFIC_SIGNALS_ICON) {
                    return; // Skip rendering this feature
                }

                // Iterate over all points in the screenPoints vector for point features
                for (const auto &p : screenPoints)
                {
                    int iconCenterX = p.first;
                    int iconCenterY = p.second;

                    if (feature.iconName == PSRAM_TRAFFIC_SIGNALS_ICON)
                    {
                        drawIcon(IconType::TrafficSignal, iconCenterX, iconCenterY, feature.color);
                    }
                    else if (feature.iconName == PSRAM_FUEL_ICON)
                    {
                        drawIcon(IconType::Fuel, iconCenterX, iconCenterY, feature.color);
                    }
                    else if (feature.iconName == PSRAM_BUS_STOP_ICON)
                    {
                        drawIcon(IconType::BusStop, iconCenterX, iconCenterY, feature.color);
                    }
                    else
                    {
                        // If it's a point feature but not a special icon type,
                        // render it as a regular point using renderRing.
                        // Create a temporary vector with just this single point.
                        std::vector<std::pair<int, int>, PSRAMAllocator<std::pair<int, int>>> singlePointVec{PSRAMAllocator<std::pair<int, int>>()};
                        singlePointVec.push_back(p);
                        renderRing(singlePointVec, feature.color, feature.isPolygon, feature.geomType, feature.hasBridge, feature.hasTunnel, params.zoomScaleFactor); // Pass hasBridge, hasTunnel and zoomScaleFactor
                    }
                }
            }
            else
            {
                // For non-point geometries, or if screenPoints is empty, draw normally
                renderRing(screenPoints, feature.color, feature.isPolygon, feature.geomType, feature.hasBridge, feature.hasTunnel, params.zoomScaleFactor); // Pass geomType, hasBridge, hasTunnel and zoomScaleFactor
            }
        }
        catch (const std::bad_alloc &e)
        {
            Serial.printf("❌ drawParsedFeature: Memory allocation failed for screenPoints: %s\n", e.what());
            // Skip drawing this ring
        }
    }
}

// Converts Latitude/Longitude to XYZ tile coordinates (and TMS Y coordinate)
void latlonToTile(double lat, double lon, int z, int &x, int &y, int &ytms)
{
    double latRad = radians(lat);
    int n = 1 << z; // Number of tiles in one dimension at zoom level z
    x = int((lon + 180.0) / 360.0 * n);
    y = int((1.0 - log(tan(latRad) + 1.0 / cos(latRad)) / PI) / 2.0 * n);
    ytms = n - 1 - y; // Convert standard Y to TMS Y (used by MBTiles)
}

// Converts Latitude/Longitude to pixel coordinates (0-255) within its containing tile
// And then scales it to the MVT extent (e.g., 0-4095)
void latLonToMVTCoords(double lat, double lon, int z, int tileX, int tileY_TMS, int &mvtX, int &mvtY, int extent)
{
    double n = pow(2, z);

    // Global pixel coordinates at Z zoom level (assuming 256x256 pixel tiles for standard mapping)
    // Adjust for TMS Y to standard Y for global pixel calculation
    double globalPx = ((lon + 180.0) / 360.0) * n * 256;
    double globalPy = (1.0 - log(tan(radians(lat)) + 1.0 / cos(radians(lat))) / PI) / 2.0 * n * 256;

    // Pixel coordinate within the current tile (0-255) relative to the tile's top-left corner
    double pixelInTileX_256 = globalPx - (tileX * 256);
    // Convert TMS Y to standard Y for the tile's global pixel origin
    double pixelInTileY_256 = globalPy - (((1 << z) - 1 - tileY_TMS) * 256);

    // Convert pixel-in-tile (0-255) to MVT extent coordinates (0-extent)
    mvtX = round(pixelInTileX_256 * (extent / 256.0));
    mvtY = round(pixelInTileY_256 * (extent / 256.0));

    // Clamp to extent boundaries if floating point math results in slightly out of bounds values
    mvtX = std::max(0, std::min(extent - 1, mvtX));
    mvtY = std::max(0, std::min(extent - 1, mvtY));
}

// Helper function to blend two RGB565 colors with a given alpha
uint16_t blendColors(uint16_t background, uint16_t foreground, float alpha)
{
    // Extract R, G, B components from background color
    uint8_t bg_r = (background >> 11) & 0x1F; // 5 bits
    uint8_t bg_g = (background >> 5) & 0x3F;  // 6 bits
    uint8_t bg_b = background & 0x1F;         // 5 bits

    // Extract R, G, B components from foreground color (black in this case)
    uint8_t fg_r = (foreground >> 11) & 0x1F;
    uint8_t fg_g = (foreground >> 5) & 0x3F;
    uint8_t fg_b = foreground & 0x1F;

    // Perform linear interpolation for each component
    uint8_t blended_r = static_cast<uint8_t>(bg_r * (1.0f - alpha) + fg_r * alpha);
    uint8_t blended_g = static_cast<uint8_t>(bg_g * (1.0f - alpha) + fg_g * alpha);
    uint8_t blended_b = static_cast<uint8_t>(bg_b * (1.0f - alpha) + fg_b * alpha);

    // Recombine into a 16-bit RGB565 color
    return ((blended_r & 0x1F) << 11) | ((blended_g & 0x3F) << 5) | (blended_b & 0x1F);
}

// Apply perspective transformation to create 3D navigation view
// Using proper perspective projection to keep straight lines straight and maintain road separation
void applyPerspective(float x, float y, float &outX, float &outY, int pivotY)
{
    // Perspective parameters - PRIORITY: maintain road separation
    const float viewerHeight = screenH * 5.0f;  // Very high camera = minimal horizontal convergence
    const float viewerDistance = screenH * 2.0f; // Very far distance = extremely gentle perspective
    const float tiltAngle = 45.0f; // Shallow tilt angle for best parallelism
    
    // Convert screen Y to a depth coordinate (bottom = near, top = far)
    // Map screen space to 3D world space
    float screenCenterX = screenW / 2.0f;
    float screenCenterY = (float)pivotY; // Use arrow tip as pivot point
    
    // Translate to origin
    float dx = x - screenCenterX;
    float dy = y - screenCenterY;
    
    // Calculate the Z depth based on Y position
    // Further up the screen = farther away in 3D space
    float tiltRad = radians(tiltAngle);
    float z = viewerDistance + (dy * cos(tiltRad));
    
    // Apply perspective division (homogeneous coordinates)
    // This preserves straight lines in 3D space
    float perspectiveFactor = viewerHeight / (viewerHeight + z);
    perspectiveFactor = constrain(perspectiveFactor, 0.92f, 1.0f); // Minimal convergence - roads stay nearly parallel
    
    // Apply perspective scaling with minimal horizontal convergence
    outX = screenCenterX + (dx * perspectiveFactor);
    
    // For Y, we need to account for the tilt and foreshortening
    // Maintains vertical separation between roads
    float projectedY = dy * cos(tiltRad) * perspectiveFactor;
    outY = screenCenterY + projectedY;
}
