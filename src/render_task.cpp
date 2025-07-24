// render_task.cpp
#include "render_task.h"
#include "common.h"
#include <cfloat> // Required for FLT_MAX and FLT_MIN

// HMC5883L compass object - changed to Adafruit_HMC5883_Unified
Adafruit_HMC5883_Unified hmc5883l = Adafruit_HMC5883_Unified(12345); // Using a sensor ID, common for unified sensors

// Global variables for compass heading filtering (moving average)
std::vector<float> headingReadings; // This vector is small, can stay in internal RAM
// FILTER_WINDOW_SIZE is now defined in common.h

// =========================================================
// ICON BITMAPS (Defined locally in render_task.cpp)
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

static const int FUEL_16_WIDTH = 10; // Width of the fuel icon
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

static const int BUS_WIDTH = 10; // Width of the bus icon
static const int BUS_HEIGHT = 16; // Height of the bus icon

// =========================================================
// ICON DRAWING HELPER FUNCTIONS
// =========================================================

// Function to draw a monochrome icon from its bit map
void drawMonochromeIcon(int centerX, int centerY, uint16_t color, const IconBitmap& iconBitmap) {
    // Calculate top-left corner to center the icon
    int startX = centerX - (iconBitmap.width / 2);
    int startY = centerY - (iconBitmap.height / 2);

    for (int col = 0; col < iconBitmap.width; ++col) {
        // Get the 16-bit data for the current column
        // Each bit represents a pixel in the column, from bottom (LSB) to top (MSB)
        uint16_t column_data = iconBitmap.bitmap[col];

        for (int row = 0; row < iconBitmap.height; ++row) {
            // Check if the bit at the current 'row' position (from bottom) is set
            if ((column_data >> row) & 0x0001) {
                // To map the bottom-up bitmap data to the screen's top-down Y-coordinate system:
                // The pixel at 'row' (0-15, where 0 is bottom-most)
                // corresponds to screen Y: startY + (total_height - 1 - row)
                sprite.drawPixel(startX + col, startY + (iconBitmap.height - 1 - row), color);
            }
        }
    }
}

// New function to draw a traffic signal icon
void drawTrafficSignalIcon(int centerX, int centerY, uint16_t color) {
    // Traffic signal body - Fixed dimensions as requested (7x16)
    const int bodyWidth = 7;
    const int bodyHeight = 16;
    sprite.fillRect(centerX - bodyWidth / 2, centerY - bodyHeight / 2, bodyWidth, bodyHeight, TFT_DARKGREY);

    // Define the size of the circles - Fixed to 5x5 pixels (radius 2) as requested
    const int CIRCLE_LIGHT_RADIUS = 2;

    // Calculate positions for the three lights (circles)
    // Positioned to be centered horizontally and distributed vertically within the 16-pixel body.
    // We'll use a spacing that divides the body height into equal segments for the light centers.

    // Red light (top) - moved 1 pixel up
    sprite.fillCircle(centerX, centerY - bodyHeight / 2 + (bodyHeight / 4) - 1, CIRCLE_LIGHT_RADIUS, TFT_RED);

    // Yellow light (middle)
    // Positioned at the vertical center of the body
    sprite.fillCircle(centerX, centerY, CIRCLE_LIGHT_RADIUS, TFT_YELLOW);

    // Green light (bottom)
    // Positioned at 3/4th of the body height from the top (or 1/4th from the bottom)
    sprite.fillCircle(centerX, centerY + bodyHeight / 2 - (bodyHeight / 4), CIRCLE_LIGHT_RADIUS, TFT_GREEN);
}

// Common function to draw icons based on IconType
void drawIcon(IconType type, int centerX, int centerY, uint16_t color) {
    // Static map to store IconBitmap data, initialized once
    static const std::map<IconType, IconBitmap> iconBitmaps = {
        {IconType::Fuel, {Fuel_16_bit_map, FUEL_16_WIDTH, FUEL_16_HEIGHT}},
        {IconType::BusStop, {Bus_bit_map, BUS_WIDTH, BUS_HEIGHT}}
        // TrafficSignal is drawn directly, no bitmap needed
    };

    switch (type) {
        case IconType::TrafficSignal:
            drawTrafficSignalIcon(centerX, centerY, color);
            break;
        case IconType::Fuel: {
            auto it = iconBitmaps.find(IconType::Fuel);
            if (it != iconBitmaps.end()) {
                drawMonochromeIcon(centerX, centerY, color, it->second);
            }
            break;
        }
        case IconType::BusStop: {
            auto it = iconBitmaps.find(IconType::BusStop);
            if (it != iconBitmaps.end()) {
                drawMonochromeIcon(centerX, centerY, color, it->second);
            }
            break;
        }
        case IconType::None:
        default:
            // No icon to draw, or unknown type
            break;
    }
}

// Function to determine road width based on zoom scale factor
// This function now provides specific pixel widths for discrete zoom levels (1x, 2x, 3x, 4x)
// to allow for more precise control over road appearance.
int getRoadWidth(float zoomScaleFactor) {
    // Round the zoomScaleFactor to the nearest integer to treat discrete zoom levels
    int discreteZoom = static_cast<int>(round(zoomScaleFactor));

    switch (discreteZoom) {
        case 1:
            return 1; // Zoom 1x: 1 pixel wide
        case 2:
            return 2; // Zoom 2x: 2 pixels wide
        case 3:
            return 3; // Zoom 3x: 3 pixels wide
        case 4:
            return 5; // Zoom 4x: 5 pixels wide (changed from 4 to 5)
        default:
            return 1; // Default to 1 pixel for any other zoom factors
    }
}

// =========================================================
// GEOMETRY DRAWING (Render Task will use this)
// =========================================================
// This function is now only used for rendering pre-parsed geometry.
// It takes a set of screen-space points and draws/fills them.
void renderRing(const std::vector<std::pair<int, int>, PSRAMAllocator<std::pair<int, int>>>& points, uint16_t color, bool isPolygon, int geomType, bool hasBridge, bool hasTunnel, float zoomScaleFactor) {
  if (points.empty()) return;

  if (geomType == 1) { // Explicitly handle Point geometry (which includes MultiPoint, where each "ring" is a single point)
      for (const auto& p : points) { // Iterate through all points in the vector
          int px = p.first;
          int py = p.second;
          sprite.fillRect(px - POINT_FEATURE_SIZE/2, py - POINT_FEATURE_SIZE/2, POINT_FEATURE_SIZE, POINT_FEATURE_SIZE, color);
      }
  } else if (geomType == 2) { // Explicitly handle LineString geometry (roads, waterways)
      if (points.size() > 1) {
          int lineWidth = getRoadWidth(zoomScaleFactor); // Get dynamic line width

          if (lineWidth == 1) {
              for (size_t k = 0; k < points.size() - 1; ++k) {
                  sprite.drawLine(points[k].first, points[k].second, points[k+1].first, points[k+1].second, color);
              }
          } else { // For lineWidth 2, 3, 5 (or any other > 1), use filled triangles for consistency
              for (size_t k = 0; k < points.size() - 1; ++k) {
                  int x1 = points[k].first;
                  int y1 = points[k].second;
                  int x2 = points[k+1].first;
                  int y2 = points[k+1].second;

                  float dx = (float)(x2 - x1);
                  float dy = (float)(y2 - y1);
                  float length = sqrt(dx*dx + dy*dy);

                  if (length > 0) {
                      float perp_dx_norm = dy / length;
                      float perp_dy_norm = -dx / length;

                      // Calculate the four corners of the widened line segment
                      int p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y;

                      if (lineWidth == 2) {
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
                      } else {
                          // For other widths (3, 5), use the standard halfWidthOffset calculation
                          float halfWidthOffset = (float)(lineWidth - 1) / 2.0f;
                          p1x = round(x1 - perp_dx_norm * halfWidthOffset);
                          p1y = round(y1 - perp_dy_norm * halfWidthOffset);
                          p2x = round(x2 - perp_dx_norm * halfWidthOffset);
                          p2y = round(y2 - perp_dy_norm * halfWidthOffset);
                          p3x = round(x2 + perp_dx_norm * halfWidthOffset);
                          p3y = round(y2 + perp_dy_norm * halfWidthOffset);
                          p4x = round(x1 + perp_dx_norm * halfWidthOffset);
                          p4y = round(y1 + perp_dy_norm * halfWidthOffset);
                      }

                      // Draw the rectangle using two triangles
                      sprite.fillTriangle(p1x, p1y, p2x, p2y, p3x, p3y, color);
                      sprite.fillTriangle(p1x, p1y, p3x, p3y, p4x, p4y, color);
                  } else { // Handle single point case for line (x1==x2 && y1==y2)
                      sprite.drawPixel(x1, y1, color);
                  }
              }
          }

          // Bridge/Tunnel borders (adjust offset for new line width)
          if (hasBridge && !hasTunnel) {
              // The border should be 1 pixel outside the total road width.
              // The road extends from -(lineWidth-1)/2.0f to +(lineWidth-1)/2.0f from the center.
              // The outermost edge of the road is at (lineWidth-1)/2.0f + 0.5f.
              // The center of the 1-pixel border should be at this edge + 0.5f (half the border's width).
              // So, borderOffset = (lineWidth-1)/2.0f + 0.5f + 0.5f = (lineWidth-1)/2.0f + 1.0f.
              float borderOffset = (float)(lineWidth - 1) / 2.0f + 1.0f;

              for (size_t k = 0; k < points.size() - 1; ++k) {
                  int x1 = points[k].first;
                  int y1 = points[k].second;
                  int x2 = points[k+1].first;
                  int y2 = points[k+1].second;

                  float dx = (float)(x2 - x1);
                  float dy = (float)(y2 - y1);
                  float length = sqrt(dx*dx + dy*dy);

                  if (length > 0) {
                      float perp_dx_norm = dy / length;
                      float perp_dy_norm = -dx / length;

                      int current_offset_x = round(perp_dx_norm * borderOffset);
                      int current_offset_y = round(perp_dy_norm * borderOffset);

                      sprite.drawLine(x1 - current_offset_x, y1 - current_offset_y,
                                     x2 - current_offset_x, y2 - current_offset_y, TFT_BLACK);
                      sprite.drawLine(x1 + current_offset_x, y1 + current_offset_y,
                                     x2 + current_offset_x, y2 + current_offset_y, TFT_BLACK);
                  }
              }
          }
      } else if (points.size() == 1) { // A single point in a LineString is still a point
          sprite.drawPixel(points[0].first, points[0].second, color);
      }
  } else if (geomType == 3) { // Explicitly handle Polygon geometry
      // Draw the outline of the polygon
      if (points.size() > 1) {
          for (size_t k = 0; k < points.size() - 1; ++k) {
              sprite.drawLine(points[k].first, points[k].second, points[k+1].first, points[k+1].second, color);
          }
          // Close the path for polygons
          if (points.front().first != points.back().first || points.front().second != points.back().second) {
              sprite.drawLine(points.back().first, points.back().second, points.front().first, points.front().second, color);
          }
      } else if (points.size() == 1) { // A single point in a Polygon is still a point
          sprite.drawPixel(points[0].first, points[0].second, color);
      }

      // Fill the polygon
      // (Keep existing scanline fill logic here)
      int minY = screenH, maxY = 0;
      for (const auto& p : points) {
          if (p.second < minY) minY = p.second;
          if (p.second > maxY) maxY = p.second;
      }

      minY = std::max(0, minY);
      maxY = std::min(screenH - 1, maxY);

      // Declare intersections vector once, outside the scanline loop
      // Using PSRAMAllocator to prevent repeated internal RAM allocations/deallocations.
      std::vector<int, PSRAMAllocator<int>> intersections_psram{PSRAMAllocator<int>()};
      // Reserve maximum possible size (number of vertices in the polygon)
      intersections_psram.reserve(points.size());

      for (int scanY = minY; scanY <= maxY; ++scanY) {
          intersections_psram.clear(); // Clear for current scanline, without deallocating memory

          for (size_t k = 0; k < points.size(); ++k) {
              int x1 = points[k].first;
              int y1 = points[k].second;
              int x2 = points[(k + 1) % points.size()].first;
              int y2 = points[(k + 1) % points.size()].second;

              if (y1 == y2) continue;
              if ((scanY < std::min(y1, y2)) || (scanY >= std::max(y1, y2))) continue;

              float intersectX = (float)(x2 - x1) * (scanY - y1) / (float)(y2 - y1) + x1;
              intersections_psram.push_back(round(intersectX));
          }
          std::sort(intersections_psram.begin(), intersections_psram.end());

          for (size_t k = 0; k + 1 < intersections_psram.size(); k += 2) {
              int startX = intersections_psram[k];
              int endX = intersections_psram[k+1];
              if (startX > endX) std::swap(startX, endX);

              startX = std::max(0, startX);
              endX = std::min(screenW - 1, endX);

              if (startX <= endX) {
                  sprite.drawFastHLine(startX, scanY, endX - startX + 1, color);
              }
          }
      }
  } else { // Fallback for unknown geomType, draw as points
      if (!points.empty()) {
          sprite.fillRect(points[0].first - POINT_FEATURE_SIZE/2, points[0].second - POINT_FEATURE_SIZE/2, POINT_FEATURE_SIZE, POINT_FEATURE_SIZE, color);
      }
  }
}

// Custom function to draw the navigation arrow using two triangles
void drawNavigationArrow(int centerX, int centerY, int size, uint16_t color) {
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
void drawParsedFeature(const ParsedFeature& feature, int layerExtent, const TileKey& tileKey, const RenderParams& params) {
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
        screenMinY_float_pre_rot > screenH + cullingBufferBottom) {
        return; // Cull feature if its unrotated bounding box is completely outside the buffered screen area
    }

    // Calculate cosine and sine of the rotation angle (in radians)
    float cosTheta = cos(radians(params.mapRotationDegrees));
    float sinTheta = sin(radians(params.mapRotationDegrees));

    for (const auto& ring : feature.geometryRings) {
        // This vector needs to use the PSRAMAllocator to match the renderRing function's signature
        std::vector<std::pair<int, int>, PSRAMAllocator<std::pair<int, int>>> screenPoints{PSRAMAllocator<std::pair<int, int>>()};
        try {
          screenPoints.reserve(ring.size()); // Pre-allocate memory

          for (const auto& p : ring) {
              // Convert MVT (x,y) to screen (x,y) coordinates
              // Assuming MVT Y is Y-down, which aligns with TFT_eSPI's coordinate system
              float screen_x = (float)p.first * scaleX + tileRenderOffsetX_float - params.displayOffsetX;
              float screen_y = (float)p.second * scaleY + tileRenderOffsetY_float - params.displayOffsetY;

              // Translate point so that the center of rotation is at the origin
              float translatedX = screen_x - centerX;
              float translatedY = screen_y - centerY;

              // Apply 2D rotation transformation
              float rotatedX = translatedX * cosTheta - translatedY * sinTheta;
              float rotatedY = translatedX * sinTheta + translatedY * cosTheta;

              screenPoints.push_back({round(rotatedX + centerX), round(rotatedY + centerY)});
          }

          if (!screenPoints.empty() && feature.geomType == 1) { // Apply only to point geometries
              // Iterate over all points in the screenPoints vector for point features
              for (const auto& p : screenPoints) {
                  int iconCenterX = p.first;
                  int iconCenterY = p.second;

                  // Use the iconName to determine which icon to draw
                  static const PSRAMString PSRAM_TRAFFIC_SIGNALS_ICON("traffic_signals", PSRAMAllocator<char>());
                  static const PSRAMString PSRAM_FUEL_ICON("fuel", PSRAMAllocator<char>());
                  static const PSRAMString PSRAM_BUS_STOP_ICON("bus_stop", PSRAMAllocator<char>());

                  if (feature.iconName == PSRAM_TRAFFIC_SIGNALS_ICON) {
                      drawIcon(IconType::TrafficSignal, iconCenterX, iconCenterY, feature.color);
                  } else if (feature.iconName == PSRAM_FUEL_ICON) {
                      drawIcon(IconType::Fuel, iconCenterX, iconCenterY, feature.color);
                  } else if (feature.iconName == PSRAM_BUS_STOP_ICON) {
                      drawIcon(IconType::BusStop, iconCenterX, iconCenterY, feature.color);
                  }
                  else {
                      // If it's a point feature but not a special icon type,
                      // render it as a regular point using renderRing.
                      // Create a temporary vector with just this single point.
                      std::vector<std::pair<int, int>, PSRAMAllocator<std::pair<int, int>>> singlePointVec{PSRAMAllocator<std::pair<int, int>>()};
                      singlePointVec.push_back(p);
                      renderRing(singlePointVec, feature.color, feature.isPolygon, feature.geomType, feature.hasBridge, feature.hasTunnel, params.zoomScaleFactor); // Pass hasBridge, hasTunnel and zoomScaleFactor
                  }
              }
          } else {
              // For non-point geometries, or if screenPoints is empty, draw normally
              renderRing(screenPoints, feature.color, feature.isPolygon, feature.geomType, feature.hasBridge, feature.hasTunnel, params.zoomScaleFactor); // Pass geomType, hasBridge, hasTunnel and zoomScaleFactor
          }

        } catch (const std::bad_alloc& e) {
          Serial.printf("❌ drawParsedFeature: Memory allocation failed for screenPoints: %s\n", e.what());
          // Skip drawing this ring
        }
    }
}


// Converts Latitude/Longitude to XYZ tile coordinates (and TMS Y coordinate)
void latlonToTile(double lat, double lon, int z, int &x, int &y, int &ytms) {
  double latRad = radians(lat);
  int n = 1 << z; // Number of tiles in one dimension at zoom level z
  x = int((lon + 180.0) / 360.0 * n);
  y = int((1.0 - log(tan(latRad) + 1.0 / cos(latRad)) / PI) / 2.0 * n);
  ytms = n - 1 - y; // Convert standard Y to TMS Y (used by MBTiles)
}

// Converts Latitude/Longitude to pixel coordinates (0-255) within its containing tile
// And then scales it to the MVT extent (e.g., 0-4095)
void latLonToMVTCoords(double lat, double lon, int z, int tileX, int tileY_TMS, int& mvtX, int& mvtY, int extent) {
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
uint16_t blendColors(uint16_t background, uint16_t foreground, float alpha) {
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


// =========================================================
// RENDER TASK (Core 1)
// Handles display updates, sensor readings, and tile requests
// =========================================================
void renderTask(void *pvParameters) {
    tft.begin();
    tft.setRotation(0); // Changed to 0 for portrait mode
    tft.fillScreen(0x1926); // Changed background color to #1a2632

    sprite.setColorDepth(16); // Set sprite color depth (RGB565, good balance of speed/quality)
    sprite.createSprite(screenW, screenH); // Create the 160x128 off-screen sprite

    // Initialize HMC5883L compass
    Wire.begin(); // Initialize I2C
    if (!hmc5883l.begin()) {
        Serial.println("❌ Render Task: Could not find a valid HMC5883L sensor, check wiring!"); // Keep this error message
    } else {
        hmc5883l.setMagGain(HMC5883_MAGGAIN_1_3); // Set gain to +/- 1.3 Gauss (default)
    }

    // Initialize currentControlParams with default values
    ControlParams currentControlParams = {
        .targetLat = 12.8273,
        .targetLon = 80.2193,
        .zoomFactor = 1.0,
        .cullingBufferPercentageLeft = DEFAULT_CULLING_BUFFER_PERCENTAGE_LEFT,
        .cullingBufferPercentageRight = DEFAULT_CULLING_BUFFER_PERCENTAGE_RIGHT,
        .cullingBufferPercentageTop = DEFAULT_CULLING_BUFFER_PERCENTAGE_TOP,
        .cullingBufferPercentageBottom = DEFAULT_CULLING_BUFFER_PERCENTAGE_BOTTOM
    };
    RenderParams currentRenderParams; // Parameters derived from control and sensor data

    // State variables for loading management
    TileKey currentRequestedCenterTile = {-1, -1, -1}; // The tile that was last requested by user input
    TileKey currentlyLoadedCenterTile = {-2, -2, -2}; // Initialize to a different invalid value to ensure initial render trigger

    float internalCurrentRotationAngle = 0.0f; // Current rotation from compass
    float lastSentRotationAngle = 0.0f; // Stores the last rotation angle used for rendering
    float lastSentZoomFactor = 0.0f; // Stores the last zoom factor used for rendering

    // Define status bar height and margin for arrow
    const int STATUS_BAR_HEIGHT = 10;
    const int ARROW_MARGIN_ABOVE_STATUS_BAR = 2; // Pixels between arrow's lowest point and status bar top
    const float STATUS_BAR_ALPHA = 0.5f; // 50% opacity for the status bar

    while (true) {
        bool screenNeedsUpdate = false; // Flag to track if screen update is needed this frame

        // 1. Check for new control parameters from the main loop (user input)
        ControlParams receivedControlParams;
        if (xQueueReceive(controlParamsQueue, &receivedControlParams, 0) == pdPASS) {
            currentControlParams = receivedControlParams;
            screenNeedsUpdate = true; // New control params always mean screen update
        }

        // 2. Read compass data and update rotation
        sensors_event_t event;
        hmc5883l.getEvent(&event);
        float rawHeading = atan2(event.magnetic.y, event.magnetic.x);
        rawHeading = rawHeading * 180 / PI;
        if (rawHeading < 0) rawHeading += 360;

        headingReadings.push_back(rawHeading);
        if (headingReadings.size() > COMPASS_FILTER_WINDOW_SIZE) {
            headingReadings.erase(headingReadings.begin());
        }
        float sumSin = 0.0f;
        float sumCos = 0.0f;
        for (float h : headingReadings) {
            sumSin += sin(radians(h));
            sumCos += cos(radians(h));
        }
        internalCurrentRotationAngle = degrees(atan2(sumSin / headingReadings.size(), sumCos / headingReadings.size()));
        if (internalCurrentRotationAngle < 0) internalCurrentRotationAngle += 360;

        // Check if rotation has changed significantly
        if (fabs(internalCurrentRotationAngle - lastSentRotationAngle) >= COMPASS_ROTATION_THRESHOLD_DEG) {
            screenNeedsUpdate = true;
        }

        // 3. Calculate current central tile and target MVT point
        int newCentralTileX, newCentralTileY_std, newCentralTileY_TMS;
        latlonToTile(currentControlParams.targetLat, currentControlParams.targetLon, currentTileZ,
                     newCentralTileX, newCentralTileY_std, newCentralTileY_TMS);

        TileKey newRequestedCenterTile = {currentTileZ, newCentralTileX, newCentralTileY_TMS};

        int internalTargetPointMVT_X, internalTargetPointMVT_Y;
        latLonToMVTCoords(currentControlParams.targetLat, currentControlParams.targetLon, currentTileZ, newCentralTileX, newCentralTileY_TMS,
                          internalTargetPointMVT_X, internalTargetPointMVT_Y, currentLayerExtent);

        // 4. Update currentRenderParams
        currentRenderParams.centralTileX = newCentralTileX;
        currentRenderParams.centralTileY_TMS = newCentralTileY_TMS;
        currentRenderParams.targetPointMVT_X = internalTargetPointMVT_X;
        currentRenderParams.targetPointMVT_Y = internalTargetPointMVT_Y;
        currentRenderParams.layerExtent = currentLayerExtent;
        currentRenderParams.zoomScaleFactor = currentControlParams.zoomFactor;
        currentRenderParams.mapRotationDegrees = internalCurrentRotationAngle;
        currentRenderParams.cullingBufferPercentageLeft = currentControlParams.cullingBufferPercentageLeft;
        currentRenderParams.cullingBufferPercentageRight = currentControlParams.cullingBufferPercentageRight;
        currentRenderParams.cullingBufferPercentageTop = currentControlParams.cullingBufferPercentageTop;
        currentRenderParams.cullingBufferPercentageBottom = currentControlParams.cullingBufferPercentageBottom;

        // Define arrow properties
        int arrowSize = NAVIGATION_ARROW_SIZE;
        int arrowHalfSize = arrowSize / 2;

        // Calculate the Y coordinate for the center of the arrow's base,
        // positioning it above the status bar with a margin.
        // The lowest point of the arrow is centerY + halfSize.
        // We want this lowest point to be at (screenH - STATUS_BAR_HEIGHT - ARROW_MARGIN_ABOVE_STATUS_BAR).
        int arrowBaseCenterY = (screenH - STATUS_BAR_HEIGHT) - ARROW_MARGIN_ABOVE_STATUS_BAR - arrowHalfSize;

        // The pivotY for map rotation is the tip of the arrow.
        currentRenderParams.pivotY = arrowBaseCenterY - arrowSize;

        // Calculate display offsets. The map's target point should align with the arrow's tip.
        float scaledPointX_global = (float)currentRenderParams.targetPointMVT_X * (screenW * currentRenderParams.zoomScaleFactor / currentRenderParams.layerExtent);
        float scaledPointY_global = (float)currentRenderParams.targetPointMVT_Y * (screenH * currentRenderParams.zoomScaleFactor / currentRenderParams.layerExtent);
        
        // Adjust display offsets to correct point positioning.
        // A positive offset here shifts the map content to the left (for X) or upwards (for Y).
        // If the displayed point is to the right of actual, increase displayOffsetX.
        // If the displayed point is below actual, increase displayOffsetY.
        currentRenderParams.displayOffsetX = round(scaledPointX_global - (screenW / 2.0f) -2); // Increased offset to +5
        currentRenderParams.displayOffsetY = round(scaledPointY_global - currentRenderParams.pivotY +1); // Increased offset to +5


        // 5. Request necessary tiles from Data Task based on the new loading strategy
        if (!(newRequestedCenterTile == currentlyLoadedCenterTile) ||
            fabs(currentControlParams.zoomFactor - lastSentZoomFactor) >= 0.01f) { // Added zoom factor change as trigger

            // Clear any pending requests for old center tile if the central tile has changed
            if (!(newRequestedCenterTile == currentlyLoadedCenterTile)) {
                Serial.printf("Render Task: Central tile changed to Z:%d X:%d Y:%d. Resetting tile request queue.\n",
                              newRequestedCenterTile.z, newRequestedCenterTile.x, newRequestedCenterTile.y_tms);
                xQueueReset(tileRequestQueue);
            }

            currentlyLoadedCenterTile = newRequestedCenterTile;
            screenNeedsUpdate = true; // New tile requests always mean screen update

            std::set<TileKey> requestedTilesInCycle; // To track what's sent this cycle

            auto sendTileRequest = [&](const TileKey& key) -> bool {
                // Check if already requested in this cycle or already loaded
                if (requestedTilesInCycle.find(key) == requestedTilesInCycle.end()) {
                    bool alreadyLoaded = false;
                    if (xSemaphoreTake(loadedTilesDataMutex, 0) == pdTRUE) {
                        alreadyLoaded = (loadedTilesData.find(key) != loadedTilesData.end());
                        xSemaphoreGive(loadedTilesDataMutex);
                    }
                    if (!alreadyLoaded) {
                        // Attempt to send the request
                        if (xQueueSend(tileRequestQueue, &key, pdMS_TO_TICKS(200)) != pdPASS) {
                            Serial.printf("❌ Render Task: Failed to send tile request Z:%d X:%d Y:%d. Queue full? (Timeout)\n",
                                          key.z, key.x, key.y_tms);
                            return false;
                        }
                        requestedTilesInCycle.insert(key); // Mark as sent
                        return true;
                    }
                }
                return false; // Already requested or loaded
            };

            // Implementing "9+16" tile loading strategy:
            // Ring 0: Central tile (1 tile)
            sendTileRequest(newRequestedCenterTile);

            // Ring 1: 3x3 grid (excluding center) = 8 tiles
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) { // Corrected: Added 'dy <= 1' to the loop condition
                    if (dx == 0 && dy == 0) continue; // Skip central tile
                    TileKey neighborKey = {currentTileZ, newRequestedCenterTile.x + dx, newRequestedCenterTile.y_tms + dy};
                    sendTileRequest(neighborKey);
                }
            }

            // Ring 2: 5x5 grid (excluding 3x3 inner grid) = 16 tiles
            for (int dx = -2; dx <= 2; ++dx) {
                for (int dy = -2; dy <= 2; ++dy) { // Corrected: Added 'dy <= 2' to the loop condition
                    // Skip the inner 3x3 grid (already handled in Ring 0 and Ring 1)
                    if (abs(dx) <= 1 && abs(dy) <= 1) continue;
                    TileKey neighborKey = {currentTileZ, newRequestedCenterTile.x + dx, newRequestedCenterTile.y_tms + dy};
                    sendTileRequest(neighborKey);
                }
            }
        }

        // 6. Check for tile parsed notifications (optional, but good for responsiveness)
        bool notification;
        if (xQueueReceive(tileParsedNotificationQueue, &notification, 0) == pdPASS) {
            if (!notification) {
                Serial.println("❌ Render Task: A tile parsing operation failed in Data Task.");
            } else {
                screenNeedsUpdate = true; // A tile was successfully parsed, so the map data has changed.
            }
        }

        // 7. Render the map (only if needed)
        if (screenNeedsUpdate) {
            sprite.fillScreen(0x1926); // Changed background color to #1a2632

            if (xSemaphoreTake(loadedTilesDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                // Eviction logic: Remove tiles that are no longer within the 5x5 grid
                // This aligns with the "9+16" loading strategy, keeping a 5x5 buffer of loaded tiles.
                int minX_5x5 = newRequestedCenterTile.x - 2;
                int maxX_5x5 = newRequestedCenterTile.x + 2;
                int minY_5x5 = newRequestedCenterTile.y_tms - 2;
                int maxY_5x5 = newRequestedCenterTile.y_tms + 2;

                for (auto it = loadedTilesData.begin(); it != loadedTilesData.end(); ) {
                    const TileKey& tileKey = it->first;
                    if (tileKey.z == currentTileZ &&
                        (tileKey.x < minX_5x5 || tileKey.x > maxX_5x5 ||
                         tileKey.y_tms < minY_5x5 || tileKey.y_tms > maxY_5x5)) {
                        it = loadedTilesData.erase(it);
                    } else {
                        ++it;
                    }
                }

                if (!loadedTilesData.empty()) {
                    for (auto it = loadedTilesData.begin(); it != loadedTilesData.end(); ++it) {
                        const TileKey& tileKey = it->first;
                        std::vector<ParsedLayer, PSRAMAllocator<ParsedLayer>>& layers = it->second; // Use non-const reference

                        // Sort layers by drawOrder before rendering
                        std::sort(layers.begin(), layers.end(), [](const ParsedLayer& a, const ParsedLayer& b) {
                            return a.drawOrder < b.drawOrder;
                        });

                        for (const auto& layer : layers) {
                            for (const auto& feature : layer.features) {
                                drawParsedFeature(feature, layer.extent, tileKey, currentRenderParams);
                            }
                        }
                    }
                } else {
                    Serial.println("Render Task: loadedTilesData is empty, no tiles to draw."); // Added debug print
                }
                xSemaphoreGive(loadedTilesDataMutex);
            } else {
                Serial.println("❌ Render Task: Failed to acquire mutex for loadedTilesData during drawing (timeout).");
            }

            // Draw the navigation arrow, using the calculated base center Y
            drawNavigationArrow(screenW / 2, arrowBaseCenterY, arrowSize, TFT_WHITE);

            int statusBarY = 0; // Set status bar to the top
            for (int y = statusBarY; y < STATUS_BAR_HEIGHT; ++y) { // Loop for the height of the status bar
                for (int x = 0; x < screenW; ++x) {
                    uint16_t currentPixelColor = sprite.readPixel(x, y);
                    uint16_t blendedColor = blendColors(currentPixelColor, TFT_BLACK, STATUS_BAR_ALPHA);
                    sprite.drawPixel(x, y, blendedColor);
                }
            }

            sprite.pushSprite(0, 0); // Only push if update is needed

            lastSentRotationAngle = internalCurrentRotationAngle;
            lastSentZoomFactor = currentControlParams.zoomFactor;
        }

        vTaskDelay(pdMS_TO_TICKS(1)); // Always yield
    }
}
