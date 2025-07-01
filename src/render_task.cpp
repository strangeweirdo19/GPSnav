// render_task.cpp
#include "render_task.h"
#include "common.h"

// =========================================================
// GEOMETRY DRAWING (Render Task will use this)
// =========================================================
// This function is now only used for rendering pre-parsed geometry.
// It takes a set of screen-space points and draws/fills them.
void renderRing(const std::vector<std::pair<int, int>, PSRAMAllocator<std::pair<int, int>>>& points, uint16_t color, bool isPolygon, int geomType) {
  if (points.empty()) return;

  if (geomType == 1) { // Explicitly handle Point geometry
      if (!points.empty()) { // Should only have one point for a single point feature
          sprite.drawPixel(points[0].first, points[0].second, color);
      }
  } else if (geomType == 2) { // Explicitly handle LineString geometry
      if (points.size() > 1) {
          for (size_t k = 0; k < points.size() - 1; ++k) {
              sprite.drawLine(points[k].first, points[k].second, points[k+1].first, points[k+1].second, color);
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

      for (int scanY = minY; scanY <= maxY; ++scanY) {
          std::vector<int> intersections; // This vector is temporary, can use default allocator
          for (size_t k = 0; k < points.size(); ++k) {
              int x1 = points[k].first;
              int y1 = points[k].second;
              int x2 = points[(k + 1) % points.size()].first;
              int y2 = points[(k + 1) % points.size()].second;

              if (y1 == y2) continue;
              if ((scanY < std::min(y1, y2)) || (scanY >= std::max(y1, y2))) continue;

              float intersectX = (float)(x2 - x1) * (scanY - y1) / (float)(y2 - y1) + x1;
              intersections.push_back(round(intersectX));
          }
          std::sort(intersections.begin(), intersections.end());

          for (size_t k = 0; k + 1 < intersections.size(); k += 2) {
              int startX = intersections[k];
              int endX = intersections[k+1];
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
          sprite.drawPixel(points[0].first, points[0].second, color);
      }
  }
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


    // --- Bounding Box Culling Optimization ---
    // Transform the MVT bounding box to screen coordinates, considering the tile's position
    // and the global display offset.
    float screenMinX_float_pre_rot = feature.minX_mvt * scaleX + tileRenderOffsetX_float - params.displayOffsetX;
    float screenMinY_float_pre_rot = feature.minY_mvt * scaleY + tileRenderOffsetY_float - params.displayOffsetY;
    float screenMaxX_float_pre_rot = feature.maxX_mvt * scaleX + tileRenderOffsetX_float - params.displayOffsetX;
    float screenMaxY_float_pre_rot = feature.maxY_mvt * scaleY + tileRenderOffsetY_float - params.displayOffsetY;

    // Check if the feature's bounding box is outside the screen. If so, skip rendering.
    if (screenMaxX_float_pre_rot < 0 || screenMinX_float_pre_rot >= screenW || screenMaxY_float_pre_rot < 0 || screenMinY_float_pre_rot >= screenH) {
        return;
    }
    // --- End Bounding Box Culling ---

    // Calculate cosine and sine of the rotation angle (in radians)
    float cosTheta = cos(radians(params.mapRotationDegrees));
    float sinTheta = sin(radians(params.mapRotationDegrees));
    
    // Define the center of rotation (screen center)
    int centerX = screenW / 2;
    int centerY = screenH / 2;

    for (const auto& ring : feature.geometryRings) {
        // This vector needs to use the PSRAMAllocator to match the renderRing function's signature
        std::vector<std::pair<int, int>, PSRAMAllocator<std::pair<int, int>>> screenPoints{PSRAMAllocator<std::pair<int, int>>()};
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

            // Translate point back to its original position relative to the screen center
            screenPoints.push_back({round(rotatedX + centerX), round(rotatedY + centerY)});
        }
        renderRing(screenPoints, feature.color, feature.isPolygon, feature.geomType); // Pass geomType
    }
}


// =========================================================
// RENDERING FUNCTION (Render Task will use this)
// =========================================================

// Renders all currently loaded tile data with the active overzoom settings
void renderTileWithOverzoom(const RenderParams& params) {
  sprite.fillScreen(TFT_BLACK); // Clear the off-screen sprite buffer for new drawing

  // Acquire mutex before reading shared data
  if (xSemaphoreTake(loadedTilesDataMutex, portMAX_DELAY) == pdTRUE) {
      if (!loadedTilesData.empty()) {
        for (const auto& pair : loadedTilesData) {
            const TileKey& tileKey = pair.first;
            const std::vector<ParsedLayer, PSRAMAllocator<ParsedLayer>>& layers = pair.second;
            for (const auto& layer : layers) {
                for (const auto& feature : layer.features) {
                    drawParsedFeature(feature, layer.extent, tileKey, params);
                }
            }
        }
      } else {
        Serial.println("No parsed tile data to render.");
      }
      xSemaphoreGive(loadedTilesDataMutex); // Release mutex
  } else {
      Serial.println("❌ Failed to acquire mutex for loadedTilesData during rendering.");
  }
}

// =========================================================
// RENDER TASK (Core 1)
// Handles display updates
// =========================================================
void renderTask(void *pvParameters) {
    Serial.println("Render Task: Initializing TFT...");
    tft.begin();
    tft.setRotation(1); // Set display to landscape mode (should be 160x128 if native is 128x160)
    tft.fillScreen(TFT_BLACK); 

    Serial.printf("Render Task: TFT Reported Width (after rotation): %d\n", tft.width());
    Serial.printf("Render Task: TFT Reported Height (after rotation): %d\n", tft.height());

    sprite.setColorDepth(16); // Set sprite color depth (RGB565, good balance of speed/quality)
    sprite.createSprite(screenW, screenH); // Create the 160x128 off-screen sprite

    RenderParams currentRenderParams; // Store the last received parameters

    while (true) {
        unsigned long renderLoopStartTime = millis();
        // Try to receive new rendering parameters from the queue
        if (xQueueReceive(renderParamsQueue, &currentRenderParams, portMAX_DELAY) == pdPASS) {
            // New parameters received, render the map
            renderTileWithOverzoom(currentRenderParams);

            sprite.setTextColor(TFT_YELLOW, TFT_BLACK);
            sprite.setCursor(5, 5);
            sprite.printf("Zoom: %.1fx", currentRenderParams.zoomScaleFactor); // Display the current zoom factor
            sprite.setCursor(5, 15); // New line for heading
            sprite.printf("Hdg: %.1f deg", currentRenderParams.mapRotationDegrees);
            
            sprite.pushSprite(0, 0); // Push the completed sprite to the physical display
            unsigned long renderLoopEndTime = millis();
            Serial.printf("Render Task: Map rendered in %lu ms.\n", renderLoopEndTime - renderLoopStartTime);
        }
        // If no new parameters, the task will block on xQueueReceive until data is available.
        // This ensures the render task only renders when there's new data to display.
    }
}

// =========================================================