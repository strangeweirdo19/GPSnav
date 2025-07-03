// render_task.cpp
#include "render_task.h"
#include "common.h"

// HMC5883L compass object - changed to Adafruit_HMC5883_Unified
Adafruit_HMC5883_Unified hmc5883l = Adafruit_HMC5883_Unified(12345); // Using a sensor ID, common for unified sensors

// Global variables for compass heading filtering (moving average)
std::vector<float> headingReadings; // This vector is small, can stay in internal RAM
const int FILTER_WINDOW_SIZE = 10; // Number of readings to average

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
    double latRad = radians(lat);

    // Global pixel coordinates at Z zoom level (assuming 256x256 pixel tiles for standard mapping)
    double globalPx = ((lon + 180.0) / 360.0) * n * 256;
    double globalPy = (1.0 - log(tan(latRad) + 1.0 / cos(latRad)) / PI) / 2.0 * n * 256;

    // Pixel coordinate within the current tile (0-255 range for 256x256 pixel tile)
    // By taking modulo of global pixel coordinates with tile size (256)
    double pixelInTileX_256 = fmod(globalPx, 256.0);
    double pixelInTileY_256 = fmod(globalPy, 256.0);

    // If globalPx or globalPy are very close to a tile boundary, fmod might give something like 255.999...
    // or 0.000... at the other end. Ensure it's correctly within [0, 256) or [0, 4096)
    if (pixelInTileX_256 < 0) pixelInTileX_256 += 256;
    if (pixelInTileY_256 < 0) pixelInTileY_256 += 256;

    // Convert pixel-in-tile (0-255) to MVT extent coordinates (0-extent)
    mvtX = round(pixelInTileX_256 * (extent / 256.0));
    mvtY = round(pixelInTileY_256 * (extent / 256.0));

    // Clamp to extent boundaries if floating point math results in slightly out of bounds values
    mvtX = std::max(0, std::min(extent - 1, mvtX));
    mvtY = std::max(0, std::min(extent - 1, mvtY));
}


// =========================================================
// RENDER TASK (Core 1)
// Handles display updates, sensor readings, and tile requests
// =========================================================
void renderTask(void *pvParameters) {
    tft.begin();
    tft.setRotation(1); // Set display to landscape mode (should be 160x128 if native is 128x160)
    tft.fillScreen(TFT_BLACK); 

    sprite.setColorDepth(16); // Set sprite color depth (RGB565, good balance of speed/quality)
    sprite.createSprite(screenW, screenH); // Create the 160x128 off-screen sprite

    // Initialize HMC5883L compass
    Wire.begin(); // Initialize I2C
    if (!hmc5883l.begin()) {
        Serial.println("❌ Render Task: Could not find a valid HMC5883L sensor, check wiring!");
    } else {
        hmc5883l.setMagGain(HMC5883_MAGGAIN_1_3); // Set gain to +/- 1.3 Gauss (default)
    }

    ControlParams currentControlParams = {12.8273, 80.2193, 1.0}; // Initialized to default values
    RenderParams currentRenderParams; // Parameters derived from control and sensor data

    // State variables for loading management
    TileKey currentRequestedCenterTile = {-1, -1, -1}; // The tile that was last requested by user input
    TileKey currentlyLoadedCenterTile = {-2, -2, -2}; // Initialize to a different invalid value to ensure initial render trigger

    float internalCurrentRotationAngle = 0.0f; // Current rotation from compass
    float lastSentRotationAngle = 0.0f; // Stores the last rotation angle used for rendering
    float lastSentZoomFactor = 0.0f; // Stores the last zoom factor used for rendering

    // Variables for FPS calculation
    unsigned long lastFpsTime = 0;
    unsigned int frameCount = 0;
    float currentFps = 0.0f;

    // Helper to check if two tiles are adjacent (including corners)
    auto areTilesAdjacent = [](const TileKey& k1, const TileKey& k2) {
        if (k1.z != k2.z) return false; // Must be same zoom level
        if (k1 == k2) return false; // Not adjacent to itself
        return (abs(k1.x - k2.x) <= 1 && abs(k1.y_tms - k2.y_tms) <= 1);
    };

    while (true) {
        unsigned long renderLoopStartTime = millis();

        // 1. Check for new control parameters from the main loop (user input)
        ControlParams receivedControlParams;
        if (xQueueReceive(controlParamsQueue, &receivedControlParams, 0) == pdPASS) {
            currentControlParams = receivedControlParams;
        }

        // 2. Read compass data and update rotation
        sensors_event_t event;
        hmc5883l.getEvent(&event);
        float rawHeading = atan2(event.magnetic.y, event.magnetic.x);
        rawHeading = rawHeading * 180 / PI;
        if (rawHeading < 0) rawHeading += 360;

        headingReadings.push_back(rawHeading);
        if (headingReadings.size() > FILTER_WINDOW_SIZE) {
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

        // Calculate display offsets
        float scaledPointX_global = (float)currentRenderParams.targetPointMVT_X * (screenW * currentRenderParams.zoomScaleFactor / currentRenderParams.layerExtent);
        float scaledPointY_global = (float)currentRenderParams.targetPointMVT_Y * (screenH * currentRenderParams.zoomScaleFactor / currentRenderParams.layerExtent);
        currentRenderParams.displayOffsetX = round(scaledPointX_global - (screenW / 2.0f));
        currentRenderParams.displayOffsetY = round(scaledPointY_global - (screenH / 2.0f));

        // 5. Request necessary tiles from Data Task based on the new loading strategy
        // Only request tiles if the center tile or zoom has changed significantly
        if (!(newRequestedCenterTile == currentlyLoadedCenterTile) || 
            fabs(currentControlParams.zoomFactor - lastSentZoomFactor) >= 0.01f) {
            
            // Update currently loaded center tile
            currentlyLoadedCenterTile = newRequestedCenterTile; 

            // Use a set to keep track of requested tiles in this cycle to avoid duplicates
            std::set<TileKey> requestedTilesInCycle;
            // This set will store all tiles that have been successfully requested/are considered "loaded" for adjacency checks
            std::set<TileKey> currentlyLoadedOrRequestedTiles;


            // Helper lambda to send tile request if not already loaded and not already requested in this cycle
            // Returns true if the tile was actually sent, false otherwise (already loaded or queue full)
            auto sendTileRequest = [&](const TileKey& key) -> bool {
                if (requestedTilesInCycle.find(key) == requestedTilesInCycle.end()) { // If not already requested in this cycle
                    bool alreadyLoaded = false;
                    if (xSemaphoreTake(loadedTilesDataMutex, 0) == pdTRUE) { // Try to get mutex without blocking
                        alreadyLoaded = (loadedTilesData.find(key) != loadedTilesData.end());
                        xSemaphoreGive(loadedTilesDataMutex);
                    }
                    if (!alreadyLoaded) {
                        if (xQueueSend(tileRequestQueue, &key, 0) != pdPASS) {
                            // Serial.println("❌ Render Task: Failed to send tile request. Queue full?"); // Keep silent for non-critical requests
                            return false; // Failed to send
                        }
                        requestedTilesInCycle.insert(key); // Mark as requested in this cycle
                        currentlyLoadedOrRequestedTiles.insert(key); // Add to the set of tiles for adjacency checks
                        return true; // Successfully sent
                    }
                    currentlyLoadedOrRequestedTiles.insert(key); // Already loaded, but still part of the "loaded" set
                }
                return false; // Already requested or loaded
            };

            // Phase 1: Request Central Tile
            sendTileRequest(newRequestedCenterTile);

            // Phase 2: Immediate Neighbors (3x3 grid excluding center)
            std::vector<std::pair<int, int>> criticalNeighborOffsets;
            std::vector<std::pair<int, int>> otherNeighborOffsets;

            float edge_threshold = currentRenderParams.layerExtent * 0.25f; // 25% threshold for criticality

            bool nearLeft = (internalTargetPointMVT_X < edge_threshold);
            bool nearRight = (internalTargetPointMVT_X > (currentRenderParams.layerExtent - edge_threshold));
            bool nearTop = (internalTargetPointMVT_Y < edge_threshold);
            bool nearBottom = (internalTargetPointMVT_Y > (currentRenderParams.layerExtent - edge_threshold));

            // Populate critical and other neighbor offsets
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    if (dx == 0 && dy == 0) continue; // Skip central tile

                    bool isCritical = false;
                    // Check for critical neighbors based on target point proximity to edges
                    if (dx == -1 && nearLeft) isCritical = true;
                    if (dx == 1 && nearRight) isCritical = true;
                    if (dy == -1 && nearTop) isCritical = true;
                    if (dy == 1 && nearBottom) isCritical = true;

                    // If it's a corner, and both axes are critical, it's critical
                    if ((dx == -1 && nearLeft && dy == -1 && nearTop) ||
                        (dx == 1 && nearRight && dy == -1 && nearTop) ||
                        (dx == -1 && nearLeft && dy == 1 && nearBottom) ||
                        (dx == 1 && nearRight && dy == 1 && nearBottom)) {
                        isCritical = true;
                    }

                    if (isCritical) {
                        criticalNeighborOffsets.push_back({dx, dy});
                    } else {
                        otherNeighborOffsets.push_back({dx, dy});
                    }
                }
            }

            // Send critical immediate neighbors first
            for (const auto& offset : criticalNeighborOffsets) {
                TileKey neighborKey = {currentTileZ, newCentralTileX + offset.first, newCentralTileY_TMS + offset.second};
                sendTileRequest(neighborKey);
            }

            // Then send other immediate neighbors
            for (const auto& offset : otherNeighborOffsets) {
                TileKey neighborKey = {currentTileZ, newCentralTileX + offset.first, newCentralTileY_TMS + offset.second};
                sendTileRequest(neighborKey);
            }

            // Phase 3: Secondary Ring (5x5 excluding 3x3)
            std::vector<TileKey> secondaryContactTiles;
            std::vector<TileKey> secondaryOtherTiles;

            for (int dx = -2; dx <= 2; ++dx) {
                for (int dy = -2; dy <= 2; ++dy) {
                    // Skip tiles already covered by the 3x3 grid
                    if (abs(dx) <= 1 && abs(dy) <= 1) continue; 

                    TileKey candidateKey = {currentTileZ, newCentralTileX + dx, newCentralTileY_TMS + dy};
                    
                    bool isContact = false;
                    // Check if candidate is adjacent to any tile in the currentlyLoadedOrRequestedTiles (which includes central and immediate neighbors)
                    for (const auto& loadedKey : currentlyLoadedOrRequestedTiles) {
                        if (areTilesAdjacent(candidateKey, loadedKey)) {
                            isContact = true;
                            break;
                        }
                    }

                    if (isContact) {
                        secondaryContactTiles.push_back(candidateKey);
                    } else {
                        secondaryOtherTiles.push_back(candidateKey);
                    }
                }
            }

            // Send secondary contact tiles first
            for (const auto& key : secondaryContactTiles) {
                sendTileRequest(key);
            }
            // Then send other secondary tiles
            for (const auto& key : secondaryOtherTiles) {
                sendTileRequest(key);
            }

            // Phase 4: Tertiary Ring (7x7 excluding 5x5)
            std::vector<TileKey> tertiaryContactTiles;
            std::vector<TileKey> tertiaryOtherTiles;

            for (int dx = -3; dx <= 3; ++dx) {
                for (int dy = -3; dy <= 3; ++dy) {
                    // Skip tiles already covered by the 5x5 grid
                    if (abs(dx) <= 2 && abs(dy) <= 2) continue;

                    TileKey candidateKey = {currentTileZ, newCentralTileX + dx, newCentralTileY_TMS + dy};

                    bool isContact = false;
                    // Check if candidate is adjacent to any tile in the currentlyLoadedOrRequestedTiles (which now includes 5x5 grid)
                    for (const auto& loadedKey : currentlyLoadedOrRequestedTiles) {
                        if (areTilesAdjacent(candidateKey, loadedKey)) {
                            isContact = true;
                            break;
                        }
                    }

                    if (isContact) {
                        tertiaryContactTiles.push_back(candidateKey);
                    } else {
                        tertiaryOtherTiles.push_back(candidateKey);
                    }
                }
            }

            // Send tertiary contact tiles first
            for (const auto& key : tertiaryContactTiles) {
                sendTileRequest(key);
            }
            // Then send other tertiary tiles
            for (const auto& key : tertiaryOtherTiles) {
                sendTileRequest(key);
            }
        }

        // 6. Check for tile parsed notifications (optional, but good for responsiveness)
        bool notification;
        if (xQueueReceive(tileParsedNotificationQueue, &notification, 0) == pdPASS) {
            if (notification) {
                // A new tile was parsed, force a redraw to include it
                // This is implicitly handled by the render loop always drawing available tiles.
            } else {
                // Handle notification of a failed tile load if necessary
            }
        }

        // 7. Render the map
        // Always clear the sprite at the beginning of each render cycle
        sprite.fillScreen(TFT_BLACK); 
        
        // Acquire mutex before reading shared data
        // Increased timeout from pdMS_TO_TICKS(1) to pdMS_TO_TICKS(100)
        if (xSemaphoreTake(loadedTilesDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) { 
            if (!loadedTilesData.empty()) {
                for (auto it = loadedTilesData.begin(); it != loadedTilesData.end(); ++it) {
                    const TileKey& tileKey = it->first;
                    const std::vector<ParsedLayer, PSRAMAllocator<ParsedLayer>>& layers = it->second;
                    for (const auto& layer : layers) {
                        for (const auto& feature : layer.features) {
                            drawParsedFeature(feature, layer.extent, tileKey, currentRenderParams);
                        }
                    }
                }
            }
            xSemaphoreGive(loadedTilesDataMutex); // Release mutex
        } else {
            Serial.println("❌ Render Task: Failed to acquire mutex for loadedTilesData during drawing.");
        }

        // Update and display FPS
        frameCount++;
        if (millis() - lastFpsTime >= 1000) { // Update FPS every 1 second
            currentFps = (float)frameCount / ((millis() - lastFpsTime) / 1000.0f);
            lastFpsTime = millis();
            frameCount = 0;
        }

        sprite.setTextColor(TFT_YELLOW, TFT_BLACK);
        sprite.setCursor(5, 5);
        sprite.printf("Zoom: %.1fx", currentRenderParams.zoomScaleFactor); // Display the current zoom factor
        sprite.setCursor(5, 15); // New line for heading
        sprite.printf("Hdg: %.1f deg", currentRenderParams.mapRotationDegrees);
        sprite.setCursor(5, 25); // New line for FPS
        sprite.printf("FPS: %.1f", currentFps);
        
        sprite.pushSprite(0, 0); // Push the completed sprite to the physical display

        lastSentRotationAngle = internalCurrentRotationAngle; // Update last sent rotation
        lastSentZoomFactor = currentControlParams.zoomFactor; // Update last sent zoom

        vTaskDelay(pdMS_TO_TICKS(1)); // Yield to other tasks
    }
}
