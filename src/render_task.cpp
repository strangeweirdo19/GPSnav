// render_task.cpp
#include "render_task.h"
#include "common.h"
#include "map_renderer.h" // Include the new map renderer header
#include "colors.h"       // Include the new colors header
#include <cfloat>         // Required for FLT_MAX and FLT_MIN

// HMC5883L compass object - changed to Adafruit_HMC5883_Unified
Adafruit_HMC5883_Unified hmc5883l = Adafruit_HMC5883_Unified(12345); // Using a sensor ID, common for unified sensors

// Global variables for compass heading filtering (moving average)
// Changed from std::vector to std::array for fixed-size circular buffer
std::array<float, COMPASS_FILTER_WINDOW_SIZE> headingReadings_buffer; // Fixed-size array
size_t headingReadings_index = 0; // Current write position
bool headingReadings_full = false; // Flag to indicate buffer is filled once
float lastSmoothedRotationAngle = 0.0f; // For exponential smoothing


// =========================================================
// RENDER TASK (Core 1)
// Handles display updates, sensor readings, and tile requests
// =========================================================
void renderTask(void *pvParameters) {
    // Boot screen still showing - don't touch display yet
    // Just initialize sprite
    sprite.setColorDepth(16);
    sprite.createSprite(screenW, screenH);

    // Initialize HMC5883L compass
    Wire.begin(); // Initialize I2C
    if (!hmc5883l.begin()) {
        Serial.println("❌ Render Task: Could not find a valid HMC5883L sensor, check wiring!");
    } else {
        hmc5883l.setMagGain(HMC5883_MAGGAIN_1_3);
    }

    // Initialize currentControlParams with default values
    ControlParams currentControlParams = {
        .targetLat = 12.8273,
        .targetLon = 80.2193,
        .zoomFactor = 1.0,
        .cullingBufferPercentageLeft = DEFAULT_CULLING_BUFFER_PERCENTAGE_LEFT,
        .cullingBufferPercentageRight = DEFAULT_CULLING_BUFFER_PERCENTAGE_RIGHT,
        .cullingBufferPercentageTop = DEFAULT_CULLING_BUFFER_PERCENTAGE_TOP,
        .cullingBufferPercentageBottom = DEFAULT_CULLING_BUFFER_PERCENTAGE_BOTTOM,
        .bleIconMode = 1,
        .showPIN = false,
        .pinCode = {0},
        .deviceName = {0}
    };
    RenderParams currentRenderParams;

    // State variables for loading management
    TileKey currentRequestedCenterTile = {-1, -1, -1};
    TileKey currentlyLoadedCenterTile = {-2, -2, -2};

    float internalCurrentRotationAngle = 0.0f;
    float lastSentRotationAngle = 0.0f;
    float lastSentZoomFactor = 0.0f;

    const int STATUS_BAR_HEIGHT = 10;
    const int ARROW_MARGIN_ABOVE_STATUS_BAR = 2;
    const float STATUS_BAR_ALPHA = 0.5f;
    
    bool bootScreenCleared = false; // Track if we've cleared the boot screen
    const int TILES_NEEDED_FOR_BOOT = 20; // Show boot screen until this many tiles load
    
    // Animated Zoom State
    float animatedZoomFactor = 1.0f; // Current animated zoom level
    const float ZOOM_LERP_SPEED = 0.25f; // 25% per frame (snappier than 0.15f)

    while (true) {
        bool screenNeedsUpdate = false;
        
        // Check if we should clear boot screen and show map
        if (!bootScreenCleared && tilesLoadedCount >= TILES_NEEDED_FOR_BOOT) {
            tft.fillScreen(MAP_BACKGROUND_COLOR);
            bootScreenCleared = true;
            screenNeedsUpdate = true;
            Serial.println("Boot screen cleared - showing map");
        }
        
        // Update boot screen progress bar based on tile loading
        if (!bootScreenCleared && tilesLoadedCount > 0) {
            // Draw a simple progress indicator on boot screen
            int barX = (screenW - 100) / 2;
            int barY = screenH / 2 + 15;
            int progress = (tilesLoadedCount * 100) / TILES_NEEDED_FOR_BOOT;
            int fillWidth = progress;
            if (fillWidth > 100) fillWidth = 100;
            tft.fillRoundRect(barX, barY, fillWidth, 2, 1, TFT_WHITE);
        }

        // 1. Check for new control parameters from the main loop (user input)
        ControlParams receivedControlParams;
        if (xQueueReceive(controlParamsQueue, &receivedControlParams, 0) == pdPASS) {
            currentControlParams = receivedControlParams;
            screenNeedsUpdate = true; // New control params always mean screen update
        }

        // Logic for blinking BLE Connected icon (Mode 0)
        static unsigned long lastIconBlinkTime = 0;
        static bool iconBlinkVisible = true;
        if (currentControlParams.bleIconMode == 0) { // Blink Red
             if (millis() - lastIconBlinkTime > 500) {
                 iconBlinkVisible = !iconBlinkVisible;
                 lastIconBlinkTime = millis();
                 screenNeedsUpdate = true; // Trigger redraw for blink update
             }
        } else {
            iconBlinkVisible = true; // Always visible in other modes
        }

        // 2. Read compass data and update rotation (rate-limited for FPS)
        static unsigned long lastCompassRead = 0;
        const unsigned long COMPASS_READ_INTERVAL_MS = 50; // Read compass at 20Hz max
        
        sensors_event_t event;
        bool compassReadThisFrame = false;
        
        if (millis() - lastCompassRead >= COMPASS_READ_INTERVAL_MS) {
            lastCompassRead = millis();
            hmc5883l.getEvent(&event);
            compassReadThisFrame = true;
        }
        
        // Only update heading if compass was read and data is valid
        if (compassReadThisFrame && !isnan(event.magnetic.x) && !isnan(event.magnetic.y)) {
            float rawHeading = atan2(event.magnetic.y, event.magnetic.x);
            rawHeading = rawHeading * 180 / PI;
            if (rawHeading < 0) rawHeading += 360;

            // Add new reading to circular buffer
            headingReadings_buffer[headingReadings_index] = rawHeading;
            headingReadings_index = (headingReadings_index + 1) % COMPASS_FILTER_WINDOW_SIZE;
            if (!headingReadings_full && headingReadings_index == 0) {
                headingReadings_full = true; // Buffer has been filled at least once
            }

            // Calculate moving average using circular statistics
            float sumSin = 0.0f;
            float sumCos = 0.0f;
            size_t count = headingReadings_full ? COMPASS_FILTER_WINDOW_SIZE : headingReadings_index;
            
            for (size_t i = 0; i < count; ++i) {
                sumSin += sin(radians(headingReadings_buffer[i]));
                sumCos += cos(radians(headingReadings_buffer[i]));
            }
            
            // Avoid division by zero if count is 0 (shouldn't happen if filter window is > 0)
            if (count > 0) {
                float averagedHeading = degrees(atan2(sumSin / count, sumCos / count));
                if (averagedHeading < 0) averagedHeading += 360;
                
                // Smooth across 0/360 by using shortest angular difference
                float angleDelta = averagedHeading - lastSmoothedRotationAngle;
                angleDelta = fmodf(angleDelta + 540.0f, 360.0f) - 180.0f; // Normalize to [-180,180]
                internalCurrentRotationAngle = lastSmoothedRotationAngle + (COMPASS_EXPONENTIAL_SMOOTHING_ALPHA * angleDelta);
                // Wrap back to [0,360)
                internalCurrentRotationAngle = fmodf(internalCurrentRotationAngle + 360.0f, 360.0f);
                lastSmoothedRotationAngle = internalCurrentRotationAngle;
            } else {
                // Keep last value instead of snapping to 0 when sensor data is briefly invalid
                internalCurrentRotationAngle = lastSmoothedRotationAngle;
            }
        }


        // Check if rotation has changed significantly
        if (fabs(internalCurrentRotationAngle - lastSentRotationAngle) >= COMPASS_ROTATION_THRESHOLD_DEG) {
            lastSentRotationAngle = internalCurrentRotationAngle; // Update Stable Angle
            screenNeedsUpdate = true;
        }

        // =========================================================
        // GPS INTERPOLATION (Dead Reckoning) for smooth animation
        // =========================================================
        double smoothLat = currentControlParams.targetLat;
        double smoothLon = currentControlParams.targetLon;
        
        if (phoneGpsActive && xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            unsigned long now = millis();
            float elapsed = (now - gpsState.timestamp) / 1000.0f; // seconds since last GPS
            
            // SMART INTERPOLATION v2: Constant Deceleration Model
            // User Request: Max 1s, start decay immediately.
            // Formula: Velocity decreases linearly from InitialSpeed to 0 over 1.0s.
            // Distance d(t) = v0 * t - 0.5 * a * t^2
            // Where a = v0 / T_max. So d(t) = v0 * (t - 0.5 * t^2 / T_max)
            
            float maxTime = 1.0f;
            
            if (gpsState.speed > 0.5f && elapsed < maxTime && elapsed > 0.0f) {
                // Apply 95% safety factor to initial speed
                float v0 = gpsState.speed * 0.95f;
                
                // Parabolic distance curve (w/ braking)
                // At t=0, speed = v0. At t=1.0, speed = 0.
                float distance = v0 * (elapsed - (0.5f * elapsed * elapsed / maxTime));
                
                float headingRad = gpsState.heading * 3.14159f / 180.0f;
                
                // Convert distance to lat/lon delta
                double dLat = (distance * cos(headingRad)) / 111320.0;
                double dLon = (distance * sin(headingRad)) / (111320.0 * cos(gpsState.lat * 3.14159 / 180.0));
                
                smoothLat = gpsState.lat + dLat;
                smoothLon = gpsState.lon + dLon;
                screenNeedsUpdate = true; // Force redraw
            } else if (elapsed >= maxTime && gpsState.speed > 0.5f) {
                // Hold position at max extrapolation (stopped)
                float v0 = gpsState.speed * 0.95f;
                float distance = v0 * (maxTime - 0.5f * maxTime); // distance at t=maxTime
                float headingRad = gpsState.heading * 3.14159f / 180.0f;
                
                double dLat = (distance * cos(headingRad)) / 111320.0;
                double dLon = (distance * sin(headingRad)) / (111320.0 * cos(gpsState.lat * 3.14159 / 180.0));
                 
                smoothLat = gpsState.lat + dLat;
                smoothLon = gpsState.lon + dLon;
                // No forced redraw needed if stopped
            } else if (gpsState.lat != 0.0 && gpsState.lon != 0.0) {
                // Use raw GPS position
                smoothLat = gpsState.lat;
                smoothLon = gpsState.lon;
            }
            xSemaphoreGive(gpsMutex);
        }


        // 3. Calculate current central tile and target MVT point (using interpolated position)
        int newCentralTileX, newCentralTileY_std, newCentralTileY_TMS;
        latlonToTile(smoothLat, smoothLon, currentTileZ,
                     newCentralTileX, newCentralTileY_std, newCentralTileY_TMS);

        TileKey newRequestedCenterTile = {currentTileZ, newCentralTileX, newCentralTileY_TMS};

        int internalTargetPointMVT_X, internalTargetPointMVT_Y;
        latLonToMVTCoords(smoothLat, smoothLon, currentTileZ, newCentralTileX, newCentralTileY_TMS,
                          internalTargetPointMVT_X, internalTargetPointMVT_Y, currentLayerExtent);


        // 4. Update currentRenderParams
        currentRenderParams.centralTileX = newCentralTileX;
        currentRenderParams.centralTileY_TMS = newCentralTileY_TMS;
        currentRenderParams.targetPointMVT_X = internalTargetPointMVT_X;
        currentRenderParams.targetPointMVT_Y = internalTargetPointMVT_Y;
        currentRenderParams.layerExtent = currentLayerExtent;
        
        // Smooth Zoom Animation (Lerp towards target)
        float targetZoom = currentControlParams.zoomFactor;
        if (fabs(animatedZoomFactor - targetZoom) > 0.001f) {
            animatedZoomFactor += (targetZoom - animatedZoomFactor) * ZOOM_LERP_SPEED;
            screenNeedsUpdate = true; // Force redraw during animation
        } else {
            animatedZoomFactor = targetZoom; // Snap when close enough
        }
        currentRenderParams.zoomScaleFactor = animatedZoomFactor;
        currentRenderParams.mapRotationDegrees = lastSentRotationAngle; // Use STABLE angle
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
                        // Attempt to send the request (short timeout to avoid blocking render)
                        if (xQueueSend(tileRequestQueue, &key, pdMS_TO_TICKS(10)) != pdPASS) {
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

        // 7. Render the map (only if needed AND boot screen is cleared)
        if ((screenNeedsUpdate || globalOTAState.active) && bootScreenCleared) {
            sprite.fillScreen(MAP_BACKGROUND_COLOR); // Changed background color to #1a2632
            
            // =========================================================
            // OTA UPDATE SCREEN
            // =========================================================
            if (globalOTAState.active) {
                // Background
                sprite.fillScreen(TFT_BLACK);
                
                // Title "UPDATING..."
                sprite.setTextColor(TFT_WHITE, TFT_BLACK);
                sprite.setTextSize(2);
                sprite.setTextDatum(MC_DATUM); // Middle Center
                sprite.drawString("UPDATING...", screenW / 2, 40);
                
                // Type "FIRMWARE" or "MAP DATA"
                sprite.setTextSize(1);
                sprite.setTextColor(TFT_CYAN, TFT_BLACK);
                sprite.drawString(globalOTAState.type, screenW / 2, 70);
                
                // Progress Bar
                int barWidth = screenW - 40;
                int barHeight = 10;
                int barX = 20;
                int barY = 90;
                
                // Draw Empty Bar
                sprite.drawRect(barX, barY, barWidth, barHeight, TFT_WHITE);
                
                // Draw Filled Bar
                int fillWidth = (barWidth - 4) * globalOTAState.percent / 100;
                if (fillWidth > 0) {
                    sprite.fillRect(barX + 2, barY + 2, fillWidth, barHeight - 4, TFT_GREEN);
                }
                
                // Percentage Text
                sprite.setTextColor(TFT_WHITE, TFT_BLACK);
                sprite.setTextSize(2);
                String percentStr = String(globalOTAState.percent) + "%";
                sprite.drawString(percentStr, screenW / 2, 120);
                
                sprite.pushSprite(0, 0);
                vTaskDelay(pdMS_TO_TICKS(50)); // Limit refresh rate
                continue; // Skip map rendering
            }

            if (xSemaphoreTake(loadedTilesDataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
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

            // =========================================================
            // DRAW ROUTE OVERLAY (Blue polyline on top of map)
            // =========================================================
            if ((routeAvailable || (isRouteSyncing && !activeRoute.empty())) && xSemaphoreTake(routeMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (!activeRoute.empty()) {
                    // Re-index trigger
                    if (lastIndexedZoom != currentTileZ) {
                        indexRouteLocked();
                    }

                    // Calculate rotation params (common for all)
                    float cosTheta = cos(radians(currentRenderParams.mapRotationDegrees));
                    float sinTheta = sin(radians(currentRenderParams.mapRotationDegrees));
                    int centerX = screenW / 2;
                    int centerY = currentRenderParams.pivotY;
                    
                    // ===========================================
                    // RENDER ALTERNATES (Light Gray)
                    // ===========================================
                    for (int rIdx = 0; rIdx < 2; rIdx++) {
                        if (!alternateRoutes[rIdx].empty()) {
                            // Use Anchor + Delta for first point
                            double altLat = alternateAnchors[rIdx].lat + alternateRoutes[rIdx][0].dLat / 100000.0;
                            double altLon = alternateAnchors[rIdx].lon + alternateRoutes[rIdx][0].dLon / 100000.0;
                            int lastX = -1, lastY = -1;
                            
                            for (size_t k = 0; k < alternateRoutes[rIdx].size(); k++) {
                                if (k > 0) {
                                    altLat += alternateRoutes[rIdx][k].dLat / 100000.0;
                                    altLon += alternateRoutes[rIdx][k].dLon / 100000.0;
                                }
                                
                                // Check bounds optimization
                                // if (abs(altLat - currentControlParams.targetLat) > 0.05) continue; // Rough check
                                
                                // Projection
                                int tileX, tileY_std, tileY_TMS;
                                latlonToTile(altLat, altLon, currentTileZ, tileX, tileY_std, tileY_TMS);
                                int mvtX, mvtY;
                                latLonToMVTCoords(altLat, altLon, currentTileZ, tileX, tileY_TMS, mvtX, mvtY, currentRenderParams.layerExtent);
                                
                                float tileRenderWidth = (float)screenW * currentRenderParams.zoomScaleFactor;
                                float tileRenderHeight = (float)screenH * currentRenderParams.zoomScaleFactor;
                                float tileOffsetX = (float)(tileX - currentRenderParams.centralTileX) * tileRenderWidth;
                                float tileOffsetY = (float)(currentRenderParams.centralTileY_TMS - tileY_TMS) * tileRenderHeight;
                                float scaleX = (float)screenW * currentRenderParams.zoomScaleFactor / currentRenderParams.layerExtent;
                                float scaleY = (float)screenH * currentRenderParams.zoomScaleFactor / currentRenderParams.layerExtent;
                                float screenX = (float)mvtX * scaleX + tileOffsetX - currentRenderParams.displayOffsetX;
                                float screenY = (float)mvtY * scaleY + tileOffsetY - currentRenderParams.displayOffsetY;
                                
                                float translatedX = screenX - centerX;
                                float translatedY = screenY - centerY;
                                float rotatedX = translatedX * cosTheta - translatedY * sinTheta;
                                float rotatedY = translatedX * sinTheta + translatedY * cosTheta;
                                int finalX = round(rotatedX + centerX);
                                int finalY = round(rotatedY + centerY);
                                
                                // Perspective removed
                                finalX = round(rotatedX + centerX);
                                finalY = round(rotatedY + centerY);
                                
                                if (lastX != -1) {
                                    sprite.drawLine(lastX, lastY, finalX, finalY, ALTERNATE_ROUTE_COLOR);
                                }
                                lastX = finalX;
                                lastY = finalY;
                            }
                        }
                    }
                    
                    // Decouple route rendering from loaded map tiles
                    // Iterate directly over the route buckets
                    // xSemaphoreTake(loadedTilesDataMutex, pdMS_TO_TICKS(50)); // No longer needed for route

                    for (const auto& item : routeTileIndex) {
                        const TileKey& tKey = item.first;
                        
                        // Visibility Check optimization (Manhattan distance)
                        // Only draw tiles that are reasonably close to the center
                        // 2 tile radius is usually enough for 320x240 screen at standard zooms
                        if (abs(tKey.x - currentRenderParams.centralTileX) > 2 || 
                            abs(tKey.y_tms - currentRenderParams.centralTileY_TMS) > 2) {
                            continue;
                        }

                        // Draw all segments in this tile
                        for (const auto& seg : item.second) {
                            double currentLat = seg.startLat;
                            double currentLon = seg.startLon;
                                
                                // Render Loop for Segment
                                for (size_t k = 0; k < seg.count; k++) {
                                    size_t idx = seg.startIndex + k;
                                    
                                    // ROUTE TRIMMING: Skip points already passed
                                    if (idx < routeProgressIndex) {
                                        // Still need to update coordinates for next iteration!
                                        if (k < seg.count - 1) {
                                            size_t nextIdx = idx + 1;
                                            if (nextIdx < activeRoute.size()) {
                                                currentLat += activeRoute[nextIdx].dLat / 100000.0;
                                                currentLon += activeRoute[nextIdx].dLon / 100000.0;
                                            }
                                        }
                                        continue; 
                                    }

                                    // If this is the FIRST visible point (idx == routeProgressIndex),
                                    // force it to NOT draw a line from the previous (hidden) point.
                                    // This is handled automatically because we skipped the previous iteration,
                                    // so 'prevScreenX' logic which depends on k-1 flow needs check.
                                    // But wait, the loop calculates prevScreenX by backtracking if k==0.
                                    // If k > 0 but we skipped k-1, we need to ensure we don't use stale prevScreenX?
                                    // Actually, I should just set a flag or let the logic handle it.
                                    
                                    // Calculate current point screen position
                                    // ------------------------------------------
                                    // Reuse existing projection logic helper? 
                                    // Or inline it as before. Inlining for now to match style.
                                    
                                    int tileX, tileY_std, tileY_TMS;
                                    latlonToTile(currentLat, currentLon, currentTileZ, tileX, tileY_std, tileY_TMS);
                                    
                                    int mvtX, mvtY;
                                    latLonToMVTCoords(currentLat, currentLon, currentTileZ, tileX, tileY_TMS, mvtX, mvtY, currentRenderParams.layerExtent);
                                    
                                    float tileRenderWidth = (float)screenW * currentRenderParams.zoomScaleFactor;
                                    float tileRenderHeight = (float)screenH * currentRenderParams.zoomScaleFactor;
                                    float tileOffsetX = (float)(tileX - currentRenderParams.centralTileX) * tileRenderWidth;
                                    float tileOffsetY = (float)(currentRenderParams.centralTileY_TMS - tileY_TMS) * tileRenderHeight;
                                    
                                    float scaleX = (float)screenW * currentRenderParams.zoomScaleFactor / currentRenderParams.layerExtent;
                                    float scaleY = (float)screenH * currentRenderParams.zoomScaleFactor / currentRenderParams.layerExtent;
                                    
                                    float screenX = (float)mvtX * scaleX + tileOffsetX - currentRenderParams.displayOffsetX;
                                    float screenY = (float)mvtY * scaleY + tileOffsetY - currentRenderParams.displayOffsetY;
                                    
                                    // Rotation
                                    float translatedX = screenX - centerX;
                                    float translatedY = screenY - centerY;
                                    float rotatedX = translatedX * cosTheta - translatedY * sinTheta;
                                    float rotatedY = translatedX * sinTheta + translatedY * cosTheta;
                                    int finalX = round(rotatedX + centerX);
                                    int finalY = round(rotatedY + centerY);
                                    
                                    // Perspective
                                    if (currentRenderParams.zoomScaleFactor >= 3.0f) {
                                        float perspX, perspY;
                                        applyPerspective((float)finalX, (float)finalY, perspX, perspY, currentRenderParams.pivotY);
                                        finalX = round(perspX);
                                        finalY = round(perspY);
                                    }
                                    
                                    // Determine Previous Point for pairing
                                    // ------------------------------------
                                    int prevScreenX = -1, prevScreenY = -1;
                                    
                                    if (k == 0) {
                                        // First point of segment.
                                        // If this is NOT the absolute start of route (idx 1 is first real delta point),
                                        // then back-calculate the PREVIOUS point from the delta.
                                        // activeRoute[idx] stores (current - prev).
                                        // So prev = current - delta.
                                        
                                        if (idx > 1) { // Index 0 is dummy, Index 1 is start. So if idx > 1, there is a previous point.
                                            double prevLat = currentLat - (activeRoute[idx].dLat / ROUTE_SCALE);
                                            double prevLon = currentLon - (activeRoute[idx].dLon / ROUTE_SCALE);
                                            
                                            // Project Previous Point
                                            int pTileX, pTileY_std, pTileY_TMS;
                                            latlonToTile(prevLat, prevLon, currentTileZ, pTileX, pTileY_std, pTileY_TMS);
                                            int pMvtX, pMvtY;
                                            latLonToMVTCoords(prevLat, prevLon, currentTileZ, pTileX, pTileY_TMS, pMvtX, pMvtY, currentRenderParams.layerExtent);
                                            
                                            float pTileOffsetX = (float)(pTileX - currentRenderParams.centralTileX) * tileRenderWidth;
                                            float pTileOffsetY = (float)(currentRenderParams.centralTileY_TMS - pTileY_TMS) * tileRenderHeight;
                                            
                                            float pScreenX = (float)pMvtX * scaleX + pTileOffsetX - currentRenderParams.displayOffsetX;
                                            float pScreenY = (float)pMvtY * scaleY + pTileOffsetY - currentRenderParams.displayOffsetY;
                                            
                                            float pTransX = pScreenX - centerX;
                                            float pTransY = pScreenY - centerY;
                                            float pRotX = pTransX * cosTheta - pTransY * sinTheta;
                                            float pRotY = pTransX * sinTheta + pTransY * cosTheta;
                                            
                                            int pFinalX = round(pRotX + centerX);
                                            int pFinalY = round(pRotY + centerY);
                                            
                                            // Perspective removed
                                            prevScreenX = pFinalX;
                                            prevScreenY = pFinalY;
                                        }
                                    } else {
                                        // Not first point, we could cache the previous finalX/Y from previous iteration?
                                        // But here we are re-calculating for simplicity of the loop structure inside block.
                                        // Actually optimization: current point becomes prev for next k.
                                        // But loop design here is "Calculate Current, Find Prev". 
                                        // Let's optimize: Store 'lastFinalX/Y' outside k loop.
                                        // But handling the k=0 case is the special logic.
                                        // Okay, let's keep it safe. k=0 handles boundary crossing.
                                        // k > 0 handles internal connections.
                                        // Need to ensure k>0 connects to k-1.
                                        // I will defer k>0 logic to the `prevScreenX` variable if I hoist it out.
                                    }
                                    
                                    // DRAW LINE
                                    // ---------
                                    // Issues with the above "k > 0" thought:
                                    // If I iterate k, I want to draw line (k-1 -> k).
                                    // For k=0, I draw line (boundary -> 0).
                                    // So I always draw a line ending at 'current'.
                                    
                                    // Wait, if I use the hoist method:
                                    // int lastX, lastY;
                                    // if k=0: calculate lastX/Y using backtrack.
                                    // else: lastX/Y = valid from previous iter.
                                    // Then draw last -> current.
                                    // Then last = current.
                                    
                                    // Refined Loop:
                                    // 1. Calculate Current Point Screen Coords (finalX, finalY)
                                    // 2. Identify Prev Screen Coords (prevScreenX, prevScreenY)
                                    //    If k==0: Backtrack calculation.
                                    //    If k>0:  Use stored value from previous iteration.
                                    
                                    // But wait, my manual backtrack code calculates prevScreenX/Y.
                                    // So:
                                    
                                    /* 
                                     int lastScreenX = -1, lastScreenY = -1;
                                     // ... inside k loop...
                                     // Calculate current finalX, finalY
                                     
                                     if (k == 0) {
                                        // Backtrack logic -> sets lastScreenX/Y
                                     } 
                                     
                                     if (lastScreenX != -1) {
                                        drawLine(lastScreenX, lastScreenY, finalX, finalY)
                                     }
                                     
                                     lastScreenX = finalX;
                                     lastScreenY = finalY;
                                     
                                     // Update lat/lon for next k
                                     if (k + 1 < seg.count) {
                                        currentLat += delta...
                                        currentLon += delta...
                                     }
                                    */
                                     
                                    // This looks cleaner!
                                } // end k loop
                                
                                // IMPLEMENTING THE CLEANER LOOP NOW
                                // Lambda to prevent code duplication for two-pass rendering
                                auto drawRoutePass = [&](bool drawFuture) {
                                    int lastScreenX = -9999, lastScreenY = -9999;
                                    double iterLat = seg.startLat;
                                    double iterLon = seg.startLon;
                                    
                                    for (size_t k = 0; k < seg.count; k++) {
                                        size_t idx = seg.startIndex + k;
                                        
                                        // ROUTE TRIMMING: Skip points already passed
                                        if (idx < routeProgressIndex) {
                                            if (k < seg.count - 1) {
                                                size_t nextIdx = idx + 1;
                                                if (nextIdx < activeRoute.size()) {
                                                    iterLat += activeRoute[nextIdx].dLat / 100000.0;
                                                    iterLon += activeRoute[nextIdx].dLon / 100000.0;
                                                }
                                            }
                                            continue; 
                                        }

                                        // Optimization: If drawing Active (Blue) pass, stop when hitting Future part
                                        // We check > switchIndex because switchIndex segment is still Blue
                                        if (!drawFuture && idx > routeLegSwitchIndex) break;

                                        if (idx == routeProgressIndex) {
                                            lastScreenX = -9999;
                                            lastScreenY = -9999;
                                        }
                                        
                                        // 1. Calculate Screen Position
                                        int tileX, tileY_std, tileY_TMS;
                                        latlonToTile(iterLat, iterLon, currentTileZ, tileX, tileY_std, tileY_TMS);
                                        int mvtX, mvtY;
                                        latLonToMVTCoords(iterLat, iterLon, currentTileZ, tileX, tileY_TMS, mvtX, mvtY, currentRenderParams.layerExtent);
                                        
                                        float tileRenderWidth = (float)screenW * currentRenderParams.zoomScaleFactor;
                                        float tileRenderHeight = (float)screenH * currentRenderParams.zoomScaleFactor;
                                        float tileOffsetX = (float)(tileX - currentRenderParams.centralTileX) * tileRenderWidth;
                                        float tileOffsetY = (float)(currentRenderParams.centralTileY_TMS - tileY_TMS) * tileRenderHeight;
                                        float scaleX = (float)screenW * currentRenderParams.zoomScaleFactor / currentRenderParams.layerExtent;
                                        float scaleY = (float)screenH * currentRenderParams.zoomScaleFactor / currentRenderParams.layerExtent;
                                        float screenX = (float)mvtX * scaleX + tileOffsetX - currentRenderParams.displayOffsetX;
                                        float screenY = (float)mvtY * scaleY + tileOffsetY - currentRenderParams.displayOffsetY;
                                        float translatedX = screenX - centerX;
                                        float translatedY = screenY - centerY;
                                        float rotatedX = translatedX * cosTheta - translatedY * sinTheta;
                                        float rotatedY = translatedX * sinTheta + translatedY * cosTheta;
                                        int finalX = round(rotatedX + centerX);
                                        int finalY = round(rotatedY + centerY);
                                        
                                        // 2. Determine Previous Point
                                        if (k == 0) {
                                            if (idx > 0) {
                                                double prevLat = iterLat - (activeRoute[idx].dLat / ROUTE_SCALE);
                                                double prevLon = iterLon - (activeRoute[idx].dLon / ROUTE_SCALE);
                                                int pTileX, pTileY_std, pTileY_TMS;
                                                latlonToTile(prevLat, prevLon, currentTileZ, pTileX, pTileY_std, pTileY_TMS);
                                                int pMvtX, pMvtY;
                                                latLonToMVTCoords(prevLat, prevLon, currentTileZ, pTileX, pTileY_TMS, pMvtX, pMvtY, currentRenderParams.layerExtent);
                                                float pTileOffsetX = (float)(pTileX - currentRenderParams.centralTileX) * tileRenderWidth;
                                                float pTileOffsetY = (float)(currentRenderParams.centralTileY_TMS - pTileY_TMS) * tileRenderHeight;
                                                float pScreenX = (float)pMvtX * scaleX + pTileOffsetX - currentRenderParams.displayOffsetX;
                                                float pScreenY = (float)pMvtY * scaleY + pTileOffsetY - currentRenderParams.displayOffsetY;
                                                float pTransX = pScreenX - centerX;
                                                float pTransY = pScreenY - centerY;
                                                float pRotX = pTransX * cosTheta - pTransY * sinTheta;
                                                float pRotY = pTransX * sinTheta + pTransY * cosTheta;
                                                int pFinalX = round(pRotX + centerX);
                                                int pFinalY = round(pRotY + centerY);
                                                lastScreenX = pFinalX;
                                                lastScreenY = pFinalY;
                                            } else {
                                                lastScreenX = -9999;
                                            }
                                        }
                                        
                                        // 3. Draw Line from Last to Current
                                        if (lastScreenX != -9999 && 
                                            lastScreenX >= -100 && lastScreenX <= screenW + 100 &&
                                            lastScreenY >= -100 && lastScreenY <= screenH + 100) {
                                            
                                            bool isFuture = (idx > routeLegSwitchIndex);
                                            
                                            // Only draw if it matches the current pass
                                            if (drawFuture == isFuture) {
                                                uint16_t color = isFuture ? FUTURE_ROUTE_COLOR : 0x059A; // Cyan #05b4d6
                                                for (int offset = -1; offset <= 1; offset++) {
                                                    drawAntiAliasedLine(lastScreenX + offset, lastScreenY, finalX + offset, finalY, color);
                                                }
                                            }
                                        }
                                        
                                        // 4. Update state
                                        lastScreenX = finalX;
                                        lastScreenY = finalY;
                                        
                                        if (k + 1 < seg.count) {
                                            iterLat += (activeRoute[idx + 1].dLat / ROUTE_SCALE);
                                            iterLon += (activeRoute[idx + 1].dLon / ROUTE_SCALE);
                                        }
                                    }
                                };
                                
                                // Pass 1: Render Future (Orange) FIRST (Bottom Layer)
                                drawRoutePass(true);
                                
                                // Pass 2: Render Active (Blue) SECOND (Top Layer)
                                drawRoutePass(false);
                            }
                            }
                        }
                        
                        // =========================================================
                        // INTERMEDIATE WAYPOINT MARKERS (Multi-Stop)
                        // =========================================================
                        if (waypointBuffer.size() >= 2) {
                            // waypointBuffer contains: [Stop1, Stop2, ..., Destination]
                            // (Start position is NOT included - only intermediate stops + destination)
                            // Draw intermediate waypoints (skip last=destination)
                            
                            // Calculate rotation (reusable for all waypoints)
                            float wpCosTheta = cos(radians(currentRenderParams.mapRotationDegrees));
                            float wpSinTheta = sin(radians(currentRenderParams.mapRotationDegrees));
                            
                            // Draw intermediate stops (indices 0 to N-2, excluding last destination)
                            for (size_t wpIdx = 0; wpIdx < waypointBuffer.size() - 1; wpIdx++) {
                                // Skip past waypoints (already visited)
                                if (wpIdx < currentWaypointIndex) continue;
                                
                                double wpLat = waypointBuffer[wpIdx].lat;
                                double wpLon = waypointBuffer[wpIdx].lon;
                                
                                // Project to screen coords
                                int wpTileX, wpTileY_std, wpTileY_TMS;
                                latlonToTile(wpLat, wpLon, currentTileZ, wpTileX, wpTileY_std, wpTileY_TMS);
                                int wpMvtX, wpMvtY;
                                latLonToMVTCoords(wpLat, wpLon, currentTileZ, wpTileX, wpTileY_TMS, wpMvtX, wpMvtY, currentRenderParams.layerExtent);
                                
                                float wpTileRenderWidth = (float)screenW * currentRenderParams.zoomScaleFactor;
                                float wpTileRenderHeight = (float)screenH * currentRenderParams.zoomScaleFactor;
                                float wpTileOffsetX = (float)(wpTileX - currentRenderParams.centralTileX) * wpTileRenderWidth;
                                float wpTileOffsetY = (float)(currentRenderParams.centralTileY_TMS - wpTileY_TMS) * wpTileRenderHeight;
                                float wpScaleX = wpTileRenderWidth / currentRenderParams.layerExtent;
                                float wpScaleY = wpTileRenderHeight / currentRenderParams.layerExtent;
                                float wpScreenX = (float)wpMvtX * wpScaleX + wpTileOffsetX - currentRenderParams.displayOffsetX;
                                float wpScreenY = (float)wpMvtY * wpScaleY + wpTileOffsetY - currentRenderParams.displayOffsetY;
                                
                                // Apply rotation
                                float wpTransX = wpScreenX - (screenW / 2);
                                float wpTransY = wpScreenY - currentRenderParams.pivotY;
                                float wpRotX = wpTransX * wpCosTheta - wpTransY * wpSinTheta;
                                float wpRotY = wpTransX * wpSinTheta + wpTransY * wpCosTheta;
                                int wpX = round(wpRotX + (screenW / 2));
                                int wpY = round(wpRotY + currentRenderParams.pivotY);
                                
                                // Clamp to screen edges if off-screen (floating marker effect)
                                const int MARKER_MARGIN = 10; // Distance from screen edge
                                int drawX = wpX;
                                int drawY = wpY;
                                
                                // Clamp X coordinate to screen bounds
                                if (drawX < MARKER_MARGIN) drawX = MARKER_MARGIN;
                                if (drawX > screenW - MARKER_MARGIN) drawX = screenW - MARKER_MARGIN;
                                
                                // Clamp Y coordinate to screen bounds  
                                if (drawY < STATUS_BAR_HEIGHT + MARKER_MARGIN) drawY = STATUS_BAR_HEIGHT + MARKER_MARGIN;
                                if (drawY > screenH - MARKER_MARGIN) drawY = screenH - MARKER_MARGIN;
                                
                                // Color based on waypoint status
                                // Current waypoint (where we're navigating to now): Blue (same as active route)
                                // Future waypoints: Orange
                                uint16_t wpColor = (wpIdx == currentWaypointIndex) ? 0x059A : WAYPOINT_MARKER_COLOR;
                                
                                // Draw waypoint marker (Circle for intermediate stops)
                                drawTriangleAndCircle(drawX, drawY, 0, wpColor, false, true);
                            }
                        }
                        
                        // =========================================================
                        // DESTINATION MARKER (Final Destination from waypointBuffer)
                        // =========================================================
                        if (!waypointBuffer.empty()) {
                            // Use the LAST waypoint as the final destination
                            double destLat = waypointBuffer.back().lat;
                            double destLon = waypointBuffer.back().lon;
                            
                            // Project to screen coords
                            int dTileX, dTileY_std, dTileY_TMS;
                            latlonToTile(destLat, destLon, currentTileZ, dTileX, dTileY_std, dTileY_TMS);
                            int dMvtX, dMvtY;
                            latLonToMVTCoords(destLat, destLon, currentTileZ, dTileX, dTileY_TMS, dMvtX, dMvtY, currentRenderParams.layerExtent);
                            
                            float dTileRenderWidth = (float)screenW * currentRenderParams.zoomScaleFactor;
                            float dTileRenderHeight = (float)screenH * currentRenderParams.zoomScaleFactor;
                            float dTileOffsetX = (float)(dTileX - currentRenderParams.centralTileX) * dTileRenderWidth;
                            float dTileOffsetY = (float)(currentRenderParams.centralTileY_TMS - dTileY_TMS) * dTileRenderHeight;
                            float dScaleX = dTileRenderWidth / currentRenderParams.layerExtent;
                            float dScaleY = dTileRenderHeight / currentRenderParams.layerExtent;
                            float dScreenX = (float)dMvtX * dScaleX + dTileOffsetX - currentRenderParams.displayOffsetX;
                            float dScreenY = (float)dMvtY * dScaleY + dTileOffsetY - currentRenderParams.displayOffsetY;
                            
                            // Apply rotation
                            float destCosTheta = cos(radians(currentRenderParams.mapRotationDegrees));
                            float destSinTheta = sin(radians(currentRenderParams.mapRotationDegrees));
                            float dTransX = dScreenX - (screenW / 2);
                            float dTransY = dScreenY - currentRenderParams.pivotY;
                            float dRotX = dTransX * destCosTheta - dTransY * destSinTheta;
                            float dRotY = dTransX * destSinTheta + dTransY * destCosTheta;
                            int destX = round(dRotX + (screenW / 2));
                            int destY = round(dRotY + currentRenderParams.pivotY);
                            
                            // Clamp to screen edges if off-screen (floating marker effect)
                            const int MARKER_MARGIN = 10;
                            int drawX = destX;
                            int drawY = destY;
                            
                            // Clamp X coordinate to screen bounds
                            if (drawX < MARKER_MARGIN) drawX = MARKER_MARGIN;
                            if (drawX > screenW - MARKER_MARGIN) drawX = screenW - MARKER_MARGIN;
                            
                            // Clamp Y coordinate to screen bounds
                            if (drawY < STATUS_BAR_HEIGHT + MARKER_MARGIN) drawY = STATUS_BAR_HEIGHT + MARKER_MARGIN;
                            if (drawY > screenH - MARKER_MARGIN) drawY = screenH - MARKER_MARGIN;
                            
                            // Color based on progress: Blue if it's the current target, Orange if future
                            // Destination is current target if currentWaypointIndex >= all intermediate waypoints
                            size_t numIntermediateWaypoints = waypointBuffer.size() - 1; // Exclude destination itself
                            bool isCurrentTarget = (currentWaypointIndex >= numIntermediateWaypoints);
                            uint16_t destColor = isCurrentTarget ? 0x059A : WAYPOINT_MARKER_COLOR; // Blue for current, Orange for future
                            
                            // Draw destination marker (Circle, not triangle)
                            drawTriangleAndCircle(drawX, drawY, 0, destColor, false, true);
                        }
                        
                xSemaphoreGive(routeMutex);
            }

            // Draw the navigation arrow, using the calculated base center Y
            drawNavigationArrow(screenW / 2, arrowBaseCenterY, arrowSize, NAVIGATION_ARROW_COLOR);

            int statusBarY = 0; // Set status bar to the top
            for (int y = statusBarY; y < STATUS_BAR_HEIGHT; ++y) { // Loop for the height of the status bar
                for (int x = 0; x < screenW; ++x) {
                    uint16_t currentPixelColor = sprite.readPixel(x, y);
                    uint16_t blendedColor = blendColors(currentPixelColor, STATUS_BAR_COLOR, STATUS_BAR_ALPHA);
                    sprite.drawPixel(x, y, blendedColor);
                }
            }
            


            // Draw GPS icon on the top left of the status bar
            // Center X for GPS icon: half its width + a small margin from left edge
            int gpsIconCenterX = (GPS_16_WIDTH / 2) + 2; // 2 pixels margin from left
            // Center Y for GPS icon: half status bar height
            int gpsIconCenterY = statusBarY + (STATUS_BAR_HEIGHT / 2);
            
            // Check for phone GPS timeout (10 seconds without any commands) to prevent flickering
            if (phoneGpsActive && (millis() - lastPhoneCommandTime > 10000)) {
                phoneGpsActive = false; // Mark as inactive after 10s timeout
            }
            
            // GPS Icon Color Logic:
            // - Red: No GPS module detected OR phone GPS timed out
            // - Blue: Phone GPS active (receiving location from phone)
            // - Green: GPS module has fix
            uint16_t gpsIconColor = TFT_RED; // Default: No module
            if (gpsModulePresent && gpsHasFix) {
                gpsIconColor = TFT_GREEN; // GPS module has fix
            } else if (phoneGpsActive) {
                gpsIconColor = TFT_BLUE; // Phone GPS active
            }
            drawIcon(IconType::GPS, gpsIconCenterX, gpsIconCenterY, gpsIconColor);
            
            // Draw GPS rate next to the icon (only when phone GPS active)
            // Decay GPS Rate if idle
            if (xSemaphoreTake(gpsMutex, 0) == pdTRUE) {
                 if (millis() - gpsState.timestamp > 1500) {
                      gpsRate = 0.0f;
                 }
                 xSemaphoreGive(gpsMutex);
            }

            if (phoneGpsActive && gpsRate > 0.1f) {
                sprite.setTextSize(1);
                sprite.setTextColor(TFT_WHITE, TFT_TRANSPARENT);
                sprite.setTextDatum(ML_DATUM); // Middle-left
                char rateStr[8];
                snprintf(rateStr, sizeof(rateStr), "%.0f/s", gpsRate);
                sprite.drawString(rateStr, gpsIconCenterX + GPS_16_WIDTH/2 + 2, gpsIconCenterY);
            }

            // Draw Connected icon on the top right of the status bar
            // Center X for Connected icon: screen width - half its width - a small margin from right edge
            int connectedIconCenterX = screenW - (CONNECTED_16_WIDTH / 2) - 2; // 2 pixels margin from right
            // Center Y for Connected icon: half status bar height
            int connectedIconCenterY = statusBarY + (STATUS_BAR_HEIGHT / 2);
            
            // Determine Color and Visibility based on bleIconMode
            // 0: Blink Red (Window Open, Not Connected)
            // 1: Solid Red (Disconnected or Connected but Unauth)
            // 2: Solid Blue (Authenticated)
            uint16_t connectedIconColor = TFT_RED; // Default Red
            bool shouldDrawConnectedIcon = true;
            
            if (currentControlParams.bleIconMode == 0) {
                connectedIconColor = TFT_RED;
                shouldDrawConnectedIcon = iconBlinkVisible;
            } else if (currentControlParams.bleIconMode == 2) {
                connectedIconColor = TFT_BLUE;
            } else {
                // Mode 1: Solid Red
                connectedIconColor = TFT_RED;
            }

            if (shouldDrawConnectedIcon) {
                drawIcon(IconType::Connected, connectedIconCenterX, connectedIconCenterY, connectedIconColor); 
            }



            // PIN Overlay - Drawn on top of everything if enabled
            if (currentControlParams.showPIN) {
                // Improved PIN Overlay UI
                int boxW = screenW - 20;
                int boxH = 90;
                int boxX = 10;
                int boxY = 40;

                // Semi-transparent background (simulated by solid color on sprite)
                sprite.fillRoundRect(boxX, boxY, boxW, boxH, 8, TFT_BLACK); 
                sprite.drawRoundRect(boxX, boxY, boxW, boxH, 8, TFT_CYAN);
                
                // Header "PAIR DEVICE"
                sprite.setTextSize(1);
                sprite.setTextColor(TFT_CYAN, TFT_BLACK); 
                int headerWidth = sprite.textWidth("PAIRING REQUEST"); // Changed text to be more descriptive but smaller font
                sprite.setCursor((screenW - headerWidth) / 2, boxY + 12);
                sprite.print("PAIRING REQUEST");
                
                // PIN Code (Large & Central)
                sprite.setTextSize(4);
                sprite.setTextColor(TFT_WHITE, TFT_BLACK);
                int pinWidth = sprite.textWidth(currentControlParams.pinCode);
                sprite.setCursor((screenW - pinWidth) / 2, boxY + 35);
                sprite.print(currentControlParams.pinCode);
                
                // Device Name (Small footer)
                sprite.setTextSize(1);
                sprite.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
                int nameWidth = sprite.textWidth(currentControlParams.deviceName);
                sprite.setCursor((screenW - nameWidth) / 2, boxY + 75);
                sprite.print(currentControlParams.deviceName);

                // Reset text size for other elements
                sprite.setTextSize(1); 

                // Debug print (throttle to not spam)
                static unsigned long lastPinLog = 0;
                if (millis() - lastPinLog > 2000) {
                   // Log less frequently
                   // Serial.printf("Render: PIN Overlay Active. PIN: %s\n", currentControlParams.pinCode);
                   lastPinLog = millis();
                }
            }

            // ROUTE SYNC OVERLAY
            if (isRouteSyncing) {
                int textY = 5; // Even higher, practically at the top

                // No Box, just text
                // sprite.fillRoundRect... (Removed)
                // sprite.drawRoundRect... (Removed)

                sprite.setTextColor(TFT_WHITE, TFT_BLACK); // High contrast text
                sprite.setTextDatum(MC_DATUM); // Middle-Center
                sprite.setTextSize(1); 
                
                if (routeTotalBytes > 0) {
                     int percent = (routeSyncProgressBytes * 100) / routeTotalBytes;
                     if (percent > 100) percent = 100;
                     if (percent < 0) percent = 0;
                     
                     char msg[32];
                     snprintf(msg, sizeof(msg), "Syncing Routes %d%%", percent);
                     sprite.drawString(msg, screenW / 2, textY + 10);
                } else {
                     sprite.drawString("Syncing Routes...", screenW / 2, textY + 10);
                }
                
                // Reset Text Datum
                sprite.setTextDatum(TL_DATUM); 
            }



            sprite.pushSprite(0, 0); // Only push if update is needed

            // lastSentRotationAngle = internalCurrentRotationAngle; // REMOVED: Managed by Deadband Logic now
            lastSentZoomFactor = currentControlParams.zoomFactor;
        }

        vTaskDelay(pdMS_TO_TICKS(1)); // Always yield
    }
}
