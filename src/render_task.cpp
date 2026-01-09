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
    tft.begin();
    tft.setRotation(0); // Changed to 0 for portrait mode
    tft.fillScreen(MAP_BACKGROUND_COLOR); // Changed background color to #1a2632

    sprite.setColorDepth(16); // Set sprite color depth (RGB565, good balance of speed/quality)
    sprite.createSprite(screenW, screenH); // Create the 160x128 off-screen sprite
    //128*160*16
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
        .cullingBufferPercentageBottom = DEFAULT_CULLING_BUFFER_PERCENTAGE_BOTTOM,
        .bleIconMode = 1,
        .showPIN = false,
        .pinCode = {0},
        .deviceName = {0}
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

        // 2. Read compass data and update rotation
        sensors_event_t event;
        hmc5883l.getEvent(&event);
        
        // Only update heading if magnetic sensor data is valid
        if (!isnan(event.magnetic.x) && !isnan(event.magnetic.y)) {
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
            sprite.fillScreen(MAP_BACKGROUND_COLOR); // Changed background color to #1a2632

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

            // =========================================================
            // DRAW ROUTE OVERLAY (Blue polyline on top of map)
            // =========================================================
            if (routeAvailable && xSemaphoreTake(routeMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                if (activeRoute.size() >= 2) {
                    // Calculate rotation params
                    float cosTheta = cos(radians(currentRenderParams.mapRotationDegrees));
                    float sinTheta = sin(radians(currentRenderParams.mapRotationDegrees));
                    int centerX = screenW / 2;
                    int centerY = currentRenderParams.pivotY;
                    
                    // Previous screen point for drawing line segments
                    int prevScreenX = -1, prevScreenY = -1;
                    
                    for (size_t i = 0; i < activeRoute.size(); i++) {
                        // Convert lat/lon to tile coordinates
                        int tileX, tileY_std, tileY_TMS;
                        latlonToTile(activeRoute[i].lat, activeRoute[i].lon, currentTileZ, tileX, tileY_std, tileY_TMS);
                        
                        // Convert lat/lon to MVT coordinates within its tile
                        int mvtX, mvtY;
                        latLonToMVTCoords(activeRoute[i].lat, activeRoute[i].lon, currentTileZ, tileX, tileY_TMS, mvtX, mvtY, currentRenderParams.layerExtent);
                        
                        // Calculate tile offset relative to central tile
                        float tileRenderWidth = (float)screenW * currentRenderParams.zoomScaleFactor;
                        float tileRenderHeight = (float)screenH * currentRenderParams.zoomScaleFactor;
                        float tileOffsetX = (float)(tileX - currentRenderParams.centralTileX) * tileRenderWidth;
                        float tileOffsetY = (float)(currentRenderParams.centralTileY_TMS - tileY_TMS) * tileRenderHeight;
                        
                        // Calculate screen position
                        float scaleX = (float)screenW * currentRenderParams.zoomScaleFactor / currentRenderParams.layerExtent;
                        float scaleY = (float)screenH * currentRenderParams.zoomScaleFactor / currentRenderParams.layerExtent;
                        
                        float screenX = (float)mvtX * scaleX + tileOffsetX - currentRenderParams.displayOffsetX;
                        float screenY = (float)mvtY * scaleY + tileOffsetY - currentRenderParams.displayOffsetY;
                        
                        // Apply rotation
                        float translatedX = screenX - centerX;
                        float translatedY = screenY - centerY;
                        float rotatedX = translatedX * cosTheta - translatedY * sinTheta;
                        float rotatedY = translatedX * sinTheta + translatedY * cosTheta;
                        int finalX = round(rotatedX + centerX);
                        int finalY = round(rotatedY + centerY);
                        
                        // Apply perspective if zoomed in
                        if (currentRenderParams.zoomScaleFactor >= 3.0f) {
                            float perspX, perspY;
                            applyPerspective((float)finalX, (float)finalY, perspX, perspY, currentRenderParams.pivotY);
                            finalX = round(perspX);
                            finalY = round(perspY);
                        }
                        
                        // Draw line segment from previous point
                        if (i > 0 && prevScreenX >= -100 && prevScreenX <= screenW + 100 &&
                            prevScreenY >= -100 && prevScreenY <= screenH + 100) {
                            // Draw bright white line for route overlay (width = 3 pixels)
                            for (int offset = -1; offset <= 1; offset++) {
                                drawAntiAliasedLine(prevScreenX + offset, prevScreenY, finalX + offset, finalY, TFT_WHITE);
                            }
                        }
                        
                        prevScreenX = finalX;
                        prevScreenY = finalY;
                    }
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
            
            // Check for phone GPS timeout (1 second without any commands = inactive)
            if (phoneGpsActive && (millis() - lastPhoneCommandTime > 1000)) {
                phoneGpsActive = false; // Mark as inactive after 1s timeout
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

            sprite.pushSprite(0, 0); // Only push if update is needed

            lastSentRotationAngle = internalCurrentRotationAngle;
            lastSentZoomFactor = currentControlParams.zoomFactor;
        }

        vTaskDelay(pdMS_TO_TICKS(1)); // Always yield
    }
}
