// main.cpp
#include <Arduino.h>
#include <esp_task_wdt.h>
#include <TinyGPSPlus.h>
#include "common.h"     // Include common definitions
#include "data_task.h"  // Include data task declarations
#include "render_task.h" // Include render task declarations
#include "mvt_parser.h" // Include mvt_parser for parseMVTForTile
#include "map_renderer.h" // Include map_renderer for latlonToTile, latLonToMVTCoords, blendColors
#include "ble_handler.h" // Include BLE handler
#include "boot_screen.h" // Include boot screen for startup animation
#include <Preferences.h>
#include <nvs_flash.h>
#include <esp_flash.h>  // For flash size detection
#include "colors.h"

// Flash size fallback if not set by build env
#ifndef FLASH_SIZE_MB
#define FLASH_SIZE_MB 4
#endif

// =========================================================
// GPS CONFIGURATION (NEO-6M on UART0 - Programming Pins)
// =========================================================
// GPIO3 (RX0) - Connect to NEO-6M TX
// GPIO1 (TX0) - Connect to NEO-6M RX  
#ifndef GPS_BAUD
#define GPS_BAUD 115200  // Target baud rate (matches monitor_speed)
#endif
#ifndef GPS_DEFAULT_BAUD
#define GPS_DEFAULT_BAUD 9600  // NEO-6M factory default - used briefly to send baud-change command
#endif

// Using Serial (UART0) shared with USB programming
TinyGPSPlus gps;

// GPS state
bool gpsInitialized = false;
unsigned long lastGPSCheck = 0;
const unsigned long GPS_CHECK_INTERVAL = 100; // Check GPS every 100ms
unsigned long lastValidGPSTime = 0;
const unsigned long GPS_TIMEOUT = 2000; // Consider GPS lost if no data for 2 seconds

// =========================================================
// TFT DISPLAY AND SPRITE SETUP (Definitions from common.h externs)
// =========================================================
TFT_eSPI tft = TFT_eSPI();              // TFT display object
TFT_eSprite sprite = TFT_eSprite(&tft); // DMA sprite for smooth rendering

// These define the logical screen dimensions AFTER rotation
int screenH = 160;
int screenW = 128;

// Actual zoom level of the tile fetched from MBTiles
int currentTileZ = 16;

// =========================================================
// BACKLIGHT CONTROL
// =========================================================
#ifndef TFT_BL
#define TFT_BL 5 // Backlight Pin
#endif
#ifndef BACKLIGHT_PWM_FREQ
#define BACKLIGHT_PWM_FREQ 5000
#endif
#ifndef BACKLIGHT_PWM_CHAN
#define BACKLIGHT_PWM_CHAN 0
#endif
#ifndef BACKLIGHT_PWM_RES
#define BACKLIGHT_PWM_RES 8
#endif

int currentBrightness = 100;

void setBrightness(int percent) {
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    currentBrightness = percent;
    
    // Map 0-100 to 0-255
    // Map 0-100 to 255-0 (Active Low Backlight)
    int duty = map(percent, 0, 100, 255, 0);
    
    // Write PWM
    ledcWrite(BACKLIGHT_PWM_CHAN, duty);
}

// =========================================================
// STATUS INDICATORS
// =========================================================
bool bleConnected = false;
bool gpsHasFix = false;
bool gpsModulePresent = false;
bool phoneGpsActive = false;
unsigned long lastPhoneCommandTime = 0;
bool bootComplete = false; // Boot screen flag
int tilesLoadedCount = 0;  // Tile loading counter

// GPS Interpolation State (Dead Reckoning)
GPSState gpsState = {0.0, 0.0, 0.0f, 0.0f, 0};
SemaphoreHandle_t gpsMutex;
float gpsRate = 0.0f;  // GPS commands per second

// Route Overlay (Using PSRAM)
RouteAnchor activeRouteAnchor = {0.0, 0.0};

// Smart Startup State
StartupState currentStartupState = STARTUP_BOOT_ANIMATION;

std::vector<RouteNode, PSRAMAllocator<RouteNode>> activeRoute;
bool routeAvailable = false;
SemaphoreHandle_t routeMutex;
bool isRouteSyncing = false; // Flag for route transfer overlay
int routeSyncProgressBytes = 0; // Bytes received for route transfer
int routeTotalBytes = 0; // Total bytes expected for route transfer
int routeProgressIndex = 0; // Trigger for route trimming
int routeLegSwitchIndex = ROUTE_LEG_SWITCH_DEFAULT; // Trigger for route coloring (Default: Infinite/One Leg)
int currentWaypointIndex = 0; // Active waypoint (0-based, 0 = first intermediate stop or destination)
double currentRouteLat = 0.0; // Absolute Latitude of route[routeProgressIndex]
double currentRouteLon = 0.0; // Absolute Longitude of route[routeProgressIndex]
float routeProgressFrac = 0.0f; // 0.0–1.0: fraction along segment [routeProgressIndex → routeProgressIndex+1]

// Alternate Routes (for heading-based auto-selection)
// Note: alternateDecoderStates is local to ble_handler.cpp
std::vector<RouteNode, PSRAMAllocator<RouteNode>> alternateRoutes[2];
RouteAnchor alternateAnchors[2] = {{0.0, 0.0}, {0.0, 0.0}}; // Anchor points for alternates
int selectedRouteIndex = -1; // -1=main, 0=alt0, 1=alt1

// Spatial Index: Map from TileKey to RouteSegments
std::map<TileKey, std::vector<RouteSegment, PSRAMAllocator<RouteSegment>>, std::less<TileKey>,
                PSRAMAllocator<std::pair<const TileKey, std::vector<RouteSegment, PSRAMAllocator<RouteSegment>>>>> routeTileIndex;
int lastIndexedZoom = -1;
size_t lastIndexedRouteSize = 0;
double lastIndexedLat = 0.0;
double lastIndexedLon = 0.0;



// Route state tracking
RouteState currentRouteState = ROUTE_NONE;
int routePointCount = 0;
uint32_t routeHash = 0;

// Route Chunk Queue (For async BLE processing)
QueueHandle_t routeChunkQueue = nullptr;

// Process queued route chunks - call from main loop
void processRouteChunks() {
    if (!routeChunkQueue) return;

    
    RouteChunk chunk;
    bool chunksProcessed = false;

    // Check if queue has items to avoid unnecessary mutex locking
    if (uxQueueMessagesWaiting(routeChunkQueue) > 0) {
        if (xSemaphoreTake(routeMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            int effectiveLimit = (ESP.getPsramSize() > 0) ? MAX_ROUTE_POINTS : 1000;
            
            // Process ALL available chunks in the queue
            while (xQueueReceive(routeChunkQueue, &chunk, 0) == pdTRUE) {
                // Add each delta pair to activeRoute
                for (int i = 0; i < chunk.count && activeRoute.size() < effectiveLimit; i++) {
                    int32_t dLat = chunk.deltas[i * 2];
                    int32_t dLon = chunk.deltas[i * 2 + 1];
                    activeRoute.push_back({dLat, dLon});
                }
                chunksProcessed = true;
            }
            
            // Mark route as partial so it renders
            if (currentRouteState == ROUTE_NONE && activeRoute.size() > 0) {
                 currentRouteState = ROUTE_PARTIAL;
                 routeAvailable = true;
            }
            
            // Allow status queries to see progress
            routePointCount = activeRoute.size();
            
            xSemaphoreGive(routeMutex);
        }
    }

    
    // Update index incrementally outside the critical section (it acquires mutex internally)
    if (chunksProcessed) {
        indexRouteIncremental();
    }
}

// Turn Direction
TurnType currentTurnType = TurnType::NONE;

// OTA State
OTAState globalOTAState = {false, "", 0};

// =========================================================
// GLOBAL SHARED DATA AND SYNCHRONIZATION OBJECTS (Definitions from common.h externs)
// =========================================================
std::map<TileKey, std::vector<ParsedLayer, PSRAMAllocator<ParsedLayer>>, std::less<TileKey>,
                PSRAMAllocator<std::pair<const TileKey, std::vector<ParsedLayer, PSRAMAllocator<ParsedLayer>>>>> loadedTilesData;
SemaphoreHandle_t loadedTilesDataMutex;
int currentLayerExtent = 4096; // Default MVT tile extent, will be updated from parsed data

// Queues for inter-task communication
QueueHandle_t controlParamsQueue;        // Loop -> RenderTask (For user input)
QueueHandle_t tileRequestQueue;          // RenderTask -> DataTask (New: for requesting tiles)
QueueHandle_t tileParsedNotificationQueue; // DataTask -> RenderTask (New: for notifying when tile is parsed)

// DMA-capable buffer for SD card operations (declared extern here, defined in main.cpp)
uint8_t *sd_dma_buffer = nullptr; // Initialize to nullptr
size_t SD_DMA_BUFFER_SIZE = 0;

// =========================================================
// GLOBAL ICON COLOR MAP (Defined here, declared extern in common.h)
// =========================================================
// Initialize iconColorsMap with PSRAMAllocator
std::map<PSRAMString, uint16_t, std::less<PSRAMString>,
         PSRAMAllocator<std::pair<const PSRAMString, uint16_t>>> iconColorsMap(PSRAMAllocator<std::pair<const PSRAMString, uint16_t>>{});

// =========================================================
// CONTROL PARAMETERS (Needed by BLE callbacks)
// =========================================================
// Global variable to track last sent control parameters
ControlParams lastSentControlParams = {
    .targetLat = 0.0,  // No hardcoded position - waits for first GPS packet
    .targetLon = 0.0,
    .zoomFactor = 1.0,
    .cullingBufferPercentageLeft = DEFAULT_CULLING_BUFFER_PERCENTAGE_LEFT,
    .cullingBufferPercentageRight = DEFAULT_CULLING_BUFFER_PERCENTAGE_RIGHT,
    .cullingBufferPercentageTop = DEFAULT_CULLING_BUFFER_PERCENTAGE_TOP,
    .cullingBufferPercentageBottom = DEFAULT_CULLING_BUFFER_PERCENTAGE_BOTTOM,
    .bleIconMode = 1, // Default Solid Red
    .showPIN = false,
    .pinCode = {0},
    .deviceName = {0}
};

// Task Handles for OTA cleanup
TaskHandle_t dataTaskHandle = NULL;
TaskHandle_t renderTaskHandle = NULL;
TaskHandle_t bleLoopTaskHandle = NULL;
TaskHandle_t backgroundTaskHandle = NULL;
void BLELoopTask(void *pvParameters);

// =========================================================
// BLE CALLBACK FUNCTIONS
// =========================================================

// Callback when coordinates are received via BLE
void onBLECoordinatesReceived(double lat, double lon) {
    Serial.printf("BLE Callback: Received coordinates - Lat: %.6f, Lon: %.6f\n", lat, lon);
    
    ControlParams newControlParams = lastSentControlParams;
    newControlParams.targetLat = lat;
    newControlParams.targetLon = lon;
    // newControlParams.zoomFactor = 1.0; // REMOVED: Keep existing zoom for auto-zoom feature
    
    // Use 0 timeout to avoid blocking the BLE task (Core 0) if the render task (Core 1) is busy.
    // Dropping a coordinate is better than starving the BLE watchdog/ACK system.
    if (xQueueSend(controlParamsQueue, &newControlParams, 0) == pdPASS) {
        lastSentControlParams = newControlParams;
        Serial.println("BLE: Coordinates sent to render task");
    } else {
        Serial.println("BLE: Queue FULL - Dropping coordinates to prevent Core 0 starvation");
        bleHandler.notify("COORDS_DROPPED"); // Optional: alert phone
    }
}

// Callback when zoom level is received via BLE
void onBLEZoomReceived(float zoom) {
    Serial.printf("BLE Callback: Received zoom level: %.2f\n", zoom);
    
    ControlParams newControlParams = lastSentControlParams;
    newControlParams.zoomFactor = zoom;
    
    if (xQueueSend(controlParamsQueue, &newControlParams, pdMS_TO_TICKS(100)) == pdPASS) {
        lastSentControlParams = newControlParams;
        Serial.println("BLE: Zoom level sent to render task");
        bleHandler.notify("ZOOM_OK");
    } else {
        Serial.println("BLE: Failed to send zoom to queue");
        bleHandler.notify("ZOOM_FAILED");
    }
}

// Callback when OTA firmware request is received via BLE
void onBLEOTAFirmware(String url) {
    Serial.printf("BLE Callback: OTA Firmware request: %s\n", url.c_str());
    // Actual OTA handling is done in ble_handler.cpp
}

// Callback when OTA map request is received via BLE
void onBLEOTAMap(String url) {
    Serial.printf("BLE Callback: OTA Map request: %s\n", url.c_str());
    // Actual download handling is done in ble_handler.cpp
}

// Forward declarations
void updateStatusIcons();

// =========================================================
// GPS INITIALIZATION AND PROCESSING
// =========================================================
void initGPS() {
#ifndef DISABLE_GPS
    Serial.println("GPS: Initializing NEO-6M on UART0 (programming pins)...");

    // NEO-6M defaults to 9600 baud. Reconfigure it to GPS_BAUD (115200) so both
    // the GPS module and this UART run at the same speed as the monitor.
    // Step 1: Drop to 9600 briefly just to send the UBX baud-change command.
    Serial.begin(GPS_DEFAULT_BAUD);
    delay(100);

    // UBX-CFG-PRT: set UART1 on NEO-6M to 115200 baud, 8N1, NMEA+UBX in/out.
    // Checksum bytes (0xC0, 0x7E) are pre-calculated for this exact payload.
    static const uint8_t ubxSetBaud[] = {
        0xB5, 0x62,              // UBX sync chars
        0x06, 0x00,              // Class: CFG, ID: PRT
        0x14, 0x00,              // Payload length: 20 bytes
        0x01,                    // PortID: UART1
        0x00,                    // Reserved
        0x00, 0x00,              // txReady (disabled)
        0xD0, 0x08, 0x00, 0x00,  // mode: 8N1
        0x00, 0xC2, 0x01, 0x00,  // baudRate: 115200 (little-endian 0x0001C200)
        0x07, 0x00,              // inProtoMask: UBX + NMEA + RTCM
        0x03, 0x00,              // outProtoMask: UBX + NMEA
        0x00, 0x00,              // flags
        0x00, 0x00,              // reserved
        0xC0, 0x7E               // checksum
    };
    Serial.write(ubxSetBaud, sizeof(ubxSetBaud));
    Serial.flush();
    delay(100);

    // Step 2: Switch our side to 115200 to match the newly configured GPS module.
    Serial.begin(GPS_BAUD);
    delay(200);
    // Flush any stale bytes that arrived during the baud transition
    while (Serial.available()) Serial.read();

    Serial.println("GPS: UART0 running at 115200 (GPS + debug shared)");

    // Check if we're receiving valid-looking NMEA data at the new baud rate
    unsigned long startTime = millis();
    bool dataReceived = false;
    int validChars = 0;

    while (millis() - startTime < 3000) {  // Wait up to 3 seconds
        if (Serial.available()) {
            char c = Serial.read();
            if (c == '$') {
                validChars = 1; // Start of sentence
            } else if (validChars == 1 && (c == 'G' || c == 'P')) {
                validChars = 2;
            } else if (validChars == 2 && (c == 'N' || c == 'P' || c == 'L')) {
                validChars = 3;
            } else if (validChars == 3 && (c == 'G' || c == 'R' || c == 'V')) {
                dataReceived = true; // Saw a reasonably complete NMEA prefix like $GPG, $GNG, $GLG, $GPR, etc.
                break;
            } else if (validChars > 0) {
                validChars = 0; // Reset if sequence breaks early
            }
        } else {
            delay(10);
        }
    }

    if (dataReceived) {
        gpsModulePresent = true;
        gpsInitialized = true;
        Serial.println("GPS: NEO-6M module detected at 115200");
    } else {
        gpsModulePresent = false;
        Serial.println("GPS: NEO-6M module not detected (no NMEA at 115200)");
    }
#else
    gpsModulePresent = false;
    Serial.println("GPS: Disabled via build flag (DISABLE_GPS)");
#endif

    updateStatusIcons();
}

void processGPS() {
#ifndef DISABLE_GPS
    if (!gpsInitialized) return;
    
    // Read GPS data from UART0
    while (Serial.available() > 0) {
        char c = Serial.read();
        gps.encode(c);
    }
    
    // Update GPS fix status
    bool previousFix = gpsHasFix;
    
    if (gps.location.isValid() && gps.location.age() < 2000) {
        gpsHasFix = true;
        lastValidGPSTime = millis();
        
        // First fix or fix regained
        if (!previousFix) {
            Serial.println("GPS: Fix acquired");
            Serial.printf("GPS: Location - Lat: %.6f, Lon: %.6f\n", 
                         gps.location.lat(), gps.location.lng());
            Serial.printf("GPS: Satellites: %d, HDOP: %.2f\n", 
                         gps.satellites.value(), gps.hdop.hdop());
            updateStatusIcons();
            
            // Auto-center map on first GPS fix
            ControlParams newParams = lastSentControlParams;
            newParams.targetLat = gps.location.lat();
            newParams.targetLon = gps.location.lng();
            
            if (xQueueSend(controlParamsQueue, &newParams, pdMS_TO_TICKS(100)) == pdPASS) {
                lastSentControlParams = newParams;
                Serial.println("GPS: Map centered on GPS location");
            }
        }
    } else {
        // Check for GPS timeout
        if (millis() - lastValidGPSTime > GPS_TIMEOUT) {
            if (gpsHasFix) {
                gpsHasFix = false;
                Serial.println("GPS: Fix lost");
                updateStatusIcons();
            }
        }
    }
#endif
}

void printGPSInfo() {
    if (!gpsInitialized || !gpsHasFix) return;
    
    Serial.println("\n--- GPS Status ---");
    Serial.printf("Location: %.6f, %.6f\n", gps.location.lat(), gps.location.lng());
    Serial.printf("Altitude: %.2f m\n", gps.altitude.meters());
    Serial.printf("Speed: %.2f km/h\n", gps.speed.kmph());
    Serial.printf("Course: %.2f°\n", gps.course.deg());
    Serial.printf("Satellites: %d\n", gps.satellites.value());
    Serial.printf("HDOP: %.2f\n", gps.hdop.hdop());
    
    if (gps.date.isValid()) {
        Serial.printf("Date: %04d-%02d-%02d\n", 
                     gps.date.year(), gps.date.month(), gps.date.day());
    }
    if (gps.time.isValid()) {
        Serial.printf("Time: %02d:%02d:%02d UTC\n", 
                     gps.time.hour(), gps.time.minute(), gps.time.second());
    }
    Serial.println("------------------\n");
}

// =========================================================
// STATUS ICON DRAWING FUNCTIONS
// =========================================================
void drawBLEIcon(bool connected, bool visible = true) {
    // Icons disabled by user request
}

void drawGPSIcon(bool hasFix, bool modulePresent) {
    // Icons disabled by user request
}

void updateStatusIcons() {
    // Icons disabled by user request
}

// PIN display moved to Render Task


// BLE connection callback - show PIN when device connects
void onBLEDeviceConnected() {
    Serial.println("BLE Callback: Device connected - showing PIN");
    // status stays RED until authenticated
    bleConnected = false; 
    updateStatusIcons();
    
    // Update Params to Show PIN
    ControlParams newParams = lastSentControlParams;
    newParams.showPIN = true;
    
    // Copy Strings safely
    String pin = bleHandler.getPIN();
    String name = bleHandler.getDeviceName();
    strncpy(newParams.pinCode, pin.c_str(), sizeof(newParams.pinCode) - 1);
    newParams.pinCode[sizeof(newParams.pinCode) - 1] = '\0';
    strncpy(newParams.deviceName, name.c_str(), sizeof(newParams.deviceName) - 1);
    newParams.deviceName[sizeof(newParams.deviceName) - 1] = '\0';
    
    // Critical UI update: Wait up to 200ms for queue space
    if (xQueueSend(controlParamsQueue, &newParams, pdMS_TO_TICKS(200)) == pdPASS) {
        lastSentControlParams = newParams;
        Serial.println("BLE: Sent PIN Display Command to Queue");
    } else {
        Serial.println("BLE ERROR: Failed to send PIN Display Command (Queue Full)");
    }
}

// BLE authentication callback - clear PIN when authenticated
void onBLEDeviceAuthenticated() {
    Serial.println("BLE Callback: Device authenticated - clearing PIN");
    bleConnected = true; // NOW we are "connected"
    updateStatusIcons(); // Turn icon BLUE
    
    // Switch to Map View immediately upon authentication
    currentStartupState = STARTUP_MAPPING;
    
    // Update Params to Hide PIN
    ControlParams newParams = lastSentControlParams;
    newParams.showPIN = false;
    
    if (xQueueSend(controlParamsQueue, &newParams, pdMS_TO_TICKS(200)) == pdPASS) {
        lastSentControlParams = newParams;
        Serial.println("BLE: Sent PIN Hide Command to Queue");
    } else {
        Serial.println("BLE ERROR: Failed to send PIN Hide Command");
    }
    
    // Request Route Sync from App
    // This allows ESP32 to recover route state after restart/reconnect
    bleHandler.notify("SYNC_REQ");
}

// High-Priority BLE Loop Task (Core 0)
// Handles session watchdog and outgoing ACKs even when App Task (Core 1) is pegged.
// Priority 4 allows it to run between NimBLE events (Priority 5).
void BLELoopTask(void *pvParameters) {
    Serial.println("BLELoopTask: Started on Core 0 (Priority 4)");
    while(true) {
        bleHandler.loop();
        vTaskDelay(pdMS_TO_TICKS(10)); // Yield frequently - BLE doesn't need high frequency
    }
}

// BLE disconnection callback
void onBLEDeviceDisconnected() {
    Serial.println("BLE Callback: Device disconnected");
    bleConnected = false;
    updateStatusIcons();
    
    // Always ensure PIN is hidden on disconnect
    ControlParams newParams = lastSentControlParams;
    newParams.showPIN = false;
    newParams.bleIconMode = 1; // Reset to red

    // Reset Route Sync State
    isRouteSyncing = false;
    
    // Clear any pending route chunks to prevent stale data processing on reconnect
    if (routeChunkQueue != NULL) {
        xQueueReset(routeChunkQueue);
    }

    if (xQueueSend(controlParamsQueue, &newParams, pdMS_TO_TICKS(200)) == pdPASS) {
        lastSentControlParams = newParams;
        Serial.println("BLE: Sent PIN Hide Command (Disconnect)");
    }
}


void prepareForWifiOTA() {
    Serial.println("OTA: Preparing memory for WiFi...");
    Serial.printf("OTA: Free Heap (Before): %u\n", ESP.getFreeHeap());

    if (dataTaskHandle != NULL) {
        Serial.println("OTA: Specific task deletion logic..."); 
        vTaskDelete(dataTaskHandle);
        dataTaskHandle = NULL;
    }
    if (renderTaskHandle != NULL) {
        vTaskDelete(renderTaskHandle);
        renderTaskHandle = NULL;
    }
    
    // Clear heavy PSRAM data
    if (xSemaphoreTake(loadedTilesDataMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
        loadedTilesData.clear();
        xSemaphoreGive(loadedTilesDataMutex);
        Serial.println("OTA: Cleared tile cache");
    }
    
    // Free SD DMA buffer
    if (sd_dma_buffer != nullptr) {
        heap_caps_free(sd_dma_buffer);
        sd_dma_buffer = nullptr;
        Serial.println("OTA: Freed SD DMA buffer");
    }
    
    Serial.printf("OTA: Free Heap (After): %u\n", ESP.getFreeHeap());
    delay(100); // Allow OS to reclaim stack
}

// =========================================================
// BACKGROUND TASK (Core 0)
// =========================================================
void backgroundTask(void *pvParameters) {
    Serial.println("BackgroundTask: Started on Core 0");
    static String inputString = ""; // Buffer for serial input
    
    // Watchdog for this task
    esp_task_wdt_add(NULL);

    while(true) {
        esp_task_wdt_reset();

        // --- GPS Processing ---
        unsigned long current = millis();
        if (current - lastGPSCheck >= GPS_CHECK_INTERVAL) {
            processGPS();
            lastGPSCheck = current;
        }

        // --- Serial Input Processing ---
        while (Serial.available()) {
            char inChar = Serial.read();
            inputString += inChar;
            if (inChar == '\n') {
                Serial.println("BackgroundTask: Newline detected. Processing input.");
                bool paramsUpdated = false;
                ControlParams newControlParams = lastSentControlParams; // Start with last sent params

                // Attempt to parse as Lat,Lon
                double requestedLat = 0.0;
                double requestedLon = 0.0;
                int parsedCount = sscanf(inputString.c_str(), "%lf,%lf", &requestedLon, &requestedLat);

                if (parsedCount == 2) {
                    newControlParams.targetLat = requestedLat;
                    newControlParams.targetLon = requestedLon;
                    newControlParams.zoomFactor = 1.0; // Reset zoom to 1x when new coordinates are entered
                    paramsUpdated = true;
                    Serial.printf("BackgroundTask: Parsed Lat: %.4f, Lon: %.4f\n", requestedLat, requestedLon);
                } else if (inputString.startsWith("gps")) {
                    // GPS info command
                    printGPSInfo();
                } else {
                    // If not Lat,Lon, attempt to parse as zoom factor
                    float requestedZoom = inputString.toFloat();
                    if (requestedZoom >= 1.0f && requestedZoom <= 4.0f) {
                        newControlParams.zoomFactor = requestedZoom;
                        paramsUpdated = true;
                        Serial.printf("BackgroundTask: Parsed Zoom: %.1f\n", requestedZoom);
                    } else {
                        Serial.println("BackgroundTask: Invalid input. Please enter coordinates (Lon,Lat) or a zoom factor (1, 2, 3, or 4).");
                    }
                }

                if (paramsUpdated) {
                    Serial.printf("BackgroundTask: Sending new control parameters - Lat: %.4f, Lon: %.4f, Zoom: %.1f, CullL: %.2f, CullR: %.2f, CullT: %.2f, CullB: %.2f. Queue space: %d\n",
                                    newControlParams.targetLat, newControlParams.targetLon, newControlParams.zoomFactor,
                                    newControlParams.cullingBufferPercentageLeft, newControlParams.cullingBufferPercentageRight,
                                    newControlParams.cullingBufferPercentageTop, newControlParams.cullingBufferPercentageBottom,
                                    uxQueueSpacesAvailable(controlParamsQueue));
                    // Send the updated control parameters to the render task
                    if (xQueueSend(controlParamsQueue, &newControlParams, 0) != pdPASS) { // Use 0 timeout for non-blocking send
                        Serial.println("❌ BackgroundTask: Failed to send control parameters to queue. Queue full? (Non-blocking send failed)");
                    } else {
                        lastSentControlParams = newControlParams; // Update last sent params if successful
                        Serial.println("BackgroundTask: Successfully sent control parameters to queue.");
                    }
                }
                inputString = ""; // Clear the input string
            }
        }
        
        // bleHandler.loop() moved to BLELoopTask (Priority 4)

        vTaskDelay(pdMS_TO_TICKS(10)); // Yield to other tasks on Core 0 (e.g. WiFi/BLE controller)
    }
}

// =========================================================
// ARDUINO SETUP AND LOOP FUNCTIONS
// =========================================================

void setup() {
    Serial.begin(115200); // Initialize serial communication
    while (!Serial && millis() < 5000); // Wait for serial port to connect (for up to 5 seconds)

    // =========================================================
    // INITIALIZE NVS EARLY (must be before ANY Preferences call)
    // =========================================================
    esp_err_t nvsErr = nvs_flash_init();
    if (nvsErr == ESP_ERR_NVS_NO_FREE_PAGES || nvsErr == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        Serial.println("Boot: NVS needs erase, erasing and reinitializing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvsErr = nvs_flash_init();
    }
    if (nvsErr != ESP_OK) {
        Serial.printf("Boot: NVS init failed: %s\n", esp_err_to_name(nvsErr));
    } else {
        Serial.println("Boot: NVS initialized OK");
    }

    // =========================================================
    // DETECT FLASH SIZE AND HARDWARE VARIANT
    // =========================================================
    uint32_t flashSize = 0;
    esp_flash_get_size(NULL, &flashSize);
    uint32_t flashMB = flashSize / (1024 * 1024);
    Serial.printf("Boot: Flash size (runtime): %u MB\n", flashMB);
    uint32_t psramSize = ESP.getPsramSize();
    Serial.printf("Boot: PSRAM size: %u MB\n", psramSize / (1024 * 1024));

#if defined(BOARD_ESP32_WROVER_IPEX_16M)
    Serial.println("Boot: Config: ESP32-WROVER-IPEX 16M 128Mbit (partitions_16mb.csv)");
    if (flashMB < 16) {
        Serial.printf("Boot: ⚠️  WARNING: Built for 16MB but only %u MB flash detected!\n", flashMB);
        Serial.println("Boot:    Use 'esp-wrover-kit' env instead for this board.");
    }
#else
    Serial.printf("Boot: Config: ESP32-WROVER-KIT %uMB (partitions_custom.csv)\n", FLASH_SIZE_MB);
    if (flashMB >= 16) {
        Serial.println("Boot: ℹ️  NOTE: 16MB flash detected but running 4MB config.");
        Serial.println("Boot:    Switch to 'esp32-wrover-16mb' env to utilize full flash.");
    }
#endif

    // =========================================================
    // LOAD THEME FROM PREFERENCES
    // Open read-write so the namespace is created on first boot
    // (read-only open fails with NOT_FOUND on a blank flash)
    // =========================================================
    Preferences prefs;
    prefs.begin("settings", false); // Read-write: creates namespace if absent
    int theme = prefs.getInt("theme", 0); // Default 0 (Dark)
    int savedBrightness = prefs.getInt("brightness", 100);
    prefs.end();
    
    Serial.printf("Boot: Loading Theme %d\n", theme);
    setTheme(theme == 0); // Apply Theme immediately

    // Initialize TFT early for boot screen
    tft.begin();
    tft.setRotation(0); // Portrait mode

    // Setup Backlight PWM
    ledcSetup(BACKLIGHT_PWM_CHAN, BACKLIGHT_PWM_FREQ, BACKLIGHT_PWM_RES);
    ledcAttachPin(TFT_BL, BACKLIGHT_PWM_CHAN);
    setBrightness(savedBrightness); // value already loaded above
    
    // Create boot screen (8 steps total)
    BootScreen bootScreen(tft, screenW, screenH, 8);
    bootScreen.show();
    vTaskDelay(pdMS_TO_TICKS(300)); // Brief pause to show logo
    
    Serial.println("Main Loop: Initializing FreeRTOS objects...");
    bootScreen.updateProgress(1);

    // Create mutex for loadedTilesData
    loadedTilesDataMutex = xSemaphoreCreateMutex();
    if (loadedTilesDataMutex == NULL) {
        Serial.println("❌ Main Loop: Failed to create loadedTilesDataMutex! Halting.");
        while(true) { vTaskDelay(1000); } // Halt on critical error
    }

    // Create mutex for route overlay data
    routeMutex = xSemaphoreCreateMutex();
    if (routeMutex == NULL) {
        Serial.println("❌ Main Loop: Failed to create routeMutex!");
    }

    // Create mutex for GPS interpolation state
    gpsMutex = xSemaphoreCreateMutex();
    if (gpsMutex == NULL) {
        Serial.println("❌ Main Loop: Failed to create gpsMutex!");
    }

    // Create queues using defined constants
    controlParamsQueue = xQueueCreate(CONTROL_PARAMS_QUEUE_SIZE, sizeof(ControlParams));
    tileRequestQueue = xQueueCreate(TILE_REQUEST_QUEUE_SIZE, sizeof(TileKey));
    tileParsedNotificationQueue = xQueueCreate(TILE_PARSED_NOTIFICATION_QUEUE_SIZE, sizeof(bool));

    if (controlParamsQueue == NULL || tileRequestQueue == NULL || tileParsedNotificationQueue == NULL) {
        Serial.println("❌ Main Loop: Failed to create one or more queues! Halting.");
        while(true) { vTaskDelay(1000); } // Halt on critical error
    }

    // Create route chunk queue for async BLE processing
    routeChunkQueue = xQueueCreate(ROUTE_CHUNK_QUEUE_SIZE, sizeof(RouteChunk));
    if (routeChunkQueue == NULL) {
        Serial.println("❌ Main Loop: Failed to create routeChunkQueue!");
    }

    bootScreen.updateProgress(2);
    Serial.println("Main Loop: Initializing SD_MMC...");
    if (!SD_MMC.begin()) {
        Serial.println("❌ Main Loop: SD_MMC Card Mount Failed! Please ensure card is inserted. Halting.");
        while(true) { vTaskDelay(1000); } // Halt on critical error
    }
    Serial.println("Main Loop: SD_MMC Card Mounted.");

    // Allocate read buffer for SD card operations.
    // ESP32 PSRAM is NOT DMA-capable; MALLOC_CAP_DMA|MALLOC_CAP_SPIRAM always
    // returns 0.  Allocate from plain PSRAM first (large), fall back to
    // internal DMA RAM (small but always works).
    size_t largestFreePsramBlock     = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
    size_t largestFreeInternalDmaBlock = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);

    // Try PSRAM first
    SD_DMA_BUFFER_SIZE = std::min(largestFreePsramBlock, MAX_SD_DMA_BUFFER_SIZE_BYTES);

    Serial.printf("Main Loop: Attempting to allocate %u bytes for SD buffer (Max %u KB).\n",
                  SD_DMA_BUFFER_SIZE, MAX_SD_DMA_BUFFER_SIZE_KB);
    Serial.printf("           Free PSRAM: %u bytes. Largest block: %u bytes.\n",
                  heap_caps_get_free_size(MALLOC_CAP_SPIRAM), largestFreePsramBlock);
    Serial.printf("           Free Internal DMA RAM: %u bytes. Largest block: %u bytes.\n",
                  heap_caps_get_free_size(MALLOC_CAP_DMA), largestFreeInternalDmaBlock);

    sd_dma_buffer = (uint8_t *)heap_caps_malloc(SD_DMA_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
    if (sd_dma_buffer == nullptr) {
        // PSRAM unavailable or exhausted – fall back to internal DMA RAM
        Serial.println("⚠️  Main Loop: PSRAM alloc failed. Falling back to internal DMA RAM...");
        SD_DMA_BUFFER_SIZE = std::min(largestFreeInternalDmaBlock, MAX_SD_DMA_BUFFER_SIZE_BYTES);
        sd_dma_buffer = (uint8_t *)heap_caps_malloc(SD_DMA_BUFFER_SIZE, MALLOC_CAP_DMA);
        if (sd_dma_buffer == nullptr) {
            Serial.println("❌ Main Loop: Failed to allocate SD buffer from internal DMA RAM! Halting.");
            while(true) { vTaskDelay(1000); }
        }
        Serial.printf("Main Loop: SD buffer allocated in internal RAM: %u bytes\n", SD_DMA_BUFFER_SIZE);
    } else {
        Serial.printf("Main Loop: SD buffer allocated in PSRAM: %u bytes. Remaining PSRAM: %u bytes\n",
                      SD_DMA_BUFFER_SIZE, heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    }

    bootScreen.updateProgress(3);
    // Initialize GPS module
    initGPS();


    bootScreen.updateProgress(4);
    // Initialize iconColorsMap with specific colors for icon types
    iconColorsMap.emplace(PSRAMString("traffic_signals", PSRAMAllocator<char>()), TFT_CYAN);
    iconColorsMap.emplace(PSRAMString("bus_stop", PSRAMAllocator<char>()), TFT_VIOLET);
    iconColorsMap.emplace(PSRAMString("fuel", PSRAMAllocator<char>()), TFT_MAGENTA);
    
    // Wait for system to stabilize before initializing BLE (prevents brownout)
    bootScreen.updateProgress(5);
    Serial.println("Main Loop: Waiting for system stabilization...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    bootScreen.updateProgress(6);
    // Initialize BLE with security
    Serial.println("Main Loop: Initializing BLE with security...");
    bleHandler.init();  // Auto-generates device name and PIN
    
    // Set BLE callbacks
    bleHandler.onCoordinatesReceived = onBLECoordinatesReceived;
    bleHandler.onZoomReceived = onBLEZoomReceived;
    bleHandler.onOTAFirmwareReceived = onBLEOTAFirmware;
    bleHandler.onOTAMapReceived = onBLEOTAMap;
    bleHandler.onDeviceConnected = onBLEDeviceConnected;         // Show PIN on connect
    bleHandler.onAuthenticated = onBLEDeviceAuthenticated;       // Clear PIN on auth
    bleHandler.onDeviceDisconnected = onBLEDeviceDisconnected;   // Update status on disconnect
    bleHandler.onZoomChange = onBLEZoomReceived;                 // Auto-zoom capability
    
    Serial.println("Main Loop: BLE initialized and ready");
    Serial.printf("Device Name: %s\n", bleHandler.getDeviceName().c_str());
    
    bootScreen.updateProgress(7);
    // Draw initial status icons
    updateStatusIcons();
    
    vTaskDelay(pdMS_TO_TICKS(300));
    bootScreen.updateProgress(8);
    bootScreen.complete();
    
    // Signal that boot is complete
    bootComplete = true;
    
    // NOW create tasks after boot screen is done
    // Create high-priority BLE loop task on Core 0
    xTaskCreatePinnedToCore(BLELoopTask, "BLELoopTask", 4096, NULL, 4, &bleLoopTaskHandle, 0);
    
    Serial.println("Main Loop: Creating tasks...");
    // Create data task (Core 0)
    xTaskCreatePinnedToCore(
        dataTask,
        "DataTask",
        DATA_TASK_STACK_SIZE,
        NULL,
        1,
        &dataTaskHandle,
        0
    );

    // Create background task (Core 0)
    xTaskCreatePinnedToCore(
        backgroundTask,
        "BackgroundTask",
        4096, // Stack size
        NULL,
        1,    // Priority 1
        &backgroundTaskHandle,
        0     // Core 0
    );

    // Create render task (Core 1) - Higher priority for smooth display
    xTaskCreatePinnedToCore(
        renderTask,
        "RenderTask",
        RENDER_TASK_STACK_SIZE,
        NULL,
        3,  // Priority 3 for smoother display
        &renderTaskHandle,
        1
    );
    
    Serial.println("Ready. Enter coordinates to load map (Longitude, Latitude):");
    Serial.println("Or enter zoom factor (1, 2, 3, or 4) to change zoom for current location.");
    Serial.println("Type 'gps' to display GPS information.");
    
    // Enable watchdog timer (5 seconds timeout)
    esp_task_wdt_init(5, true);
    esp_task_wdt_add(NULL);  // Add current task
    Serial.println("Main Loop: Watchdog timer enabled (5s timeout)");
}

// Global variable to store input string from Serial
String inputString = "";

// BLE heartbeat timer - REMOVED per user request
// unsigned long lastHeartbeatTime = 0;
// const unsigned long HEARTBEAT_INTERVAL = 5000; // 5 seconds

void loop() {
  // Reset watchdog timer
  esp_task_wdt_reset();
  
  // bleHandler.loop() moved to BackgroundTask on Core 0
  
  // Process any queued route chunks from BLE
  processRouteChunks();

  // Smart Startup State Machine
  if (bootComplete) {
      if (bleHandler.waitingForToken) {
          // Absolute priority: If we are actively paired and waiting for the PIN, forcibly show the BLE screen
          currentStartupState = STARTUP_WAITING_BLE;
      } else if (gpsHasFix || (bleHandler.isConnected() && bleHandler.isDeviceAuthenticated())) {
          currentStartupState = STARTUP_MAPPING;
      } else {
          // No GPS and Not Connected
          // Check if we are still in initial grace period for GPS (e.g. 5 seconds after boot)
          // We use 8000ms to allow 5-8 seconds for GPS to warm up
          if (millis() < 8000) { 
               currentStartupState = STARTUP_WAITING_GPS;
          } else {
               // Timeout - Check if we have paired devices
               if (bleHandler.getBondedDeviceCount() > 0) {
                   currentStartupState = STARTUP_WAITING_BLE;
               } else {
                   currentStartupState = STARTUP_QR_CODE;
               }
          }
      }
  }
  
    // Send BLE heartbeat every 10 seconds for debugging - REMOVED
  /*
  unsigned long heartbeatTime = millis();
  if (heartbeatTime - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
      // Log memory stats for debugging
      Serial.printf("MEM: Free Heap: %d, Free PSRAM: %d\n", ESP.getFreeHeap(), ESP.getFreePsram()); 
      lastHeartbeatTime = heartbeatTime;
  }
  */

  // Yield control more frequently
  vTaskDelay(pdMS_TO_TICKS(10));
  
  // -- Check and Update BLE Icon Mode --
  int currentBleIconMode = 1; // Default Solid Red
  if (bleHandler.isConnected()) {
      if (bleHandler.isDeviceAuthenticated()) {
          currentBleIconMode = 2; // Solid Blue
      } else {
          currentBleIconMode = 1; // Solid Red (Waiting for Auth)
      }
  } else {
      if (bleHandler.isWithinPairingWindow()) {
          currentBleIconMode = 0; // Blink Red (Pairing Window Open)
      } else {
          currentBleIconMode = 1; // Solid Red (Window Closed)
      }
  }
  
  // If mode changed, send update to Render Task
  if (currentBleIconMode != lastSentControlParams.bleIconMode) {
      ControlParams newParams = lastSentControlParams;
      newParams.bleIconMode = currentBleIconMode;
      
      // Use non-blocking send to avoid stalling loop
      if (xQueueSend(controlParamsQueue, &newParams, 0) == pdPASS) {
          lastSentControlParams = newParams;
          // Serial.printf("Main Loop: Updated BLE Icon Mode to %d\n", currentBleIconMode);
      }
  }
}
