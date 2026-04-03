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

// Hardware name fallback if not set by build env
#ifndef DEVICE_HW_NAME
#define DEVICE_HW_NAME "ESP32-GENERIC"
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

#ifdef USE_GPS_UART1
#ifndef GPS_RX_PIN
#define GPS_RX_PIN 47
#endif
#ifndef GPS_TX_PIN
#define GPS_TX_PIN 48
#endif
HardwareSerial GPSSerial(1);

static void beginGpsPort(uint32_t baud) {
    GPSSerial.begin(baud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

static void flushGpsRx() {
    while (GPSSerial.available()) GPSSerial.read();
}
#else
static void beginGpsPort(uint32_t baud) {
    Serial.begin(baud);
}

static void flushGpsRx() {
    while (Serial.available()) Serial.read();
}
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
#ifndef BACKLIGHT_ACTIVE_LOW
#define BACKLIGHT_ACTIVE_LOW 1
#endif
#ifndef BACKLIGHT_SWEEP_TEST
#define BACKLIGHT_SWEEP_TEST 0
#endif
#ifndef BACKLIGHT_SWEEP_STEP_MS
#define BACKLIGHT_SWEEP_STEP_MS 30
#endif
#ifndef AUTO_BRIGHTNESS_DEFAULT
#define AUTO_BRIGHTNESS_DEFAULT 1
#endif
#ifndef AUTO_BRIGHTNESS_SAMPLE_MS
#define AUTO_BRIGHTNESS_SAMPLE_MS 120
#endif
#ifndef AUTO_BRIGHTNESS_MIN_PERCENT
#define AUTO_BRIGHTNESS_MIN_PERCENT 10
#endif
#ifndef AUTO_BRIGHTNESS_MAX_PERCENT
#define AUTO_BRIGHTNESS_MAX_PERCENT 90
#endif
#ifndef AUTO_BRIGHTNESS_ADC_DARK
#define AUTO_BRIGHTNESS_ADC_DARK 500
#endif
#ifndef AUTO_BRIGHTNESS_ADC_BRIGHT
#define AUTO_BRIGHTNESS_ADC_BRIGHT 3200
#endif
#ifndef AUTO_BRIGHTNESS_LDR_INVERT
#define AUTO_BRIGHTNESS_LDR_INVERT 0
#endif
#ifndef AUTO_BRIGHTNESS_ADC_PIN
#ifdef ALADC_PIN
#define AUTO_BRIGHTNESS_ADC_PIN ALADC_PIN
#else
#define AUTO_BRIGHTNESS_ADC_PIN 20
#endif
#endif
#ifndef AUTO_BRIGHTNESS_SMOOTHING_NUM
#define AUTO_BRIGHTNESS_SMOOTHING_NUM 1
#endif
#ifndef AUTO_BRIGHTNESS_SMOOTHING_DEN
#define AUTO_BRIGHTNESS_SMOOTHING_DEN 2
#endif
#ifndef AUTO_BRIGHTNESS_WINDOW_MS
#define AUTO_BRIGHTNESS_WINDOW_MS 300000UL
#endif
#ifndef AUTO_BRIGHTNESS_WINDOW_SAMPLE_MS
#define AUTO_BRIGHTNESS_WINDOW_SAMPLE_MS 500UL
#endif
#ifndef AUTO_BRIGHTNESS_MIN_ADC_SPAN
#define AUTO_BRIGHTNESS_MIN_ADC_SPAN 200
#endif
#ifndef AUTO_BRIGHTNESS_BOUND_SMOOTH_NUM
#define AUTO_BRIGHTNESS_BOUND_SMOOTH_NUM 7
#endif
#ifndef AUTO_BRIGHTNESS_BOUND_SMOOTH_DEN
#define AUTO_BRIGHTNESS_BOUND_SMOOTH_DEN 8
#endif
#ifndef AUTO_BRIGHTNESS_DEADBAND_PERCENT
#define AUTO_BRIGHTNESS_DEADBAND_PERCENT 2
#endif
#ifndef AUTO_BRIGHTNESS_MAX_STEP_PERCENT
#define AUTO_BRIGHTNESS_MAX_STEP_PERCENT 2
#endif
#define AUTO_BRIGHTNESS_WINDOW_SAMPLES (AUTO_BRIGHTNESS_WINDOW_MS / AUTO_BRIGHTNESS_WINDOW_SAMPLE_MS)

// =========================================================
// SD_MMC CONFIGURATION
// =========================================================
#ifndef SDMMC_CLK
#define SDMMC_CLK 14
#endif
#ifndef SDMMC_CMD
#define SDMMC_CMD 15
#endif
#ifndef SDMMC_D0
#define SDMMC_D0 2
#endif
#ifndef SDMMC_D1
#define SDMMC_D1 4
#endif
#ifndef SDMMC_D2
#define SDMMC_D2 12
#endif
#ifndef SDMMC_D3
#define SDMMC_D3 13
#endif
#ifndef SDMMC_MODE_1BIT
#define SDMMC_MODE_1BIT 0
#endif

int currentBrightness = 100;
bool autoBrightnessEnabled = false;
unsigned long lastAutoBrightnessSampleMs = 0;
unsigned long lastAutoBrightnessWindowSampleMs = 0;
int autoBrightnessWindow[AUTO_BRIGHTNESS_WINDOW_SAMPLES] = {0};
int autoBrightnessWindowCount = 0;
int autoBrightnessWindowIndex = 0;
int autoBrightnessDynamicDark = AUTO_BRIGHTNESS_ADC_DARK;
int autoBrightnessDynamicBright = AUTO_BRIGHTNESS_ADC_BRIGHT;

static void updateAutoBrightnessWindow(int raw, unsigned long now) {
    if (now - lastAutoBrightnessWindowSampleMs < AUTO_BRIGHTNESS_WINDOW_SAMPLE_MS) return;
    lastAutoBrightnessWindowSampleMs = now;

    autoBrightnessWindow[autoBrightnessWindowIndex] = raw;
    autoBrightnessWindowIndex = (autoBrightnessWindowIndex + 1) % AUTO_BRIGHTNESS_WINDOW_SAMPLES;
    if (autoBrightnessWindowCount < AUTO_BRIGHTNESS_WINDOW_SAMPLES) {
        autoBrightnessWindowCount++;
    }

    int minRaw = 4095;
    int maxRaw = 0;
    for (int i = 0; i < autoBrightnessWindowCount; i++) {
        int v = autoBrightnessWindow[i];
        if (v < minRaw) minRaw = v;
        if (v > maxRaw) maxRaw = v;
    }

    // Keep some minimum span so tiny ambient changes do not over-amplify brightness swings.
    if ((maxRaw - minRaw) < AUTO_BRIGHTNESS_MIN_ADC_SPAN) {
        int mid = (minRaw + maxRaw) / 2;
        minRaw = mid - (AUTO_BRIGHTNESS_MIN_ADC_SPAN / 2);
        maxRaw = mid + (AUTO_BRIGHTNESS_MIN_ADC_SPAN / 2);
    }

    int targetDark = constrain(minRaw, 0, 4094);
    int targetBright = constrain(maxRaw, targetDark + 1, 4095);

    // Move dynamic bounds gradually to avoid brightness pumping.
    autoBrightnessDynamicDark =
        (autoBrightnessDynamicDark * AUTO_BRIGHTNESS_BOUND_SMOOTH_NUM + targetDark) /
        AUTO_BRIGHTNESS_BOUND_SMOOTH_DEN;
    autoBrightnessDynamicBright =
        (autoBrightnessDynamicBright * AUTO_BRIGHTNESS_BOUND_SMOOTH_NUM + targetBright) /
        AUTO_BRIGHTNESS_BOUND_SMOOTH_DEN;

    // Keep bounds valid after smoothing.
    autoBrightnessDynamicDark = constrain(autoBrightnessDynamicDark, 0, 4094);
    autoBrightnessDynamicBright = constrain(autoBrightnessDynamicBright, autoBrightnessDynamicDark + 1, 4095);
}

static void updateAutoBrightness() {
    if (!autoBrightnessEnabled) return;

    unsigned long now = millis();
    if (now - lastAutoBrightnessSampleMs < AUTO_BRIGHTNESS_SAMPLE_MS) return;
    lastAutoBrightnessSampleMs = now;

    long rawSum = 0;
    for (int i = 0; i < 4; i++) {
        rawSum += analogRead(AUTO_BRIGHTNESS_ADC_PIN);
    }
    int raw = (int)(rawSum / 4);

#if AUTO_BRIGHTNESS_LDR_INVERT
    raw = 4095 - raw;
#endif

    updateAutoBrightnessWindow(raw, now);

    int clampedRaw = constrain(raw, autoBrightnessDynamicDark, autoBrightnessDynamicBright);
    int target = map(clampedRaw,
                     autoBrightnessDynamicDark,
                     autoBrightnessDynamicBright,
                     AUTO_BRIGHTNESS_MIN_PERCENT,
                     AUTO_BRIGHTNESS_MAX_PERCENT);

    // Faster smoothing for more responsive brightness updates.
    int smoothed = (currentBrightness * AUTO_BRIGHTNESS_SMOOTHING_NUM + target) /
                   (AUTO_BRIGHTNESS_SMOOTHING_NUM + 1);

    // Ignore tiny changes and cap per-step movement to reduce visible flutter.
    int delta = smoothed - currentBrightness;
    if (abs(delta) < AUTO_BRIGHTNESS_DEADBAND_PERCENT) {
        return;
    }
    if (delta > AUTO_BRIGHTNESS_MAX_STEP_PERCENT) delta = AUTO_BRIGHTNESS_MAX_STEP_PERCENT;
    if (delta < -AUTO_BRIGHTNESS_MAX_STEP_PERCENT) delta = -AUTO_BRIGHTNESS_MAX_STEP_PERCENT;

    setBrightness(currentBrightness + delta);
}

void setBrightness(int percent) {
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    currentBrightness = percent;

#if BACKLIGHT_ACTIVE_LOW
    int duty = map(percent, 0, 100, 255, 0);
#else
    int duty = map(percent, 0, 100, 0, 255);
#endif

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

#if BACKLIGHT_SWEEP_TEST
unsigned long lastBacklightSweepMs = 0;
int backlightSweepPercent = 0;
int backlightSweepDirection = 1;
#endif

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
    Serial.println("GPS: Initializing NEO-6M...");

    // NEO-6M defaults to 9600 baud. Reconfigure it to GPS_BAUD (115200) so both
    // the GPS module and this UART run at the same speed as the monitor.
    // Step 1: Drop to 9600 briefly just to send the UBX baud-change command.
    beginGpsPort(GPS_DEFAULT_BAUD);
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
#ifdef USE_GPS_UART1
    GPSSerial.write(ubxSetBaud, sizeof(ubxSetBaud));
    GPSSerial.flush();
#else
    Serial.write(ubxSetBaud, sizeof(ubxSetBaud));
    Serial.flush();
#endif
    delay(100);

    // Step 2: Switch our side to 115200 to match the newly configured GPS module.
    beginGpsPort(GPS_BAUD);
    delay(200);
    // Flush any stale bytes that arrived during the baud transition
    flushGpsRx();

#ifdef USE_GPS_UART1
    Serial.printf("GPS: UART1 running at %d (RX=%d TX=%d)\n", GPS_BAUD, GPS_RX_PIN, GPS_TX_PIN);
#else
    Serial.println("GPS: UART0 running at 115200 (GPS + debug shared)");
#endif

    // Check if we're receiving valid-looking NMEA data at the new baud rate
    unsigned long startTime = millis();
    bool dataReceived = false;
    int validChars = 0;

    while (millis() - startTime < 3000) {  // Wait up to 3 seconds
#ifdef USE_GPS_UART1
        if (GPSSerial.available()) {
            char c = GPSSerial.read();
#else
        if (Serial.available()) {
            char c = Serial.read();
#endif
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
    
    // Read GPS data from configured UART
#ifdef USE_GPS_UART1
    while (GPSSerial.available() > 0) {
        char c = GPSSerial.read();
#else
    while (Serial.available() > 0) {
        char c = Serial.read();
#endif
        gps.encode(c);
    }
    
    // Update GPS fix status
    bool previousFix = gpsHasFix;
    
    if (gps.location.isValid() && gps.location.age() < 2000) {
        gpsHasFix = true;
        lastValidGPSTime = millis();

        // Hardware GPS is authoritative when present: keep shared GPS state fresh
        if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            gpsState.lat = gps.location.lat();
            gpsState.lon = gps.location.lng();
            gpsState.timestamp = millis();
            if (gps.speed.isValid()) {
                gpsState.speed = gps.speed.mps();
            }
            if (gps.course.isValid()) {
                gpsState.heading = gps.course.deg();
            }
            xSemaphoreGive(gpsMutex);
        }
        phoneGpsActive = false;
        
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
    
    // Request Route Sync from App - Decommissioned: App detects mismatch via State Byte
    // bleHandler.notify("SYNC_REQ");
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
    Serial.printf("Boot: Config: %s %uMB (partitions: %s)\n", DEVICE_HW_NAME, FLASH_SIZE_MB, "custom");
    if (flashMB >= 16 && FLASH_SIZE_MB < 16) {
        Serial.println("Boot: ℹ️  NOTE: 16MB flash detected but running <16MB config.");
        Serial.println("Boot:    Switch to a 16MB build environment to utilize full flash.");
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
    bool hasAutoBrightnessKey = prefs.isKey("auto_brightness");
    bool savedAutoBrightness = prefs.getBool("auto_brightness", AUTO_BRIGHTNESS_DEFAULT != 0);
#if AUTO_BRIGHTNESS_DEFAULT
    autoBrightnessEnabled = savedAutoBrightness;
    if (!hasAutoBrightnessKey) {
        prefs.putBool("auto_brightness", autoBrightnessEnabled);
    }
#else
    autoBrightnessEnabled = false;
#endif
    prefs.end();
    
    Serial.printf("Boot: Loading Theme %d\n", theme);
    setTheme(theme == 0); // Apply Theme immediately

    // Initialize TFT early for boot screen
    tft.begin();
    tft.setRotation(0); // Portrait mode

    // Backlight PWM: use saved brightness preference.
    ledcSetup(BACKLIGHT_PWM_CHAN, BACKLIGHT_PWM_FREQ, BACKLIGHT_PWM_RES);
    ledcAttachPin(TFT_BL, BACKLIGHT_PWM_CHAN);
    setBrightness(savedBrightness); // value already loaded above

    analogReadResolution(12);
    analogSetPinAttenuation(AUTO_BRIGHTNESS_ADC_PIN, ADC_11db);
    pinMode(AUTO_BRIGHTNESS_ADC_PIN, INPUT);

    Serial.printf("Boot: Auto brightness %s (pin=%d, sample=%ums)\n",
                  autoBrightnessEnabled ? "ON" : "OFF",
                  AUTO_BRIGHTNESS_ADC_PIN,
                  AUTO_BRIGHTNESS_SAMPLE_MS);
    
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
    if (!SD_MMC.setPins(SDMMC_CLK, SDMMC_CMD, SDMMC_D0, SDMMC_D1, SDMMC_D2, SDMMC_D3)) {
        Serial.println("❌ Main Loop: SD_MMC.setPins failed! Halting.");
        while(true) { vTaskDelay(1000); }
    }

    if (!SD_MMC.begin("/sdcard", SDMMC_MODE_1BIT, false)) {
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
    xTaskCreatePinnedToCore(BLELoopTask, "BLELoopTask", BLE_TASK_STACK_SIZE, NULL, 4, &bleLoopTaskHandle, 0);
    
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

#if BACKLIGHT_SWEEP_TEST
    Serial.println("Backlight: Sweep test enabled (0% <-> 100%)");
#endif
}

// Global variable to store input string from Serial
String inputString = "";

// BLE heartbeat timer - REMOVED per user request
// unsigned long lastHeartbeatTime = 0;
// const unsigned long HEARTBEAT_INTERVAL = 5000; // 5 seconds

void loop() {
  // Reset watchdog timer
  esp_task_wdt_reset();

#if BACKLIGHT_SWEEP_TEST
    unsigned long now = millis();
    if (now - lastBacklightSweepMs >= BACKLIGHT_SWEEP_STEP_MS) {
            lastBacklightSweepMs = now;
            setBrightness(backlightSweepPercent);
            backlightSweepPercent += backlightSweepDirection;
            if (backlightSweepPercent >= 100) {
                    backlightSweepPercent = 100;
                    backlightSweepDirection = -1;
            } else if (backlightSweepPercent <= 0) {
                    backlightSweepPercent = 0;
                    backlightSweepDirection = 1;
            }
    }
#endif

    updateAutoBrightness();
  
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
