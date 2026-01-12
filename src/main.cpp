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

// =========================================================
// GPS CONFIGURATION (NEO-6M on UART0 - Programming Pins)
// =========================================================
// GPIO3 (RX0) - Connect to NEO-6M TX
// GPIO1 (TX0) - Connect to NEO-6M RX  
#define GPS_BAUD 9600  // NEO-6M default baud rate

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
// STATUS INDICATORS
// =========================================================
bool bleConnected = false;
bool gpsHasFix = false;
bool gpsModulePresent = false;
bool phoneGpsActive = false;
unsigned long lastPhoneCommandTime = 0;

// Route Overlay
// Route Overlay
std::vector<RoutePoint, PSRAMAllocator<RoutePoint>> activeRoute;
bool routeAvailable = false;
SemaphoreHandle_t routeMutex;

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
    .targetLat = 12.8273,
    .targetLon = 80.2193,
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

// =========================================================
// BLE CALLBACK FUNCTIONS
// =========================================================

// Callback when coordinates are received via BLE
void onBLECoordinatesReceived(double lat, double lon) {
    Serial.printf("BLE Callback: Received coordinates - Lat: %.6f, Lon: %.6f\n", lat, lon);
    
    ControlParams newControlParams = lastSentControlParams;
    newControlParams.targetLat = lat;
    newControlParams.targetLon = lon;
    newControlParams.zoomFactor = 1.0; // Reset zoom when new coordinates received
    
    if (xQueueSend(controlParamsQueue, &newControlParams, pdMS_TO_TICKS(100)) == pdPASS) {
        lastSentControlParams = newControlParams;
        Serial.println("BLE: Coordinates sent to render task");
        bleHandler.sendNotification("COORDS_OK");
    } else {
        Serial.println("BLE: Failed to send coordinates to queue");
        bleHandler.sendNotification("COORDS_FAILED");
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
        bleHandler.sendNotification("ZOOM_OK");
    } else {
        Serial.println("BLE: Failed to send zoom to queue");
        bleHandler.sendNotification("ZOOM_FAILED");
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
    Serial.println("GPS: Initializing NEO-6M on UART0 (programming pins)...");
    Serial.begin(GPS_BAUD);  // UART0 already initialized at 115200 in setup, reinitialize for GPS
    
    // Wait a bit for GPS module to respond
    delay(200);
    
    // Check if we're receiving NMEA data
    unsigned long startTime = millis();
    bool dataReceived = false;
    
    while (millis() - startTime < 3000) {  // Wait up to 3 seconds
        if (Serial.available()) {
            char c = Serial.read();
            if (c == '$') {  // NMEA sentences start with '$'
                dataReceived = true;
                break;
            }
        }
        delay(100);
    }
    
    if (dataReceived) {
        gpsModulePresent = true;
        gpsInitialized = true;
        Serial.println("GPS: NEO-6M module detected");
    } else {
        gpsModulePresent = false;
        Serial.println("GPS: NEO-6M module not detected");
    }
    
    updateStatusIcons();
}

void processGPS() {
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
    
    // Update Params to Hide PIN
    ControlParams newParams = lastSentControlParams;
    newParams.showPIN = false;
    
    if (xQueueSend(controlParamsQueue, &newParams, pdMS_TO_TICKS(200)) == pdPASS) {
        lastSentControlParams = newParams;
        Serial.println("BLE: Sent PIN Hide Command to Queue");
    } else {
        Serial.println("BLE ERROR: Failed to send PIN Hide Command");
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
// ARDUINO SETUP AND LOOP FUNCTIONS
// =========================================================

void setup() {
    Serial.begin(115200); // Initialize serial communication
    while (!Serial && millis() < 5000); // Wait for serial port to connect (for up to 5 seconds)

    Serial.println("Main Loop: Initializing FreeRTOS objects...");

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

    // Create queues using defined constants
    controlParamsQueue = xQueueCreate(CONTROL_PARAMS_QUEUE_SIZE, sizeof(ControlParams));
    tileRequestQueue = xQueueCreate(TILE_REQUEST_QUEUE_SIZE, sizeof(TileKey));
    tileParsedNotificationQueue = xQueueCreate(TILE_PARSED_NOTIFICATION_QUEUE_SIZE, sizeof(bool));

    if (controlParamsQueue == NULL || tileRequestQueue == NULL || tileParsedNotificationQueue == NULL) {
        Serial.println("❌ Main Loop: Failed to create one or more queues! Halting.");
        while(true) { vTaskDelay(1000); } // Halt on critical error
    }

    Serial.println("Main Loop: Initializing SD_MMC...");
    if (!SD_MMC.begin()) {
        Serial.println("❌ Main Loop: SD_MMC Card Mount Failed! Please ensure card is inserted. Halting.");
        while(true) { vTaskDelay(1000); } // Halt on critical error
    }
    Serial.println("Main Loop: SD_MMC Card Mounted.");

    // Allocate DMA-capable buffer for SD card operations
    // Prioritize PSRAM for larger buffer, fallback to internal DMA RAM
    size_t largestFreePsramDmaBlock = heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
    size_t largestFreeInternalDmaBlock = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);

    // Determine target buffer size, capped by MAX_SD_DMA_BUFFER_SIZE_BYTES
    SD_DMA_BUFFER_SIZE = std::min(largestFreePsramDmaBlock, MAX_SD_DMA_BUFFER_SIZE_BYTES);

    Serial.printf("Main Loop: Attempting to allocate %u bytes for SD DMA buffer (Max %u KB).\n",
                  SD_DMA_BUFFER_SIZE, MAX_SD_DMA_BUFFER_SIZE_KB);
    Serial.printf("           Total Free PSRAM (DMA-capable): %u bytes. Largest Block: %u bytes.\n",
                  heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM), largestFreePsramDmaBlock);
    Serial.printf("           Total Free Internal RAM (DMA-capable): %u bytes. Largest Block: %u bytes.\n",
                  heap_caps_get_free_size(MALLOC_CAP_DMA), largestFreeInternalDmaBlock);


    sd_dma_buffer = (uint8_t *)heap_caps_malloc(SD_DMA_BUFFER_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
    if (sd_dma_buffer == nullptr) {
        Serial.println("❌ Main Loop: Failed to allocate SD DMA buffer from PSRAM. Trying internal DMA RAM...");
        SD_DMA_BUFFER_SIZE = std::min(largestFreeInternalDmaBlock, MAX_SD_DMA_BUFFER_SIZE_BYTES); // Re-evaluate size for internal RAM
        sd_dma_buffer = (uint8_t *)heap_caps_malloc(SD_DMA_BUFFER_SIZE, MALLOC_CAP_DMA);
        if (sd_dma_buffer == nullptr) {
            Serial.println("❌ Main Loop: Failed to allocate SD DMA buffer from internal DMA RAM either! Critical error. Halting.");
            while(true) { vTaskDelay(1000); } // Halt
        }
    }
    Serial.printf("Main Loop: Successfully allocated %u bytes for SD DMA buffer. Current PSRAM free: %u bytes\n",
                  SD_DMA_BUFFER_SIZE, heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    // Initialize GPS module
    initGPS();

    // Initialize iconColorsMap with specific colors for icon types
    iconColorsMap.emplace(PSRAMString("traffic_signals", PSRAMAllocator<char>()), TFT_CYAN);
    iconColorsMap.emplace(PSRAMString("bus_stop", PSRAMAllocator<char>()), TFT_VIOLET);
    iconColorsMap.emplace(PSRAMString("fuel", PSRAMAllocator<char>()), TFT_MAGENTA);


    Serial.println("Main Loop: Creating tasks...");
    // Create data task (Core 0)
    xTaskCreatePinnedToCore(
        dataTask,          // Task function
        "DataTask",        // Name of task
        DATA_TASK_STACK_SIZE, // Stack size (bytes)
        NULL,              // Parameter of the task
        1,                 // Priority of the task (Lower than render task)
        &dataTaskHandle,   // Task handle to keep track of created task
        0                  // Core to run on (Core 0)
    );

    // Create render task (Core 1)
    xTaskCreatePinnedToCore(
        renderTask,          // Task function
        "RenderTask",        // Name of task
        RENDER_TASK_STACK_SIZE, // Stack size (bytes)
        NULL,                // Parameter of the task
        2,                   // Priority of the task (Higher than data task)
        &renderTaskHandle,   // Task handle to keep track of created task
        1                    // Core to run on (Core 1)
    );

    Serial.println("Ready. Enter coordinates to load map (Longitude, Latitude):");
    Serial.println("Or enter zoom factor (1, 2, 3, or 4) to change zoom for current location.");
    Serial.println("Type 'gps' to display GPS information.");
    
    // Wait for system to stabilize before initializing BLE (prevents brownout)
    Serial.println("Main Loop: Waiting for system stabilization...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
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
    
    Serial.println("Main Loop: BLE initialized and ready");
    Serial.printf("Device Name: %s\n", bleHandler.getDeviceName().c_str());
    
    // Draw initial status icons
    updateStatusIcons();
    
    // Enable watchdog timer (5 seconds timeout)
    esp_task_wdt_init(5, true);
    esp_task_wdt_add(NULL);  // Add current task
    Serial.println("Main Loop: Watchdog timer enabled (5s timeout)");
}

// Global variable to store input string from Serial
String inputString = "";

// BLE heartbeat timer
unsigned long lastHeartbeatTime = 0;
const unsigned long HEARTBEAT_INTERVAL = 5000; // 5 seconds

void loop() {
  // Reset watchdog timer
  esp_task_wdt_reset();
  
  // Call BLE handler loop (for session timeout check)
  bleHandler.loop();
  
  // Process GPS data
  unsigned long currentTime = millis();
  if (currentTime - lastGPSCheck >= GPS_CHECK_INTERVAL) {
      processGPS();
      lastGPSCheck = currentTime;
  }
  
  // Read serial input
  while (Serial.available()) {
      char inChar = Serial.read();
      inputString += inChar;
      if (inChar == '\n') {
          Serial.println("Main Loop: Newline detected. Processing input.");
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
              Serial.printf("Main Loop: Parsed Lat: %.4f, Lon: %.4f\n", requestedLat, requestedLon);
          } else if (inputString.startsWith("gps")) {
              // GPS info command
              printGPSInfo();
          } else {
              // If not Lat,Lon, attempt to parse as zoom factor
              float requestedZoom = inputString.toFloat();
              if (requestedZoom >= 1.0f && requestedZoom <= 4.0f) {
                  newControlParams.zoomFactor = requestedZoom;
                  paramsUpdated = true;
                  Serial.printf("Main Loop: Parsed Zoom: %.1f\n", requestedZoom);
              } else {
                  Serial.println("Main Loop: Invalid input. Please enter coordinates (Lon,Lat) or a zoom factor (1, 2, 3, or 4).");
              }
          }

          if (paramsUpdated) {
              Serial.printf("Main Loop: Sending new control parameters - Lat: %.4f, Lon: %.4f, Zoom: %.1f, CullL: %.2f, CullR: %.2f, CullT: %.2f, CullB: %.2f. Queue space: %d\n",
                            newControlParams.targetLat, newControlParams.targetLon, newControlParams.zoomFactor,
                            newControlParams.cullingBufferPercentageLeft, newControlParams.cullingBufferPercentageRight,
                            newControlParams.cullingBufferPercentageTop, newControlParams.cullingBufferPercentageBottom,
                            uxQueueSpacesAvailable(controlParamsQueue));
              // Send the updated control parameters to the render task
              if (xQueueSend(controlParamsQueue, &newControlParams, 0) != pdPASS) { // Use 0 timeout for non-blocking send
                  Serial.println("❌ Main Loop: Failed to send control parameters to queue. Queue full? (Non-blocking send failed)");
              } else {
                  lastSentControlParams = newControlParams; // Update last sent params if successful
                  Serial.println("Main Loop: Successfully sent control parameters to queue.");
              }
          }
          inputString = ""; // Clear the input string
          vTaskDelay(pdMS_TO_TICKS(50)); // Add a small delay after processing input
      }
  }
  
  // Send BLE heartbeat every 10 seconds for debugging
  unsigned long heartbeatTime = millis();
  if (heartbeatTime - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
      if (bleHandler.isConnected() && bleHandler.isDeviceAuthenticated()) {
          bleHandler.sendNotification("HEARTBEAT");
      }
      lastHeartbeatTime = heartbeatTime;
  }
  
  /* Blinking logic removed (Icons disabled)
  // Blink BLE icon if in pairing window and not connected (physically)
  static unsigned long lastBlinkTime = 0;
  static bool iconVisible = true;
  
  // Only blink if NOT physically connected AND in pairing window
  if (!bleHandler.isConnected() && bleHandler.isWithinPairingWindow()) {
      if (millis() - lastBlinkTime > 500) { // Blink every 500ms
          iconVisible = !iconVisible;
          drawBLEIcon(false, iconVisible); // Flash RED
          lastBlinkTime = millis();
      }
  } else {
      // All other states: Ensure icon is visible (Solid Red or Blue)
      // If we were blinking (invisible), restore visibility immediately
      if (!iconVisible) {
          iconVisible = true;
          // Refresh to correct color (Red if unauth, Blue if auth)
          updateStatusIcons();
      }
  }
  */
  
  // Yield control more frequently
  bleHandler.loop();
  vTaskDelay(pdMS_TO_TICKS(10));
  
  // -- Check and Update BLE Icon Mode --
  uint8_t currentBleIconMode = 1; // Default Solid Red
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
      
      if (xQueueSend(controlParamsQueue, &newParams, 0) == pdPASS) {
          lastSentControlParams = newParams;
          Serial.printf("Main Loop: Updated BLE Icon Mode to %d\n", currentBleIconMode);
      }
  }
}
