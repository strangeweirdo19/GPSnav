// main.cpp
#include <Arduino.h>
#include "common.h"     // Include common definitions
#include "data_task.h"  // Include data task declarations
#include "render_task.h" // Include render task declarations

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
        5,                 // Priority of the task
        NULL,              // Task handle to keep track of created task
        0                  // Core to run on (Core 0)
    );

    // Create render task (Core 1)
    xTaskCreatePinnedToCore(
        renderTask,          // Task function
        "RenderTask",        // Name of task
        RENDER_TASK_STACK_SIZE, // Stack size (bytes)
        NULL,                // Parameter of the task
        4,                   // Priority of the task (lower than data task, but still high)
        NULL,                // Task handle to keep track of created task
        1                    // Core to run on (Core 1)
    );

    Serial.println("Ready. Enter coordinates to load map (Longitude, Latitude):");
    Serial.println("Or enter zoom factor (1, 2, 3, or 4) to change zoom for current location.");
}

// Global variable to store input string from Serial
String inputString = "";
// Global variable to track last sent control parameters to avoid redundant sends
ControlParams lastSentControlParams = {
    .targetLat = 12.8273,
    .targetLon = 80.2193,
    .zoomFactor = 1.0,
    .cullingBufferPercentageLeft = DEFAULT_CULLING_BUFFER_PERCENTAGE_LEFT,
    .cullingBufferPercentageRight = DEFAULT_CULLING_BUFFER_PERCENTAGE_RIGHT,
    .cullingBufferPercentageTop = DEFAULT_CULLING_BUFFER_PERCENTAGE_TOP,
    .cullingBufferPercentageBottom = DEFAULT_CULLING_BUFFER_PERCENTAGE_BOTTOM
};

void loop() {
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
  // Yield control more frequently to allow other tasks to run, improving responsiveness
  vTaskDelay(pdMS_TO_TICKS(10));
}
