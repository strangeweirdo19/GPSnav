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
int screenW = 160;
int screenH = 128;

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
// ARDUINO SETUP AND LOOP FUNCTIONS
// =========================================================

void setup() {
    Serial.begin(115200); // Initialize serial communication
    while (!Serial && millis() < 5000); // Wait for serial port to connect (for up to 5 seconds)

    Serial.println("Main Loop: Initializing FreeRTOS objects...");

    // Create mutex for loadedTilesData
    loadedTilesDataMutex = xSemaphoreCreateMutex();
    if (loadedTilesDataMutex == NULL) {
        Serial.println("❌ Main Loop: Failed to create loadedTilesDataMutex!");
        // Handle error, perhaps halt
    }

    // Create queues
    controlParamsQueue = xQueueCreate(5, sizeof(ControlParams)); // Small queue for user input
    // Increased tileRequestQueue size to accommodate 9+16 tiles (25 tiles) plus some buffer
    tileRequestQueue = xQueueCreate(30, sizeof(TileKey)); 
    tileParsedNotificationQueue = xQueueCreate(5, sizeof(bool)); // Small queue for notifications

    if (controlParamsQueue == NULL || tileRequestQueue == NULL || tileParsedNotificationQueue == NULL) {
        Serial.println("❌ Main Loop: Failed to create one or more queues!");
        // Handle error, perhaps halt
    }

    Serial.println("Main Loop: Initializing SD_MMC...");
    if (!SD_MMC.begin()) {
        Serial.println("❌ Main Loop: SD_MMC Card Mount Failed!");
        // Handle error, perhaps halt
        return;
    }
    Serial.println("Main Loop: SD_MMC Card Mounted.");

    // Allocate DMA-capable buffer for SD card operations
    // Try to allocate the largest contiguous block of DMA-capable PSRAM
    SD_DMA_BUFFER_SIZE = heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
    if (SD_DMA_BUFFER_SIZE == 0) {
        // Fallback to internal DMA-capable RAM if PSRAM is not available or full
        SD_DMA_BUFFER_SIZE = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);
    }
    
    // Cap the buffer size to a reasonable maximum if it's excessively large, e.g., 256KB
    // A typical MVT tile is usually much smaller, but some can be large.
    // If your largest tile is, for example, 100KB, set a cap slightly above that.
    const size_t MAX_SD_DMA_BUFFER_SIZE = 150 * 1024; // Example: Cap at 150KB
    if (SD_DMA_BUFFER_SIZE > MAX_SD_DMA_BUFFER_SIZE) {
        SD_DMA_BUFFER_SIZE = MAX_SD_DMA_BUFFER_SIZE;
    }

    Serial.printf("Main Loop: Attempting to allocate %u bytes for SD DMA buffer. Total Free DMA-compatible RAM: %u bytes. Largest Free DMA-compatible Block: %u bytes.\n", 
                  SD_DMA_BUFFER_SIZE, 
                  heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM),
                  heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM));

    sd_dma_buffer = (uint8_t *)heap_caps_malloc(SD_DMA_BUFFER_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
    if (sd_dma_buffer == nullptr) {
        Serial.println("❌ Main Loop: Failed to allocate SD DMA buffer! Map loading will fail.");
        // Try allocating from internal DMA RAM as a last resort, but performance might be worse
        sd_dma_buffer = (uint8_t *)heap_caps_malloc(SD_DMA_BUFFER_SIZE, MALLOC_CAP_DMA);
        if (sd_dma_buffer == nullptr) {
            Serial.println("❌ Main Loop: Failed to allocate SD DMA buffer from internal DMA RAM either! Critical error.");
            // Halt or enter a safe mode
            while(true) { vTaskDelay(1000); } // Halt
        }
    }
    Serial.printf("Main Loop: Successfully allocated %u bytes for SD DMA buffer. Current PSRAM free: %u bytes\n", 
                  SD_DMA_BUFFER_SIZE, heap_caps_get_free_size(MALLOC_CAP_SPIRAM));


    Serial.println("Main Loop: Creating tasks...");
    // Create data task (Core 0)
    xTaskCreatePinnedToCore(
        dataTask,          // Task function
        "DataTask",        // Name of task
        8 * 1024,          // Stack size (bytes) - Increased for parsing complexity
        NULL,              // Parameter of the task
        5,                 // Priority of the task
        NULL,              // Task handle to keep track of created task
        0                  // Core to run on (Core 0)
    );

    // Create render task (Core 1)
    xTaskCreatePinnedToCore(
        renderTask,          // Task function
        "RenderTask",        // Name of task
        10 * 1024,           // Stack size (bytes) - Increased for rendering complexity
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
// Initialize with default culling percentages for all four directions
ControlParams lastSentControlParams = {12.8273, 80.2193, 1.0, 0.15f, 0.15f, 0.13f, 0.50f}; 

void loop() {
  // Read serial input
  while (Serial.available()) {
      char inChar = Serial.read();
      inputString += inChar;
      if (inChar == '\n') {
          Serial.println("Main Loop: Newline detected. Processing input."); // Debugging
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
              Serial.printf("Main Loop: Parsed Lat: %.4f (valid: %d), Lon: %.4f (valid: %d)\n", 
                            requestedLat, true, requestedLon, true); // Debug: Log parsed lat/lon
          } else {
              // If not Lat,Lon, attempt to parse as zoom factor
              float requestedZoom = inputString.toFloat();
              if (requestedZoom >= 1.0f && requestedZoom <= 4.0f) {
                  newControlParams.zoomFactor = requestedZoom;
                  paramsUpdated = true;
                  Serial.printf("Main Loop: Parsed Zoom: %.1f (valid: %d)\n", requestedZoom, true); // Debug: Log parsed zoom
              } else {
                  Serial.println("Main Loop: Invalid input. Please enter coordinates (Lon,Lat) or a zoom factor (1, 2, 3, or 4).");
                  Serial.printf("Main Loop: Parsed Zoom: %.1f (valid: %d)\n", requestedZoom, false); // Debug: Log invalid zoom
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
