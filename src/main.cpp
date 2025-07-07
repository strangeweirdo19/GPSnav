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

// DMA-capable buffer for SD card operations (defined here)
uint8_t *sd_dma_buffer = nullptr;
size_t SD_DMA_BUFFER_SIZE; // No longer const, will be determined dynamically

// =========================================================
// ARDUINO SETUP AND LOOP FUNCTIONS
// =========================================================
void setup() {
  Serial.begin(115200); // Initialize serial communication for debugging

  // Initialize mutex and queues
  loadedTilesDataMutex = xSemaphoreCreateMutex();
  controlParamsQueue = xQueueCreate(10, sizeof(ControlParams)); // Increased queue size for control parameters
  tileRequestQueue = xQueueCreate(15, sizeof(TileKey)); // Increased queue size for tile requests
  tileParsedNotificationQueue = xQueueCreate(5, sizeof(bool)); // Queue for tile parsed notifications

  if (loadedTilesDataMutex == NULL || controlParamsQueue == NULL || 
      tileRequestQueue == NULL || tileParsedNotificationQueue == NULL) {
      Serial.println("❌ Main: Failed to create FreeRTOS objects. Not enough memory?");
      while(true); // Halt if essential FreeRTOS objects cannot be created
  }

  // Initialize SD_MMC here, before tasks are created
  Serial.println("Main: Initializing SD_MMC...");
  if (!SD_MMC.begin()) {
      Serial.println("❌ Main: SD_MMC mount failed. Check wiring and formatting.");
      // Do not halt, allow dataTask to report this as well, but it won't proceed
  } else {
      Serial.println("Main: SD_MMC mounted successfully.");
  }

  // --- Memory Information ---
  Serial.println("\n--- Heap Memory Information (at startup) ---");
  Serial.printf("Total Free Internal RAM (DRAM): %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  Serial.printf("Largest Free Internal RAM Block (DRAM): %u bytes\n", heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  Serial.printf("Total Free IRAM: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_32BIT | MALLOC_CAP_EXEC));
  Serial.printf("Largest Free IRAM Block: %u bytes\n", heap_caps_get_largest_free_block(MALLOC_CAP_32BIT | MALLOC_CAP_EXEC));
  Serial.printf("Total Free PSRAM: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  Serial.printf("Largest Free PSRAM Block: %u bytes\n", heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  Serial.printf("Total Free DMA-compatible RAM: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_DMA));
  Serial.printf("Largest Free DMA-compatible Block: %u bytes\n", heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
  Serial.println("------------------------------------------\n");

  // Dynamically determine the largest free DMA-compatible block and allocate the buffer
  // We will attempt to allocate the exact largest free block.
  SD_DMA_BUFFER_SIZE = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);
  
  Serial.printf("Main: Attempting to allocate %u bytes for SD DMA buffer. Total Free DMA-compatible RAM: %u bytes. Largest Free DMA-compatible Block: %u bytes.\n", 
                SD_DMA_BUFFER_SIZE, heap_caps_get_free_size(MALLOC_CAP_DMA), heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
  
  // Check if there's any DMA-compatible memory available before attempting allocation
  if (SD_DMA_BUFFER_SIZE == 0) {
      Serial.println("❌ Main: No contiguous DMA-compatible RAM block available for allocation. Map operations will fail.");
      sd_dma_buffer = nullptr; // Explicitly set to nullptr
  } else {
      sd_dma_buffer = (uint8_t*) heap_caps_malloc(SD_DMA_BUFFER_SIZE, MALLOC_CAP_DMA);
      if (sd_dma_buffer == nullptr) {
          Serial.printf("❌ Main: Failed to allocate %u bytes for SD DMA buffer in DMA-compatible RAM. Map operations will fail.\n", SD_DMA_BUFFER_SIZE);
          // Do not halt, but dataTask will terminate if this fails
      } else {
          Serial.printf("Main: Successfully allocated %u bytes for SD DMA buffer. Current PSRAM free: %u bytes\n", 
                        SD_DMA_BUFFER_SIZE, heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
      }
  }


  // Create tasks
  xTaskCreatePinnedToCore(
    dataTask,           // Task function
    "DataTask",         // Name of task
    8 * 1024,           // Stack size (bytes) - Increased for SQLite and parsing
    NULL,               // Parameter of the task
    2,                  // Priority of the task
    NULL,               // Task handle to keep track of created task
    0                   // Core where task should run (Core 0)
  );

  xTaskCreatePinnedToCore(
    renderTask,         // Task function
    "RenderTask",       // Name of task
    8 * 1024,           // Stack size (bytes) - Increased for rendering and compass
    NULL,               // Parameter of the task
    1,                  // Priority of the task
    NULL,               // Task handle to keep track of created task
    1                   // Core where task should run (Core 1)
  );

  Serial.println("Ready. Enter coordinates to load map (Longitude, Latitude):");
  Serial.println("Or enter zoom factor (1, 2, 3, or 4) to change zoom for current location.");
}

void loop() {
  static String inputString = "";
  static bool newData = false;
  static ControlParams lastSentControlParams = {12.8273, 80.2193, 1.0}; // Keep track of last sent params

  while (Serial.available()) {
    char inChar = Serial.read();
    Serial.printf("Main Loop: Received char: '%c' (0x%X)\n", inChar, inChar); // Debug: Log each character
    if (inChar == '\n' || inChar == '\r') { // Handle both LF and CR for newline
      newData = true;
      Serial.println("Main Loop: Newline detected. Processing input.");
    } else {
      inputString += inChar;
    }
  }

  if (newData) {
      newData = false;
      inputString.trim(); // Remove leading/trailing whitespace
      Serial.printf("Main Loop: Processing input string: '%s'\n", inputString.c_str()); // Debug: Log trimmed input

      ControlParams newControlParams = lastSentControlParams; // Start with last sent params
      bool paramsUpdated = false;

      if (inputString.indexOf(',') != -1) { // Coordinate input
        int commaIndex = inputString.indexOf(',');
        String lonStr = inputString.substring(0, commaIndex);
        String latStr = inputString.substring(commaIndex + 1);

        double newLon = lonStr.toDouble();
        double newLat = latStr.toDouble();

        // Check if toDouble() actually parsed a valid number, or if it returned 0.0 for non-numeric input
        // Also explicitly check for "0" or "0.0" strings if 0.0 is a valid coordinate
        bool latValid = (newLat != 0.0f || latStr.equals("0") || latStr.equals("0.0"));
        bool lonValid = (newLon != 0.0f || lonStr.equals("0") || lonStr.equals("0.0"));

        Serial.printf("Main Loop: Parsed Lat: %.4f (valid: %d), Lon: %.4f (valid: %d)\n", newLat, latValid, newLon, lonValid); // Debug: Log parsed coords

        if (latValid && lonValid) {
            newControlParams.targetLat = newLat;
            newControlParams.targetLon = newLon;
            paramsUpdated = true;
        } else {
            Serial.println("Main Loop: Invalid coordinate format or values. Please enter valid floating-point numbers separated by a comma.");
        }
      } else { // Zoom factor input
        float requestedZoom = inputString.toFloat();
        // Check if toFloat() actually parsed a valid number
        if (requestedZoom >= 1.0f && requestedZoom <= 4.0f && fmod(requestedZoom, 1.0f) == 0.0f) {
            newControlParams.zoomFactor = requestedZoom;
            paramsUpdated = true;
            Serial.printf("Main Loop: Parsed Zoom: %.1f (valid: %d)\n", requestedZoom, true); // Debug: Log parsed zoom
        } else {
            Serial.println("Main Loop: Invalid zoom factor. Please enter 1, 2, 3, or 4.");
            Serial.printf("Main Loop: Parsed Zoom: %.1f (valid: %d)\n", requestedZoom, false); // Debug: Log invalid zoom
        }
      }

      if (paramsUpdated) {
          Serial.printf("Main Loop: Sending new control parameters - Lat: %.4f, Lon: %.4f, Zoom: %.1f. Queue space: %d\n", 
                        newControlParams.targetLat, newControlParams.targetLon, newControlParams.zoomFactor, uxQueueSpacesAvailable(controlParamsQueue));
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
  // Yield control more frequently to allow other tasks to run, improving responsiveness
  vTaskDelay(pdMS_TO_TICKS(10)); 
}
