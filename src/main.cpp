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
int currentTileZ = 17;

// =========================================================
// GLOBAL SHARED DATA AND SYNCHRONIZATION OBJECTS (Definitions from common.h externs)
// =========================================================
std::map<TileKey, std::vector<ParsedLayer, PSRAMAllocator<ParsedLayer>>> loadedTilesData;
SemaphoreHandle_t loadedTilesDataMutex;
int currentLayerExtent = 4096; // Default MVT tile extent, will be updated from parsed data

// Queues for inter-task communication
QueueHandle_t renderParamsQueue;         // DataTask -> RenderTask (Removed, RenderTask now calculates its own)
QueueHandle_t controlParamsQueue;        // Loop -> RenderTask (For user input)
QueueHandle_t tileRequestQueue;          // RenderTask -> DataTask (New: for requesting tiles)
QueueHandle_t tileParsedNotificationQueue; // DataTask -> RenderTask (New: for notifying when tile is parsed)


// Global variable to store the last sent control parameters, for persistent state
// This will now be sent to renderTask directly.
ControlParams lastSentControlParams = {12.8273, 80.2193, 1.0}; // Initialized to default values

// =========================================================
// ARDUINO SETUP AND LOOP
// =========================================================

void setup() {
  Serial.begin(115200);
  delay(100);

  loadedTilesDataMutex = xSemaphoreCreateMutex();
  // Renamed and added new queues for the restructured communication
  controlParamsQueue = xQueueCreate(1, sizeof(ControlParams)); // Loop -> RenderTask
  tileRequestQueue = xQueueCreate(5, sizeof(TileKey));        // RenderTask -> DataTask (Buffer for a few tile requests)
  tileParsedNotificationQueue = xQueueCreate(1, sizeof(bool)); // DataTask -> RenderTask (Simple notification)


  if (loadedTilesDataMutex == NULL || controlParamsQueue == NULL || tileRequestQueue == NULL || tileParsedNotificationQueue == NULL) {
      Serial.println("❌ Main Setup: Failed to create FreeRTOS objects. Out of memory?");
      return;
  }

  // Create tasks with their new roles
  xTaskCreatePinnedToCore(
      dataTask,           // Task function (now purely data processing)
      "DataTask",         // Name of task
      8192,               // Stack size (bytes) - increased for SQLite/parsing
      NULL,               // Parameter to pass to function
      1,                  // Task priority
      NULL,               // Task handle
      0                   // Core where the task should run (Core 0)
  );

  xTaskCreatePinnedToCore(
      renderTask,         // Task function (now handles control, sensors, and rendering)
      "RenderTask",       // Name of task
      8192,               // Stack size (bytes) - increased for TFT/sprite and sensor logic
      NULL,               // Parameter to pass to function
      2,                  // Task priority (higher for responsiveness)
      NULL,               // Task handle
      1                   // Core where the task should run (Core 1)
  );

  // Send initial control parameters to the render task to trigger first map load
  if (xQueueSend(controlParamsQueue, &lastSentControlParams, 0) != pdPASS) {
      Serial.println("Main Setup: Failed to send initial control parameters to queue. This might delay map display.");
  }
  vTaskDelay(pdMS_TO_TICKS(50)); // Small delay to allow tasks to start

  Serial.println("\nReady. Enter coordinates to load map (Longitude, Latitude):");
  Serial.println("Or enter zoom factor (1, 2, 3, or 4) to change zoom for current location.");
}

void loop() {
  // The loop function will now primarily handle serial input for changing coordinates/zoom.
  // The main application logic is moved to FreeRTOS tasks.
  if (Serial.available() > 0) {
      String inputString = Serial.readStringUntil('\n');
      inputString.trim();

      int commaIndex = inputString.indexOf(',');

      // Start with the last known good parameters
      ControlParams newControlParams = lastSentControlParams;
      bool paramsUpdated = false;

      if (commaIndex != -1) { // Coordinate input
        String lonStr = inputString.substring(0, commaIndex);
        String latStr = inputString.substring(commaIndex + 1);

        double newLat = latStr.toFloat();
        double newLon = lonStr.toFloat();

        bool latValid = (newLat != 0.0f || latStr.equals("0") || latStr.equals("0.0"));
        bool lonValid = (newLon != 0.0f || lonStr.equals("0") || lonStr.equals("0.0"));

        if (latValid && lonValid) {
            newControlParams.targetLat = newLat;
            newControlParams.targetLon = newLon;
            paramsUpdated = true;
        } else {
            Serial.println("Main Loop: Invalid coordinate format or values. Please enter valid floating-point numbers separated by a comma.");
        }
      } else { // Zoom factor input
        float requestedZoom = inputString.toFloat();
        if (requestedZoom >= 1.0f && requestedZoom <= 4.0f && fmod(requestedZoom, 1.0f) == 0.0f) {
            newControlParams.zoomFactor = requestedZoom;
            paramsUpdated = true;
        } else {
            Serial.println("Main Loop: Invalid zoom factor. Please enter 1, 2, 3, or 4.");
        }
      }

      if (paramsUpdated) {
          // Send the updated control parameters to the render task
          if (xQueueSend(controlParamsQueue, &newControlParams, 0) != pdPASS) {
              Serial.println("Main Loop: Failed to send control parameters to queue. Queue full?");
          } else {
              lastSentControlParams = newControlParams; // Update last sent params if successful
          }
      }
  }
  // Yield control more frequently to allow other tasks to run, improving responsiveness
  vTaskDelay(pdMS_TO_TICKS(10)); // Reduced delay from 100ms to 10ms
}
