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

QueueHandle_t renderParamsQueue;
QueueHandle_t controlParamsQueue;

// Global variable to store the last sent control parameters, for persistent state
ControlParams lastSentControlParams = {12.8273, 80.2193, 1.0}; // Initialized to default values

// =========================================================
// ARDUINO SETUP AND LOOP
// =========================================================

void setup() {
  unsigned long setupStartTime = millis(); // Start timing setup duration

  Serial.begin(115200);
  delay(100);

  Serial.println("Main Setup: Creating FreeRTOS objects...");
  loadedTilesDataMutex = xSemaphoreCreateMutex();
  renderParamsQueue = xQueueCreate(1, sizeof(RenderParams)); // Queue size 1, only latest params needed
  controlParamsQueue = xQueueCreate(1, sizeof(ControlParams)); // Queue for control parameters

  if (loadedTilesDataMutex == NULL || renderParamsQueue == NULL || controlParamsQueue == NULL) {
      Serial.println("❌ Main Setup: Failed to create FreeRTOS objects. Out of memory?");
      return;
  }
  Serial.println("✅ Main Setup: FreeRTOS objects created.");

  Serial.println("Main Setup: Creating tasks...");
  xTaskCreatePinnedToCore(
      dataTask,           // Task function
      "DataTask",         // Name of task
      8192,               // Stack size (bytes) - increased for SQLite/parsing
      NULL,               // Parameter to pass to function
      1,                  // Task priority (higher is more important)
      NULL,               // Task handle
      0                   // Core where the task should run (Core 0)
  );

  xTaskCreatePinnedToCore(
      renderTask,         // Task function
      "RenderTask",       // Name of task
      8192,               // Stack size (bytes) - increased for TFT/sprite
      NULL,               // Parameter to pass to function
      2,                  // Task priority (higher than data task for responsiveness)
      NULL,               // Task handle
      1                   // Core where the task should run (Core 1)
  );

  unsigned long setupDuration = millis() - setupStartTime; // Calculate setup duration
  Serial.printf("Main Setup: Initial hardware and FreeRTOS setup completed in %lu ms.\n", setupDuration); // Print setup duration

  // --- Re-added: Send initial control parameters to data task to trigger first load ---
  if (xQueueSend(controlParamsQueue, &lastSentControlParams, 0) != pdPASS) {
      Serial.println("Main Setup: Failed to send initial control parameters to queue. This might delay map display.");
  }
  vTaskDelay(pdMS_TO_TICKS(50)); // Small delay to allow dataTask to process initial params
  // --- End Re-added ---

  Serial.println("\nReady. Enter coordinates to load map (Longitude, Latitude):"); // Updated prompt
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
            Serial.printf("Main Loop: Received new target coordinates: Lon %.6f, Lat %.6f\n", newLon, newLat);
            newControlParams.targetLat = newLat;
            newControlParams.targetLon = newLon;
            paramsUpdated = true;
        } else {
            Serial.println("Main Loop: Invalid coordinate format or values. Please enter valid floating-point numbers separated by a comma.");
        }
      } else { // Zoom factor input
        float requestedZoom = inputString.toFloat();
        if (requestedZoom >= 1.0f && requestedZoom <= 4.0f && fmod(requestedZoom, 1.0f) == 0.0f) {
            Serial.printf("Main Loop: Received new zoom factor: %.1fx\n", requestedZoom);
            newControlParams.zoomFactor = requestedZoom;
            paramsUpdated = true;
        } else {
            Serial.println("Main Loop: Invalid zoom factor. Please enter 1, 2, 3, or 4.");
        }
      }

      if (paramsUpdated) {
          // Send the updated control parameters to the data task
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
