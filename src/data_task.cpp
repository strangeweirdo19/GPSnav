// data_task.cpp
#include "data_task.h"
#include "common.h"
#include "mvt_parser.h" // Include the new MVT parser header
#include "colors.h"     // Include the colors header

// Global SQLite DB handle (careful with this for multi-threading, but fine for single-threaded loop)
static sqlite3 *mbtiles_db = nullptr; // Changed to static, managed internally by dataTask

// =========================================================
// MBTILES DATABASE AND COORDINATE HELPERS
// =========================================================

// Fetches tile data (BLOB) from the SQLite MBTiles database
// This function should only be called from the dataTask, ensuring single-threaded DB access.
// tileDataPtr will be allocated in PSRAM, the caller is responsible for freeing it.
bool fetchTile(sqlite3 *db, int z, int x, int y, uint8_t *&tileDataPtr, size_t &tileDataLen) {
  sqlite3_stmt *stmt; // Prepared statement object
  const char *sql = "SELECT tile_data FROM tiles WHERE zoom_level=? AND tile_column=? AND tile_row=?;";

  if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    Serial.printf("❌ SQLite prepare failed: %s.\n", sqlite3_errmsg(db)); // Re-enabled debug print
    return false;
  }

  sqlite3_bind_int(stmt, 1, z);
  sqlite3_bind_int(stmt, 2, x);
  sqlite3_bind_int(stmt, 3, y); // Bind the TMS Y coordinate

  bool ok = false;
  int rc = sqlite3_step(stmt);
  if (rc == SQLITE_ROW) {
    const void *blob = sqlite3_column_blob(stmt, 0); // Corrected: Pass stmt to sqlite3_column_blob
    int len = sqlite3_column_bytes(stmt, 0); // Corrected: Pass stmt to sqlite3_column_bytes

    // Check if tile data size exceeds the pre-allocated buffer
    if (len > SD_DMA_BUFFER_SIZE) {
        Serial.printf("❌ Data Task: Tile data size (%d bytes) exceeds pre-allocated SD DMA buffer size (%u bytes) for Z:%d X:%d Y:%d.\n", // Re-enabled debug print
                      len, SD_DMA_BUFFER_SIZE, z, x, y);
        sqlite3_finalize(stmt);
        return false;
    }

    // Use the pre-allocated DMA buffer
    memcpy(sd_dma_buffer, blob, len);
    tileDataPtr = sd_dma_buffer; // Point to the pre-allocated buffer
    tileDataLen = len;
    ok = true;
  } else if (rc == SQLITE_DONE) {
    // Tile not found - silent (normal for empty areas)
    // Serial.printf("Data Task: Tile Z:%d X:%d Y:%d not found in DB.\n", z, x, y);
  } else {
    Serial.printf("❌ SQLite step failed for Z:%d X:%d Y:%d: %s\n", z, x, y, sqlite3_errmsg(db)); // Re-enabled debug print
  }

  sqlite3_finalize(stmt);
  return ok;
}


// =========================================================
// DATA TASK (Core 0)
// Fetches, parses, and manages map tiles based on requests
// =========================================================
void dataTask(void *pvParameters) {
//    Serial.println("Data Task: Running."); // Silent startup

    // Check if the DMA buffer was successfully allocated in setup()
    if (sd_dma_buffer == nullptr) {
        Serial.println("❌ Data Task: SD DMA buffer not allocated. Terminating task."); // Re-enabled debug print
        vTaskDelete(NULL);
    }

    char currentMbTilesPath[96] = ""; // Max path length for MBTiles file

    TileKey receivedTileRequest;
    bool tileParsedSuccess = true; // For notification to renderTask

    while (true) {
        // Wait indefinitely for a tile request
        if (xQueueReceive(tileRequestQueue, &receivedTileRequest, portMAX_DELAY) == pdPASS) {
            char newMbTilesPath[96];
            // Convert TMS Y to OSM Y for MBTiles file naming convention
            int y_osm = (1 << receivedTileRequest.z) - 1 - receivedTileRequest.y_tms;
            snprintf(newMbTilesPath, sizeof(newMbTilesPath), "/sdcard/tiles/%d_%d.mbtiles",
                     (int)floor(tileYToLat(y_osm, receivedTileRequest.z)),
                     (int)floor(tileXToLon(receivedTileRequest.x, receivedTileRequest.z)));

            // Check if the MBTiles file needs to be switched
            if (strcmp(newMbTilesPath, currentMbTilesPath) != 0) {
                if (mbtiles_db) {
                    sqlite3_close(mbtiles_db);
                    mbtiles_db = nullptr;
//                    Serial.printf("Data Task: Closed previous MBTiles DB: %s\n", currentMbTilesPath);
                }
//                Serial.printf("Data Task: Opening new MBTiles DB: %s\n", newMbTilesPath);
                if (sqlite3_open(newMbTilesPath, &mbtiles_db) != SQLITE_OK) {
                    Serial.printf("❌ Data Task: Failed to open MBTiles database: %s. Error: %s\n", newMbTilesPath, sqlite3_errmsg(mbtiles_db)); // Re-enabled debug print
                    mbtiles_db = nullptr;
                    tileParsedSuccess = false; // Mark as failure for notification
                } else {
                    strcpy(currentMbTilesPath, newMbTilesPath);
                    tileParsedSuccess = true; // Mark as success for notification
                }
            }

            // Proceed only if DB is open and previous operations were successful
            if (mbtiles_db && tileParsedSuccess) {
                uint8_t *tileDataBuffer = nullptr; // This will point to sd_dma_buffer
                size_t tileDataLen = 0;

                bool alreadyLoaded = false;
                // Check if tile is already in loadedTilesData (protected by mutex)
                if (xSemaphoreTake(loadedTilesDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    alreadyLoaded = (loadedTilesData.find(receivedTileRequest) != loadedTilesData.end());
                    xSemaphoreGive(loadedTilesDataMutex);
                } else {
                    Serial.println("❌ Data Task: Failed to acquire mutex for loadedTilesData during initial check."); // Re-enabled debug print
                    // If mutex cannot be acquired, assume not loaded to attempt fetch, but log error
                }

                if (!alreadyLoaded) {
                    if (fetchTile(mbtiles_db, receivedTileRequest.z, receivedTileRequest.x, receivedTileRequest.y_tms, tileDataBuffer, tileDataLen)) {
                        try {
                            // tileDataBuffer now points to the static sd_dma_buffer
                            parseMVTForTile(tileDataBuffer, tileDataLen, receivedTileRequest);
                            tileParsedSuccess = true;
                            tilesLoadedCount++; // Increment for boot screen progress
                        } catch (const std::bad_alloc& e) {
                            Serial.printf("❌ Data Task: Caught std::bad_alloc during parseMVTForTile: %s\n", e.what()); // Re-enabled debug print
                            tileParsedSuccess = false;
                        } catch (...) {
                            Serial.println("❌ Data Task: Caught unknown exception during parseMVTForTile."); // Re-enabled debug print
                            tileParsedSuccess = false;
                        }
                        // No need to free tileDataBuffer here, as it's the static sd_dma_buffer
                    } else {
                        // Fetch failed silently (may be empty tile)
                        // Serial.printf(\"❌ Data Task: Failed to fetch tile\\n\");
                        tileParsedSuccess = false;
                    }
                } else {
                    // Already loaded - silent
                    tileParsedSuccess = true;
                }
            } else {
                // DB was not open or previous DB open failed, so parsing cannot proceed
                tileParsedSuccess = false;
            }

            // Send notification to render task about parsing success/failure
            if (xQueueSend(tileParsedNotificationQueue, &tileParsedSuccess, pdMS_TO_TICKS(50)) != pdPASS) {
                Serial.println("❌ Data Task: Failed to send tile parsed notification to queue. Queue full?"); // Re-enabled debug print
            }

            // IMPORTANT: Free the raw tile data buffer after it has been parsed
            // This ensures the memory allocated for the raw MVT data is released.
            // tileDataBuffer points to sd_dma_buffer, which is a global buffer.
            // We only need to free it if it was dynamically allocated within fetchTile,
            // but in this case, fetchTile uses the global sd_dma_buffer.
            // So, no explicit free for tileDataBuffer here, as it's a shared global buffer.
            // The PSRAM for parsedLayers is managed by PSRAMAllocator and will be freed
            // when entries are removed from loadedTilesData map.
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // Yield to other tasks
    }
}

// Helper functions for tile coordinates (needed by dataTask to determine MBTiles file)
double tileXToLon(int x, int z) {
    return (x / pow(2.0, z) * 360.0) - 180.0;
}

double tileYToLat(int y, int z) {
    double n = PI - (2.0 * PI * y) / pow(2.0, z);
    return (180.0 / PI) * atan(0.5 * (exp(n) - exp(-n)));
}
