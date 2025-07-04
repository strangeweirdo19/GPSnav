// data_task.cpp
#include "data_task.h"
#include "common.h"

// Global SQLite DB handle (careful with this for multi-threading, but fine for single-threaded loop)
static sqlite3 *mbtiles_db = nullptr; // Changed to static, managed internally by dataTask

// =========================================================
// MVT DECODING HELPER FUNCTIONS
// =========================================================

// Decodes a variable-length integer (Varint) as per Protocol Buffers
inline uint64_t varint(const uint8_t *data, size_t &i, size_t dataSize) {
  uint64_t result = 0;
  int shift = 0;
  // Ensure we don't read beyond the data buffer
  while (i < dataSize) {
    uint8_t byte = data[i++];
    result |= (uint64_t)(byte & 0x7F) << shift;
    if (!(byte & 0x80)) break; // Stop if MSB is not set (end of varint)
    shift += 7;
  }
  return result;
}

// Decodes a ZigZag-encoded integer (used for signed values in MVT geometry)
inline int64_t zigzag(uint64_t n) {
  return (n >> 1) ^ -(n & 1);
}

// Reads a length-prefixed string from the MVT data and allocates it in PSRAM
PSRAMString readPSRAMString(const uint8_t *data, size_t &i, size_t dataSize) {
  uint64_t len = varint(data, i, dataSize); // Get string length
  if (i + len > dataSize) { // Prevent reading past buffer
      len = dataSize - i;
  }
  // Construct PSRAMString with PSRAMAllocator
  PSRAMString str{PSRAMAllocator<char>()}; // Corrected initialization
  str.reserve(len); // Pre-allocate memory for efficiency
  for (uint64_t j = 0; j < len; j++) {
    str += (char)data[i++];
  }
  return str;
}

// =========================================================
// MVT LAYER AND TILE DECODING (Data Task will use this)
// =========================================================

// Constants for MVT Protocol Buffer fields
enum MVT_Layer_Fields {
    MVT_LAYER_NAME = 1,
    MVT_LAYER_FEATURE = 2,
    MVT_LAYER_KEYS = 3,
    MVT_LAYER_VALUES = 4,
    MVT_LAYER_EXTENT = 5
};

enum MVT_Feature_Fields {
    MVT_FEATURE_ID = 1,
    MVT_FEATURE_TAGS = 2,
    MVT_FEATURE_GEOMETRY_TYPE = 3,
    MVT_FEATURE_GEOMETRY = 4
};

enum MVT_Geometry_Cmds {
    MVT_CMD_MOVETO = 1,
    MVT_CMD_LINETO = 2,
    MVT_CMD_CLOSEPATH = 7
};


// Parses a single layer from the MVT data into our structured format.
// This is called only once when a new tile is loaded.
ParsedLayer parseLayer(const uint8_t *data, size_t len) {
  ParsedLayer layer;
  size_t i = 0;
  // Use PSRAMString for layer keys and values
  std::vector<PSRAMString, PSRAMAllocator<PSRAMString>> layerKeys{PSRAMAllocator<PSRAMString>()};
  std::vector<PSRAMString, PSRAMAllocator<PSRAMString>> layerValues{PSRAMAllocator<PSRAMString>()};

  std::vector<std::pair<size_t, size_t>> featureOffsets; // Temporary, uses internal heap

  // First pass: Parse layer metadata (name, extent, keys, values) and collect feature offsets
  while (i < len) {
    uint64_t tag = varint(data, i, len);
    int field = tag >> 3;
    int type = tag & 0x07;

    switch (field) {
      case MVT_LAYER_NAME: layer.name = readPSRAMString(data, i, len); break;
      case MVT_LAYER_FEATURE: {
        size_t featureLen = varint(data, i, len);
        featureOffsets.push_back({i, featureLen});
        i += featureLen;
        break;
      }
      case MVT_LAYER_KEYS: layerKeys.push_back(readPSRAMString(data, i, len)); break;
      case MVT_LAYER_VALUES: {
        size_t value_block_len = varint(data, i, len); // Length of the entire value block
        size_t value_block_end = i + value_block_len;
        while (i < value_block_end) {
          uint64_t tag2 = varint(data, i, value_block_end); // Use value_block_end as boundary
          int f = tag2 >> 3, t = tag2 & 0x07;
          if (f == 1 && t == 2) { // string_value (field 1, wire type 2 = Length-delimited)
            layerValues.push_back(readPSRAMString(data, i, value_block_end)); // Use value_block_end as boundary
          } else { // Skip other unknown value types
            if (t == 0) varint(data, i, value_block_end);
            else if (t == 2) i += varint(data, i, value_block_end);
            else if (t == 5) i += 4;
            else if (t == 1) i += 8;
          }
          vTaskDelay(0); // Yield after processing each value
        }
        break;
      }
      case MVT_LAYER_EXTENT: layer.extent = varint(data, i, len); break;
      default: // Skip unknown layer fields
        if (type == 0) varint(data, i, len);
        else if (type == 2) i += varint(data, i, len);
        else if (type == 5) i += 4;
        else if (type == 1) i += 8;
        break;
    }
    vTaskDelay(0); // Yield after processing each layer field
  }

  // Second pass: Parse each feature's properties and geometry
  for (const auto &offset : featureOffsets) {
    size_t fi = offset.first;
    size_t fend = fi + offset.second;
    
    ParsedFeature feature; // This object is created on stack, its vectors use PSRAM
    feature.isPolygon = false;
    feature.minX_mvt = layer.extent + 1; // Initialize with values outside possible range
    feature.minY_mvt = layer.extent + 1;
    feature.maxX_mvt = -1;
    feature.maxY_mvt = -1;

    size_t currentFeatureIdx = fi;

    while (currentFeatureIdx < fend) {
        uint64_t tag = varint(data, currentFeatureIdx, fend);
        int field = tag >> 3;
        int type = tag & 0x07;

        if (field == MVT_FEATURE_GEOMETRY_TYPE && type == 0) {
            feature.geomType = varint(data, currentFeatureIdx, fend);
            if (feature.geomType == 3) feature.isPolygon = true;
        } else if (field == MVT_FEATURE_GEOMETRY && type == 2) {
            size_t geomDataLen = varint(data, currentFeatureIdx, fend);
            size_t geomDataEnd = currentFeatureIdx + geomDataLen;

            int x = 0, y = 0;
            // This currentRing is temporary, it uses PSRAMAllocator now
            std::vector<std::pair<int, int>, PSRAMAllocator<std::pair<int, int>>> currentRing{PSRAMAllocator<std::pair<int, int>>()};
            while (currentFeatureIdx < geomDataEnd) {
                uint64_t cmdlen = varint(data, currentFeatureIdx, geomDataEnd);
                int cmd = cmdlen & 0x7;
                int count = cmdlen >> 3;

                switch (cmd) {
                    case MVT_CMD_MOVETO:
                        if (!currentRing.empty()) {
                            feature.geometryRings.push_back(currentRing);
                            currentRing.clear();
                        }
                        for (int j = 0; j < count; ++j) {
                            x += zigzag(varint(data, currentFeatureIdx, geomDataEnd));
                            y += zigzag(varint(data, currentFeatureIdx, geomDataEnd));
                            currentRing.push_back({x, y});
                            // Update feature bounding box in MVT coordinates
                            if (x < feature.minX_mvt) feature.minX_mvt = x;
                            if (x > feature.maxX_mvt) feature.maxX_mvt = x;
                            if (y < feature.minY_mvt) feature.minY_mvt = y;
                            if (y > feature.maxY_mvt) feature.maxY_mvt = y;
                            vTaskDelay(0); // Yield after each coordinate pair
                        }
                        break;
                    case MVT_CMD_LINETO:
                        for (int j = 0; j < count; ++j) {
                            x += zigzag(varint(data, currentFeatureIdx, geomDataEnd));
                            y += zigzag(varint(data, currentFeatureIdx, geomDataEnd));
                            currentRing.push_back({x, y});
                            // Update feature bounding box in MVT coordinates
                            if (x < feature.minX_mvt) feature.minX_mvt = x;
                            if (x > feature.maxX_mvt) feature.maxX_mvt = x;
                            if (y < feature.minY_mvt) feature.minY_mvt = y;
                            if (y > feature.maxY_mvt) feature.maxY_mvt = y;
                            vTaskDelay(0); // Yield after each coordinate pair
                        }
                        break;
                    case MVT_CMD_CLOSEPATH:
                        if (!currentRing.empty()) {
                            if (currentRing.front() != currentRing.back()) {
                                currentRing.push_back(currentRing.front());
                            }
                            feature.geometryRings.push_back(currentRing);
                            currentRing.clear();
                        }
                        vTaskDelay(0); // Yield after closing path
                        break;
                    default: // Unknown command
                        // Skip remaining coordinates for this command
                        for(int j=0; j<count; ++j) {
                            varint(data, currentFeatureIdx, geomDataEnd); // x
                            varint(data, currentFeatureIdx, geomDataEnd); // y
                            vTaskDelay(0); // Yield after each skipped coordinate
                        }
                        break;
                }
            }
            if (!currentRing.empty()) feature.geometryRings.push_back(currentRing);
        } else if (field == MVT_FEATURE_TAGS && type == 2) {
            size_t propLen = varint(data, currentFeatureIdx, fend);
            size_t propEnd = currentFeatureIdx + propLen;
            while (currentFeatureIdx < propEnd) {
                uint64_t keyIdx = varint(data, currentFeatureIdx, propEnd);
                uint64_t valIdx = varint(data, currentFeatureIdx, propEnd);
                if (keyIdx < layerKeys.size() && valIdx < layerValues.size()) {
                    feature.properties[layerKeys[keyIdx]] = layerValues[valIdx];
                }
                vTaskDelay(0); // Yield after each property tag
            }
        } else if (field == MVT_FEATURE_ID && type == 0) {
            varint(data, currentFeatureIdx, fend);
        } else {
            if (type == 0) varint(data, currentFeatureIdx, fend);
            else if (type == 2) currentFeatureIdx += varint(data, currentFeatureIdx, fend);
            else if (type == 5) currentFeatureIdx += 4;
            else if (type == 1) currentFeatureIdx += 8;
        }
        vTaskDelay(0); // Yield after processing each feature field
    }

    // --- COLOR ASSIGNMENT LOGIC (done once during parsing) ---
    feature.color = TFT_WHITE; // Default color
    if (layer.name == "water") {
        feature.color = TFT_BLUE;
        feature.isPolygon = true;
    } else if (layer.name == "landcover") {
        if (feature.properties.count("class")) {
            PSRAMString lcClass = feature.properties.at("class"); // Use .at() for PSRAMString map
            if (lcClass == "forest") {
                feature.color = TFT_DARKGREEN;
                feature.isPolygon = true;
            } else if (lcClass == "grass") {
                feature.color = TFT_GREEN;
                feature.isPolygon = true;
            } else if (lcClass == "wetland") {
                feature.color = TFT_BLUE; // Wetlands often appear greenish-blue
                feature.isPolygon = true;
            }
        }
    } else if (layer.name == "landuse") {
        if (feature.properties.count("class")) {
            PSRAMString luClass = feature.properties.at("class"); // Use .at() for PSRAMString map
            if (luClass == "residential") {
                feature.color = TFT_LIGHTGREY; // Residential often lighter
                feature.isPolygon = true;
            } else if (luClass == "commercial") {
                feature.color = TFT_ORANGE;
                feature.isPolygon = true;
            } else if (luClass == "industrial") {
                feature.color = TFT_BROWN;
                feature.isPolygon = true;
            }
        }
    } else if (layer.name == "road" || layer.name == "transportation") {
        feature.color = TFT_DARKGREY;
        if (feature.properties.count("class")) {
            PSRAMString transportClass = feature.properties.at("class"); // Use .at() for PSRAMString map
            if (transportClass == "motorway" || transportClass == "trunk") {
                feature.color = TFT_RED;
            } else if (transportClass == "street" || transportClass == "primary" || transportClass == "secondary") {
                feature.color = TFT_LIGHTGREY;
            } else if (transportClass == "rail") {
                feature.color = TFT_DARKCYAN;
            }
        } else if (feature.properties.count("highway")) { // common in OpenStreetMap data
             PSRAMString highwayType = feature.properties.at("highway"); // Use .at() for PSRAMString map
             if (highwayType == "motorway" || highwayType == "trunk") {
                 feature.color = TFT_RED;
             } else if (highwayType == "primary" || highwayType == "secondary") {
                 feature.color = TFT_ORANGE;
             } else if (highwayType == "tertiary" || highwayType == "residential" || highwayType == "unclassified") {
                 feature.color = TFT_LIGHTGREY;
             } else if (highwayType == "footway" || highwayType == "path") {
                 feature.color = TFT_GREENYELLOW;
             }
        }
    } else if (layer.name == "transportation_name") {
        feature.color = TFT_CYAN;
    } else if (layer.name == "place") {
        feature.color = TFT_MAGENTA;
        if (feature.geomType == 3) {
            feature.isPolygon = true;
        }
    } else if (layer.name == "poi") {
        feature.color = TFT_YELLOW;
        if (feature.properties.count("class")) {
            PSRAMString poiClass = feature.properties.at("class"); // Use .at() for PSRAMString map
            if (poiClass == "park") {
                feature.color = TFT_GREEN;
                feature.isPolygon = true;
            } else if (poiClass == "building") {
                feature.color = TFT_SILVER;
                feature.isPolygon = true;
            } else if (poiClass == "hospital") {
                feature.color = TFT_RED;
            }
        }
    }
    // --- END COLOR ASSIGNMENT LOGIC ---

    layer.features.push_back(feature); // This push_back will use PSRAMAllocator for ParsedFeature
    vTaskDelay(0); // Yield after processing each feature
  }
  return layer;
}

// Parses the entire MVT tile into our structured format and stores it under the given TileKey.
void parseMVTForTile(const uint8_t *data_buffer, size_t data_len, const TileKey& key) {
  // This vector is temporary and will be copied into loadedTilesData (which uses PSRAM)
  std::vector<ParsedLayer, PSRAMAllocator<ParsedLayer>> currentTileLayers{PSRAMAllocator<ParsedLayer>()};
  size_t i = 0;
  while (i < data_len) {
    uint64_t tag = varint(data_buffer, i, data_len);
    int field = tag >> 3;
    int type = tag & 0x07;

    if (field == 3 && type == 2) { // Layer field (tag 3, wire type 2 = Length-delimited)
      size_t len = varint(data_buffer, i, data_len); // Get layer data length
      currentTileLayers.push_back(parseLayer(data_buffer + i, len)); // Parse the layer and add to our vector
      i += len;                            // Advance index past this layer's data
      vTaskDelay(1); // Yield after parsing each layer to prevent watchdog timeout
    } else if (type == 2) { // Generic length-delimited field (skip if not layer)
      size_t len = varint(data_buffer, i, data_len);
      i += len;
    } else { // Generic fixed-length or varint field (skip)
      varint(data_buffer, i, data_len); // Just consume its value
    }
    vTaskDelay(0); // Yield after processing each top-level field in the MVT tile
  }
  // After parsing, grab the extent from the first layer (assuming it's consistent across all layers/tiles)
  if (!currentTileLayers.empty()) {
      // Acquire mutex before writing to shared data
      if (xSemaphoreTake(loadedTilesDataMutex, portMAX_DELAY) == pdTRUE) {
          // This assignment will trigger deep copy using PSRAMAllocator for elements
          loadedTilesData[key] = currentTileLayers;
          currentLayerExtent = currentTileLayers[0].extent;
          xSemaphoreGive(loadedTilesDataMutex); // Release mutex
      } else {
          Serial.println("❌ Data Task: Failed to acquire mutex for loadedTilesData during parsing.");
      }
  }
}


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
    Serial.printf("❌ SQLite prepare failed: %s. Please ensure your MBTiles file is valid and contains a 'tiles' table.\n", sqlite3_errmsg(db));
    return false;
  }
  
  sqlite3_bind_int(stmt, 1, z);
  sqlite3_bind_int(stmt, 2, x);
  sqlite3_bind_int(stmt, 3, y); // Bind the TMS Y coordinate

  bool ok = false;
  int rc = sqlite3_step(stmt);
  if (rc == SQLITE_ROW) {
    const void *blob = sqlite3_column_blob(stmt, 0);
    int len = sqlite3_column_bytes(stmt, 0);
    
    // Allocate buffer in PSRAM
    tileDataPtr = (uint8_t*) heap_caps_malloc(len, MALLOC_CAP_SPIRAM);
    if (tileDataPtr == nullptr) {
        Serial.printf("❌ Failed to allocate %d bytes for tile data in PSRAM for Z:%d X:%d Y:%d\n", len, z, x, y);
        sqlite3_finalize(stmt);
        return false;
    }
    memcpy(tileDataPtr, blob, len);
    tileDataLen = len;
    ok = true;
  } else if (rc == SQLITE_DONE) {
    // Tile not found is not an error, just means no data for this tile
  } else {
    Serial.printf("❌ SQLite step failed for Z:%d X:%d Y:%d: %s\n", z, x, y, sqlite3_errmsg(db));
  }
  
  sqlite3_finalize(stmt);
  return ok;
}


// =========================================================
// DATA TASK (Core 0)
// Fetches, parses, and manages map tiles based on requests
// =========================================================
void dataTask(void *pvParameters) {
    if (!SD_MMC.begin()) {
        Serial.println("❌ Data Task: SD_MMC mount failed. Check wiring and formatting.");
        vTaskDelete(NULL); // Delete this task if SD fails
    }

    // Keep track of the currently open MBTiles file path
    char currentMbTilesPath[96] = "";

    TileKey receivedTileRequest;
    bool tileParsedSuccess = true; // For notification to renderTask

    while (true) {
        // Wait indefinitely for a tile request from the render task
        if (xQueueReceive(tileRequestQueue, &receivedTileRequest, portMAX_DELAY) == pdPASS) {
            char newMbTilesPath[96];
            // Determine the MBTiles file based on the tile's coordinates (e.g., for global coverage)
            // Convert TMS Y to standard (OSM) Y for correct latitude calculation
            int y_osm = (1 << receivedTileRequest.z) - 1 - receivedTileRequest.y_tms;
            snprintf(newMbTilesPath, sizeof(newMbTilesPath), "/sdcard/tile/%d_%d.mbtiles", 
                     (int)floor(tileYToLat(y_osm, receivedTileRequest.z)), 
                     (int)floor(tileXToLon(receivedTileRequest.x, receivedTileRequest.z)));

            // --- DEBUG PRINT: Show the generated MBTiles file path ---
            Serial.printf("Data Task: Attempting to open MBTiles file: %s\n", newMbTilesPath);
            // --- END DEBUG PRINT ---

            // If the requested tile is from a different MBTiles file, close the old one and open the new one
            if (strcmp(newMbTilesPath, currentMbTilesPath) != 0) {
                if (mbtiles_db) {
                    sqlite3_close(mbtiles_db);
                    mbtiles_db = nullptr;
                }
                if (sqlite3_open(newMbTilesPath, &mbtiles_db) != SQLITE_OK) {
                    Serial.printf("❌ Data Task: Failed to open MBTiles database: %s\n", sqlite3_errmsg(mbtiles_db));
                    mbtiles_db = nullptr;
                    Serial.println("Data Task: Error: Could not open MBTiles file. Please ensure it exists on the SD card.");
                    tileParsedSuccess = false; // Mark as failed
                } else {
                    strcpy(currentMbTilesPath, newMbTilesPath);
                    tileParsedSuccess = true;
                }
            }

            if (mbtiles_db && tileParsedSuccess) { // Only proceed if DB is open and valid
                uint8_t *tileDataBuffer = nullptr;
                size_t tileDataLen = 0;

                // Check if the tile is already loaded in shared memory
                bool alreadyLoaded = false;
                if (xSemaphoreTake(loadedTilesDataMutex, 0) == pdTRUE) { // Try to get mutex without blocking
                    alreadyLoaded = (loadedTilesData.find(receivedTileRequest) != loadedTilesData.end());
                    xSemaphoreGive(loadedTilesDataMutex);
                }

                if (!alreadyLoaded) {
                    if (fetchTile(mbtiles_db, receivedTileRequest.z, receivedTileRequest.x, receivedTileRequest.y_tms, tileDataBuffer, tileDataLen)) {
                        parseMVTForTile(tileDataBuffer, tileDataLen, receivedTileRequest);
                        heap_caps_free(tileDataBuffer);
                        tileParsedSuccess = true;
                    } else {
                        Serial.printf("❌ Data Task: Failed to fetch tile Z:%d X:%d Y:%d.\n", 
                                      receivedTileRequest.z, receivedTileRequest.x, receivedTileRequest.y_tms);
                        tileParsedSuccess = false;
                    }
                } else {
                    // Tile already loaded, no need to fetch/parse again
                    tileParsedSuccess = true;
                }
            }

            // Notify render task that a tile was processed (success or failure)
            // Block for a short time if queue is full to ensure notification is sent
            if (xQueueSend(tileParsedNotificationQueue, &tileParsedSuccess, pdMS_TO_TICKS(50)) != pdPASS) {
                Serial.println("❌ Data Task: Failed to send tile parsed notification to queue. Queue full?");
            }
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
