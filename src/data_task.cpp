// data_task.cpp
#include "data_task.h"
#include "common.h"

// Global SQLite DB handle (careful with this for multi-threading, but fine for single-threaded loop)
sqlite3 *mbtiles_db = nullptr; // This will now be opened/closed per animation sequence

// HMC5883L compass object - changed to Adafruit_HMC5883_Unified
Adafruit_HMC5883_Unified hmc5883l = Adafruit_HMC5883_Unified(12345); // Using a sensor ID, common for unified sensors

// Global variables for compass heading filtering (moving average)
std::vector<float> headingReadings; // This vector is small, can stay in internal RAM
const int FILTER_WINDOW_SIZE = 10; // Number of readings to average

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

  // Serial.printf("🗂️ Parsing Layer: %s (Features: %d, Extent: %d)\n", layer.name.c_str(), featureOffsets.size(), layer.extent);

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
          Serial.printf("Data Task: Added tile Z:%d X:%d Y:%d to loadedTilesData. Current size: %d\n", key.z, key.x, key.y_tms, loadedTilesData.size()); // Debug print
          // Update currentLayerExtent from the newly parsed tile (assuming consistency across tiles)
          currentLayerExtent = currentTileLayers[0].extent;
          xSemaphoreGive(loadedTilesDataMutex); // Release mutex
      } else {
          Serial.println("❌ Failed to acquire mutex for loadedTilesData during parsing.");
      }
  }
}


// =========================================================
// MBTILES DATABASE AND COORDINATE HELPERS
// =========================================================

// Converts Latitude/Longitude to XYZ tile coordinates (and TMS Y coordinate)
void latlonToTile(double lat, double lon, int z, int &x, int &y, int &ytms) {
  double latRad = radians(lat);
  int n = 1 << z; // Number of tiles in one dimension at zoom level z
  x = int((lon + 180.0) / 360.0 * n);
  y = int((1.0 - log(tan(latRad) + 1.0 / cos(latRad)) / PI) / 2.0 * n);
  ytms = n - 1 - y; // Convert standard Y to TMS Y (used by MBTiles)
}

// Converts Latitude/Longitude to pixel coordinates (0-255) within its containing tile
// And then scales it to the MVT extent (e.g., 0-4095)
void latLonToMVTCoords(double lat, double lon, int z, int tileX, int tileY_TMS, int& mvtX, int& mvtY, int extent) {
    double n = pow(2, z);
    double latRad = radians(lat);

    // Global pixel coordinates at Z zoom level (assuming 256x256 pixel tiles for standard mapping)
    double globalPx = ((lon + 180.0) / 360.0) * n * 256;
    double globalPy = (1.0 - log(tan(latRad) + 1.0 / cos(latRad)) / PI) / 2.0 * n * 256;

    // Pixel coordinate within the current tile (0-255 range for 256x256 pixel tile)
    // By taking modulo of global pixel coordinates with tile size (256)
    double pixelInTileX_256 = fmod(globalPx, 256.0);
    double pixelInTileY_256 = fmod(globalPy, 256.0);

    // If globalPx or globalPy are very close to a tile boundary, fmod might give something like 255.999...
    // or 0.000... at the other end. Ensure it's correctly within [0, 256) or [0, 4096)
    if (pixelInTileX_256 < 0) pixelInTileX_256 += 256;
    if (pixelInTileY_256 < 0) pixelInTileY_256 += 256;

    // Convert pixel-in-tile (0-255) to MVT extent coordinates (0-extent)
    mvtX = round(pixelInTileX_256 * (extent / 256.0));
    mvtY = round(pixelInTileY_256 * (extent / 256.0));

    // Clamp to extent boundaries if floating point math results in slightly out of bounds values
    mvtX = std::max(0, std::min(extent - 1, mvtX));
    mvtY = std::max(0, std::min(extent - 1, mvtY));
}


// Fetches tile data (BLOB) from the SQLite MBTiles database
// This function should only be called from the dataTask, ensuring single-threaded DB access.
// tileDataPtr will be allocated in PSRAM, the caller is responsible for freeing it.
bool fetchTile(sqlite3 *db, int z, int x, int y, uint8_t *&tileDataPtr, size_t &tileDataLen) {
  sqlite3_stmt *stmt; // Prepared statement object
  const char *sql = "SELECT tile_data FROM tiles WHERE zoom_level=? AND tile_column=? AND tile_row=?;";
  
  if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    Serial.printf("❌ SQLite prepare failed: %s\n", sqlite3_errmsg(db));
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
    Serial.printf("❌ Tile not found in DB for Z:%d X:%d Y:%d.\n", z, x, y);
  } else {
    Serial.printf("❌ SQLite step failed for Z:%d X:%d Y:%d: %s\n", z, x, y, sqlite3_errmsg(db));
  }
  
  sqlite3_finalize(stmt);
  return ok;
}


// =========================================================
// DATA/GPS TASK (Core 0)
// Manages tile loading and parsing based on current location
// =========================================================
void dataTask(void *pvParameters) {
    // Initialize SD_MMC and SQLite DB here, as this task will manage them
    Serial.println("Data Task: Initializing SD_MMC...");
    if (!SD_MMC.begin()) {
        Serial.println("❌ Data Task: SD_MMC mount failed. Check wiring and formatting.");
        vTaskDelete(NULL); // Delete this task if SD fails
    }
    Serial.println("✅ Data Task: SD_MMC mounted successfully.");

    // Initialize HMC5883L compass
    Serial.println("Data Task: Initializing HMC5883L compass...");
    Wire.begin(); // Initialize I2C
    if (!hmc5883l.begin()) {
        Serial.println("❌ Data Task: Could not find a valid HMC5883L sensor, check wiring!");
    } else {
        Serial.println("✅ Data Task: HMC5883L sensor found.");
        hmc5883l.setMagGain(HMC5883_MAGGAIN_1_3); // Set gain to +/- 1.3 Gauss (default)
    }


    // Internal variables for dataTask to manage current state
    double internalCurrentTargetLat = 12.8273; // Default starting coordinates
    double internalCurrentTargetLon = 80.2193; // Default starting coordinates
    float internalCurrentZoomFactor = 1.0; // Default zoom factor
    float internalCurrentRotationAngle = 0.0f; // New: to store current rotation from compass
    float lastSentRotationAngle = 0.0f; // Stores the last rotation angle sent to render task
    float lastSentZoomFactor = 0.0f; // Stores the last zoom factor sent to render task
    bool forceRenderUpdate = true; // New flag to force initial render update

    // State variables for loading management
    enum LoadingPhase {
        IDLE,
        LOADING_CENTER,
        LOADING_PRIMARY_GRID,
        LOADING_SECONDARY_GRID
    } currentLoadingPhase = IDLE;

    TileKey currentRequestedCenterTile = {-1, -1, -1}; // The tile that was last requested by user input
    TileKey currentlyLoadedCenterTile = {-2, -2, -2}; // Initialize to a different invalid value to ensure initial render trigger

    // Queues/vectors for tiles to be loaded in each phase, ordered by priority
    std::vector<TileKey> tilesToLoadPrimary; // All 8 direct neighbors
    std::vector<TileKey> tilesToLoadSecondary; // All 16 outer ring tiles of 5x5 grid

    // MVT pixel coordinates of the target point relative to the central tile's MVT extent.
    int internalTargetPointMVT_X = 0;
    int internalTargetPointMVT_Y = 0;

    ControlParams receivedControlParams;

    // Explicitly trigger initial loading phase by setting the first requested tile
    int initialCentralTileX, initialCentralTileY_std, initialCentralTileY_TMS;
    latlonToTile(internalCurrentTargetLat, internalCurrentTargetLon, currentTileZ, initialCentralTileX, initialCentralTileY_std, initialCentralTileY_TMS);
    currentRequestedCenterTile = {currentTileZ, initialCentralTileX, initialCentralTileY_TMS};
    currentLoadingPhase = LOADING_CENTER;
    Serial.printf("Data Task: Initial central tile set to Z:%d X:%d Y:%d (TMS). Starting in LOADING_CENTER phase.\n",
                  currentRequestedCenterTile.z, currentRequestedCenterTile.x, currentRequestedCenterTile.y_tms);


    while (true) {
        unsigned long taskLoopStartTime = millis();

        // Log free heap memory at the start of the loop
        Serial.printf("Data Task: Free Internal Heap: %u bytes, Largest Internal Block: %u bytes\n",
                      heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                      heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
        Serial.printf("Data Task: Free PSRAM: %u bytes, Largest PSRAM Block: %u bytes\n",
                      heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                      heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
        Serial.printf("Data Task: Largest Internal DMA-capable Block: %u bytes\n",
                      heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));


        // Check for new control parameters from the main loop
        if (xQueueReceive(controlParamsQueue, &receivedControlParams, 0) == pdPASS) {
            internalCurrentTargetLat = receivedControlParams.targetLat;
            internalCurrentTargetLon = receivedControlParams.targetLon;
            internalCurrentZoomFactor = receivedControlParams.zoomFactor;
            
            // Calculate the new central tile based on received coordinates
            int newCentralTileX, newCentralTileY_std, newCentralTileY_TMS;
            latlonToTile(internalCurrentTargetLat, internalCurrentTargetLon, currentTileZ, newCentralTileX, newCentralTileY_std, newCentralTileY_TMS);
            
            TileKey newRequestedTile = {currentTileZ, newCentralTileX, newCentralTileY_TMS};

            if (!(newRequestedTile == currentRequestedCenterTile)) { // If a new center tile is requested
                currentRequestedCenterTile = newRequestedTile;
                currentLoadingPhase = LOADING_CENTER; // Reset loading phase
                Serial.printf("Data Task: New central tile requested: Z:%d X:%d Y:%d (TMS). Resetting loading.\n",
                              newRequestedTile.z, newRequestedTile.x, newRequestedTile.y_tms);
                
                // Clear previous loading queues/sets
                tilesToLoadPrimary.clear();
                tilesToLoadSecondary.clear();

                // Close existing DB connection if any
                if (mbtiles_db) {
                    sqlite3_close(mbtiles_db);
                    mbtiles_db = nullptr;
                    Serial.println("Data Task: MBTiles database closed (previous).");
                }
            }
        }

        // Read compass data
        sensors_event_t event;
        hmc5883l.getEvent(&event);

        // Calculate raw heading in degrees (0-360)
        float rawHeading = atan2(event.magnetic.y, event.magnetic.x);
        rawHeading = rawHeading * 180 / PI; // Convert radians to degrees
        if (rawHeading < 0) {
            rawHeading += 360; // Normalize to 0-360 degrees
        }

        // Apply moving average filter to heading using sine/cosine components for smooth wrap-around
        headingReadings.push_back(rawHeading);
        if (headingReadings.size() > FILTER_WINDOW_SIZE) {
            headingReadings.erase(headingReadings.begin()); // Remove oldest reading
        }

        float sumSin = 0.0f;
        float sumCos = 0.0f;
        for (float h : headingReadings) {
            sumSin += sin(radians(h));
            sumCos += cos(radians(h));
        }

        internalCurrentRotationAngle = degrees(atan2(sumSin / headingReadings.size(), sumCos / headingReadings.size()));
        if (internalCurrentRotationAngle < 0) {
            internalCurrentRotationAngle += 360; // Normalize to 0-360 degrees
        }

        // --- Tile Loading State Machine ---
        switch (currentLoadingPhase) {
            case IDLE:
                // If all tiles are loaded and no new request, just wait
                // This state will be re-entered if a new request comes in.
                break;

            case LOADING_CENTER: {
                Serial.printf("Data Task: Loading Phase: LOADING_CENTER (Z:%d X:%d Y:%d)\n",
                              currentRequestedCenterTile.z, currentRequestedCenterTile.x, currentRequestedCenterTile.y_tms);
                
                // Open DB for the current target location
                char path[96];
                snprintf(path, sizeof(path), "/sdcard/tile/%d_%d.mbtiles", (int)floor(internalCurrentTargetLat), (int)floor(internalCurrentTargetLon));
                Serial.printf("Data Task: Attempting to open MBTiles file: %s\n", path); // Added debug print for path

                if (sqlite3_open(path, &mbtiles_db) != SQLITE_OK) {
                    Serial.printf("❌ Data Task: Failed to open MBTiles database: %s\n", sqlite3_errmsg(mbtiles_db));
                    mbtiles_db = nullptr;
                    Serial.println("Data Task: Error: Could not open MBTiles file. Please ensure it exists on the SD card.");
                    vTaskDelay(pdMS_TO_TICKS(5000)); // Wait before retrying
                    currentLoadingPhase = IDLE; // Go back to idle or retry
                    break;
                }
                Serial.println("✅ Data Task: MBTiles database opened successfully for new location.");

                uint8_t *tileDataBuffer = nullptr;
                size_t tileDataLen = 0;
                // Check if the center tile is already loaded
                bool alreadyLoaded = false;
                if (xSemaphoreTake(loadedTilesDataMutex, portMAX_DELAY) == pdTRUE) {
                    alreadyLoaded = (loadedTilesData.find(currentRequestedCenterTile) != loadedTilesData.end());
                    xSemaphoreGive(loadedTilesDataMutex);
                }

                if (!alreadyLoaded) {
                    if (mbtiles_db && fetchTile(mbtiles_db, currentRequestedCenterTile.z, currentRequestedCenterTile.x, currentRequestedCenterTile.y_tms, tileDataBuffer, tileDataLen)) {
                        parseMVTForTile(tileDataBuffer, tileDataLen, currentRequestedCenterTile);
                        heap_caps_free(tileDataBuffer);
                        Serial.printf("Data Task: Loaded and parsed central tile Z:%d X:%d Y:%d.\n",
                                      currentRequestedCenterTile.z, currentRequestedCenterTile.x, currentRequestedCenterTile.y_tms);
                    } else {
                        Serial.printf("❌ Data Task: Failed to load central tile Z:%d X:%d Y:%d.\n",
                                      currentRequestedCenterTile.z, currentRequestedCenterTile.x, currentRequestedCenterTile.y_tms);
                    }
                } else {
                    Serial.printf("Data Task: Central tile Z:%d X:%d Y:%d already loaded.\n",
                                  currentRequestedCenterTile.z, currentRequestedCenterTile.x, currentRequestedCenterTile.y_tms);
                }
                currentlyLoadedCenterTile = currentRequestedCenterTile; // Update loaded center
                currentLoadingPhase = LOADING_PRIMARY_GRID; // Move to next phase
                forceRenderUpdate = true; // Force a render update after loading the center tile
                vTaskDelay(1); // Yield after processing center tile
                break;
            }

            case LOADING_PRIMARY_GRID: {
                Serial.println("Data Task: Loading Phase: LOADING_PRIMARY_GRID.");
                // Populate tilesToLoadPrimary if empty
                if (tilesToLoadPrimary.empty()) {
                    // Collect all 8 direct neighbors
                    for (int dx = -1; dx <= 1; ++dx) {
                        for (int dy = -1; dy <= 1; ++dy) {
                            if (dx == 0 && dy == 0) continue; // Skip central tile

                            int neighborX = currentlyLoadedCenterTile.x + dx;
                            int neighborY_TMS = currentlyLoadedCenterTile.y_tms + dy;
                            int maxTileCoord = (1 << currentTileZ) - 1;
                            if (neighborX >= 0 && neighborX <= maxTileCoord && neighborY_TMS >= 0 && neighborY_TMS <= maxTileCoord) {
                                TileKey neighborKey = {currentTileZ, neighborX, neighborY_TMS};
                                bool alreadyLoaded = false;
                                if (xSemaphoreTake(loadedTilesDataMutex, portMAX_DELAY) == pdTRUE) {
                                    alreadyLoaded = (loadedTilesData.find(neighborKey) != loadedTilesData.end());
                                    xSemaphoreGive(loadedTilesDataMutex);
                                }
                                if (!alreadyLoaded) {
                                    tilesToLoadPrimary.push_back(neighborKey);
                                }
                            }
                        }
                    }
                    // No specific sorting for primary tiles as they are all equally "primary" for a 3x3 grid
                }

                // Load one primary tile per loop iteration
                if (!tilesToLoadPrimary.empty()) {
                    TileKey tileToLoad = tilesToLoadPrimary.front();
                    tilesToLoadPrimary.erase(tilesToLoadPrimary.begin()); // Remove from queue

                    uint8_t *tileDataBuffer = nullptr;
                    size_t tileDataLen = 0;
                    if (mbtiles_db && fetchTile(mbtiles_db, tileToLoad.z, tileToLoad.x, tileToLoad.y_tms, tileDataBuffer, tileDataLen)) {
                        parseMVTForTile(tileDataBuffer, tileDataLen, tileToLoad);
                        heap_caps_free(tileDataBuffer);
                        Serial.printf("Data Task: Loaded and parsed primary tile Z:%d X:%d Y:%d.\n", tileToLoad.z, tileToLoad.x, tileToLoad.y_tms);
                        forceRenderUpdate = true; // Force a render update after each primary tile is loaded
                    } else {
                        Serial.printf("❌ Data Task: Failed to load primary tile Z:%d X:%d Y:%d.\n", tileToLoad.z, tileToLoad.x, tileToLoad.y_tms);
                    }
                    vTaskDelay(1); // Yield after processing each primary tile
                } else {
                    // All primary tiles processed. Move to secondary loading.
                    currentLoadingPhase = LOADING_SECONDARY_GRID;
                    Serial.println("Data Task: All primary tiles processed. Moving to secondary loading.");
                }
                break;
            }

            case LOADING_SECONDARY_GRID: {
                Serial.println("Data Task: Loading Phase: LOADING_SECONDARY_GRID.");
                // Populate tilesToLoadSecondary if empty
                if (tilesToLoadSecondary.empty()) {
                    // Collect all 16 outer ring tiles of the 5x5 grid
                    for (int dx = -2; dx <= 2; ++dx) {
                        for (int dy = -2; dy <= 2; ++dy) {
                            if (abs(dx) <= 1 && abs(dy) <= 1) continue; // Skip the 3x3 primary grid

                            int neighborX = currentlyLoadedCenterTile.x + dx;
                            int neighborY_TMS = currentlyLoadedCenterTile.y_tms + dy;
                            int maxTileCoord = (1 << currentTileZ) - 1;
                            if (neighborX >= 0 && neighborX <= maxTileCoord && neighborY_TMS >= 0 && neighborY_TMS <= maxTileCoord) {
                                TileKey neighborKey = {currentTileZ, neighborX, neighborY_TMS};
                                bool alreadyLoaded = false;
                                if (xSemaphoreTake(loadedTilesDataMutex, portMAX_DELAY) == pdTRUE) {
                                    alreadyLoaded = (loadedTilesData.find(neighborKey) != loadedTilesData.end());
                                    xSemaphoreGive(loadedTilesDataMutex);
                                }
                                if (!alreadyLoaded) {
                                    tilesToLoadSecondary.push_back(neighborKey);
                                }
                            }
                        }
                    }
                    // Sort tilesToLoadSecondary by distance to the currentlyLoadedCenterTile
                    std::sort(tilesToLoadSecondary.begin(), tilesToLoadSecondary.end(),
                              [&](const TileKey& a, const TileKey& b) {
                                  // Calculate squared Euclidean distance from currentlyLoadedCenterTile
                                  // (dx*dx + dy*dy) is sufficient for comparison, no need for sqrt
                                  long long distSqA = (long long)(a.x - currentlyLoadedCenterTile.x) * (a.x - currentlyLoadedCenterTile.x) +
                                                      (long long)(a.y_tms - currentlyLoadedCenterTile.y_tms) * (a.y_tms - currentlyLoadedCenterTile.y_tms);
                                  long long distSqB = (long long)(b.x - currentlyLoadedCenterTile.x) * (b.x - currentlyLoadedCenterTile.x) +
                                                      (long long)(b.y_tms - currentlyLoadedCenterTile.y_tms) * (b.y_tms - currentlyLoadedCenterTile.y_tms);
                                  return distSqA < distSqB;
                              });
                }

                // Load one secondary tile per loop iteration
                if (!tilesToLoadSecondary.empty()) {
                    TileKey tileToLoad = tilesToLoadSecondary.front();
                    tilesToLoadSecondary.erase(tilesToLoadSecondary.begin()); // Remove from queue

                    uint8_t *tileDataBuffer = nullptr;
                    size_t tileDataLen = 0;
                    if (mbtiles_db && fetchTile(mbtiles_db, tileToLoad.z, tileToLoad.x, tileToLoad.y_tms, tileDataBuffer, tileDataLen)) {
                        parseMVTForTile(tileDataBuffer, tileDataLen, tileToLoad);
                        heap_caps_free(tileDataBuffer);
                        Serial.printf("Data Task: Loaded and parsed secondary tile Z:%d X:%d Y:%d.\n", tileToLoad.z, tileToLoad.x, tileToLoad.y_tms);
                        // No forceRenderUpdate here, as secondary tiles are less critical for immediate display
                    } else {
                        Serial.printf("❌ Data Task: Failed to load secondary tile Z:%d X:%d Y:%d.\n", tileToLoad.z, tileToLoad.x, tileToLoad.y_tms);
                    }
                    vTaskDelay(1); // Yield after processing each secondary tile
                } else {
                    // All secondary tiles processed. Go back to idle.
                    currentLoadingPhase = IDLE;
                    Serial.println("Data Task: All secondary tiles processed. Entering IDLE.");
                }
                break;
            }
        }
        // --- End Tile Loading State Machine ---

        // Eviction logic: Evict tiles that are outside the 5x5 grid around `currentlyLoadedCenterTile`.
        // This runs continuously, ensuring memory is freed.
        std::set<TileKey> currentRequiredTiles; // 5x5 grid
        for (int dx = -2; dx <= 2; ++dx) {
            for (int dy = -2; dy <= 2; ++dy) {
                int neighborX = currentlyLoadedCenterTile.x + dx;
                int neighborY_TMS = currentlyLoadedCenterTile.y_tms + dy;
                int maxTileCoord = (1 << currentTileZ) - 1;
                if (neighborX >= 0 && neighborX <= maxTileCoord && neighborY_TMS >= 0 && neighborY_TMS <= maxTileCoord) {
                    currentRequiredTiles.insert({currentTileZ, neighborX, neighborY_TMS});
                }
            }
        }

        if (xSemaphoreTake(loadedTilesDataMutex, portMAX_DELAY) == pdTRUE) {
            std::vector<TileKey> keysToEvict;
            for (const auto& pair : loadedTilesData) {
                if (currentRequiredTiles.find(pair.first) == currentRequiredTiles.end()) {
                    keysToEvict.push_back(pair.first);
                }
            }
            for (const auto& key : keysToEvict) {
                loadedTilesData.erase(key);
                Serial.printf("Data Task: Evicted tile Z:%d X:%d Y:%d (TMS) - outside 5x5 grid.\n", key.z, key.x, key.y_tms);
            }
            xSemaphoreGive(loadedTilesDataMutex);
        } else {
            Serial.println("❌ Data Task: Failed to acquire mutex for tile eviction.");
        }


        // Calculate the MVT coordinates of the target point within the fetched central tile
        // This needs to be based on the *requested* target, not necessarily the *loaded* center.
        latLonToMVTCoords(internalCurrentTargetLat, internalCurrentTargetLon, currentTileZ, currentRequestedCenterTile.x, currentRequestedCenterTile.y_tms,
                        internalTargetPointMVT_X, internalTargetPointMVT_Y, currentLayerExtent);

        // Prepare and send rendering parameters to the render task
        RenderParams paramsToSend;
        paramsToSend.centralTileX = currentRequestedCenterTile.x; // Render based on requested center
        paramsToSend.centralTileY_TMS = currentRequestedCenterTile.y_tms;
        paramsToSend.targetPointMVT_X = internalTargetPointMVT_X;
        paramsToSend.targetPointMVT_Y = internalTargetPointMVT_Y;
        paramsToSend.layerExtent = currentLayerExtent;
        paramsToSend.zoomScaleFactor = internalCurrentZoomFactor;
        paramsToSend.mapRotationDegrees = internalCurrentRotationAngle;
        
        // Calculate offsets to keep the specific target point centered during zoom.
        float scaledPointX_global = (float)paramsToSend.targetPointMVT_X * (screenW * paramsToSend.zoomScaleFactor / paramsToSend.layerExtent);
        float scaledPointY_global = (float)paramsToSend.targetPointMVT_Y * (screenH * paramsToSend.zoomScaleFactor / paramsToSend.layerExtent);

        paramsToSend.displayOffsetX = round(scaledPointX_global - (screenW / 2.0f));
        paramsToSend.displayOffsetY = round(scaledPointY_global - (screenH / 2.0f));

        // Send parameters to the render queue ONLY if rotation has changed by 1 degree or more,
        // or if other parameters (like location or zoom) have changed, or if forced.
        if (forceRenderUpdate ||
            fabs(internalCurrentRotationAngle - lastSentRotationAngle) >= 1.0f ||
            fabs(internalCurrentZoomFactor - lastSentZoomFactor) >= 0.01f ||
            !(currentRequestedCenterTile == currentlyLoadedCenterTile)) // Check if requested center changed
        {
            if (xQueueSend(renderParamsQueue, &paramsToSend, 0) != pdPASS) {
                // Serial.println("Data Task: Failed to send render parameters to queue. Queue full?");
            } else {
                lastSentRotationAngle = internalCurrentRotationAngle;
                lastSentZoomFactor = internalCurrentZoomFactor;
                forceRenderUpdate = false; // Reset the flag after sending
            }
        }
        Serial.printf("Data Task: Free Internal Heap (end loop): %u bytes, Largest Internal Block: %u bytes\n",
                      heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                      heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
        Serial.printf("Data Task: Free PSRAM: %u bytes, Largest PSRAM Block: %u bytes\n",
                      heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                      heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
        Serial.printf("Data Task: Largest Internal DMA-capable Block (end loop): %u bytes\n",
                      heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));

        // Reduced delay to allow more frequent yielding and responsiveness
        vTaskDelay(pdMS_TO_TICKS(10)); // Changed from 100ms to 10ms
    }
}
