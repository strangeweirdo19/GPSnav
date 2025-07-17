// data_task.cpp
#include "data_task.h"
#include "common.h"

// Global SQLite DB handle (careful with this for multi-threading, but fine for single-threaded loop)
static sqlite3 *mbtiles_db = nullptr; // Changed to static, managed internally by dataTask

// sd_dma_buffer and SD_DMA_BUFFER_SIZE are now extern, defined in main.cpp
// Remove the 'static' declaration here as it conflicts with 'extern' in common.h
// extern uint8_t *sd_dma_buffer; // This line is implicitly handled by common.h include
// const size_t SD_DMA_BUFFER_SIZE; // This line is implicitly handled by common.h include

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
  try {
    str.reserve(len); // Pre-allocate memory for efficiency
    for (uint64_t j = 0; j < len; j++) {
      str += (char)data[i++];
    }
  } catch (const std::bad_alloc& e) {
    Serial.printf("❌ PSRAMString: Failed to allocate string of length %u: %s\n", len, e.what());
    // Return an empty string or handle appropriately
    return PSRAMString{PSRAMAllocator<char>()};
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
// This is called only only once when a new tile is loaded.
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

    try {
      switch (field) {
        case MVT_LAYER_NAME: {
            layer.name = readPSRAMString(data, i, len);
            // Assign draw order based on layer name
            if (layer.name == "landcover") layer.drawOrder = 10;
            else if (layer.name == "water" || layer.name == "waterway") layer.drawOrder = 20;
            else if (layer.name == "landuse") layer.drawOrder = 30;
            else if (layer.name == "road" || layer.name == "transportation") layer.drawOrder = 40;
            else if (layer.name == "boundary") layer.drawOrder = 50;
            else if (layer.name == "poi" || layer.name == "mountain_peak" || layer.name == "aeroway") layer.drawOrder = 60;
            else if (layer.name == "water_name" || layer.name == "transportation_name" || layer.name == "place") layer.drawOrder = 70;
            else layer.drawOrder = 99; // Default for unknown layers
            break;
        }
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
            uint64_t value_tag = varint(data, i, value_block_end);
            int value_field = value_tag >> 3;
            int value_type = value_tag & 0x07;
            // Removed: Serial.printf("        Value Debug: Tag=%llu, Field=%d, Type=%d\n", value_tag, value_field, value_type); // ADDED DEBUG

            PSRAMString value_str{PSRAMAllocator<char>()}; // Temporary string to store the value
            char temp_str_buffer[64]; // Sufficiently large buffer for number/bool to string conversion
            bool value_parsed = false; // Flag to indicate if a value was successfully parsed

            switch (value_field) {
                case 1: // string_value (wire type 2: Length-delimited)
                    if (value_type == 2) {
                        value_str = readPSRAMString(data, i, value_block_end);
                        value_parsed = true;
                    } else {
                        Serial.printf("❌ parseLayer: Unexpected wire type %d for string_value (field 1).\n", value_type);
                        // Skip the data for this field
                        if (value_type == 0) varint(data, i, value_block_end);
                        else if (value_type == 2) i += varint(data, i, value_block_end);
                        else if (value_type == 5) i += 4;
                        else if (value_type == 1) i += 8;
                    }
                    break;
                case 2: // float_value (wire type 5: 32-bit)
                    if (value_type == 5) {
                        float f_val;
                        memcpy(&f_val, data + i, 4); // Copy 4 bytes
                        i += 4;
                        snprintf(temp_str_buffer, sizeof(temp_str_buffer), "%.6f", f_val); // Convert float to string
                        value_str = PSRAMString(temp_str_buffer, PSRAMAllocator<char>());
                        value_parsed = true;
                    } else {
                        Serial.printf("❌ parseLayer: Unexpected wire type %d for float_value (field 2).\n", value_type);
                        // Skip the data for this field
                        if (value_type == 0) varint(data, i, value_block_end);
                        else if (value_type == 2) i += varint(data, i, value_block_end);
                        else if (value_type == 5) i += 4;
                        else if (value_type == 1) i += 8;
                    }
                    break;
                case 3: // double_value (wire type 1: 64-bit)
                    if (value_type == 1) {
                        double d_val;
                        memcpy(&d_val, data + i, 8); // Copy 8 bytes
                        i += 8;
                        snprintf(temp_str_buffer, sizeof(temp_str_buffer), "%.10f", d_val); // Convert double to string
                        value_str = PSRAMString(temp_str_buffer, PSRAMAllocator<char>());
                        value_parsed = true;
                    } else {
                        Serial.printf("❌ parseLayer: Unexpected wire type %d for double_value (field 3).\n", value_type);
                        // Skip the data for this field
                        if (value_type == 0) varint(data, i, value_block_end);
                        else if (value_type == 2) i += varint(data, i, value_block_end);
                        else if (value_type == 5) i += 4;
                        else if (value_type == 1) i += 8;
                    }
                    break;
                case 4: // int64 int_value (wire type 0: Varint)
                case 5: // uint64 uint_value (wire type 0: Varint)
                case 6: // sint64 sint_value (wire type 0: Varint)
                    if (value_type == 0) {
                        int64_t i_val = varint(data, i, value_block_end); // Read as varint
                        if (value_field == 6) i_val = zigzag(i_val); // Zigzag decode if sint64
                        snprintf(temp_str_buffer, sizeof(temp_str_buffer), "%lld", i_val); // Convert int64 to string
                        value_str = PSRAMString(temp_str_buffer, PSRAMAllocator<char>());
                        value_parsed = true;
                    } else {
                        Serial.printf("❌ parseLayer: Unexpected wire type %d for int/uint/sint_value (field %d).\n", value_type, value_field);
                        // Skip the data for this field
                        if (value_type == 0) varint(data, i, value_block_end);
                        else if (value_type == 2) i += varint(data, i, value_block_end);
                        else if (value_type == 5) i += 4;
                        else if (value_type == 1) i += 8;
                    }
                    break;
                case 7: // bool_value (wire type 0: Varint, 0 for false, >0 for true)
                    if (value_type == 0) {
                        bool b_val = (varint(data, i, value_block_end) != 0);
                        value_str = b_val ? PSRAMString("true", PSRAMAllocator<char>()) : PSRAMString("false", PSRAMAllocator<char>());
                        value_parsed = true;
                    } else {
                        Serial.printf("❌ parseLayer: Unexpected wire type %d for bool_value (field 7).\n", value_type);
                        // Skip the data for this field
                        if (value_type == 0) varint(data, i, value_block_end);
                        else if (value_type == 2) i += varint(data, i, value_block_end);
                        else if (value_type == 5) i += 4;
                        else if (value_type == 1) i += 8;
                    }
                    break;
                default: // Unknown value field type
                    Serial.printf("❌ parseLayer: Unknown value field %d with wire type %d. Skipping data.\n", value_field, value_type);
                    // Skip the data for this field
                    if (value_type == 0) varint(data, i, value_block_end);
                    else if (value_type == 2) i += varint(data, i, value_block_end);
                    else if (value_type == 5) i += 4;
                    else if (value_type == 1) i += 8;
                    // No value_str assignment, value_parsed remains false
                    break;
            }
            // Always push a value to maintain index alignment. If not parsed, push an empty string.
            if (value_parsed) {
                layerValues.push_back(value_str);
            } else {
                layerValues.push_back(PSRAMString("", PSRAMAllocator<char>())); // Push placeholder
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
          vTaskDelay(0); // Yield after skipping
          break;
      }
    } catch (const std::bad_alloc& e) {
      Serial.printf("❌ parseLayer: Memory allocation failed during layer metadata parsing: %s\n", e.what());
      // Handle the error, perhaps return a partially parsed layer or an invalid one
      return ParsedLayer();
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

        try { // Added try-catch for feature parsing
            if (field == MVT_FEATURE_GEOMETRY_TYPE && type == 0) {
                feature.geomType = varint(data, currentFeatureIdx, fend);
                if (feature.geomType == 3) feature.isPolygon = true;
                // Removed: Debug print for MultiPoint
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
                        // Removed: Serial.printf(" Property Debug: KeyIdx=%llu, ValIdx=%llu, Key='%s', Value='%s'\n", keyIdx, valIdx, layerKeys[keyIdx].c_str(), layerValues[valIdx].c_str());
                    } else {
                        Serial.printf("        Warning: Invalid KeyIdx=%llu or ValIdx=%llu (Sizes: Keys=%u, Values=%u) for feature properties.\n",
                                      keyIdx, valIdx, layerKeys.size(), layerValues.size());
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
        } catch (const std::bad_alloc& e) {
            Serial.printf("❌ parseLayer: Memory allocation failed during feature parsing: %s\n", e.what());
            // Continue to next feature or return
            currentFeatureIdx = fend; // Skip remaining of this feature to avoid infinite loop
        }
        vTaskDelay(0); // Yield after processing each feature field
    }

    // --- COLOR ASSIGNMENT LOGIC (done once during parsing) ---
    feature.color = TFT_WHITE; // Default color for all features

    // Prioritize POI layer color assignments
    if (layer.name == "poi") {
        PSRAMString poiClass;
        if (feature.properties.count("class")) {
            poiClass = feature.properties.at("class");
        }
        PSRAMString poiSubclass;
        if (feature.properties.count("subclass")) {
            poiSubclass = feature.properties.at("subclass");
        }
        PSRAMString poiHighway;
        if (feature.properties.count("highway")) {
            poiHighway = feature.properties.at("highway");
        }
        PSRAMString poiLayer;
        if (feature.properties.count("layer")) {
            poiLayer = feature.properties.at("layer");
        }
        PSRAMString poiIndoor;
        if (feature.properties.count("indoor")) {
            poiIndoor = feature.properties.at("indoor");
        }

        // Removed: Serial.printf("POI Debug - Class: '%s', Subclass: '%s', Highway: '%s', Layer: '%s', Indoor: '%s'\n", poiClass.c_str(), poiSubclass.c_str(), poiHighway.c_str(), poiLayer.c_str(), poiIndoor.c_str());

        bool classified = false; // Flag to track if classified

        // 1. Highest priority: Traffic Signals (check class, subclass, or highway)
        if (poiClass == "traffic_signals" || poiSubclass == "traffic_signals" || poiHighway == "traffic_signals") {
            feature.color = TFT_CYAN;
            feature.geomType = 1;
            // Removed: Serial.println(" -> Traffic Signal (Cyan)");
            classified = true;
        }
        // 2. Specific POI Classes (direct class match or class/subclass combination)
        else if (poiClass == "bus_stop" || (poiClass == "bus" && poiSubclass == "bus_stop")) { // Added (poiClass == "bus" && poiSubclass == "bus_stop")
            feature.color = TFT_MAROON;
            feature.geomType = 1;
            // Removed: Serial.println(" -> Bus Stop (Maroon)");
            classified = true;
        } else if (poiClass == "shelter") {
            feature.color = TFT_DARKCYAN;
            feature.geomType = 1;
            // Removed: Serial.println(" -> Shelter (Dark Cyan)");
            classified = true;
        } else if (poiClass == "mobile_phone") {
            feature.color = TFT_VIOLET;
            feature.geomType = 1;
            // Removed: Serial.println(" -> Mobile Phone (Violet)");
            classified = true;
        } else if (poiClass == "pawnbroker") {
            feature.color = TFT_BROWN;
            feature.geomType = 1;
            // Removed: Serial.println(" -> Pawnbroker (Brown)");
            classified = true;
        } else if (poiClass == "agrarian") {
            feature.color = TFT_DARKGREEN;
            feature.geomType = 1;
            // Removed: Serial.println(" -> Agrarian (Dark Green)");
            classified = true;
        }
        // 3. Fuel Stations (check class, layer or indoor for "Petrol Bunk", or amenity subclass fuel)
        else if (poiClass == "fuel" || poiLayer == "fuel" || poiIndoor == "Petrol Bunk" || (poiClass == "amenity" && poiSubclass == "fuel")) { // Added poiClass == "fuel"
            feature.color = TFT_ORANGE;
            feature.geomType = 1;
            // Removed: Serial.println(" -> Fuel (Orange)");
            classified = true;
        }
        // 4. General Shops (check class or indoor, or specific shop-related classes)
        else if (poiClass == "shop" || poiIndoor == "shop" ||
                 poiClass == "bicycle" || poiClass == "supermarket" || poiClass == "bakery" ||
                 poiClass == "alcohol" || poiClass == "houseware" || poiClass == "stationery" ||
                 poiClass == "clothes" || poiClass == "bag" || poiClass == "hardware" ||
                 poiClass == "greengrocer" || poiClass == "shoes") // Added more direct shop classes
        {
            if (feature.properties.count("subclass")) {
                PSRAMString subclass = feature.properties.at("subclass");
                if (subclass == "bicycle") feature.color = TFT_BLUE;
                else if (subclass == "supermarket") feature.color = TFT_RED;
                else if (subclass == "bakery") feature.color = TFT_BROWN;
                else if (subclass == "alcohol") feature.color = TFT_PURPLE;
                else if (subclass == "houseware") feature.color = TFT_GOLD; // Specific color for houseware
                else feature.color = TFT_MAGENTA; // Other specific shops
            } else {
                feature.color = TFT_MAGENTA; // General shops without subclass
            }
            // Removed: Serial.printf(" -> Shop (Color: %04X)\n", feature.color);
            classified = true;
        }
        // 5. General Amenities (check class and subclass, if not caught by specific fuel)
        else if (poiClass == "amenity") {
            if (feature.properties.count("subclass")) {
                PSRAMString subclass = feature.properties.at("subclass");
                // "fuel" subclass is handled at top, so no need to repeat here
                if (subclass == "bicycle_parking") feature.color = TFT_DARKGREEN;
                else if (subclass == "bicycle_rental") feature.color = TFT_GREEN;
                else if (subclass == "toilets") feature.color = TFT_WHITE;
                else if (subclass == "taxi") feature.color = TFT_MAROON;
                else if (subclass == "parking") feature.color = TFT_DARKGREY;
                else if (subclass == "hospital") feature.color = TFT_RED;
                else feature.color = TFT_YELLOW; // Default for other amenities
            } else {
                feature.color = TFT_YELLOW; // General amenities without subclass
            }
            // Removed: Serial.printf(" -> Amenity (Color: %04X)\n", feature.color);
            classified = true;
        }
        // 6. Tourism
        else if (poiClass == "tourism") {
            if (feature.properties.count("subclass")) {
                PSRAMString subclass = feature.properties.at("subclass");
                if (subclass == "attraction") feature.color = TFT_PINK; // Tourist attractions
                else if (subclass == "hotel" || subclass == "motel" || subclass == "hostel" || subclass == "bed_and_breakfast" || subclass == "guest_house") feature.color = TFT_BROWN; // Lodging
                else if (subclass == "camp_site" || subclass == "caravan_site") feature.color = TFT_DARKGREEN; // Camping
                else if (subclass == "museum" || subclass == "gallery") feature.color = TFT_ORANGE; // Museums/Galleries
                else if (subclass == "viewpoint") feature.color = TFT_CYAN; // Viewpoints
                else feature.color = TFT_PURPLE; // Other tourism
            } else {
                feature.color = TFT_PURPLE; // General tourism without subclass
            }
            // Removed: Serial.printf(" -> Tourism (Color: %04X)\n", feature.color);
            classified = true;
        }
        // 7. Other top-level POI classes
        else if (poiClass == "sport") {
            feature.color = TFT_RED;
            // Removed: Serial.println(" -> Sport (Red)");
            classified = true;
        } else if (poiClass == "historic") {
            feature.color = TFT_BROWN;
            // Removed: Serial.println(" -> Historic (Brown)");
            classified = true;
        } else if (poiClass == "aerodrome_label") {
            feature.color = TFT_CYAN;
            // Removed: Serial.println(" -> Aerodrome Label (Cyan)");
            classified = true;
        }

        // Final fallback if not classified by any specific rule
        if (!classified) {
            feature.color = TFT_LIGHTGREY;
            // Removed: Serial.println(" -> Unclassified POI (Light Grey)");
        }

        // Removed: Debugging for unclassified POIs
        // if (!classified) { // Only print if it remained unclassified
        //     Serial.println("    --- All POI Properties (for unclassified/defaulted) ---");
        //     for (const auto& prop : feature.properties) {
        //         Serial.printf("    Key: '%s', Value: '%s'\n", prop.first.c_str(), prop.second.c_str());
        //     }
        //     Serial.println("    -----------------------------------------------------");
        // }

    }
    // Handle water features
    else if (layer.name == "water") {
        if (feature.properties.count("class") && feature.properties.at("class") == "lake") {
            feature.color = TFT_BLUE; // Blue for lakes
        } else if (feature.properties.count("class") && feature.properties.at("class") == "water") { // Added for landcover water
            feature.color = TFT_BLUE; // Blue for water class in landcover
        }
        else {
            feature.color = TFT_BLUE; // Standard blue for other large water bodies
        }
        feature.isPolygon = true;
    } else if (layer.name == "waterway") {
        feature.color = TFT_BLUE; // Blue for rivers, canals, drains
    } else if (layer.name == "water_name") {
        feature.color = TFT_BLUE; // Names of water features
    }
    // Handle green features (landcover and landuse)
    else if (layer.name == "landcover") {
        if (feature.properties.count("class")) {
            PSRAMString lcClass = feature.properties.at("class");
            if (lcClass == "farmland" || lcClass == "grass" || lcClass == "wood" || lcClass == "forest" || lcClass == "park" || lcClass == "garden") {
                feature.color = TFT_GREEN; // Green for all vegetation
                feature.isPolygon = true;
            } else if (lcClass == "wetland") {
                feature.color = TFT_BLUE; // Wetlands as blue (water-related)
                feature.isPolygon = true;
            } else if (lcClass == "water") { // Explicitly handle 'water' class in landcover
                feature.color = TFT_BLUE; // Blue for water in landcover
                feature.isPolygon = true;
            }
            else if (lcClass == "sand" || lcClass == "beach") {
                feature.color = 0xE64E; // Sandy color
                feature.isPolygon = true;
            } else if (lcClass == "ice" || lcClass == "glacier") {
                feature.color = TFT_WHITE;
                feature.isPolygon = true;
            } else if (lcClass == "rock" || lcClass == "bare_rock") {
                feature.color = TFT_DARKGREY;
                feature.isPolygon = true;
            }
        }
    } else if (layer.name == "landuse") {
        if (feature.properties.count("class")) {
            PSRAMString luClass = feature.properties.at("class");
            if (luClass == "forest" || luClass == "park" || luClass == "garden" || luClass == "grass" || luClass == "recreation_ground" || luClass == "village_green" || luClass == "orchard" || luClass == "vineyard" || luClass == "allotments") {
                feature.color = TFT_GREEN; // Green for landuse vegetation
                feature.isPolygon = true;
            } else if (luClass == "residential") {
                feature.color = TFT_LIGHTGREY;
                feature.isPolygon = true;
            } else if (luClass == "commercial") {
                feature.color = TFT_ORANGE;
                feature.isPolygon = true;
            } else if (luClass == "industrial") {
                feature.color = TFT_BROWN;
                feature.isPolygon = true;
            } else if (luClass == "cemetery") {
                feature.color = TFT_DARKGREEN; // Cemetery as dark green
                feature.isPolygon = true;
            } else if (luClass == "retail") {
                feature.color = TFT_MAGENTA; // Retail areas
                feature.isPolygon = true;
            }
        }
    }
    // Handle Roads and Transportation
    else if (layer.name == "road" || layer.name == "transportation") {
        feature.color = TFT_DARKGREY; // Default road color
        if (feature.properties.count("class")) {
            PSRAMString transportClass = feature.properties.at("class");
            if (transportClass == "motorway" || transportClass == "trunk") {
                feature.color = TFT_RED;
            } else if (transportClass == "primary" || transportClass == "secondary") {
                feature.color = TFT_ORANGE;
            } else if (transportClass == "tertiary" || transportClass == "street") {
                feature.color = TFT_LIGHTGREY;
            } else if (transportClass == "rail") {
                feature.color = TFT_DARKCYAN;
            } else if (transportClass == "ferry") {
                feature.color = TFT_NAVY; // Distinct blue for ferries
            } else if (transportClass == "aerialway") {
                feature.color = TFT_PINK;
            } else if (transportClass == "minor" || transportClass == "residential" || transportClass == "unclassified" || transportClass == "living_street") {
                feature.color = TFT_SILVER; // Minor roads
            } else if (transportClass == "footway" || transportClass == "path" || transportClass == "cycleway" || transportClass == "steps") {
                feature.color = TFT_GREENYELLOW; // Paths, cycleways, steps
            }
        } else if (feature.properties.count("highway")) { // common in OpenStreetMap data
             PSRAMString highwayType = feature.properties.at("highway");
             if (highwayType == "motorway" || highwayType == "trunk") {
                 feature.color = TFT_RED;
             } else if (highwayType == "primary" || highwayType == "secondary") {
                 feature.color = TFT_ORANGE;
             } else if (highwayType == "tertiary" || highwayType == "residential" || highwayType == "unclassified") {
                 feature.color = TFT_LIGHTGREY;
             } else if (highwayType == "footway" || highwayType == "path" || highwayType == "cycleway" || highwayType == "steps") {
                 feature.color = TFT_GREENYELLOW;
             }
        }
    } else if (layer.name == "transportation_name") {
        feature.color = TFT_WHITE; // Road names in white for contrast
    }
    // Handle Places
    else if (layer.name == "place") {
        if (feature.properties.count("class")) {
            PSRAMString placeClass = feature.properties.at("class");
            if (placeClass == "country") feature.color = TFT_WHITE;
            else if (placeClass == "state") feature.color = TFT_SILVER;
            else if (placeClass == "city") feature.color = TFT_YELLOW;
            else if (placeClass == "town") feature.color = TFT_ORANGE;
            else if (placeClass == "village") feature.color = TFT_GREENYELLOW;
            else if (placeClass == "hamlet" || placeClass == "suburb" || placeClass == "neighbourhood") feature.color = TFT_LIGHTGREY; // Smaller places
            else feature.color = TFT_MAGENTA; // Default for other places
        } else {
            feature.color = TFT_MAGENTA; // Fallback for place without class
        }
        if (feature.geomType == 3) {
            feature.isPolygon = true;
        }
    }
    // Handle other layers
    else if (layer.name == "boundary") {
        feature.color = TFT_WHITE; // White for boundaries
    } else if (layer.name == "mountain_peak") {
        feature.color = TFT_YELLOW; // Mountain peaks
    } else if (layer.name == "aeroway") {
        feature.color = TFT_LIGHTGREY; // Aeroway features (runways, taxiways)
        feature.isPolygon = true; // Most aeroways are areas
    }
    // --- END COLOR ASSIGNMENT LOGIC ---

    try {
      layer.features.push_back(feature); // This push_back will use PSRAMAllocator for ParsedFeature
    } catch (const std::bad_alloc& e) {
      Serial.printf("❌ parseLayer: Memory allocation failed when adding feature to layer: %s\n", e.what());
      // Continue to next feature or return
    }
    vTaskDelay(0); // Yield after processing each feature
  }
  return layer;
}


// Parses the entire MVT tile into our structured format and stores it under the given TileKey.
void parseMVTForTile(const uint8_t *data_buffer, size_t data_len, const TileKey& key) {
  Serial.printf("Data Task: Parsing MVT for Z:%d X:%d Y:%d.\n",
                key.z, key.x, key.y_tms);

  std::vector<ParsedLayer, PSRAMAllocator<ParsedLayer>> currentTileLayers{PSRAMAllocator<ParsedLayer>()};
  size_t i = 0;
  while (i < data_len) {
    uint64_t tag = varint(data_buffer, i, data_len);
    int field = tag >> 3;
    int type = tag & 0x07;

    if (field == 3 && type == 2) { // Layer field (tag 3, wire type 2 = Length-delimited)
      size_t len = varint(data_buffer, i, data_len); // Get layer data length
      try {
        currentTileLayers.push_back(parseLayer(data_buffer + i, len)); // Parse the layer and add to our vector
      } catch (const std::bad_alloc& e) {
        Serial.printf("❌ Data Task: Memory allocation failed when adding layer: %s\n", e.what());
        // Continue to next layer or break
      }
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

  if (!currentTileLayers.empty()) {
      if (xSemaphoreTake(loadedTilesDataMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
          try {
              loadedTilesData.emplace(key, std::move(currentTileLayers));
              currentLayerExtent = loadedTilesData.at(key)[0].extent;
              Serial.printf("Data Task: Successfully stored tile Z:%d X:%d Y:%d. Free PSRAM: %u bytes\n",
                            key.z, key.x, key.y_tms, heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
          } catch (const std::bad_alloc& e) {
              Serial.printf("❌ Data Task: Memory allocation failed when storing tile data: %s\n", e.what());
          } catch (const std::out_of_range& e) {
              Serial.printf("❌ Data Task: Out of range error accessing extent after move: %s\n", e.what());
          }
          xSemaphoreGive(loadedTilesDataMutex);
      } else {
          Serial.println("❌ Data Task: Failed to acquire mutex for storing tile (timeout).");
      }
  } else {
      Serial.printf("Data Task: No layers parsed for tile Z:%d X:%d Y:%d. Not storing.\n", key.z, key.x, key.y_tms);
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
    Serial.printf("❌ SQLite prepare failed: %s.\n", sqlite3_errmsg(db));
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
        Serial.printf("❌ Data Task: Tile data size (%d bytes) exceeds pre-allocated SD DMA buffer size (%u bytes) for Z:%d X:%d Y:%d.\n",
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
    // Tile not found is not an error, just means no data for this tile
    Serial.printf("Data Task: Tile Z:%d X:%d Y:%d not found in DB.\n", z, x, y); // Keep this, it's an important status
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
    // SD_MMC.begin() and sd_dma_buffer allocation are now handled in setup() in main.cpp
    Serial.println("Data Task: Running."); // Keep this initial startup message

    // Check if the DMA buffer was successfully allocated in setup()
    if (sd_dma_buffer == nullptr) {
        Serial.println("❌ Data Task: SD DMA buffer not allocated. Terminating task."); // Keep this error message
        vTaskDelete(NULL);
    }

    char currentMbTilesPath[96] = "";

    TileKey receivedTileRequest;
    bool tileParsedSuccess = true; // For notification to renderTask

    while (true) {
        if (xQueueReceive(tileRequestQueue, &receivedTileRequest, portMAX_DELAY) == pdPASS) {
            char newMbTilesPath[96];
            int y_osm = (1 << receivedTileRequest.z) - 1 - receivedTileRequest.y_tms;
            snprintf(newMbTilesPath, sizeof(newMbTilesPath), "/sdcard/tiles/%d_%d.mbtiles",
                     (int)floor(tileYToLat(y_osm, receivedTileRequest.z)),
                     (int)floor(tileXToLon(receivedTileRequest.x, receivedTileRequest.z)));

            if (strcmp(newMbTilesPath, currentMbTilesPath) != 0) {
                if (mbtiles_db) {
                    sqlite3_close(mbtiles_db);
                    mbtiles_db = nullptr;
                }
                if (sqlite3_open(newMbTilesPath, &mbtiles_db) != SQLITE_OK) {
                    Serial.printf("❌ Data Task: Failed to open MBTiles database: %s. Error: %s\n", newMbTilesPath, sqlite3_errmsg(mbtiles_db)); // Keep this error message
                    mbtiles_db = nullptr;
                    tileParsedSuccess = false;
                } else {
                    strcpy(currentMbTilesPath, newMbTilesPath);
                    tileParsedSuccess = true;
                }
            }

            if (mbtiles_db && tileParsedSuccess) {
                uint8_t *tileDataBuffer = nullptr; // This will point to sd_dma_buffer
                size_t tileDataLen = 0;

                bool alreadyLoaded = false;
                if (xSemaphoreTake(loadedTilesDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    alreadyLoaded = (loadedTilesData.find(receivedTileRequest) != loadedTilesData.end());
                    xSemaphoreGive(loadedTilesDataMutex);
                } else {
                    Serial.println("❌ Data Task: Failed to acquire mutex for loadedTilesData during initial check."); // Keep this error message
                }

                if (!alreadyLoaded) {
                    if (fetchTile(mbtiles_db, receivedTileRequest.z, receivedTileRequest.x, receivedTileRequest.y_tms, tileDataBuffer, tileDataLen)) {
                        try {
                            // tileDataBuffer now points to the static sd_dma_buffer
                            parseMVTForTile(tileDataBuffer, tileDataLen, receivedTileRequest);
                            tileParsedSuccess = true;
                        } catch (const std::bad_alloc& e) {
                            Serial.printf("❌ Data Task: Caught std::bad_alloc during parseMVTForTile: %s\n", e.what()); // Keep this error message
                            tileParsedSuccess = false;
                        } catch (...) {
                            Serial.println("❌ Data Task: Caught unknown exception during parseMVTForTile."); // Keep this error message
                            tileParsedSuccess = false;
                        }
                        // No need to free tileDataBuffer here, as it's the static sd_dma_buffer
                    } else {
                        Serial.printf("❌ Data Task: Failed to fetch tile Z:%d X:%d Y:%d. (fetchTile returned false)\n", // Added explicit log for fetchTile failure
                                      receivedTileRequest.z, receivedTileRequest.x, receivedTileRequest.y_tms);
                        tileParsedSuccess = false;
                    }
                } else {
                    tileParsedSuccess = true;
                }
            } else {
                tileParsedSuccess = false;
            }

            if (xQueueSend(tileParsedNotificationQueue, &tileParsedSuccess, pdMS_TO_TICKS(50)) != pdPASS) {
                Serial.println("❌ Data Task: Failed to send tile parsed notification to queue. Queue full?"); // Keep this error message
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
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
