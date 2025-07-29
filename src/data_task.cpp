// data_task.cpp
#include "data_task.h"
#include "common.h"
#include "colors.h" // Include the colors header

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
    const int MAX_SHIFT = 63; // Prevent integer overflow
    
    while (i < dataSize && shift <= MAX_SHIFT) {
        uint8_t byte = data[i++];
        result |= (uint64_t)(byte & 0x7F) << shift;
        if (!(byte & 0x80)) break;
        shift += 7;
    }
    
    if (shift > MAX_SHIFT) {
        Serial.println("❌ Varint overflow detected");
        return 0; // Or throw exception
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
    Serial.printf("❌ PSRAMString: Failed to allocate string of length %u: %s\n", len, e.what()); // Re-enabled debug print
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

// Function to get the color for a given POI class/subclass
uint16_t getIconColor(const PSRAMString& poiClass, const PSRAMString& poiSubclass) {
    // Use static PSRAMString instances for frequently compared strings to avoid re-allocation
    static const PSRAMString PSRAM_TRAFFIC_SIGNALS("traffic_signals", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_BUS_STOP("bus_stop", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_FUEL("fuel", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_BUS("bus", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_AMENITY("amenity", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_SHELTER("shelter", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_MOBILE_PHONE("mobile_phone", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_PAWNBROKER("pawnbroker", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_AGRARIAN("agrarian", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_SHOP("shop", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_BICYCLE("bicycle", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_SUPERMARKET("supermarket", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_BAKERY("bakery", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_ALCOHOL("alcohol", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_HOUSEWARE("houseware", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_BICYCLE_PARKING("bicycle_parking", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_BICYCLE_RENTAL("bicycle_rental", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_TOILETS("toilets", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_TAXI("taxi", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_PARKING("parking", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_HOSPITAL("hospital", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_TOURISM("tourism", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_ATTRACTION("attraction", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_HOTEL("hotel", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_MOTEL("motel", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_HOSTEL("hostel", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_BED_AND_BREAKFAST("bed_and_breakfast", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_GUEST_HOUSE("guest_house", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_CAMP_SITE("camp_site", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_CARAVAN_SITE("caravan_site", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_MUSEUM("museum", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_GALLERY("gallery", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_VIEWPOINT("viewpoint", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_SPORT("sport", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_HISTORIC("historic", PSRAMAllocator<char>());
    static const PSRAMString PSRAM_AERODROME_LABEL("aerodrome_label", PSRAMAllocator<char>());


    // Check for specific icon types first (these colors are set in main.cpp's iconColorsMap)
    if (poiClass == PSRAM_TRAFFIC_SIGNALS || poiSubclass == PSRAM_TRAFFIC_SIGNALS) {
        return iconColorsMap.at(PSRAM_TRAFFIC_SIGNALS);
    }
    if (poiClass == PSRAM_BUS_STOP || (poiClass == PSRAM_BUS && poiSubclass == PSRAM_BUS_STOP)) {
        return iconColorsMap.at(PSRAM_BUS_STOP);
    }
    if (poiClass == PSRAM_FUEL || (poiClass == PSRAM_AMENITY && poiSubclass == PSRAM_FUEL)) {
        return iconColorsMap.at(PSRAM_FUEL);
    }

    // Existing color logic for other POI classes, now using colors.h
    if (poiClass == PSRAM_SHELTER) return POI_SHELTER_COLOR;
    if (poiClass == PSRAM_MOBILE_PHONE) return POI_MOBILE_PHONE_COLOR;
    if (poiClass == PSRAM_PAWNBROKER) return POI_PAWNBROKER_COLOR;
    if (poiClass == PSRAM_AGRARIAN) return POI_AGRARIAN_COLOR;

    if (poiClass == PSRAM_SHOP) {
        if (poiSubclass == PSRAM_BICYCLE) return POI_BICYCLE_SHOP_COLOR;
        if (poiSubclass == PSRAM_SUPERMARKET) return POI_SUPERMARKET_COLOR;
        if (poiSubclass == PSRAM_BAKERY) return POI_BAKERY_COLOR;
        if (poiSubclass == PSRAM_ALCOHOL) return POI_ALCOHOL_SHOP_COLOR;
        if (poiSubclass == PSRAM_HOUSEWARE) return POI_HOUSEWARE_SHOP_COLOR;
        return POI_GENERIC_SHOP_COLOR; // Other specific shops
    }

    if (poiClass == PSRAM_AMENITY) {
        if (poiSubclass == PSRAM_BICYCLE_PARKING) return POI_BICYCLE_PARKING_COLOR;
        if (poiSubclass == PSRAM_BICYCLE_RENTAL) return POI_BICYCLE_RENTAL_COLOR;
        if (poiSubclass == PSRAM_TOILETS) return POI_TOILETS_COLOR;
        if (poiSubclass == PSRAM_TAXI) return POI_TAXI_COLOR;
        if (poiSubclass == PSRAM_PARKING) return POI_PARKING_COLOR;
        if (poiSubclass == PSRAM_HOSPITAL) return POI_HOSPITAL_COLOR;
        return POI_GENERIC_AMENITY_COLOR; // Default for other amenities
    }

    if (poiClass == PSRAM_TOURISM) {
        if (poiSubclass == PSRAM_ATTRACTION) return POI_ATTRACTION_COLOR;
        if (poiSubclass == PSRAM_HOTEL || poiSubclass == PSRAM_MOTEL || poiSubclass == PSRAM_HOSTEL || poiSubclass == PSRAM_BED_AND_BREAKFAST || poiSubclass == PSRAM_GUEST_HOUSE) return POI_ACCOMMODATION_COLOR;
        if (poiSubclass == PSRAM_CAMP_SITE || poiSubclass == PSRAM_CARAVAN_SITE) return POI_CAMP_SITE_COLOR;
        if (poiSubclass == PSRAM_MUSEUM || poiSubclass == PSRAM_GALLERY) return POI_MUSEUM_GALLERY_COLOR;
        if (poiSubclass == PSRAM_VIEWPOINT) return POI_VIEWPOINT_COLOR;
        return POI_GENERIC_TOURISM_COLOR; // Other tourism
    }

    if (poiClass == PSRAM_SPORT) return POI_SPORT_COLOR;
    if (poiClass == PSRAM_HISTORIC) return POI_HISTORIC_COLOR;
    if (poiClass == PSRAM_AERODROME_LABEL) return POI_AERODROME_COLOR;

    return POI_DEFAULT_COLOR; // Default for unclassified POIs
}


// Parses a single layer from the MVT data into our structured format.
// This is called only only once when a new tile is loaded.
ParsedLayer parseLayer(const uint8_t *data, size_t len) {
  ParsedLayer layer;
  size_t i = 0;
  // Use PSRAMString for layer keys and values
  std::vector<PSRAMString, PSRAMAllocator<PSRAMString>> layerKeys{PSRAMAllocator<PSRAMString>()};
  std::vector<PSRAMString, PSRAMAllocator<PSRAMString>> layerValues{PSRAMAllocator<PSRAMString>()};
  layerKeys.reserve(50); // Estimate typical number of keys
  layerValues.reserve(100); // Estimate typical number of values

  std::vector<std::pair<size_t, size_t>> featureOffsets; // Temporary, uses internal heap
  featureOffsets.reserve(100); // Estimate typical number of features

  // Static PSRAMString instances for frequently compared strings within this function
  static const PSRAMString PSRAM_WATER_CLASS("water", PSRAMAllocator<char>()); // Moved declaration here

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
            if (layer.name == PSRAMString("landcover", PSRAMAllocator<char>())) layer.drawOrder = 10;
            else if (layer.name == PSRAMString("water", PSRAMAllocator<char>()) || layer.name == PSRAMString("waterway", PSRAMAllocator<char>())) layer.drawOrder = 20;
            else if (layer.name == PSRAMString("landuse", PSRAMAllocator<char>())) layer.drawOrder = 30;
            else if (layer.name == PSRAMString("road", PSRAMAllocator<char>()) || layer.name == PSRAMString("transportation", PSRAMAllocator<char>())) layer.drawOrder = 40;
            else if (layer.name == PSRAMString("boundary", PSRAMAllocator<char>())) layer.drawOrder = 50;
            else if (layer.name == PSRAMString("poi", PSRAMAllocator<char>()) || layer.name == PSRAMString("mountain_peak", PSRAMAllocator<char>()) || layer.name == PSRAMString("aeroway", PSRAMAllocator<char>())) layer.drawOrder = 60;
            else if (layer.name == PSRAMString("water_name", PSRAMAllocator<char>()) || layer.name == PSRAMString("transportation_name", PSRAMAllocator<char>()) || layer.name == PSRAMString("place", PSRAMAllocator<char>())) layer.drawOrder = 70;
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

            PSRAMString value_str{PSRAMAllocator<char>()}; // Temporary string to store the value
            char temp_str_buffer[64]; // Sufficiently large buffer for number/bool to string conversion
            bool value_parsed = false; // Flag to indicate if a value was successfully parsed

            switch (value_field) {
                case 1: // string_value (wire type 2: Length-delimited)
                    if (value_type == 2) {
                        value_str = readPSRAMString(data, i, value_block_end);
                        value_parsed = true;
                    } else {
                        Serial.printf("❌ parseLayer: Unexpected wire type %d for string_value (field 1).\n", value_type); // Re-enabled debug print
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
                        Serial.printf("❌ parseLayer: Unexpected wire type %d for float_value (field 2).\n", value_type); // Re-enabled debug print
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
                        Serial.printf("❌ parseLayer: Unexpected wire type %d for double_value (field 3).\n", value_type); // Re-enabled debug print
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
                        Serial.printf("❌ parseLayer: Unexpected wire type %d for int/uint/sint_value (field %d).\n", value_type, value_field); // Re-enabled debug print
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
                        Serial.printf("❌ parseLayer: Unexpected wire type %d for bool_value (field 7).\n", value_type); // Re-enabled debug print
                        // Skip the data for this field
                        if (value_type == 0) varint(data, i, value_block_end);
                        else if (value_type == 2) i += varint(data, i, value_block_end);
                        else if (value_type == 5) i += 4;
                        else if (value_type == 1) i += 8;
                    }
                    break;
                default: // Unknown value field type
                    Serial.printf("❌ parseLayer: Unknown value field %d with wire type %d. Skipping data.\n", value_field, value_type); // Re-enabled debug print
                    // Skip the data for this field
                    if (value_type == 0) varint(data, i, value_block_end);
                    else if (type == 2) i += varint(data, i, value_block_end);
                    else if (type == 5) i += 4;
                    else if (type == 1) i += 8;
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
      Serial.printf("❌ parseLayer: Memory allocation failed during layer metadata parsing: %s\n", e.what()); // Re-enabled debug print
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
    feature.hasBridge = false; // Initialize hasBridge flag
    feature.hasTunnel = false; // Initialize hasTunnel flag

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
                currentRing.reserve(50); // Estimate typical number of points in a ring
                while (currentFeatureIdx < geomDataEnd) {
                    uint64_t cmdlen = varint(data, currentFeatureIdx, geomDataEnd);
                    int cmd = cmdlen & 0x7;
                    int count = cmdlen >> 3;

                    switch (cmd) {
                        case MVT_CMD_MOVETO:
                            if (!currentRing.empty()) {
                                feature.geometryRings.push_back(currentRing);
                                currentRing.clear();
                                currentRing.reserve(50); // Re-reserve for next ring
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
                                currentRing.reserve(50); // Re-reserve for next ring
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
                    } else {
                        Serial.printf("        Warning: Invalid KeyIdx=%llu or ValIdx=%llu (Sizes: Keys=%u, Values=%u) for feature properties.\n", // Re-enabled debug print
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
            Serial.printf("❌ parseLayer: Memory allocation failed during feature parsing: %s\n", e.what()); // Re-enabled debug print
            // Continue to next feature or return
            currentFeatureIdx = fend; // Skip remaining of this feature to avoid infinite loop
        }
        vTaskDelay(0); // Yield after processing each feature
    }

    // --- COLOR AND ICON ASSIGNMENT LOGIC (done once during parsing) ---
    feature.color = DEFAULT_FEATURE_COLOR; // Default color for all features
    feature.iconName = PSRAMString("", PSRAMAllocator<char>()); // Default to no icon
    feature.hasBridge = false; // Initialize hasBridge flag for each feature
    feature.hasTunnel = false; // Initialize hasTunnel flag for each feature

    // Check for "bridge" property
    if (feature.properties.count(PSRAMString("bridge", PSRAMAllocator<char>()))) {
        PSRAMString bridgeVal = feature.properties.at(PSRAMString("bridge", PSRAMAllocator<char>()));
        if (bridgeVal == PSRAMString("yes", PSRAMAllocator<char>()) || bridgeVal == PSRAMString("true", PSRAMAllocator<char>()) || bridgeVal == PSRAMString("1", PSRAMAllocator<char>())) {
            feature.hasBridge = true;
        }
    }
    // Check for "tunnel" property
    if (feature.properties.count(PSRAMString("tunnel", PSRAMAllocator<char>()))) {
        PSRAMString tunnelVal = feature.properties.at(PSRAMString("tunnel", PSRAMAllocator<char>()));
        if (tunnelVal == PSRAMString("yes", PSRAMAllocator<char>()) || tunnelVal == PSRAMString("true", PSRAMAllocator<char>()) || tunnelVal == PSRAMString("1", PSRAMAllocator<char>())) {
            feature.hasTunnel = true;
        }
    }
    // NEW: Check for "brunnel" property
    if (feature.properties.count(PSRAMString("brunnel", PSRAMAllocator<char>()))) {
        PSRAMString brunnelVal = feature.properties.at(PSRAMString("brunnel", PSRAMAllocator<char>()));
        if (brunnelVal == PSRAMString("bridge", PSRAMAllocator<char>())) {
            feature.hasBridge = true;
        } else if (brunnelVal == PSRAMString("tunnel", PSRAMAllocator<char>())) {
            feature.hasTunnel = true;
        }
    }


    // Prioritize POI layer color assignments
    if (layer.name == PSRAMString("poi", PSRAMAllocator<char>())) {
        PSRAMString poiClass;
        if (feature.properties.count(PSRAMString("class", PSRAMAllocator<char>()))) {
            poiClass = feature.properties.at(PSRAMString("class", PSRAMAllocator<char>()));
        }
        PSRAMString poiSubclass;
        if (feature.properties.count(PSRAMString("subclass", PSRAMAllocator<char>()))) {
            poiSubclass = feature.properties.at(PSRAMString("subclass", PSRAMAllocator<char>()));
        }
        PSRAMString poiHighway;
        if (feature.properties.count(PSRAMString("highway", PSRAMAllocator<char>()))) {
            poiHighway = feature.properties.at(PSRAMString("highway", PSRAMAllocator<char>()));
        }
        PSRAMString poiLayer;
        if (feature.properties.count(PSRAMString("layer", PSRAMAllocator<char>()))) {
            poiLayer = feature.properties.at(PSRAMString("layer", PSRAMAllocator<char>()));
        }
        PSRAMString poiIndoor;
        if (feature.properties.count(PSRAMString("indoor", PSRAMAllocator<char>()))) {
            poiIndoor = feature.properties.at(PSRAMString("indoor", PSRAMAllocator<char>()));
        }

        // Assign iconName and color based on specific POI types
        static const PSRAMString PSRAM_TRAFFIC_SIGNALS_ICON("traffic_signals", PSRAMAllocator<char>());
        static const PSRAMString PSRAM_FUEL_ICON("fuel", PSRAMAllocator<char>());
        static const PSRAMString PSRAM_BUS_STOP_ICON("bus_stop", PSRAMAllocator<char>());
        static const PSRAMString PSRAM_BUS_CLASS("bus", PSRAMAllocator<char>());
        static const PSRAMString PSRAM_AMENITY_CLASS("amenity", PSRAMAllocator<char>());
        static const PSRAMString PSRAM_PETROL_BUNK("Petrol Bunk", PSRAMAllocator<char>());

        if (poiClass == PSRAM_TRAFFIC_SIGNALS_ICON || poiSubclass == PSRAM_TRAFFIC_SIGNALS_ICON || poiHighway == PSRAM_TRAFFIC_SIGNALS_ICON) {
            feature.color = iconColorsMap.at(PSRAM_TRAFFIC_SIGNALS_ICON);
            feature.iconName = PSRAM_TRAFFIC_SIGNALS_ICON;
            feature.geomType = 1; // Ensure it's treated as a point for icon drawing
        } else if (poiClass == PSRAM_BUS_STOP_ICON || (poiClass == PSRAM_BUS_CLASS && poiSubclass == PSRAM_BUS_STOP_ICON)) {
            feature.color = iconColorsMap.at(PSRAM_BUS_STOP_ICON);
            feature.iconName = PSRAM_BUS_STOP_ICON;
            feature.geomType = 1;
        } else if (poiClass == PSRAM_FUEL_ICON || poiLayer == PSRAM_FUEL_ICON || poiIndoor == PSRAM_PETROL_BUNK || (poiClass == PSRAM_AMENITY_CLASS && poiSubclass == PSRAM_FUEL_ICON)) {
            feature.color = iconColorsMap.at(PSRAM_FUEL_ICON);
            feature.iconName = PSRAM_FUEL_ICON;
            feature.geomType = 1;
        }
        // For other POI types, use the general color assignment logic
        else {
            feature.color = getIconColor(poiClass, poiSubclass);
        }
    }
    // Handle water features
    else if (layer.name == PSRAMString("water", PSRAMAllocator<char>())) {
        static const PSRAMString PSRAM_LAKE("lake", PSRAMAllocator<char>());
        // PSRAM_WATER_CLASS is already declared at the top of the function
        if (feature.properties.count(PSRAMString("class", PSRAMAllocator<char>())) && feature.properties.at(PSRAMString("class", PSRAMAllocator<char>())) == PSRAM_LAKE) {
            feature.color = WATER_COLOR; // Blue for lakes
        } else if (feature.properties.count(PSRAMString("class", PSRAMAllocator<char>())) && feature.properties.at(PSRAMString("class", PSRAMAllocator<char>())) == PSRAM_WATER_CLASS) { // Added for landcover water
            feature.color = WATER_COLOR; // Blue for water in landcover
        }
        else {
            feature.color = WATER_COLOR; // Standard blue for other large water bodies
        }
        feature.isPolygon = true;
    } else if (layer.name == PSRAMString("waterway", PSRAMAllocator<char>())) {
        feature.color = WATER_COLOR; // Blue for rivers, canals, drains
    } else if (layer.name == PSRAMString("water_name", PSRAMAllocator<char>())) {
        feature.color = WATER_COLOR; // Names of water features
    }
    // Handle green features (landcover and landuse)
    else if (layer.name == PSRAMString("landcover", PSRAMAllocator<char>())) {
        if (feature.properties.count(PSRAMString("class", PSRAMAllocator<char>()))) {
            PSRAMString lcClass = feature.properties.at(PSRAMString("class", PSRAMAllocator<char>())); // Fixed: Added missing ')'
            static const PSRAMString PSRAM_FARMLAND("farmland", PSRAMAllocator<char>());
            static const PSRAMString PSRAM_GRASS("grass", PSRAMAllocator<char>());
            static const PSRAMString PSRAM_WOOD("wood", PSRAMAllocator<char>());
            static const PSRAMString PSRAM_FOREST("forest", PSRAMAllocator<char>());
            static const PSRAMString PSRAM_PARK("park", PSRAMAllocator<char>());
            static const PSRAMString PSRAM_GARDEN("garden", PSRAMAllocator<char>());
            static const PSRAMString PSRAM_WETLAND("wetland", PSRAMAllocator<char>());
            static const PSRAMString PSRAM_SAND("sand", PSRAMAllocator<char>());
            static const PSRAMString PSRAM_BEACH("beach", PSRAMAllocator<char>());
            static const PSRAMString PSRAM_ICE("ice", PSRAMAllocator<char>());
            static const PSRAMString PSRAM_GLACIER("glacier", PSRAMAllocator<char>());
            static const PSRAMString PSRAM_ROCK("rock", PSRAMAllocator<char>());
            static const PSRAMString PSRAM_BARE_ROCK("bare_rock", PSRAMAllocator<char>());

            if (lcClass == PSRAM_FARMLAND || lcClass == PSRAM_GRASS || lcClass == PSRAM_WOOD || lcClass == PSRAM_FOREST || lcClass == PSRAM_PARK || lcClass == PSRAM_GARDEN) {
                feature.color = LANDCOVER_FARMLAND_GRASS_WOOD_FOREST_PARK_GARDEN_COLOR; // Green for all vegetation
                feature.isPolygon = true;
            } else if (lcClass == PSRAM_WETLAND) {
                feature.color = LANDCOVER_WETLAND_COLOR; // Wetlands as blue (water-related)
                feature.isPolygon = true;
            } else if (lcClass == PSRAM_WATER_CLASS) { // Explicitly handle 'water' class in landcover
                feature.color = WATER_COLOR; // Blue for water in landcover
                feature.isPolygon = true;
            }
            else if (lcClass == PSRAM_SAND || lcClass == PSRAM_BEACH) {
                feature.color = LANDCOVER_SAND_BEACH_COLOR; // Sandy color
                feature.isPolygon = true;
            } else if (lcClass == PSRAM_ICE || lcClass == PSRAM_GLACIER) {
                feature.color = LANDCOVER_ICE_GLACIER_COLOR;
                feature.isPolygon = true;
            } else if (lcClass == PSRAM_ROCK || lcClass == PSRAM_BARE_ROCK) {
                feature.color = LANDCOVER_ROCK_COLOR;
                feature.isPolygon = true;
            } else {
                feature.color = LANDCOVER_DEFAULT_GREEN; // Default green for other landcover features
                feature.isPolygon = true;
            }
        } else {
            feature.color = LANDCOVER_DEFAULT_GREEN; // Default green if no class property
            feature.isPolygon = true;
        }
    } else if (layer.name == PSRAMString("landuse", PSRAMAllocator<char>())) {
        if (feature.properties.count(PSRAMString("class", PSRAMAllocator<char>()))) {
            PSRAMString luClass = feature.properties.at(PSRAMString("class", PSRAMAllocator<char>())); // Fixed: Added missing ')'
            static const PSRAMString PSRAM_FOREST_LU("forest", PSRAMAllocator<char>());
            static const PSRAMString PSRAM_PARK_LU("park", PSRAMAllocator<char>());
            static const PSRAMString PSRAM_GARDEN_LU("garden", PSRAMAllocator<char>());
            static const PSRAMString PSRAM_GRASS_LU("grass", PSRAMAllocator<char>());
            static const PSRAMString PSRAM_RECREATION_GROUND("recreation_ground", PSRAMAllocator<char>());
            static const PSRAMString PSRAM_VILLAGE_GREEN("village_green", PSRAMAllocator<char>());
            static const PSRAMString PSRAM_ORCHARD("orchard", PSRAMAllocator<char>());
            static const PSRAMString PSRAM_VINEYARD("vineyard", PSRAMAllocator<char>());
            static const PSRAMString PSRAM_ALLOTMENTS("allotments", PSRAMAllocator<char>());
            static const PSRAMString PSRAM_RESIDENTIAL("residential", PSRAMAllocator<char>());
            static const PSRAMString PSRAM_COMMERCIAL("commercial", PSRAMAllocator<char>());
            static const PSRAMString PSRAM_INDUSTRIAL("industrial", PSRAMAllocator<char>());
            static const PSRAMString PSRAM_CEMETERY("cemetery", PSRAMAllocator<char>());
            static const PSRAMString PSRAM_RETAIL("retail", PSRAMAllocator<char>());

            if (luClass == PSRAM_FOREST_LU || luClass == PSRAM_PARK_LU || luClass == PSRAM_GARDEN_LU || luClass == PSRAM_GRASS_LU || luClass == PSRAM_RECREATION_GROUND || luClass == PSRAM_VILLAGE_GREEN || luClass == PSRAM_ORCHARD || luClass == PSRAM_VINEYARD || luClass == PSRAM_ALLOTMENTS) {
                feature.color = LANDUSE_VEGETATION_COLOR; // Green for landuse vegetation
                feature.isPolygon = true;
            } else if (luClass == PSRAM_RESIDENTIAL) {
                feature.color = LANDUSE_RESIDENTIAL_COLOR;
                feature.isPolygon = true;
            } else if (luClass == PSRAM_COMMERCIAL) {
                feature.color = LANDUSE_COMMERCIAL_COLOR;
                feature.isPolygon = true;
            } else if (luClass == PSRAM_INDUSTRIAL) {
                feature.color = LANDUSE_INDUSTRIAL_COLOR;
                feature.isPolygon = true;
            } else if (luClass == PSRAM_CEMETERY) {
                feature.color = LANDUSE_CEMETERY_COLOR; // Cemetery as dark green
                feature.isPolygon = true;
            } else if (luClass == PSRAM_RETAIL) {
                feature.color = LANDUSE_RETAIL_COLOR; // Retail areas
                feature.isPolygon = true;
            } else {
                feature.color = LANDUSE_VEGETATION_COLOR; // Default green for other landuse features
                feature.isPolygon = true;
            }
        } else {
            feature.color = LANDUSE_VEGETATION_COLOR; // Default green if no class property
            feature.isPolygon = true;
        }
    }
    // Handle Roads and Transportation
    else if (layer.name == PSRAMString("road", PSRAMAllocator<char>()) || layer.name == PSRAMString("transportation", PSRAMAllocator<char>())) {
        feature.color = OTHER_ROAD_COLOR; // Default road color
        static const PSRAMString PSRAM_CLASS("class", PSRAMAllocator<char>());
        static const PSRAMString PSRAM_HIGHWAY("highway", PSRAMAllocator<char>());
        static const PSRAMString PSRAM_MOTORWAY("motorway", PSRAMAllocator<char>());
        static const PSRAMString PSRAM_TRUNK("trunk", PSRAMAllocator<char>());
        static const PSRAMString PSRAM_PRIMARY("primary", PSRAMAllocator<char>());
        static const PSRAMString PSRAM_SECONDARY("secondary", PSRAMAllocator<char>());
        static const PSRAMString PSRAM_TERTIARY("tertiary", PSRAMAllocator<char>());
        static const PSRAMString PSRAM_STREET("street", PSRAMAllocator<char>());
        static const PSRAMString PSRAM_RAIL("rail", PSRAMAllocator<char>());
        static const PSRAMString PSRAM_FERRY("ferry", PSRAMAllocator<char>());
        static const PSRAMString PSRAM_AERIALWAY("aerialway", PSRAMAllocator<char>());
        static const PSRAMString PSRAM_MINOR("minor", PSRAMAllocator<char>());
        static const PSRAMString PSRAM_RESIDENTIAL_TR("residential", PSRAMAllocator<char>());
        static const PSRAMString PSRAM_UNCLASSIFIED("unclassified", PSRAMAllocator<char>());
        static const PSRAMString PSRAM_LIVING_STREET("living_street", PSRAMAllocator<char>());
        static const PSRAMString PSRAM_FOOTWAY("footway", PSRAMAllocator<char>());
        static const PSRAMString PSRAM_PATH("path", PSRAMAllocator<char>());
        static const PSRAMString PSRAM_CYCLEWAY("cycleway", PSRAMAllocator<char>());
        static const PSRAMString PSRAM_STEPS("steps", PSRAMAllocator<char>());

        if (feature.properties.count(PSRAM_CLASS)) {
            PSRAMString transportClass = feature.properties.at(PSRAM_CLASS);
            if (transportClass == PSRAM_MOTORWAY || transportClass == PSRAM_TRUNK || transportClass == PSRAM_PRIMARY) {
                feature.color = ROAD_IMPORTANT_COLOR; // New color for these road types
            } else if (transportClass == PSRAM_SECONDARY) {
                feature.color = SECONDARY_ROAD_COLOR;
            } else if (transportClass == PSRAM_TERTIARY || transportClass == PSRAM_STREET) {
                feature.color = TERTIARY_ROAD_COLOR;
            } else if (transportClass == PSRAM_RAIL) {
                feature.color = ROAD_RAIL_COLOR;
            } else if (transportClass == PSRAM_FERRY) {
                feature.color = ROAD_FERRY_COLOR; // Distinct blue for ferries
            } else if (transportClass == PSRAM_AERIALWAY) {
                feature.color = ROAD_AERIALWAY_COLOR;
            } else if (transportClass == PSRAM_MINOR || transportClass == PSRAM_RESIDENTIAL_TR || transportClass == PSRAM_UNCLASSIFIED || transportClass == PSRAM_LIVING_STREET) {
                feature.color = ROAD_MINOR_COLOR; // Minor roads
            } else if (transportClass == PSRAM_FOOTWAY || transportClass == PSRAM_PATH || transportClass == PSRAM_CYCLEWAY || transportClass == PSRAM_STEPS) {
                feature.color = ROAD_PATH_CYCLE_STEPS_COLOR; // Paths, cycleways, steps
            }
        } else if (feature.properties.count(PSRAM_HIGHWAY)) { // common in OpenStreetMap data
             PSRAMString highwayType = feature.properties.at(PSRAM_HIGHWAY);
             if (highwayType == PSRAM_MOTORWAY || highwayType == PSRAM_TRUNK || highwayType == PSRAM_PRIMARY) {
                 feature.color = ROAD_IMPORTANT_COLOR; // New color for these road types
             } else if (highwayType == PSRAM_SECONDARY) {
                 feature.color = SECONDARY_ROAD_COLOR;
             } else if (highwayType == PSRAM_TERTIARY || highwayType == PSRAM_RESIDENTIAL_TR || highwayType == PSRAM_UNCLASSIFIED) {
                 feature.color = TERTIARY_ROAD_COLOR;
             } else if (highwayType == PSRAM_FOOTWAY || highwayType == PSRAM_PATH || highwayType == PSRAM_CYCLEWAY || highwayType == PSRAM_STEPS) {
                 feature.color = ROAD_PATH_CYCLE_STEPS_COLOR;
             }
        }
    } else if (layer.name == PSRAMString("transportation_name", PSRAMAllocator<char>())) {
        feature.color = TFT_WHITE; // Road names in white for contrast
    }
    // Handle Places
    else if (layer.name == PSRAMString("place", PSRAMAllocator<char>())) {
        static const PSRAMString PSRAM_CLASS("class", PSRAMAllocator<char>());
        static const PSRAMString PSRAM_COUNTRY("country", PSRAMAllocator<char>());
        static const PSRAMString PSRAM_STATE("state", PSRAMAllocator<char>());
        static const PSRAMString PSRAM_CITY("city", PSRAMAllocator<char>());
        static const PSRAMString PSRAM_TOWN("town", PSRAMAllocator<char>());
        static const PSRAMString PSRAM_VILLAGE("village", PSRAMAllocator<char>());
        static const PSRAMString PSRAM_HAMLET("hamlet", PSRAMAllocator<char>());
        static const PSRAMString PSRAM_SUBURB("suburb", PSRAMAllocator<char>());
        static const PSRAMString PSRAM_NEIGHBOURHOOD("neighbourhood", PSRAMAllocator<char>());

        if (feature.properties.count(PSRAM_CLASS)) {
            PSRAMString placeClass = feature.properties.at(PSRAM_CLASS);
            if (placeClass == PSRAM_COUNTRY) feature.color = PLACE_COUNTRY_COLOR;
            else if (placeClass == PSRAM_STATE) feature.color = PLACE_STATE_COLOR;
            else if (placeClass == PSRAM_CITY) feature.color = PLACE_CITY_COLOR;
            else if (placeClass == PSRAM_TOWN) feature.color = PLACE_TOWN_COLOR;
            else if (placeClass == PSRAM_VILLAGE) feature.color = PLACE_VILLAGE_COLOR;
            else if (placeClass == PSRAM_HAMLET || placeClass == PSRAM_SUBURB || placeClass == PSRAM_NEIGHBOURHOOD) feature.color = PLACE_SMALL_SETTLEMENT_COLOR; // Smaller places
            else feature.color = PLACE_DEFAULT_COLOR; // Default for other places
        }
        if (feature.geomType == 3) {
            feature.isPolygon = true;
        }
    }
    // Handle other layers
    else if (layer.name == PSRAMString("boundary", PSRAMAllocator<char>())) {
        feature.color = BOUNDARY_COLOR; // White for boundaries
    } else if (layer.name == PSRAMString("mountain_peak", PSRAMAllocator<char>())) {
        feature.color = MOUNTAIN_PEAK_COLOR; // Mountain peaks
    } else if (layer.name == PSRAMString("aeroway", PSRAMAllocator<char>())) {
        feature.color = AEROWAY_COLOR; // Aeroway features (runways, taxiways)
        feature.isPolygon = true; // Most aeroways are areas
    }
    // --- END COLOR AND ICON ASSIGNMENT LOGIC ---

    try {
      layer.features.push_back(feature); // This push_back will use PSRAMAllocator for ParsedFeature
    } catch (const std::bad_alloc& e) {
      Serial.printf("❌ parseLayer: Memory allocation failed when adding feature to layer: %s\n", e.what()); // Re-enabled debug print
      // Continue to next feature or return
    }
    vTaskDelay(0); // Yield after processing each feature
  }
  return layer;
}

// Definition of parseMVTForTile
void parseMVTForTile(const uint8_t *data_buffer, size_t data_len, const TileKey& key) {
    // Acquire mutex before modifying loadedTilesData
    if (xSemaphoreTake(loadedTilesDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Clear existing data for this tile if it was already loaded (e.g., re-request)
        if (loadedTilesData.count(key)) {
            loadedTilesData.erase(key);
        }

        std::vector<ParsedLayer, PSRAMAllocator<ParsedLayer>> tileLayers{PSRAMAllocator<ParsedLayer>()};
        tileLayers.reserve(10); // Estimate typical number of layers per tile
        size_t current_pos = 0;

        while (current_pos < data_len) {
            uint64_t tag = varint(data_buffer, current_pos, data_len);
            int field = tag >> 3;
            int type = tag & 0x07;

            if (field == 3 && type == 2) { // MVT Layer field (tag 3, wire type 2: length-delimited)
                size_t layer_len = varint(data_buffer, current_pos, data_len);
                if (current_pos + layer_len > data_len) {
                    Serial.printf("❌ parseMVTForTile: Layer length exceeds tile data bounds. Skipping remaining data.\n"); // Re-enabled debug print
                    break;
                }
                ParsedLayer layer = parseLayer(data_buffer + current_pos, layer_len);
                if (!layer.name.empty()) { // Only add if parsing was successful and layer has a name
                    tileLayers.push_back(layer);
                }
                current_pos += layer_len;
            } else {
                // Skip unknown fields at the tile level
                if (type == 0) varint(data_buffer, current_pos, data_len);
                else if (type == 2) current_pos += varint(data_buffer, current_pos, data_len);
                else if (type == 5) current_pos += 4;
                else if (type == 1) current_pos += 8;
            }
            vTaskDelay(0); // Yield regularly during parsing
        }

        if (!tileLayers.empty()) {
            loadedTilesData[key] = tileLayers;
            // Update currentLayerExtent from the first layer, assuming all layers in a tile have the same extent
            if (!tileLayers.empty()) {
                currentLayerExtent = tileLayers[0].extent;
            }
        }
        xSemaphoreGive(loadedTilesDataMutex);
    } else {
        Serial.printf("❌ parseMVTForTile: Failed to acquire mutex for loadedTilesData for tile Z:%d X:%d Y:%d.\n", key.z, key.x, key.y_tms); // Re-enabled debug print
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
    // Tile not found is not an error, just means no data for this tile
    Serial.printf("Data Task: Tile Z:%d X:%d Y:%d not found in DB.\n", z, x, y); // Re-enabled debug print
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
    Serial.println("Data Task: Running."); // Re-enabled debug print

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
                    Serial.printf("Data Task: Closed previous MBTiles DB: %s\n", currentMbTilesPath); // Re-enabled debug print
                }
                Serial.printf("Data Task: Opening new MBTiles DB: %s\n", newMbTilesPath); // Re-enabled debug print
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
                        } catch (const std::bad_alloc& e) {
                            Serial.printf("❌ Data Task: Caught std::bad_alloc during parseMVTForTile: %s\n", e.what()); // Re-enabled debug print
                            tileParsedSuccess = false;
                        } catch (...) {
                            Serial.println("❌ Data Task: Caught unknown exception during parseMVTForTile."); // Re-enabled debug print
                            tileParsedSuccess = false;
                        }
                        // No need to free tileDataBuffer here, as it's the static sd_dma_buffer
                    } else {
                        Serial.printf("❌ Data Task: Failed to fetch tile Z:%d X:%d Y:%d. (fetchTile returned false)\n", // Re-enabled debug print
                                      receivedTileRequest.z, receivedTileRequest.x, receivedTileRequest.y_tms);
                        tileParsedSuccess = false;
                    }
                } else {
                    // Tile was already loaded, no need to re-parse
                    Serial.printf("Data Task: Tile Z:%d X:%d Y:%d already loaded. Skipping re-parse.\n", // Re-enabled debug print
                                  receivedTileRequest.z, receivedTileRequest.x, receivedTileRequest.y_tms);
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
