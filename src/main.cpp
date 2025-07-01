#include <Arduino.h>
#include <SPI.h>
#include <FS.h>
#include "SD_MMC.h"
#include <sqlite3.h>
#include <vector>
#include <map>
#include <TFT_eSPI.h> // Make sure this library is installed
#include <algorithm>  // For std::min, std::max, std::sort, std::swap
#include <cmath>      // For log2, pow, round, and PI (often defined here or Arduino.h)
#include <set>        // For std::set to manage active tile keys

// FreeRTOS includes for multi-core tasks and synchronization
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h" // For mutex
#include "freertos/queue.h"  // For queue

// HMC5883L Compass includes
#include <Wire.h> // Required for I2C communication
#include <Adafruit_Sensor.h> // Required for Adafruit unified sensor system
#include <Adafruit_HMC5883_U.h> // Required for Adafruit HMC5883L Unified Sensor

// =========================================================
// TFT DISPLAY AND SPRITE SETUP
// =========================================================
TFT_eSPI tft = TFT_eSPI();              // TFT display object
TFT_eSprite sprite = TFT_eSprite(&tft); // DMA sprite for smooth rendering

// These define the logical screen dimensions AFTER rotation
int screenW = 160;
int screenH = 128;

// Actual zoom level of the tile fetched from MBTiles
int currentTileZ = 17;

// =========================================================
// GLOBAL SHARED DATA AND SYNCHRONIZATION OBJECTS
// =========================================================

// Define structs to hold parsed MVT data. This allows us to parse the tile
// once and then render the structured data many times, which is much faster.
struct ParsedFeature {
    int geomType; // 1=Point, 2=LineString, 3=Polygon
    // Geometry is stored as raw MVT coordinates (0-extent).
    // Each inner vector is a ring of points.
    std::vector<std::vector<std::pair<int, int>>> geometryRings;
    std::map<String, String> properties; // Using String for flexibility, consider char* if keys/values are fixed.
    uint16_t color; // Pre-calculated color for fast rendering
    bool isPolygon; // Pre-calculated polygon flag
    // Add bounding box for culling
    int minX_mvt, minY_mvt, maxX_mvt, maxY_mvt; // Bounding box in MVT coordinates
};

struct ParsedLayer {
    String name;
    int extent; // Tile extent for this layer (typically 4096)
    std::vector<ParsedFeature> features;
};

// Structure to uniquely identify a tile
struct TileKey {
    int z;
    int x;
    int y_tms; // TMS Y coordinate
    // Operator for map key comparison
    bool operator<(const TileKey& other) const {
        if (z != other.z) return z < other.z;
        if (x != other.x) return x < other.x;
        return y_tms < other.y_tms;
    }
};

// Global storage for parsed data from multiple tiles. PROTECTED BY MUTEX.
std::map<TileKey, std::vector<ParsedLayer>> loadedTilesData;
SemaphoreHandle_t loadedTilesDataMutex; // Mutex to protect loadedTilesData

// Global variable for current layer extent, updated by dataTask, read by renderTask
int currentLayerExtent = 4096; // Default MVT tile extent, will be updated from parsed data

// Structure for rendering parameters to be sent via queue
struct RenderParams {
    int centralTileX;
    int centralTileY_TMS;
    int targetPointMVT_X;
    int targetPointMVT_Y;
    int layerExtent; // Changed from currentLayerExtent to layerExtent for clarity in struct
    float zoomScaleFactor;
    int displayOffsetX;
    int displayOffsetY;
    float mapRotationDegrees; // New: for map rotation based on compass
};
QueueHandle_t renderParamsQueue; // Queue to send RenderParams from data task to render task

// Structure for control parameters (Lat, Lon, Zoom) to be sent from loop() to dataTask
struct ControlParams {
    double targetLat;
    double targetLon;
    float zoomFactor;
};
QueueHandle_t controlParamsQueue; // Queue to send ControlParams from loop() to data task

// Global SQLite DB handle (careful with this for multi-threading, but fine for single-threaded loop)
sqlite3 *mbtiles_db = nullptr; // This will now be opened/closed per animation sequence

// HMC5883L compass object - changed to Adafruit_HMC5883_Unified
Adafruit_HMC5883_Unified hmc5883l = Adafruit_HMC5883_Unified(12345); // Using a sensor ID, common for unified sensors

// Global variables for compass heading filtering (moving average)
std::vector<float> headingReadings;
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

// Reads a length-prefixed string from the MVT data
// Using String for convenience, but be aware of its overhead.
String readString(const uint8_t *data, size_t &i, size_t dataSize) {
  uint64_t len = varint(data, i, dataSize); // Get string length
  if (i + len > dataSize) { // Prevent reading past buffer
      len = dataSize - i;
  }
  String str;
  str.reserve(len); // Pre-allocate memory for efficiency
  for (uint64_t j = 0; j < len; j++) {
    str += (char)data[i++];
  }
  return str;
}

// =========================================================
// GEOMETRY DRAWING (Render Task will use this)
// =========================================================
// This function is now only used for rendering pre-parsed geometry.
// It takes a set of screen-space points and draws/fills them.
void renderRing(const std::vector<std::pair<int, int>>& points, uint16_t color, bool isPolygon, int geomType) {
  if (points.empty()) return;

  if (geomType == 1) { // Explicitly handle Point geometry
      if (!points.empty()) { // Should only have one point for a single point feature
          sprite.drawPixel(points[0].first, points[0].second, color);
      }
  } else if (geomType == 2) { // Explicitly handle LineString geometry
      if (points.size() > 1) {
          for (size_t k = 0; k < points.size() - 1; ++k) {
              sprite.drawLine(points[k].first, points[k].second, points[k+1].first, points[k+1].second, color);
          }
      } else if (points.size() == 1) { // A single point in a LineString is still a point
          sprite.drawPixel(points[0].first, points[0].second, color);
      }
  } else if (geomType == 3) { // Explicitly handle Polygon geometry
      // Draw the outline of the polygon
      if (points.size() > 1) {
          for (size_t k = 0; k < points.size() - 1; ++k) {
              sprite.drawLine(points[k].first, points[k].second, points[k+1].first, points[k+1].second, color);
          }
          // Close the path for polygons
          if (points.front().first != points.back().first || points.front().second != points.back().second) {
              sprite.drawLine(points.back().first, points.back().second, points.front().first, points.front().second, color);
          }
      } else if (points.size() == 1) { // A single point in a Polygon is still a point
          sprite.drawPixel(points[0].first, points[0].second, color);
      }

      // Fill the polygon
      // (Keep existing scanline fill logic here)
      int minY = screenH, maxY = 0;
      for (const auto& p : points) {
          if (p.second < minY) minY = p.second;
          if (p.second > maxY) maxY = p.second;
      }

      minY = std::max(0, minY);
      maxY = std::min(screenH - 1, maxY);

      for (int scanY = minY; scanY <= maxY; ++scanY) {
          std::vector<int> intersections;
          for (size_t k = 0; k < points.size(); ++k) {
              int x1 = points[k].first;
              int y1 = points[k].second;
              int x2 = points[(k + 1) % points.size()].first;
              int y2 = points[(k + 1) % points.size()].second;

              if (y1 == y2) continue;
              if ((scanY < std::min(y1, y2)) || (scanY >= std::max(y1, y2))) continue;

              float intersectX = (float)(x2 - x1) * (scanY - y1) / (float)(y2 - y1) + x1;
              intersections.push_back(round(intersectX));
          }
          std::sort(intersections.begin(), intersections.end());

          for (size_t k = 0; k + 1 < intersections.size(); k += 2) {
              int startX = intersections[k];
              int endX = intersections[k+1];
              if (startX > endX) std::swap(startX, endX);

              startX = std::max(0, startX);
              endX = std::min(screenW - 1, endX);

              if (startX <= endX) {
                  sprite.drawFastHLine(startX, scanY, endX - startX + 1, color);
              }
          }
      }
  } else { // Fallback for unknown geomType, draw as points
      if (!points.empty()) {
          sprite.drawPixel(points[0].first, points[0].second, color);
      }
  }
}

// New function to draw a pre-parsed feature. It transforms MVT coordinates
// to screen coordinates and then calls renderRing. It now takes a TileKey
// to correctly offset features from neighboring tiles.
void drawParsedFeature(const ParsedFeature& feature, int layerExtent, const TileKey& tileKey, const RenderParams& params) {
    float scaleX = (float)screenW * params.zoomScaleFactor / layerExtent;
    float scaleY = (float)screenH * params.zoomScaleFactor / layerExtent;

    // Calculate the pixel offset for this specific tile relative to the central tile (params.centralTileX, params.centralTileY_TMS)
    // A full MVT extent (e.g., 4096 units) maps to `screenW` pixels at scale 1.0.
    // When zoomed, it maps to `screenW * params.zoomScaleFactor` pixels.
    float tileRenderWidth = (float)screenW * params.zoomScaleFactor;
    float tileRenderHeight = (float)screenH * params.zoomScaleFactor;

    // Corrected Y-axis offset calculation for TMS Y (which increases upwards) to screen Y (which increases downwards).
    // If tileKey.y_tms is smaller (tile is "above" in TMS, meaning it's "below" in standard Y/screen Y),
    // then (params.centralTileY_TMS - tileKey.y_tms) will be positive, shifting content downwards.
    // If tileKey.y_tms is larger (tile is "below" in TMS, meaning it's "above" in standard Y/screen Y),
    // then (params.centralTileY_TMS - tileKey.y_tms) will be negative, shifting content upwards.
    float tileRenderOffsetX_float = (float)(tileKey.x - params.centralTileX) * tileRenderWidth;
    float tileRenderOffsetY_float = (float)(params.centralTileY_TMS - tileKey.y_tms) * tileRenderHeight;


    // --- Bounding Box Culling Optimization ---
    // Transform the MVT bounding box to screen coordinates, considering the tile's position
    // and the global display offset.
    float screenMinX_float_pre_rot = feature.minX_mvt * scaleX + tileRenderOffsetX_float - params.displayOffsetX;
    float screenMinY_float_pre_rot = feature.minY_mvt * scaleY + tileRenderOffsetY_float - params.displayOffsetY;
    float screenMaxX_float_pre_rot = feature.maxX_mvt * scaleX + tileRenderOffsetX_float - params.displayOffsetX;
    float screenMaxY_float_pre_rot = feature.maxY_mvt * scaleY + tileRenderOffsetY_float - params.displayOffsetY;

    // Check if the feature's bounding box is outside the screen. If so, skip rendering.
    if (screenMaxX_float_pre_rot < 0 || screenMinX_float_pre_rot >= screenW || screenMaxY_float_pre_rot < 0 || screenMinY_float_pre_rot >= screenH) {
        return;
    }
    // --- End Bounding Box Culling ---

    // Calculate cosine and sine of the rotation angle (in radians)
    float cosTheta = cos(radians(params.mapRotationDegrees));
    float sinTheta = sin(radians(params.mapRotationDegrees));
    
    // Define the center of rotation (screen center)
    int centerX = screenW / 2;
    int centerY = screenH / 2;

    for (const auto& ring : feature.geometryRings) {
        std::vector<std::pair<int, int>> screenPoints;
        screenPoints.reserve(ring.size()); // Pre-allocate memory

        for (const auto& p : ring) {
            // Convert MVT (x,y) to screen (x,y) coordinates
            // Assuming MVT Y is Y-down, which aligns with TFT_eSPI's coordinate system
            float screen_x = (float)p.first * scaleX + tileRenderOffsetX_float - params.displayOffsetX;
            float screen_y = (float)p.second * scaleY + tileRenderOffsetY_float - params.displayOffsetY;

            // Translate point so that the center of rotation is at the origin
            float translatedX = screen_x - centerX;
            float translatedY = screen_y - centerY;

            // Apply 2D rotation transformation
            float rotatedX = translatedX * cosTheta - translatedY * sinTheta;
            float rotatedY = translatedX * sinTheta + translatedY * cosTheta;

            // Translate point back to its original position relative to the screen center
            screenPoints.push_back({round(rotatedX + centerX), round(rotatedY + centerY)});
        }
        renderRing(screenPoints, feature.color, feature.isPolygon, feature.geomType); // Pass geomType
    }
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
  std::vector<String> layerKeys;
  std::vector<String> layerValues;

  std::vector<std::pair<size_t, size_t>> featureOffsets; // Stores start index and length of each feature

  // First pass: Parse layer metadata (name, extent, keys, values) and collect feature offsets
  while (i < len) {
    uint64_t tag = varint(data, i, len);
    int field = tag >> 3;
    int type = tag & 0x07;

    switch (field) {
      case MVT_LAYER_NAME: layer.name = readString(data, i, len); break;
      case MVT_LAYER_FEATURE: {
        size_t featureLen = varint(data, i, len);
        featureOffsets.push_back({i, featureLen});
        i += featureLen;
        break;
      }
      case MVT_LAYER_KEYS: layerKeys.push_back(readString(data, i, len)); break;
      case MVT_LAYER_VALUES: {
        size_t valLen = varint(data, i, len);
        size_t valEnd = i + valLen;
        while (i < valEnd) {
          uint64_t tag2 = varint(data, i, valEnd);
          int f = tag2 >> 3, t = tag2 & 0x07;
          if (f == 1 && t == 2) { // string_value (field 1, wire type 2 = Length-delimited)
            layerValues.push_back(readString(data, i, valEnd));
          } else { // Skip other unknown value types
            if (t == 0) varint(data, i, valEnd);
            else if (t == 2) i += varint(data, i, valEnd);
            else if (t == 5) i += 4;
            else if (t == 1) i += 8;
          }
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
  }

  // Serial.printf("\n🗂️ Parsing Layer: %s (Features: %d, Extent: %d)\n", layer.name.c_str(), featureOffsets.size(), layer.extent);

  // Second pass: Parse each feature's properties and geometry
  for (const auto &offset : featureOffsets) {
    size_t fi = offset.first;
    size_t fend = fi + offset.second;
    
    ParsedFeature feature;
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
            std::vector<std::pair<int, int>> currentRing;
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
                        break;
                    default: // Unknown command
                        // Skip remaining coordinates for this command
                        for(int j=0; j<count; ++j) {
                            varint(data, currentFeatureIdx, geomDataEnd); // x
                            varint(data, currentFeatureIdx, geomDataEnd); // y
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
            }
        } else if (field == MVT_FEATURE_ID && type == 0) {
            varint(data, currentFeatureIdx, fend);
        } else {
            if (type == 0) varint(data, currentFeatureIdx, fend);
            else if (type == 2) currentFeatureIdx += varint(data, currentFeatureIdx, fend);
            else if (type == 5) currentFeatureIdx += 4;
            else if (type == 1) currentFeatureIdx += 8;
        }
    }

    // --- COLOR ASSIGNMENT LOGIC (done once during parsing) ---
    feature.color = TFT_WHITE; // Default color
    if (layer.name == "water") {
        feature.color = TFT_BLUE;
        feature.isPolygon = true;
    } else if (layer.name == "landcover") {
        if (feature.properties.count("class")) {
            String lcClass = feature.properties["class"];
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
            String luClass = feature.properties["class"];
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
            String transportClass = feature.properties["class"];
            if (transportClass == "motorway" || transportClass == "trunk") {
                feature.color = TFT_RED;
            } else if (transportClass == "street" || transportClass == "primary" || transportClass == "secondary") {
                feature.color = TFT_LIGHTGREY;
            } else if (transportClass == "rail") {
                feature.color = TFT_DARKCYAN;
            }
        } else if (feature.properties.count("highway")) { // common in OpenStreetMap data
             String highwayType = feature.properties["highway"];
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
            String poiClass = feature.properties["class"];
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

    layer.features.push_back(feature);
  }
  return layer;
}

// Parses the entire MVT tile into our structured format and stores it under the given TileKey.
void parseMVTForTile(const std::vector<uint8_t> &data, const TileKey& key) {
  std::vector<ParsedLayer> currentTileLayers; // Temporarily store layers for this tile
  size_t i = 0;
  while (i < data.size()) {
    uint64_t tag = varint(data.data(), i, data.size());
    int field = tag >> 3;
    int type = tag & 0x07;

    if (field == 3 && type == 2) { // Layer field (tag 3, wire type 2 = Length-delimited)
      size_t len = varint(data.data(), i, data.size()); // Get layer data length
      currentTileLayers.push_back(parseLayer(data.data() + i, len)); // Parse the layer and add to our vector
      i += len;                            // Advance index past this layer's data
      vTaskDelay(1); // Yield after parsing each layer to prevent watchdog timeout
    } else if (type == 2) { // Generic length-delimited field (skip if not layer)
      size_t len = varint(data.data(), i, data.size());
      i += len;
    } else { // Generic fixed-length or varint field (skip)
      varint(data.data(), i, data.size()); // Just consume its value
    }
  }
  // After parsing, grab the extent from the first layer (assuming it's consistent across all layers/tiles)
  if (!currentTileLayers.empty()) {
      // Acquire mutex before writing to shared data
      if (xSemaphoreTake(loadedTilesDataMutex, portMAX_DELAY) == pdTRUE) {
          loadedTilesData[key] = currentTileLayers; // Store the parsed layers for this tile
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
bool fetchTile(sqlite3 *db, int z, int x, int y, std::vector<uint8_t> &out) {
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
    int len = sqlite3_column_bytes(stmt, 0); // Corrected: Pass stmt instead of blob
    out.assign((const uint8_t *)blob, (const uint8_t *)blob + len); // Corrected uint8_2 to uint8_t
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
// RENDERING FUNCTION (Render Task will use this)
// =========================================================

// Renders all currently loaded tile data with the active overzoom settings
void renderTileWithOverzoom(const RenderParams& params) {
  sprite.fillScreen(TFT_BLACK); // Clear the off-screen sprite buffer for new drawing

  // Acquire mutex before reading shared data
  if (xSemaphoreTake(loadedTilesDataMutex, portMAX_DELAY) == pdTRUE) {
      if (!loadedTilesData.empty()) {
        for (const auto& pair : loadedTilesData) {
            const TileKey& tileKey = pair.first;
            const std::vector<ParsedLayer>& layers = pair.second;
            for (const auto& layer : layers) {
                for (const auto& feature : layer.features) {
                    drawParsedFeature(feature, layer.extent, tileKey, params);
                }
            }
        }
      } else {
        Serial.println("No parsed tile data to render.");
      }
      xSemaphoreGive(loadedTilesDataMutex); // Release mutex
  } else {
      Serial.println("❌ Failed to acquire mutex for loadedTilesData during rendering.");
  }
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

    // Internal variables for dataTask to manage current state
    double internalCurrentTargetLat = 12.8273; // Default starting coordinates
    double internalCurrentTargetLon = 80.2193; // Default starting coordinates
    float internalCurrentZoomFactor = 1.0; // Default zoom factor
    float internalCurrentRotationAngle = 0.0f; // New: to store current rotation from compass
    float lastSentRotationAngle = 0.0f; // Stores the last rotation angle sent to render task
    float lastSentZoomFactor = 0.0f; // Stores the last zoom factor sent to render task

    // Variables to track the currently loaded central tile (for optimization)
    int loadedCentralTileX = -1;
    int loadedCentralTileY_TMS = -1;
    
    // MVT pixel coordinates of the target point relative to the central tile's MVT extent.
    int internalTargetPointMVT_X = 0;
    int internalTargetPointMVT_Y = 0;

    ControlParams receivedControlParams;

    while (true) {
        unsigned long taskLoopStartTime = millis();

        // Check for new control parameters from the main loop
        if (xQueueReceive(controlParamsQueue, &receivedControlParams, 0) == pdPASS) {
            internalCurrentTargetLat = receivedControlParams.targetLat;
            internalCurrentTargetLon = receivedControlParams.targetLon;
            internalCurrentZoomFactor = receivedControlParams.zoomFactor;
            Serial.printf("Data Task: Received new control params: Lat %.6f, Lon %.6f, Zoom %.1fx\n",
                          internalCurrentTargetLat, internalCurrentTargetLon, internalCurrentZoomFactor);
        }
        
        int newCentralTileX, newCentralTileY_std, newCentralTileY_TMS;
        latlonToTile(internalCurrentTargetLat, internalCurrentTargetLon, currentTileZ, newCentralTileX, newCentralTileY_std, newCentralTileY_TMS);

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

        // Check if the central tile has changed
        if (newCentralTileX != loadedCentralTileX || newCentralTileY_TMS != loadedCentralTileY_TMS) {
            Serial.printf("\nData Task: Central tile changed from %d,%d to %d,%d. Managing tiles.\n",
                          loadedCentralTileX, loadedCentralTileY_TMS, newCentralTileX, newCentralTileY_TMS);
            
            loadedCentralTileX = newCentralTileX;
            loadedCentralTileY_TMS = newCentralTileY_TMS;

            // Close existing DB connection if any
            if (mbtiles_db) {
                sqlite3_close(mbtiles_db);
                mbtiles_db = nullptr;
                Serial.println("Data Task: MBTiles database closed (previous).");
            }

            char path[96];
            snprintf(path, sizeof(path), "/sdcard/tile/%d_%d.mbtiles", (int)floor(internalCurrentTargetLat), (int)floor(internalCurrentTargetLon));
            Serial.printf("Data Task: Attempting to open MBTiles file: %s\n", path);

            if (sqlite3_open(path, &mbtiles_db) != SQLITE_OK) {
                Serial.printf("❌ Data Task: Failed to open MBTiles database: %s\n", sqlite3_errmsg(mbtiles_db));
                mbtiles_db = nullptr;
                Serial.println("Data Task: Error: Could not open MBTiles file. Please ensure it exists on the SD card.");
                vTaskDelay(pdMS_TO_TICKS(5000)); // Wait before retrying
                continue; // Skip to next loop iteration
            }
            Serial.println("✅ Data Task: MBTiles database opened successfully.");

            // --- Optimized Tile Management: Evict old, load new ---
            std::set<TileKey> requiredTileKeys;
            // Add central tile
            requiredTileKeys.insert({currentTileZ, loadedCentralTileX, loadedCentralTileY_TMS});

            // Add 8 neighbors to the required set
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    if (dx == 0 && dy == 0) continue; // Skip central tile, already added
                    int neighborX = loadedCentralTileX + dx;
                    int neighborY_TMS = loadedCentralTileY_TMS + dy;
                    int maxTileCoord = (1 << currentTileZ) - 1;
                    if (neighborX >= 0 && neighborX <= maxTileCoord && neighborY_TMS >= 0 && neighborY_TMS <= maxTileCoord) {
                        requiredTileKeys.insert({currentTileZ, neighborX, neighborY_TMS});
                    }
                }
            }

            // Evict tiles that are no longer needed
            if (xSemaphoreTake(loadedTilesDataMutex, portMAX_DELAY) == pdTRUE) {
                std::vector<TileKey> keysToEvict;
                for (const auto& pair : loadedTilesData) {
                    if (requiredTileKeys.find(pair.first) == requiredTileKeys.end()) {
                        keysToEvict.push_back(pair.first);
                    }
                }
                for (const auto& key : keysToEvict) {
                    loadedTilesData.erase(key);
                    Serial.printf("Data Task: Evicted tile Z:%d X:%d Y:%d (TMS)\n", key.z, key.x, key.y_tms);
                }
                xSemaphoreGive(loadedTilesDataMutex);
            } else {
                Serial.println("❌ Data Task: Failed to acquire mutex for tile eviction.");
            }
            vTaskDelay(1); // Yield after eviction

            // Load missing tiles
            for (const auto& key : requiredTileKeys) {
                bool alreadyLoaded = false;
                if (xSemaphoreTake(loadedTilesDataMutex, portMAX_DELAY) == pdTRUE) {
                    alreadyLoaded = (loadedTilesData.find(key) != loadedTilesData.end());
                    xSemaphoreGive(loadedTilesDataMutex);
                } else {
                    Serial.println("❌ Data Task: Failed to acquire mutex for loadedTilesData check during new tile load.");
                }

                if (!alreadyLoaded) { 
                    std::vector<uint8_t> tileData;
                    unsigned long parseStartTime = millis();
                    if (mbtiles_db && fetchTile(mbtiles_db, key.z, key.x, key.y_tms, tileData)) {
                        Serial.printf("✅ Data Task: Loading new tile Z:%d X:%d Y:%d (TMS), size: %u bytes. Parsing...\n", key.z, key.x, key.y_tms, tileData.size());
                        parseMVTForTile(tileData, key); // parseMVTForTile handles mutex internally and yields
                        unsigned long parseEndTime = millis();
                        Serial.printf("Data Task: Parsing tile Z:%d X:%d Y:%d took %lu ms.\n", key.z, key.x, key.y_tms, parseEndTime - parseStartTime);
                    } else {
                        Serial.printf("❌ Data Task: Tile Z:%d X:%d Y:%d (TMS) not found or fetch failed.\n", key.z, key.x, key.y_tms);
                    }
                }
                vTaskDelay(1); // Yield after processing each required tile
            }
            // --- End Optimized Tile Management ---

            // Close the database connection after all necessary tiles are loaded for this display cycle
            if (mbtiles_db) {
                sqlite3_close(mbtiles_db); 
                mbtiles_db = nullptr;
                Serial.println("Data Task: MBTiles database closed (after tile loading cycle).");
            }
        }

        // Calculate the MVT coordinates of the target point within the fetched central tile
        latLonToMVTCoords(internalCurrentTargetLat, internalCurrentTargetLon, currentTileZ, loadedCentralTileX, newCentralTileY_TMS, 
                        internalTargetPointMVT_X, internalTargetPointMVT_Y, currentLayerExtent);
        // Serial.printf("Data Task: Target point MVT coords within central tile (0-%d extent): X=%d, Y=%d\n", currentLayerExtent, internalTargetPointMVT_X, internalTargetPointMVT_Y);


        // Prepare and send rendering parameters to the render task
        RenderParams paramsToSend;
        paramsToSend.centralTileX = loadedCentralTileX;
        paramsToSend.centralTileY_TMS = loadedCentralTileY_TMS;
        paramsToSend.targetPointMVT_X = internalTargetPointMVT_X;
        paramsToSend.targetPointMVT_Y = internalTargetPointMVT_Y;
        paramsToSend.layerExtent = currentLayerExtent; // Use the global currentLayerExtent
        paramsToSend.zoomScaleFactor = internalCurrentZoomFactor; 
        paramsToSend.mapRotationDegrees = internalCurrentRotationAngle; // Pass the current compass heading
        
        // Calculate offsets to keep the specific target point centered during zoom.
        float scaledPointX_global = (float)paramsToSend.targetPointMVT_X * (screenW * paramsToSend.zoomScaleFactor / paramsToSend.layerExtent);
        float scaledPointY_global = (float)paramsToSend.targetPointMVT_Y * (screenH * paramsToSend.zoomScaleFactor / paramsToSend.layerExtent);

        paramsToSend.displayOffsetX = round(scaledPointX_global - (screenW / 2.0f));
        paramsToSend.displayOffsetY = round(scaledPointY_global - (screenH / 2.0f));

        // Send parameters to the render queue ONLY if rotation has changed by 1 degree or more,
        // or if other parameters (like location or zoom) have changed.
        // We use a small epsilon for float comparison to account for precision issues.
        if (fabs(internalCurrentRotationAngle - lastSentRotationAngle) >= 1.0f ||
            fabs(internalCurrentZoomFactor - lastSentZoomFactor) >= 0.01f || // Check for zoom change
            newCentralTileX != loadedCentralTileX || newCentralTileY_TMS != loadedCentralTileY_TMS)
        {
            if (xQueueSend(renderParamsQueue, &paramsToSend, 0) != pdPASS) {
                // Serial.println("Data Task: Failed to send render parameters to queue. Queue full?");
            } else {
                lastSentRotationAngle = internalCurrentRotationAngle; // Update last sent angle only if sent
                lastSentZoomFactor = internalCurrentZoomFactor; // Update last sent zoom factor
            }
        }

        unsigned long taskLoopEndTime = millis();
        // Serial.printf("Data Task: Loop took %lu ms.\n", taskLoopEndTime - taskLoopStartTime);
        vTaskDelay(pdMS_TO_TICKS(100)); // Run every 100ms
    }
}

// =========================================================
// RENDER TASK (Core 1)
// Handles display updates
// =========================================================
void renderTask(void *pvParameters) {
    Serial.println("Render Task: Initializing TFT...");
    tft.begin();
    tft.setRotation(1); // Set display to landscape mode (should be 160x128 if native is 128x160)
    tft.fillScreen(TFT_BLACK); 

    Serial.printf("Render Task: TFT Reported Width (after rotation): %d\n", tft.width());
    Serial.printf("Render Task: TFT Reported Height (after rotation): %d\n", tft.height());

    sprite.setColorDepth(16); // Set sprite color depth (RGB565, good balance of speed/quality)
    sprite.createSprite(screenW, screenH); // Create the 160x128 off-screen sprite

    RenderParams currentRenderParams; // Store the last received parameters

    while (true) {
        unsigned long renderLoopStartTime = millis();
        // Try to receive new rendering parameters from the queue
        if (xQueueReceive(renderParamsQueue, &currentRenderParams, portMAX_DELAY) == pdPASS) {
            // New parameters received, render the map
            renderTileWithOverzoom(currentRenderParams);

            sprite.setTextColor(TFT_YELLOW, TFT_BLACK);
            sprite.setCursor(5, 5);
            sprite.printf("Zoom: %.1fx", currentRenderParams.zoomScaleFactor); // Display the current zoom factor
            sprite.setCursor(5, 15); // New line for heading
            sprite.printf("Hdg: %.1f deg", currentRenderParams.mapRotationDegrees);
            
            sprite.pushSprite(0, 0); // Push the completed sprite to the physical display
            unsigned long renderLoopEndTime = millis();
            Serial.printf("Render Task: Map rendered in %lu ms.\n", renderLoopEndTime - renderLoopStartTime);
        }
        // If no new parameters, the task will block on xQueueReceive until data is available.
        // This ensures the render task only renders when there's new data to display.
    }
}

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

  // Initialize HMC5883L compass
  Serial.println("Main Setup: Initializing HMC5883L compass...");
  Wire.begin(); // Initialize I2C
  if (!hmc5883l.begin()) {
    Serial.println("❌ Main Setup: Could not find a valid HMC5883L sensor, check wiring!");
  } else {
    Serial.println("✅ Main Setup: HMC5883L sensor found.");
    // The Adafruit_HMC5883_Unified library uses setMagGain for setting the gain/range.
    // The mode is typically handled internally or set via begin().
    // Available gains are: HMC5883_MAGGAIN_1_3, HMC5883_MAGGAIN_1_9, HMC5883_MAGGAIN_2_5,
    // HMC5883_MAGGAIN_4_0, HMC5883_MAGGAIN_4_7, HMC5883_MAGGAIN_5_6, HMC5883_MAGGAIN_8_1
    hmc5883l.setMagGain(HMC5883_MAGGAIN_1_3); // Set gain to +/- 1.3 Gauss (default)
  }


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

      ControlParams newControlParams;
      bool paramsUpdated = false;

      // Initialize with current values so only changed values are sent
      // This requires dataTask to send its current state back or loop to track it.
      // For simplicity, let's assume default values for initial state,
      // and subsequent updates will override.
      newControlParams.targetLat = 12.8273; // Default
      newControlParams.targetLon = 80.2193; // Default
      newControlParams.zoomFactor = 1.0;    // Default

      // In a more complex system, you might retrieve the *current* values from dataTask
      // if you want to modify them incrementally. For this setup, we're just sending
      // a complete new set of parameters.

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
          }
      }
  }
  vTaskDelay(pdMS_TO_TICKS(100)); // Small delay to prevent busy-waiting in loop
}
