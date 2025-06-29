#include <Arduino.h>
#include <SPI.h>
#include <FS.h>
#include "SD_MMC.h"
#include <sqlite3.h>
#include <vector>
#include <map>
#include <TFT_eSPI.h> // Make sure this library is installed
#include <algorithm>  // For std::min, std::max, std::sort, std::swap
#include <cmath>      // For log2, pow, round

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
// Actual X and TMS Y coordinates of the central tile
int currentTileX, currentTileY_TMS;

// The conceptual zoom level we want to DISPLAY (can be higher than currentTileZ for overzooming)
int displayZoomLevel = 17;
// How much to scale the fetched tile (e.g., 2.0 for Z+1, 4.0 for Z+2)
float zoomScaleFactor = 1.0;

// Pixel offset for panning within the magnified central tile (relative to screenW/H)
// This offset is applied to the entire rendered map, effectively moving the viewport.
int displayOffsetX = 0;
int displayOffsetY = 0;

// Global SQLite DB handle (careful with this for multi-threading, but fine for single-threaded loop)
sqlite3 *mbtiles_db = nullptr; // This will now be opened/closed per animation sequence

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

// Global storage for parsed data from multiple tiles
std::map<TileKey, std::vector<ParsedLayer>> loadedTilesData;

// =========================================================
// MANUAL ZOOM CONTROL PARAMETERS
// =========================================================
bool waitingForCoordInput = true; // State flag: true when waiting for Lat,Lon
bool waitingForZoomInput = false; // State flag: true when waiting for zoom factor

double currentTargetLat = 0.0; // Stores the last entered latitude
double currentTargetLon = 0.0; // Stores the last entered longitude

// Global variables to store the MVT pixel coordinates of the target point
// These are relative to the *central* tile's MVT extent.
int targetPointMVT_X = 0;
int targetPointMVT_Y = 0;
int currentLayerExtent = 4096; // Default MVT tile extent, will be updated from parsed data

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
// GEOMETRY DRAWING
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
void drawParsedFeature(const ParsedFeature& feature, int layerExtent, const TileKey& tileKey) {
    float scaleX = (float)screenW * zoomScaleFactor / layerExtent;
    float scaleY = (float)screenH * zoomScaleFactor / layerExtent;

    // Calculate the pixel offset for this specific tile relative to the central tile (currentTileX, currentTileY_TMS)
    // A full MVT extent (e.g., 4096 units) maps to `screenW` pixels at scale 1.0.
    // When zoomed, it maps to `screenW * zoomScaleFactor` pixels.
    float tileRenderWidth = (float)screenW * zoomScaleFactor;
    float tileRenderHeight = (float)screenH * zoomScaleFactor;

    // Calculate the pixel offset for this tile's (0,0) MVT coordinate relative to the central tile's (0,0) MVT coordinate
    float tileRenderOffsetX_float = (float)(tileKey.x - currentTileX) * tileRenderWidth;
    // Corrected: The difference in TMS Y directly corresponds to screen Y offset.
    // If tileKey.y_tms is smaller (further north), the offset should be negative (up).
    // If tileKey.y_tms is larger (further south), the offset should be positive (down).
    float tileRenderOffsetY_float = (float)(tileKey.y_tms - currentTileY_TMS) * tileRenderHeight;


    // --- Bounding Box Culling Optimization ---
    // Transform the MVT bounding box to screen coordinates, considering the tile's position
    // and the global display offset.
    float screenMinX_float = feature.minX_mvt * scaleX + tileRenderOffsetX_float - displayOffsetX;
    float screenMinY_float = (layerExtent - feature.maxY_mvt) * scaleY + tileRenderOffsetY_float - displayOffsetY; // Flipped Y for min
    float screenMaxX_float = feature.maxX_mvt * scaleX + tileRenderOffsetX_float - displayOffsetX;
    float screenMaxY_float = (layerExtent - feature.minY_mvt) * scaleY + tileRenderOffsetY_float - displayOffsetY; // Flipped Y for max

    // Check if the feature's bounding box is outside the screen. If so, skip rendering.
    if (screenMaxX_float < 0 || screenMinX_float >= screenW || screenMaxY_float < 0 || screenMinY_float >= screenH) {
        return;
    }
    // --- End Bounding Box Culling ---

    for (const auto& ring : feature.geometryRings) {
        std::vector<std::pair<int, int>> screenPoints;
        screenPoints.reserve(ring.size()); // Pre-allocate memory

        for (const auto& p : ring) {
            // Apply tile offset and then global display offset
            float px_float = p.first * scaleX + tileRenderOffsetX_float - displayOffsetX;
            // Invert MVT Y coordinate for screen Y (MVT Y is Y-up, screen Y is Y-down)
            float py_float = (layerExtent - p.second) * scaleY + tileRenderOffsetY_float - displayOffsetY;
            
            // Final inversion for the entire display output
            screenPoints.push_back({round(px_float), round(screenH - 1 - py_float)});
        }
        renderRing(screenPoints, feature.color, feature.isPolygon, feature.geomType); // Pass geomType
    }
}


// =========================================================
// MVT LAYER AND TILE DECODING
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

  Serial.printf("\n🗂️ Parsing Layer: %s (Features: %d, Extent: %d)\n", layer.name.c_str(), featureOffsets.size(), layer.extent);

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
            } else if (lcClass == "grass" || lcClass == "wood") {
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
    } else if (type == 2) { // Generic length-delimited field (skip if not layer)
      size_t len = varint(data.data(), i, data.size());
      i += len;
    } else { // Generic fixed-length or varint field (skip)
      varint(data.data(), i, data.size()); // Just consume its value
    }
  }
  // After parsing, grab the extent from the first layer (assuming it's consistent across all layers/tiles)
  if (!currentTileLayers.empty()) {
      currentLayerExtent = currentTileLayers[0].extent; // This assumes consistency
  }
  loadedTilesData[key] = currentTileLayers; // Store the parsed layers for this tile
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
    out.assign((const uint8_t *)blob, (const uint8_t *)blob + len);
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
// RENDERING FUNCTION
// =========================================================

// Renders all currently loaded tile data with the active overzoom settings
void renderTileWithOverzoom() {
  sprite.fillScreen(TFT_BLACK); // Clear the off-screen sprite buffer for new drawing

  if (!loadedTilesData.empty()) {
    // Iterate through all loaded tiles and their layers/features to draw them
    // You might want to sort layers by type (e.g., draw polygons first, then lines, then points)
    // to ensure proper z-ordering if features overlap.
    for (const auto& pair : loadedTilesData) {
        const TileKey& tileKey = pair.first;
        const std::vector<ParsedLayer>& layers = pair.second;
        for (const auto& layer : layers) {
            for (const auto& feature : layer.features) {
                drawParsedFeature(feature, layer.extent, tileKey);
            }
        }
    }
  } else {
    Serial.println("No parsed tile data to render.");
  }
}

// =========================================================
// Function to load and parse tiles for a new location
// =========================================================
void loadTilesForLocation(double newLat, double newLon) {
  Serial.println("\nLoading tiles for new location...");
  loadedTilesData.clear(); // Clear previously loaded tile data

  // Close existing DB connection if any
  if (mbtiles_db) {
      sqlite3_close(mbtiles_db);
      mbtiles_db = nullptr;
      Serial.println("MBTiles database closed (previous).");
  }

  char path[96];
  // Dynamically construct the MBTiles file path based on the floor of the newLat/newLon
  snprintf(path, sizeof(path), "/sdcard/tile/%d_%d.mbtiles", (int)floor(newLat), (int)floor(newLon));
  Serial.printf("Attempting to open MBTiles file: %s\n", path);

  // Open the database for the new coordinates
  if (sqlite3_open(path, &mbtiles_db) != SQLITE_OK) {
    Serial.printf("❌ Failed to open MBTiles database: %s\n", sqlite3_errmsg(mbtiles_db));
    mbtiles_db = nullptr; // Ensure it's null if open failed
    Serial.println("Error: Could not open MBTiles file. Please ensure it exists on the SD card.");
    waitingForCoordInput = true; // Stay in this state to re-prompt
    waitingForZoomInput = false;
    return;
  }
  Serial.println("✅ MBTiles database opened successfully.");

  int y_std_unused; // Standard Y, not TMS
  latlonToTile(newLat, newLon, currentTileZ, currentTileX, y_std_unused, currentTileY_TMS);

  Serial.printf("New Target Lat: %.4f, Lon: %.4f at base Z: %d -> Central Tile X:%d, Y:%d (TMS Y:%d)\n",
                newLat, newLon, currentTileZ, currentTileX, y_std_unused, currentTileY_TMS);
  
  // Always load the central tile first
  TileKey centralKey = {currentTileZ, currentTileX, currentTileY_TMS};
  std::vector<uint8_t> tempTileData;
  if (mbtiles_db && fetchTile(mbtiles_db, centralKey.z, centralKey.x, centralKey.y_tms, tempTileData)) {
      Serial.printf("✅ Central Tile Z:%d X:%d Y:%d (TMS) fetched, size: %u bytes. Now parsing...\n", centralKey.z, centralKey.x, centralKey.y_tms, tempTileData.size());
      parseMVTForTile(tempTileData, centralKey);
  } else {
      Serial.printf("❌ Central Tile Z:%d X:%d Y:%d (TMS) not found or fetch failed. Cannot proceed.\n", centralKey.z, centralKey.x, centralKey.y_tms);
      // Close DB on fetch failure for central tile
      if (mbtiles_db) {
          sqlite3_close(mbtiles_db); 
          mbtiles_db = nullptr;
          Serial.println("MBTiles database closed (central tile fetch failed).");
      }
      Serial.println("Error: Central tile not found or could not be fetched. Please check your MBTiles file.");
      waitingForCoordInput = true; // Stay in this state to re-prompt
      waitingForZoomInput = false;
      return;
  }

  // After central tile is loaded and parsed, currentLayerExtent is set.
  // Calculate the MVT coordinates of the target point within the fetched central tile
  latLonToMVTCoords(newLat, newLon, currentTileZ, currentTileX, currentTileY_TMS, 
                    targetPointMVT_X, targetPointMVT_Y, currentLayerExtent);
  Serial.printf("Target point MVT coords within central tile (0-%d extent): X=%d, Y=%d\n", currentLayerExtent, targetPointMVT_X, targetPointMVT_Y);

  // --- Optimized Neighbor Loading Logic ---
  // Define a threshold for how close to the edge the target point needs to be
  // to warrant loading a neighboring tile. 0.25 (25%) is a good starting point.
  const float EDGE_THRESHOLD_RATIO = 0.25f;
  const int EDGE_THRESHOLD_X = round(currentLayerExtent * EDGE_THRESHOLD_RATIO);
  const int EDGE_THRESHOLD_Y = round(currentLayerExtent * EDGE_THRESHOLD_RATIO);

  // List of (dx, dy) offsets for neighbors to consider loading
  std::vector<std::pair<int, int>> neighborOffsets;

  // Horizontal neighbors
  if (targetPointMVT_X > currentLayerExtent - EDGE_THRESHOLD_X) { // Close to right edge (high MVT X)
      neighborOffsets.push_back({1, 0}); // Load tile to the East (X+1)
  } else if (targetPointMVT_X < EDGE_THRESHOLD_X) { // Close to left edge (low MVT X)
      neighborOffsets.push_back({-1, 0}); // Load tile to the West (X-1)
  }

  // Vertical neighbors (MVT Y is Y-up, TMS Y is Y-down)
  // If target point is close to the MVT top (high MVT Y), load the northern tile (TMS Y - 1)
  if (targetPointMVT_Y > currentLayerExtent - EDGE_THRESHOLD_Y) {
      neighborOffsets.push_back({0, -1}); // Load tile to the North (TMS Y decreases)
  }
  // If target point is close to the MVT bottom (low MVT Y), load the southern tile (TMS Y + 1)
  else if (targetPointMVT_Y < EDGE_THRESHOLD_Y) {
      neighborOffsets.push_back({0, 1}); // Load tile to the South (TMS Y increases)
  }

  // Diagonal neighbors (only if both horizontal and vertical conditions met)
  // North-East (MVT X high, MVT Y high -> TMS X+1, TMS Y-1)
  if (targetPointMVT_X > currentLayerExtent - EDGE_THRESHOLD_X && targetPointMVT_Y > currentLayerExtent - EDGE_THRESHOLD_Y) {
      neighborOffsets.push_back({1, -1});
  }
  // North-West (MVT X low, MVT Y high -> TMS X-1, TMS Y-1)
  if (targetPointMVT_X < EDGE_THRESHOLD_X && targetPointMVT_Y > currentLayerExtent - EDGE_THRESHOLD_Y) {
      neighborOffsets.push_back({-1, -1});
  }
  // South-East (MVT X high, MVT Y low -> TMS X+1, TMS Y+1)
  if (targetPointMVT_X > currentLayerExtent - EDGE_THRESHOLD_X && targetPointMVT_Y < EDGE_THRESHOLD_Y) {
      neighborOffsets.push_back({1, 1});
  }
  // South-West (MVT X low, MVT Y low -> TMS X-1, TMS Y+1)
  if (targetPointMVT_X < EDGE_THRESHOLD_X && targetPointMVT_Y < EDGE_THRESHOLD_Y) {
      neighborOffsets.push_back({-1, 1});
  }

  // Fetch and parse the selected neighbors
  for (const auto& offset : neighborOffsets) {
      int neighborX = currentTileX + offset.first;
      int neighborY_TMS = currentTileY_TMS + offset.second;

      int maxTileCoord = (1 << currentTileZ) - 1;
      if (neighborX < 0 || neighborX > maxTileCoord || neighborY_TMS < 0 || neighborY_TMS > maxTileCoord) {
          Serial.printf("Skipping out-of-bounds neighbor Z:%d X:%d Y:%d (TMS)\n", currentTileZ, neighborX, neighborY_TMS);
          continue;
      }

      TileKey key = {currentTileZ, neighborX, neighborY_TMS};
      if (loadedTilesData.find(key) == loadedTilesData.end()) { 
          std::vector<uint8_t> neighborTileData;
          if (mbtiles_db && fetchTile(mbtiles_db, key.z, key.x, key.y_tms, neighborTileData)) {
              Serial.printf("✅ Neighbor Tile Z:%d X:%d Y:%d (TMS) fetched, size: %u bytes. Now parsing...\n", key.z, key.x, key.y_tms, neighborTileData.size());
              parseMVTForTile(neighborTileData, key);
          } else {
              Serial.printf("❌ Neighbor Tile Z:%d X:%d Y:%d (TMS) not found or fetch failed.\n", key.z, key.x, key.y_tms);
          }
      }
  }
  // --- End Optimized Neighbor Loading Logic ---

  // Close the database connection after all necessary tiles are loaded for this display
  if (mbtiles_db) {
      sqlite3_close(mbtiles_db); 
      mbtiles_db = nullptr;
      Serial.println("MBTiles database closed (after tile loading).");
  }

  // Successfully loaded tiles, now wait for zoom input
  waitingForCoordInput = false;
  waitingForZoomInput = true;
  Serial.println("\nTiles loaded. Enter zoom factor (1, 2, 3, or 4):");
}

// =========================================================
// Function to apply zoom and render the map
// =========================================================
void applyZoomAndRender(float zoomFactor) {
  zoomScaleFactor = zoomFactor;
  // displayZoomLevel is conceptually currentTileZ + log2(zoomScaleFactor)
  // For manual zoom, we just use the zoomScaleFactor directly for rendering.

  // Calculate offsets to keep the specific target point centered during zoom.
  // This calculation is based on the target point's MVT coordinates within its tile
  // and the current zoomScaleFactor.
  float scaledPointX_global = (float)targetPointMVT_X * (screenW * zoomScaleFactor / currentLayerExtent);
  // Apply the same Y-axis inversion to the global target point for consistent centering
  float scaledPointY_global = (float)(currentLayerExtent - targetPointMVT_Y) * (screenH * zoomScaleFactor / currentLayerExtent);

  displayOffsetX = round(scaledPointX_global - (screenW / 2.0f));
  displayOffsetY = round(scaledPointY_global - (screenH / 2.0f));

  renderTileWithOverzoom(); // Render the map with the new zoom and offset

  sprite.setTextColor(TFT_YELLOW, TFT_BLACK);
  sprite.setCursor(5, 5);
  sprite.printf("Zoom: %.1fx", zoomFactor); // Display the current zoom factor
  
  sprite.pushSprite(0, 0); // Push the completed sprite to the physical display
  Serial.printf("Map rendered at %.1fx zoom.\n", zoomFactor);
}


// =========================================================
// ARDUINO SETUP AND LOOP
// =========================================================

void setup() {
  unsigned long setupStartTime = millis(); // Start timing setup duration

  Serial.begin(115200);
  delay(100);

  Serial.println("Initializing SD_MMC...");
  if (!SD_MMC.begin()) {
    Serial.println("❌ SD_MMC mount failed. Check wiring and formatting.");
    return;
  }
  Serial.println("✅ SD_MMC mounted successfully.");

  Serial.println("Initializing TFT...");
  tft.begin();
  tft.setRotation(1); // Set display to landscape mode (should be 160x128 if native is 128x160)
  
  tft.fillScreen(TFT_BLACK); 

  Serial.printf("TFT Reported Width (after rotation): %d\n", tft.width());
  Serial.printf("TFT Reported Height (after rotation): %d\n", tft.height());

  sprite.setColorDepth(16); // Set sprite color depth (RGB565, good balance of speed/quality)
  sprite.createSprite(screenW, screenH); // Create the 160x128 off-screen sprite

  unsigned long setupDuration = millis() - setupStartTime; // Calculate setup duration
  Serial.printf("Initial hardware setup completed in %lu ms.\n", setupDuration); // Print setup duration

  // The first prompt for coordinates will happen in loop()
  Serial.println("\nReady. Enter coordinates to load map (Longitude, Latitude):"); // Updated prompt
}

void loop() {
  if (waitingForCoordInput) {
    // Check if Serial input is available for coordinates
    if (Serial.available() > 0) {
      String inputString;

      Serial.println("Reading Longitude, Latitude (e.g., 79.807507, 11.941773):"); // Updated prompt
      inputString = Serial.readStringUntil('\n');
      inputString.trim(); // Remove leading/trailing whitespace

      int commaIndex = inputString.indexOf(',');

      if (commaIndex != -1) {
        // Swapped latStr and lonStr assignment as requested
        String lonStr = inputString.substring(0, commaIndex);
        String latStr = inputString.substring(commaIndex + 1);

        double newLat = latStr.toFloat();
        double newLon = lonStr.toFloat();
    
        // Basic validation: Check if toFloat() returned 0.0 for non-zero strings
        bool latValid = (newLat != 0.0f || latStr.equals("0") || latStr.equals("0.0"));
        bool lonValid = (newLon != 0.0f || lonStr.equals("0") || lonStr.equals("0.0"));

        if (latValid && lonValid) {
            Serial.printf("Received Latitude: %.6f, Longitude: %.6f\n", newLat, newLon);
            currentTargetLat = newLat; // Store for later zoom application
            currentTargetLon = newLon; // Store for later zoom application
            loadTilesForLocation(currentTargetLat, currentTargetLon); // Load tiles for the new location
        } else {
            Serial.println("Invalid coordinate format or values. Please enter valid floating-point numbers separated by a comma.");
            // Stay in waitingForCoordInput state
        }
      } else {
        Serial.println("Invalid input format. Please enter Longitude and Latitude separated by a comma."); // Updated message
        // Stay in waitingForCoordInput state
      }
    } else {
      delay(100); // Small delay if no input
    }
  } else if (waitingForZoomInput) {
    // Check if Serial input is available for zoom factor
    if (Serial.available() > 0) {
      String inputString;
      Serial.println("Reading zoom factor (1, 2, 3, or 4):");
      inputString = Serial.readStringUntil('\n');
      inputString.trim();
      float requestedZoom = inputString.toFloat();

      // Validate input: must be an integer from 1 to 4
      if (requestedZoom >= 1.0f && requestedZoom <= 4.0f && fmod(requestedZoom, 1.0f) == 0.0f) {
          applyZoomAndRender(requestedZoom);
          // After rendering, go back to waiting for new coordinates
          waitingForZoomInput = false;
          waitingForCoordInput = true;
          Serial.println("\nEnter new coordinates to load map (Longitude, Latitude):"); // Updated prompt
      } else {
          Serial.println("Invalid zoom factor. Please enter 1, 2, 3, or 4.");
          // Stay in waitingForZoomInput state
      }
    } else {
      delay(100); // Small delay if no input
    }
  }
  // No continuous loop for animation, just waiting for input
}
