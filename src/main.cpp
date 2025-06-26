#include <Arduino.h>
#include <SPI.h>
#include <FS.h>
#include "SD_MMC.h"
#include <sqlite3.h>
#include <vector>
#include <map>
#include <TFT_eSPI.h> // Make sure this library is installed
#include <algorithm>  // For std::min, std::max, std::sort, std::swap

// =========================================================
// TFT DISPLAY AND SPRITE SETUP
// =========================================================
TFT_eSPI tft = TFT_eSPI();              // TFT display object
TFT_eSprite sprite = TFT_eSprite(&tft); // DMA sprite for smooth rendering

// These define the logical screen dimensions AFTER rotation
static int screenW = 160;
static int screenH = 128;

// Actual zoom level of the tile fetched from MBTiles
static int currentTileZ = 17;
// Actual X and TMS Y coordinates of the tile fetched
static int currentTileX, currentTileY_TMS;

// The conceptual zoom level we want to DISPLAY (can be higher than currentTileZ for overzooming)
static int displayZoomLevel = 17;
// How much to scale the fetched tile (e.g., 2.0 for Z+1, 4.0 for Z+2)
static float zoomScaleFactor = 1.0;

// Pixel offset for panning within the magnified tile (relative to screenW/H)
static int displayOffsetX = 0;
static int displayOffsetY = 0;

// Global SQLite DB handle (careful with this for multi-threading, but fine for single-threaded loop)
static sqlite3 *mbtiles_db = nullptr;

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
    int extent; // Tile extent for this layer
    std::vector<ParsedFeature> features;
};

// =========================================================
// ANIMATION AND BENCHMARKING PARAMETERS
// =========================================================
const float ANIMATION_DURATION_MS = 5000; // 5 seconds for the zoom animation
const float TARGET_ZOOM_LEVELS = 2;       // How many levels to zoom in (e.g., 2 = Z17->Z19)
static unsigned long animationStartTime = 0;
static bool animationFinished = false;

static unsigned long frameCount = 0;
static unsigned long benchmarkStartTime = 0;
static float averageFPS = 0.0;

static std::vector<uint8_t> tileData; // Stores the raw MVT tile data from SD card
static std::vector<ParsedLayer> parsedMVTData; // Global storage for parsed data

// =========================================================
// MVT DECODING HELPER FUNCTIONS
// =========================================================

// Decodes a variable-length integer (Varint) as per Protocol Buffers
inline uint64_t varint(const uint8_t *data, size_t &i) {
  uint64_t result = 0;
  int shift = 0;
  // Ensure we don't read beyond the data buffer
  while (i < tileData.size()) { // Use tileData.size() as overall buffer limit
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
String readString(const uint8_t *data, size_t &i) {
  uint64_t len = varint(data, i); // Get string length
  if (i + len > tileData.size()) { // Prevent reading past buffer
      len = tileData.size() - i;
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
void renderRing(const std::vector<std::pair<int, int>>& points, uint16_t color, bool isPolygon) {
  if (points.empty()) return;

  // Draw the outline of the ring for ALL cases (polygons or lines)
  if (points.size() == 1) { // It's a single point
      sprite.drawPixel(points[0].first, points[0].second, color);
  } else { // It's a line string or a polygon ring
      for (size_t k = 0; k < points.size() - 1; ++k) {
          sprite.drawLine(points[k].first, points[k].second, points[k+1].first, points[k+1].second, color);
      }
      // Ensure the loop is closed if it's a polygon or a closed line string
      if (isPolygon && points.size() > 1 && (points.front().first != points.back().first || points.front().second != points.back().second)) {
          sprite.drawLine(points.back().first, points.back().second, points.front().first, points.front().second, color);
      }
  }

  if (isPolygon) { // If it's a polygon feature, perform basic scanline fill
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
              int x2 = points[(k + 1) % points.size()].first; // Next point, wrapping around for closing
              int y2 = points[(k + 1) % points.size()].second;

              if (y1 == y2) continue; // Horizontal edge
              if ((scanY < std::min(y1, y2)) || (scanY >= std::max(y1, y2))) continue; // Scanline outside edge's Y range

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

              if (startX <= endX) { // Only draw if segment is valid
                  sprite.drawFastHLine(startX, scanY, endX - startX + 1, color);
              }
          }
      }
  }
}

// New function to draw a pre-parsed feature. It transforms MVT coordinates
// to screen coordinates and then calls renderRing.
void drawParsedFeature(const ParsedFeature& feature, int layerExtent) {
    float scaleX = (float)screenW * zoomScaleFactor / layerExtent;
    float scaleY = (float)screenH * zoomScaleFactor / layerExtent;

    // --- Bounding Box Culling Optimization ---
    // Transform the MVT bounding box to screen coordinates
    int screenMinX = round(feature.minX_mvt * scaleX - displayOffsetX);
    int screenMinY = round(feature.minY_mvt * scaleY - displayOffsetY);
    int screenMaxX = round(feature.maxX_mvt * scaleX - displayOffsetX);
    int screenMaxY = round(feature.maxY_mvt * scaleY - displayOffsetY);

    // Check if the feature's bounding box is outside the screen. If so, skip rendering.
    if (screenMaxX < 0 || screenMinX >= screenW || screenMaxY < 0 || screenMinY >= screenH) {
        // Serial.printf("Culling feature: minX:%d, minY:%d, maxX:%d, maxY:%d\n", screenMinX, screenMinY, screenMaxX, screenMaxY);
        return;
    }
    // --- End Bounding Box Culling ---

    for (const auto& ring : feature.geometryRings) {
        std::vector<std::pair<int, int>> screenPoints;
        screenPoints.reserve(ring.size()); // Pre-allocate memory

        for (const auto& p : ring) {
            // Use integer arithmetic where possible after initial float scale
            int px = (int)round(p.first * scaleX) - displayOffsetX;
            int py = (int)round(p.second * scaleY) - displayOffsetY;
            screenPoints.push_back({px, py});
        }
        renderRing(screenPoints, feature.color, feature.isPolygon);
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

  std::vector<std::pair<size_t, size_t>> featureOffsets;

  // First pass: Parse layer metadata (name, extent, keys, values) and collect feature offsets
  while (i < len) {
    uint64_t tag = varint(data, i);
    int field = tag >> 3;
    int type = tag & 0x07;

    switch (field) {
      case MVT_LAYER_NAME: layer.name = readString(data, i); break;
      case MVT_LAYER_FEATURE: {
        size_t featureLen = varint(data, i);
        featureOffsets.push_back({i, featureLen});
        i += featureLen;
        break;
      }
      case MVT_LAYER_KEYS: layerKeys.push_back(readString(data, i)); break;
      case MVT_LAYER_VALUES: {
        size_t valLen = varint(data, i);
        size_t valEnd = i + valLen;
        while (i < valEnd) {
          uint64_t tag2 = varint(data, i);
          int f = tag2 >> 3, t = tag2 & 0x07;
          if (f == 1 && t == 2) { // string_value (field 1, wire type 2 = Length-delimited)
            layerValues.push_back(readString(data, i));
          } else { // Skip other unknown value types
            if (t == 0) varint(data, i);
            else if (t == 2) i += varint(data, i);
            else if (t == 5) i += 4;
            else if (t == 1) i += 8;
          }
        }
        break;
      }
      case MVT_LAYER_EXTENT: layer.extent = varint(data, i); break;
      default: // Skip unknown layer fields
        if (type == 0) varint(data, i);
        else if (type == 2) i += varint(data, i);
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
        uint64_t tag = varint(data, currentFeatureIdx);
        int field = tag >> 3;
        int type = tag & 0x07;

        if (field == MVT_FEATURE_GEOMETRY_TYPE && type == 0) {
            feature.geomType = varint(data, currentFeatureIdx);
            if (feature.geomType == 3) feature.isPolygon = true;
        } else if (field == MVT_FEATURE_GEOMETRY && type == 2) {
            size_t geomDataLen = varint(data, currentFeatureIdx);
            size_t geomDataEnd = currentFeatureIdx + geomDataLen;

            int x = 0, y = 0;
            std::vector<std::pair<int, int>> currentRing;
            while (currentFeatureIdx < geomDataEnd) {
                uint64_t cmdlen = varint(data, currentFeatureIdx);
                int cmd = cmdlen & 0x7;
                int count = cmdlen >> 3;

                switch (cmd) {
                    case MVT_CMD_MOVETO:
                        if (!currentRing.empty()) {
                            feature.geometryRings.push_back(currentRing);
                            currentRing.clear();
                        }
                        for (int j = 0; j < count; ++j) {
                            x += zigzag(varint(data, currentFeatureIdx));
                            y += zigzag(varint(data, currentFeatureIdx));
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
                            x += zigzag(varint(data, currentFeatureIdx));
                            y += zigzag(varint(data, currentFeatureIdx));
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
                            varint(data, currentFeatureIdx); // x
                            varint(data, currentFeatureIdx); // y
                        }
                        break;
                }
            }
            if (!currentRing.empty()) feature.geometryRings.push_back(currentRing);
        } else if (field == MVT_FEATURE_TAGS && type == 2) {
            size_t propLen = varint(data, currentFeatureIdx);
            size_t propEnd = currentFeatureIdx + propLen;
            while (currentFeatureIdx < propEnd) {
                uint64_t keyIdx = varint(data, currentFeatureIdx);
                uint64_t valIdx = varint(data, currentFeatureIdx);
                if (keyIdx < layerKeys.size() && valIdx < layerValues.size()) {
                    feature.properties[layerKeys[keyIdx]] = layerValues[valIdx];
                }
            }
        } else if (field == MVT_FEATURE_ID && type == 0) {
            varint(data, currentFeatureIdx);
        } else {
            if (type == 0) varint(data, currentFeatureIdx);
            else if (type == 2) currentFeatureIdx += varint(data, currentFeatureIdx);
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

// Parses the entire MVT tile into our structured format.
void parseMVT(const std::vector<uint8_t> &data) {
  parsedMVTData.clear(); // Clear any data from a previous tile
  size_t i = 0;
  while (i < data.size()) {
    uint64_t tag = varint(data.data(), i);
    int field = tag >> 3;
    int type = tag & 0x07;

    if (field == 3 && type == 2) { // Layer field (tag 3, wire type 2 = Length-delimited)
      size_t len = varint(data.data(), i); // Get layer data length
      parsedMVTData.push_back(parseLayer(data.data() + i, len)); // Parse the layer and add to our vector
      i += len;                            // Advance index past this layer's data
    } else if (type == 2) { // Generic length-delimited field (skip if not layer)
      size_t len = varint(data.data(), i);
      i += len;
    } else { // Generic fixed-length or varint field (skip)
      varint(data.data(), i); // Just consume its value
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
    int len = sqlite3_column_bytes(stmt, 0);
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

// Renders the currently loaded tile data with the active overzoom settings
void renderTileWithOverzoom() {
  sprite.fillScreen(TFT_BLACK); // Clear the off-screen sprite buffer for new drawing

  if (!parsedMVTData.empty()) {
    // Iterate through layers and features to draw them
    // You might want to sort layers by type (e.g., draw polygons first, then lines, then points)
    // to ensure proper z-ordering if features overlap.
    for (const auto& layer : parsedMVTData) {
        for (const auto& feature : layer.features) {
            drawParsedFeature(feature, layer.extent);
        }
    }
  } else {
    Serial.println("No parsed tile data to render.");
  }
}


// =========================================================
// ARDUINO SETUP AND LOOP
// =========================================================

void setup() {
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

  double lat = 12.7839; // Chennai, India approximate
  double lon = 80.2463;
  
  int y_tms_unused_for_display;
  latlonToTile(lat, lon, currentTileZ, currentTileX, y_tms_unused_for_display, currentTileY_TMS);

  Serial.printf("Target Lat: %.4f, Lon: %.4f at base Z: %d -> Tile X:%d, Y:%d (TMS Y:%d)\n",
                lat, lon, currentTileZ, currentTileX, y_tms_unused_for_display, currentTileY_TMS);

  char path[96];
  snprintf(path, sizeof(path), "/sdcard/tile/%d_%d.mbtiles", (int)floor(lat), (int)floor(lon));
  Serial.printf("Attempting to open MBTiles file: %s\n", path);

  // Open the database globally if you plan to keep it open
  if (sqlite3_open(path, &mbtiles_db) != SQLITE_OK) {
    Serial.printf("❌ Failed to open MBTiles database: %s\n", sqlite3_errmsg(mbtiles_db));
    mbtiles_db = nullptr; // Ensure it's null if open failed
    return;
  }
  Serial.println("✅ MBTiles database opened successfully.");

  Serial.println("Fetching tile data from database...");
  if (mbtiles_db && fetchTile(mbtiles_db, currentTileZ, currentTileX, currentTileY_TMS, tileData)) {
    Serial.printf("✅ Tile data fetched, size: %u bytes. Now parsing...\n", tileData.size());
    parseMVT(tileData); // Parse the MVT data once after fetching
  } else {
    Serial.println("❌ Tile not found in database or fetch failed. Check ZXY and file path.");
    if (mbtiles_db) {
        sqlite3_close(mbtiles_db); // Close DB on fetch failure
        mbtiles_db = nullptr;
    }
    return;
  }
  // For a single fetch and then animation, it's generally fine to close the DB.
  // If you were panning/loading more tiles, you'd keep mbtiles_db open.
  if (mbtiles_db) {
      sqlite3_close(mbtiles_db); 
      mbtiles_db = nullptr;
      Serial.println("MBTiles database closed.");
  }
  
  displayZoomLevel = currentTileZ;
  zoomScaleFactor = 1.0;
  displayOffsetX = 0;
  displayOffsetY = 0;
  renderTileWithOverzoom();
  sprite.pushSprite(0, 0);
  delay(1000);

  Serial.println("\nSetup complete. Starting smooth zoom animation in loop().");
}

void loop() {
  if (animationFinished) {
    delay(1000);
    return;
  }

  if (animationStartTime == 0) {
      animationStartTime = millis();
      benchmarkStartTime = millis();
      frameCount = 0;
      Serial.printf("🚀 Starting smooth zoom animation over %.1f seconds...\n", ANIMATION_DURATION_MS / 1000.0f);
  }

  unsigned long currentTime = millis();
  float elapsed = currentTime - animationStartTime;

  float progress = elapsed / ANIMATION_DURATION_MS;
  if (progress >= 1.0f) {
      progress = 1.0f;
      animationFinished = true;
  }

  float startScale = 1.0f;
  float endScale = pow(2, TARGET_ZOOM_LEVELS);

  zoomScaleFactor = startScale + (endScale - startScale) * progress;
  displayZoomLevel = currentTileZ + log2(zoomScaleFactor);

  // Calculate offsets to keep the zoom centered on the screen
  // (screenW / 2) is the center of the screen
  // (screenW * zoomScaleFactor / 2) is the center of the magnified tile area
  displayOffsetX = (int)round((screenW * zoomScaleFactor / 2.0f) - (screenW / 2.0f));
  displayOffsetY = (int)round((screenH * zoomScaleFactor / 2.0f) - (screenH / 2.0f));

  renderTileWithOverzoom();

  frameCount++;
  unsigned long totalElapsed = currentTime - benchmarkStartTime;
  if (totalElapsed > 0) {
      averageFPS = (float)frameCount * 1000.0f / totalElapsed;
  }
  sprite.setTextColor(TFT_YELLOW, TFT_BLACK);
  sprite.setCursor(5, 5);
  sprite.printf("FPS: %.1f", averageFPS);
  sprite.setCursor(5, 15);
  sprite.printf("Zoom: %.2fx", zoomScaleFactor);

  sprite.pushSprite(0, 0); // Push the completed sprite to the physical display

  if (animationFinished) {
      unsigned long finalTime = millis() - benchmarkStartTime;
      Serial.println("\n✅ Animation Finished.");
      Serial.printf("  - Total time: %lu ms\n", finalTime);
      Serial.printf("  - Total frames: %lu\n", frameCount);
      Serial.printf("  - Average FPS: %.2f\n", averageFPS);
  }
}