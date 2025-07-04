// common.h
#ifndef COMMON_H
#define COMMON_H

#include <vector>     // For std::vector
#include <map>        // For std::map
#include <string>     // For std::basic_string and std::char_traits
#include <set>        // For std::set to manage active tile keys
#include <utility>    // For std::pair
#include <algorithm>  // For std::min, std::max, std::sort, std::swap
#include <cmath>      // For log2, pow, round, and PI (often defined here or Arduino.h)

// FreeRTOS includes for multi-core tasks and synchronization
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h" // For mutex
#include "freertos/queue.h"  // For queue

// PSRAM includes for memory monitoring
#include "esp_heap_caps.h" // For memory monitoring

// TFT Display Library
#include <TFT_eSPI.h> // Make sure this library is installed

// =========================================================
// GLOBAL TFT DISPLAY OBJECTS (Declared extern here, defined in main.cpp)
// =========================================================
extern TFT_eSPI tft;
extern TFT_eSprite sprite;

// These define the logical screen dimensions AFTER rotation
extern int screenW;
extern int screenH;

// Actual zoom level of the tile fetched from MBTiles
extern int currentTileZ;

// =========================================================
// GLOBAL SHARED DATA AND SYNCHRONIZATION OBJECTS
// =========================================================

// Custom allocator for std::vector, std::map, and std::string to use PSRAM
template <class T>
struct PSRAMAllocator {
    typedef T value_type;

    PSRAMAllocator() = default;
    template <class U> constexpr PSRAMAllocator(const PSRAMAllocator<U>&) noexcept {}

    T* allocate(std::size_t n) {
        if (n == 0) return nullptr;
        if (n > static_cast<std::size_t>(-1) / sizeof(T)) throw std::bad_alloc();
        void* p = heap_caps_malloc(n * sizeof(T), MALLOC_CAP_SPIRAM);
        if (p == nullptr) {
            // In a real application, you might want more robust error handling
            // than just printing to Serial, especially in an allocator.
            // For now, we'll keep the Serial print for debugging.
            // Serial.printf("❌ PSRAMAllocator: Failed to allocate %u bytes in PSRAM!\n", n * sizeof(T));
            throw std::bad_alloc();
        }
        return static_cast<T*>(p);
    }

    void deallocate(T* p, std::size_t n) noexcept {
        heap_caps_free(p);
    }
};

template <class T, class U>
bool operator==(const PSRAMAllocator<T>&, const PSRAMAllocator<U>&) { return true; }
template <class T, class U>
bool operator!=(const PSRAMAllocator<T>&, const PSRAMAllocator<U>&) { return false; }

// Define a PSRAM-allocated string type
using PSRAMString = std::basic_string<char, std::char_traits<char>, PSRAMAllocator<char>>;

// Define structs to hold parsed MVT data. This allows us to parse the tile
// once and then render the structured data many times, which is much faster.
struct ParsedFeature {
    int geomType; // 1=Point, 2=LineString, 3=Polygon
    // Geometry is stored as raw MVT coordinates (0-extent).
    // Each inner vector is a ring of points.
    // Use PSRAMAllocator for geometry data
    std::vector<std::vector<std::pair<int, int>, PSRAMAllocator<std::pair<int, int>>>, PSRAMAllocator<std::vector<std::pair<int, int>, PSRAMAllocator<std::pair<int, int>>>>> geometryRings;
    // Properties map using PSRAMString for keys and values
    std::map<PSRAMString, PSRAMString> properties; 
    uint16_t color; // Pre-calculated color for fast rendering
    bool isPolygon; // Pre-calculated polygon flag
    // Add bounding box for culling
    int minX_mvt, minY_mvt, maxX_mvt, maxY_mvt; // Bounding box in MVT coordinates
};

struct ParsedLayer {
    PSRAMString name; // Layer name now uses PSRAMString
    int extent; // Tile extent for this layer (typically 4096)
    std::vector<ParsedFeature, PSRAMAllocator<ParsedFeature>> features; // Use PSRAMAllocator for features
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
    // Operator for equality comparison (useful for std::set::find)
    bool operator==(const TileKey& other) const {
        return z == other.z && x == other.x && y_tms == other.y_tms;
    }
};

// Global storage for parsed data from multiple tiles. PROTECTED BY MUTEX.
// The map itself is in internal RAM, but its values (vectors of ParsedLayer) are in PSRAM.
extern std::map<TileKey, std::vector<ParsedLayer, PSRAMAllocator<ParsedLayer>>> loadedTilesData;
extern SemaphoreHandle_t loadedTilesDataMutex; // Mutex to protect loadedTilesData

// Global variable for current layer extent, updated by dataTask, read by renderTask
extern int currentLayerExtent; // Default MVT tile extent, will be updated from parsed data

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
    int pivotY; // Added pivotY for map rotation pivot
    float cosRotationDegrees; // Added for optimization
    float sinRotationDegrees; // Added for optimization
};
// Removed: extern QueueHandle_t renderParamsQueue; // DataTask -> RenderTask (Removed, RenderTask now calculates its own)

// Structure for control parameters (Lat, Lon, Zoom) to be sent from loop() to dataTask
struct ControlParams {
    double targetLat;
    double targetLon;
    float zoomFactor;
};
extern QueueHandle_t controlParamsQueue;        // Loop -> RenderTask (For user input)
extern QueueHandle_t tileRequestQueue;          // RenderTask -> DataTask (New: for requesting tiles)
extern QueueHandle_t tileParsedNotificationQueue; // DataTask -> RenderTask (New: for notifying when tile is parsed)

#endif // COMMON_H
