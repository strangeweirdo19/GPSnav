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

// Custom allocator for std::vector, std::map, std::string to use PSRAM
template <typename T>
struct PSRAMAllocator {
    using value_type = T;

    PSRAMAllocator() = default;
    template <typename U> PSRAMAllocator(const PSRAMAllocator<U>&) {}

    T* allocate(size_t num_objects) {
        if (num_objects == 0) return nullptr;
        size_t size_bytes = num_objects * sizeof(T);
        T* ptr = static_cast<T*>(heap_caps_malloc(size_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
        if (!ptr) {
            Serial.printf("❌ PSRAMAllocator: Failed to allocate %u bytes for %u objects of size %u. Free PSRAM: %u bytes\n", 
                          size_bytes, num_objects, sizeof(T), heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
            // Consider more robust error handling, e.g., throw std::bad_alloc
            abort(); // For critical allocations that cannot fail
        }
        return ptr;
    }

    void deallocate(T* p, size_t num_objects) {
        heap_caps_free(p);
    }
};

// Equality comparison for PSRAMAllocator
template <typename T1, typename T2>
bool operator==(const PSRAMAllocator<T1>&, const PSRAMAllocator<T2>&) { return true; }
template <typename T1, typename T2>
bool operator!=(const PSRAMAllocator<T1>&, const PSRAMAllocator<T2>&) { return false; }

// Define PSRAMString as std::string using PSRAMAllocator
using PSRAMString = std::basic_string<char, std::char_traits<char>, PSRAMAllocator<char>>;

// Structure to hold parsed MVT feature data
struct ParsedFeature {
    std::vector<std::vector<std::pair<int, int>, PSRAMAllocator<std::pair<int, int>>>, 
                PSRAMAllocator<std::vector<std::pair<int, int>, PSRAMAllocator<std::pair<int, int>>>>> geometryRings;
    std::map<PSRAMString, PSRAMString, std::less<PSRAMString>, PSRAMAllocator<std::pair<const PSRAMString, PSRAMString>>> properties;
    uint16_t color;
    bool isPolygon;
    int geomType; // 1: Point, 2: LineString, 3: Polygon
    int minX_mvt, minY_mvt, maxX_mvt, maxY_mvt; // Bounding box in MVT coordinates
    
    // Default constructor to ensure PSRAMAllocator is used for members
    ParsedFeature() : 
        geometryRings(PSRAMAllocator<std::vector<std::pair<int, int>, PSRAMAllocator<std::pair<int, int>>>>()),
        properties(PSRAMAllocator<std::pair<const PSRAMString, PSRAMString>>()) {}
};

// Structure to hold parsed MVT layer data
struct ParsedLayer {
    PSRAMString name;
    std::vector<ParsedFeature, PSRAMAllocator<ParsedFeature>> features;
    int extent; // MVT tile extent (e.g., 4096)

    // Default constructor to ensure PSRAMAllocator is used for members
    ParsedLayer() : 
        name(PSRAMAllocator<char>()),
        features(PSRAMAllocator<ParsedFeature>()) {}
};

// Structure to represent a tile key (Z, X, Y_TMS) for map storage
struct TileKey {
    int z;
    int x;
    int y_tms; // Y coordinate in TMS (Tile Map Service) scheme

    // Custom comparison operator for std::map key
    bool operator<(const TileKey& other) const {
        if (z != other.z) return z < other.z;
        if (x != other.x) return x < other.x;
        return y_tms < other.y_tms;
    }

    // Custom equality operator for comparison
    bool operator==(const TileKey& other) const {
        return z == other.z && x == other.x && y_tms == other.y_tms;
    }
};

// Global map to store loaded tile data, using PSRAM for values
extern std::map<TileKey, std::vector<ParsedLayer, PSRAMAllocator<ParsedLayer>>, std::less<TileKey>, 
                PSRAMAllocator<std::pair<const TileKey, std::vector<ParsedLayer, PSRAMAllocator<ParsedLayer>>>>> loadedTilesData;
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

// DMA-capable buffer for SD card operations (declared extern here, defined in main.cpp)
extern uint8_t *sd_dma_buffer;
extern size_t SD_DMA_BUFFER_SIZE; // Removed 'const' keyword

#endif // COMMON_H
