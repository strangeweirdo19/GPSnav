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
// PSRAM ALLOCATOR FOR STL CONTAINERS
// =========================================================
// Custom allocator to force STL containers to use PSRAM
template <typename T>
struct PSRAMAllocator {
    typedef T value_type;

    PSRAMAllocator() = default;
    template <typename U> PSRAMAllocator(const PSRAMAllocator<U>&) {}

    T* allocate(size_t count) {
        if (count == 0) return nullptr;
        void* ptr = heap_caps_malloc(count * sizeof(T), MALLOC_CAP_SPIRAM);
        if (ptr == nullptr) {
            // Log error and attempt to recover if possible, or indicate critical failure
            Serial.printf("❌ PSRAMAllocator: Failed to allocate %u bytes in PSRAM! Free PSRAM: %u bytes.\n",
                          count * sizeof(T), heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
            // In a real application, you might try to free some cached data,
            // or return a null pointer and let the calling code handle it.
            // For now, we'll return nullptr and let the caller potentially catch std::bad_alloc.
            // If the caller doesn't handle, this will lead to a crash.
            return nullptr;
        }
        return static_cast<T*>(ptr);
    }

    void deallocate(T* ptr, size_t) {
        heap_caps_free(ptr);
    }
};

template <typename T, typename U>
bool operator==(const PSRAMAllocator<T>&, const PSRAMAllocator<U>&) { return true; }
template <typename T, typename U>
bool operator!=(const PSRAMAllocator<T>&, const PSRAMAllocator<U>&) { return false; }

// Define PSRAMString for convenience
using PSRAMString = std::basic_string<char, std::char_traits<char>, PSRAMAllocator<char>>;

// =========================================================
// MVT DATA STRUCTURES (Using PSRAM Allocator)
// =========================================================

// Represents a single geometric ring (e.g., polygon exterior, hole, or line segment)
// Points are in MVT tile coordinates (0 to extent-1)
struct ParsedFeature {
    // Geometry can be multiple rings (e.g., polygon with holes, multi-line string)
    std::vector<std::vector<std::pair<int, int>, PSRAMAllocator<std::pair<int, int>>>,
                 PSRAMAllocator<std::vector<std::pair<int, int>, PSRAMAllocator<std::pair<int, int>>>>> geometryRings;
    std::map<PSRAMString, PSRAMString, std::less<PSRAMString>,
             PSRAMAllocator<std::pair<const PSRAMString, PSRAMString>>> properties;
    uint16_t color;
    bool isPolygon; // True if geometry is a polygon (for filling)
    int geomType;   // 1=Point, 2=LineString, 3=Polygon (from MVT spec)
    PSRAMString iconName; // New: To store the name of the icon to draw
    bool hasBridge; // New: Flag to indicate if the feature is a bridge
    bool hasTunnel; // New: Flag to indicate if the feature is a tunnel

    // Bounding box in MVT coordinates (0 to extent-1) for culling
    int minX_mvt, minY_mvt, maxX_mvt, maxY_mvt;

    // Constructor to ensure PSRAMAllocator is used for members
    ParsedFeature() : geometryRings(PSRAMAllocator<std::vector<std::pair<int, int>, PSRAMAllocator<std::pair<int, int>>>>()),
                      properties(PSRAMAllocator<std::pair<const PSRAMString, PSRAMString>>()),
                      color(0), isPolygon(false), geomType(0), iconName(PSRAMAllocator<char>()), hasBridge(false), hasTunnel(false), // Initialize iconName, hasBridge, hasTunnel
                      minX_mvt(0), minY_mvt(0), maxX_mvt(0), maxY_mvt(0) {}
};

// Represents a single layer within an MVT tile
struct ParsedLayer {
    PSRAMString name;
    std::vector<ParsedFeature, PSRAMAllocator<ParsedFeature>> features;
    int extent; // Typically 4096 for MVT tiles
    int drawOrder; // New: Added for controlling rendering order

    // Constructor to ensure PSRAMAllocator is used for members
    ParsedLayer() : name(PSRAMAllocator<char>()), features(PSRAMAllocator<ParsedFeature>()), extent(0), drawOrder(99) {} // Initialize drawOrder
};

// Unique identifier for a map tile
struct TileKey {
    int z;          // Zoom level
    int x;          // Tile X coordinate
    int y_tms;      // Tile Y coordinate (TMS convention: Y increases upwards)

    // Comparison operator for use in std::map and std::set
    bool operator<(const TileKey& other) const {
        if (z != other.z) return z < other.z;
        if (x != other.x) return x < other.x;
        return y_tms < other.y_tms;
    }

    bool operator==(const TileKey& other) const {
        return z == other.z && x == other.x && y_tms == other.y_tms;
    }
};

// Global map to store loaded and parsed tile data
// Protected by loadedTilesDataMutex
extern std::map<TileKey, std::vector<ParsedLayer, PSRAMAllocator<ParsedLayer>>, std::less<TileKey>,
                PSRAMAllocator<std::pair<const TileKey, std::vector<ParsedLayer, PSRAMAllocator<ParsedLayer>>>>> loadedTilesData;
extern SemaphoreHandle_t loadedTilesDataMutex;

// Global variable for current layer extent, updated by dataTask, read by renderTask
extern int currentLayerExtent; // Default MVT tile extent, will be updated from parsed data

// Structure for rendering parameters to be sent via queue
struct RenderParams {
    int centralTileX;
    int centralTileY_TMS;
    int targetPointMVT_X;
    int targetPointMVT_Y;
    int layerExtent;
    float zoomScaleFactor;
    int displayOffsetX;
    int displayOffsetY;
    float mapRotationDegrees;
    int pivotY;
    float cullingBufferPercentageLeft;
    float cullingBufferPercentageRight;
    float cullingBufferPercentageTop;
    float cullingBufferPercentageBottom;
};

// Structure for control parameters (Lat, Lon, Zoom) to be sent from loop() to dataTask
struct ControlParams {
    double targetLat;
    double targetLon;
    float zoomFactor;
    float cullingBufferPercentageLeft;
    float cullingBufferPercentageRight;
    float cullingBufferPercentageTop;
    float cullingBufferPercentageBottom;
};
extern QueueHandle_t controlParamsQueue;        // Loop -> RenderTask (For user input)
extern QueueHandle_t tileRequestQueue;          // RenderTask -> DataTask (New: for requesting tiles)
extern QueueHandle_t tileParsedNotificationQueue; // DataTask -> RenderTask (New: for notifying when tile is parsed)

// DMA-capable buffer for SD card operations (declared extern here, defined in main.cpp)
extern uint8_t *sd_dma_buffer;
extern size_t SD_DMA_BUFFER_SIZE;

// =========================================================
// ICON DEFINITIONS (Common to DataTask and RenderTask)
// =========================================================

// Enum for different icon types
enum class IconType {
    None,
    TrafficSignal,
    Fuel,
    BusStop
};

// Struct to hold icon bitmap properties
struct IconBitmap {
    const uint16_t* bitmap;
    int width;
    int height;
};

// Map to define default colors for specific POI classes that have dedicated icons
// This map will be initialized in main.cpp
extern std::map<PSRAMString, uint16_t, std::less<PSRAMString>,
                PSRAMAllocator<std::pair<const PSRAMString, uint16_t>>> iconColorsMap;

// Function to get the color for a given POI class/subclass
uint16_t getIconColor(const PSRAMString& poiClass, const PSRAMString& poiSubclass);

// Function to draw a specific icon type at given coordinates
void drawIcon(IconType type, int centerX, int centerY, uint16_t color);

// =========================================================
// CONFIGURATION CONSTANTS
// =========================================================
// Task Stack Sizes
const configSTACK_DEPTH_TYPE DATA_TASK_STACK_SIZE = 8 * 1024;
const configSTACK_DEPTH_TYPE RENDER_TASK_STACK_SIZE = 10 * 1024;

// Queue Sizes
const UBaseType_t CONTROL_PARAMS_QUEUE_SIZE = 25;
const UBaseType_t TILE_REQUEST_QUEUE_SIZE = 30; // 9+16 tiles + buffer
const UBaseType_t TILE_PARSED_NOTIFICATION_QUEUE_SIZE = 20;

// SD Card DMA Buffer
const size_t MAX_SD_DMA_BUFFER_SIZE_KB = 150; // Max buffer size in KB
const size_t MAX_SD_DMA_BUFFER_SIZE_BYTES = MAX_SD_DMA_BUFFER_SIZE_KB * 1024;

// Compass Filter
const int COMPASS_FILTER_WINDOW_SIZE = 20;
const float COMPASS_ROTATION_THRESHOLD_DEG = 0.5f;

// Render Parameters
const int NAVIGATION_ARROW_SIZE = 10;
const int NAVIGATION_ARROW_BASE_UP_SHIFT = 7;
const int POINT_FEATURE_SIZE = 3; // Size for rendering generic point features

// Default culling buffer percentages (can be overridden by ControlParams)
const float DEFAULT_CULLING_BUFFER_PERCENTAGE_LEFT = 0.15f;
const float DEFAULT_CULLING_BUFFER_PERCENTAGE_RIGHT = 0.15f;
const float DEFAULT_CULLING_BUFFER_PERCENTAGE_TOP = 0.13f;
const float DEFAULT_CULLING_BUFFER_PERCENTAGE_BOTTOM = 0.50f;
const int MIN_CULLING_BUFFER_PIXELS = 5; // Minimum buffer size in pixels

#endif // COMMON_H
