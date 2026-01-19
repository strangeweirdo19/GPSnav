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
#include <array>      // Added for std::array

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
// STATUS INDICATORS
// =========================================================
extern bool bleConnected;      // BLE connection status
extern bool gpsHasFix;          // GPS has valid location
extern bool gpsModulePresent;   // GPS module detected
extern bool phoneGpsActive;     // Phone GPS data being received
extern unsigned long lastPhoneCommandTime;  // Last time any command was received from phone (millis)
extern bool bootComplete;       // Boot screen finished, render task can start
extern int tilesLoadedCount;    // Count of tiles loaded (for boot screen progress)

// =========================================================
// GPS INTERPOLATION STATE (Dead Reckoning)
// =========================================================
struct GPSState {
    double lat;                 // Last received latitude
    double lon;                 // Last received longitude
    float speed;                // Speed in m/s
    float heading;              // Heading in degrees (0=North, 90=East)
    unsigned long timestamp;    // millis() when this GPS update was received
};
extern GPSState gpsState;       // Global GPS state for interpolation
extern SemaphoreHandle_t gpsMutex;  // Mutex for GPS state access
extern float gpsRate;              // GPS commands received per second

// =========================================================
// ROUTE OVERLAY DATA
// =========================================================
struct RouteNode {
    int32_t dLat;
    int32_t dLon;
};

#define MAX_ROUTE_POINTS 65536  // Maximum route points (65536 * 8 bytes = 512KB)
#define ROUTE_SCALE 100000.0    // Scale factor for relative coordinates

// =========================================================
// NAVIGATION CONSTANTS (replacing magic numbers)
// =========================================================
#define METERS_PER_DEGREE 111320.0       // Meters per degree of latitude
#define WAYPOINT_ARRIVAL_THRESHOLD_M 25.0 // Distance in meters to consider waypoint reached
#define ROUTE_LEG_SWITCH_DEFAULT 2147483647 // INT32_MAX - no leg switch by default
#define PI_CONST 3.14159                   // Pi constant for calculations

// Forward declaration of PSRAMAllocator since it is a template defined later
template <typename T> struct PSRAMAllocator;

struct RouteAnchor {
    double lat;
    double lon;
};

extern RouteAnchor activeRouteAnchor;
extern std::vector<RouteNode, PSRAMAllocator<RouteNode>> activeRoute;  // Current route overlay (uses PSRAM)
extern bool routeAvailable;                   // Flag indicating route is ready
extern bool routeAvailable;                   // Flag indicating route is ready
extern SemaphoreHandle_t routeMutex;          // Mutex for route data access
extern bool isRouteSyncing;                   // Flag for route transfer overlay
extern int routeSyncProgressBytes;            // Bytes received for route transfer
extern int routeTotalBytes;                   // Total bytes expected for route transfer
extern int routeProgressIndex;                // Trigger for route trimming: points before this index are hidden
extern int routeLegSwitchIndex;               // Trigger for route coloring: points after this index are ORANGE
extern int currentWaypointIndex;              // Active waypoint index (0-based): which stop user is currently navigating to
extern double currentRouteLat;                // Absolute Latitude of route[routeProgressIndex]
extern double currentRouteLon;                // Absolute Longitude of route[routeProgressIndex]

// Alternate Routes (for auto-selection feature)
extern std::vector<RouteNode, PSRAMAllocator<RouteNode>> alternateRoutes[2]; // Alt route 0 and 1
extern RouteAnchor alternateAnchors[2];       // Anchor points for alternates
extern int selectedRouteIndex;                 // -1=main, 0=alt0, 1=alt1
// Note: alternateDecoderStates is local to ble_handler.cpp (avoids circular include with polyline.h)

// Multi-Stop Waypoints (for intermediate stop markers)
struct Waypoint {
    double lat;
    double lon;
};
#define MAX_WAYPOINTS 10
extern std::vector<Waypoint> waypointBuffer;

// Spatial Indexing for Route
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

struct RouteSegment {
    size_t startIndex; // Index into activeRoute
    size_t count;      // Number of points in this segment within the tile
    double startLat;   // Absolute latitude of the first point in this segment
    double startLon;   // Absolute longitude of the first point in this segment
};

// Map from TileKey to list of segments in that tile
extern std::map<TileKey, std::vector<RouteSegment, PSRAMAllocator<RouteSegment>>, std::less<TileKey>,
                PSRAMAllocator<std::pair<const TileKey, std::vector<RouteSegment, PSRAMAllocator<RouteSegment>>>>> routeTileIndex;

extern int lastIndexedZoom;
extern size_t lastIndexedRouteSize;
extern double lastIndexedLat;
extern double lastIndexedLon;


void indexRoute(); // Helper function to populate the index (acquires mutex)
void indexRouteLocked(); // Index route assuming mutex is held (for render task)
void indexRouteIncremental(); // Index only new points (acquires mutex)


// Route state tracking for synchronization
enum RouteState {
    ROUTE_NONE = 0,      // No route loaded
    ROUTE_PARTIAL = 1,   // Transfer in progress or incomplete
    ROUTE_COMPLETE = 2   // Route fully loaded and ready
};

extern RouteState currentRouteState;
extern int routePointCount;
extern uint32_t routeHash;  // Simple hash for sync verification

// =========================================================
// ROUTE CHUNK QUEUE (For async processing from BLE)
// =========================================================
#define ROUTE_CHUNK_QUEUE_SIZE 10      // Buffer up to 10 chunks
#define MAX_DELTAS_PER_CHUNK 40        // 20 points = 40 deltas (dLat, dLon)

struct RouteChunk {
    int32_t deltas[MAX_DELTAS_PER_CHUNK];  // dLat1, dLon1, dLat2, dLon2, ...
    uint8_t count;                          // Number of delta pairs (count * 2 = total values)
};

extern QueueHandle_t routeChunkQueue;     // Queue for pending chunks
void processRouteChunks();                // Process queued chunks (call from main loop or task)

// =========================================================
// TURN DIRECTION INDICATOR
// =========================================================
enum class TurnType {
    NONE,          // No turn indicator
    STRAIGHT,      // Go straight
    LEFT,          // Turn left
    RIGHT,         // Turn right
    SLIGHT_LEFT,   // Slight left
    SLIGHT_RIGHT,  // Slight right
    UTURN          // U-turn
};

extern TurnType currentTurnType;              // Current turn direction to display

// =========================================================
// OTA UPDATE STATE
// =========================================================
struct OTAState {
    volatile bool active;        // Is an OTA update currently in progress?
    char type[16];      // "FIRMWARE" or "MAP"
    volatile int percent;        // 0-100
};

extern OTAState globalOTAState;               // Global OTA state

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
        
        // 1. Try PSRAM first
        void* ptr = heap_caps_malloc(count * sizeof(T), MALLOC_CAP_SPIRAM);
        
        // 2. Fallback to Internal RAM if PSRAM failed (or not present)
        if (ptr == nullptr) {
             ptr = malloc(count * sizeof(T));
        }

        // 3. Safety check
        if (ptr == nullptr) {
            Serial.println("❌ CRITICAL: Memory Allocation Failed!");
            return nullptr; // Will likely cause STL issues, but better than immediate reset
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
    uint8_t bleIconMode; // 0=BlinkRed, 1=SolidRed, 2=SolidBlue
    bool showPIN;        // New: Toggle PIN overlay
    char pinCode[8];     // New: Buffer for 6-digit PIN
    char deviceName[32]; // New: Buffer for device Name
};
extern QueueHandle_t controlParamsQueue;        // Loop -> RenderTask (For user input)
extern QueueHandle_t tileRequestQueue;          // RenderTask -> DataTask (New: for requesting tiles)
extern QueueHandle_t tileParsedNotificationQueue; // DataTask -> RenderTask (New: for notifying when tile is parsed)

// DMA-capable buffer for SD card operations (declared extern here, defined in main.cpp)
extern uint8_t *sd_dma_buffer;
extern size_t SD_DMA_BUFFER_SIZE;

// =========================================================
// GLOBAL ICON COLOR MAP (Declared extern here, defined in main.cpp)
// =========================================================
extern std::map<PSRAMString, uint16_t, std::less<PSRAMString>,
         PSRAMAllocator<std::pair<const PSRAMString, uint16_t>>> iconColorsMap;

// =========================================================
// ICON DEFINITIONS (Common to DataTask and RenderTask)
// =========================================================

// Enum for different icon types
enum class IconType {
    None,
    TrafficSignal,
    Fuel,
    BusStop,
    GPS, // New: GPS icon
    Connected // New: Connected icon
};

// External declarations for icon dimensions (moved from map_renderer.h)
extern const int GPS_16_WIDTH;
extern const int GPS_16_HEIGHT;
extern const int CONNECTED_16_WIDTH;
extern const int CONNECTED_16_HEIGHT;

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
const int COMPASS_FILTER_WINDOW_SIZE = 3; // Size of the circular buffer for averaging (Reduced to 3 for faster rotation response)
const float COMPASS_ROTATION_THRESHOLD_DEG = 2.0f; // Reduced threshold for smoother rotation
const float COMPASS_EXPONENTIAL_SMOOTHING_ALPHA = 0.35f; // Increased alpha for faster response

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
 
// Optimization for OTA
void prepareForWifiOTA();

#endif // COMMON_H
