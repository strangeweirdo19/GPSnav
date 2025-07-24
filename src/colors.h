// colors.h
#ifndef COLORS_H
#define COLORS_H

#include <stdint.h> // For uint16_t

// General UI Colors
const uint16_t MAP_BACKGROUND_COLOR = 0x1926; // Dark blue-grey (#1a2632)
const uint16_t NAVIGATION_ARROW_COLOR = 0xFFFF; // White (TFT_WHITE)
const uint16_t STATUS_BAR_COLOR = 0x0000;     // Black (TFT_BLACK)

// Traffic Signal Colors
const uint16_t TRAFFIC_SIGNAL_RED = 0xF800;   // Red (TFT_RED)
const uint16_t TRAFFIC_SIGNAL_YELLOW = 0xFFE0; // Yellow (TFT_YELLOW)
const uint16_t TRAFFIC_SIGNAL_GREEN = 0x07E0; // Green (TFT_GREEN)
const uint16_t TRAFFIC_SIGNAL_BODY_COLOR = 0x7BEF; // Dark Grey (TFT_DARKGREY)

// Road Colors
// These are general suggestions, you can adjust them to your preference
const uint16_t MOTORWAY_COLOR = 0x001F;      // Dark Blue
const uint16_t TRUNK_COLOR = 0x07FF;         // Cyan
const uint16_t PRIMARY_ROAD_COLOR = 0xF800;  // Red
const uint16_t SECONDARY_ROAD_COLOR = 0xFFE0; // Yellow
const uint16_t TERTIARY_ROAD_COLOR = 0x07E0; // Green
const uint16_t RESIDENTIAL_ROAD_COLOR = 0xAD55; // Light Grey (TFT_LIGHTGREY)
const uint16_t SERVICE_ROAD_COLOR = 0x8410;  // Grey
const uint16_t PATH_COLOR = 0x8800;          // Brown
const uint16_t FOOTWAY_COLOR = 0x8800;       // Same as path
const uint16_t CYCLEWAY_COLOR = 0x07E0;      // Green
const uint16_t PEDESTRIAN_COLOR = 0x8800;    // Same as path
const uint16_t OTHER_ROAD_COLOR = 0x8410;    // Default grey for unclassified roads

// Bridge Border Color (always black regardless of road type)
const uint16_t BRIDGE_BORDER_COLOR = 0x0000; // Black

// Other Map Feature Colors
const uint16_t WATER_COLOR = 0x001F;         // Blue
const uint16_t BUILDING_COLOR = 0x6B4D;      // Medium Grey
const uint16_t LANDUSE_PARK_COLOR = 0x07E0;  // Green (for parks/forests)
const uint16_t LANDUSE_RESIDENTIAL_COLOR = 0x8410; // Light brown/tan
const uint16_t NATURAL_WOOD_COLOR = 0x0400;  // Dark Green
const uint16_t NATURAL_GRASS_COLOR = 0x07E0; // Green
const uint16_t DEFAULT_FEATURE_COLOR = 0x8410; // A neutral grey for unclassified features

#endif // COLORS_H
