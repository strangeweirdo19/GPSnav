#include "colors.h"
#include <Arduino.h>

// =========================================================
// COLOR DEFINITIONS
// =========================================================

// General UI Colors
uint16_t MAP_BACKGROUND_COLOR; 
uint16_t NAVIGATION_ARROW_COLOR;
uint16_t ALTERNATE_ROUTE_COLOR;
uint16_t DESTINATION_MARKER_COLOR;
uint16_t WAYPOINT_MARKER_COLOR;
uint16_t FUTURE_ROUTE_COLOR;
uint16_t STATUS_BAR_COLOR;

// New Dynamic UI Colors
uint16_t UI_TEXT_COLOR;
uint16_t UI_BACKGROUND_COLOR; 

// Common Road Colors
uint16_t ROAD_MAJOR_COLOR;
uint16_t ROAD_MEDIUM_COLOR;
uint16_t ROAD_MINOR_PATH_COLOR;

// Road Colors (Aliases)
uint16_t MOTORWAY_COLOR;
uint16_t TRUNK_COLOR;
uint16_t PRIMARY_ROAD_COLOR;
uint16_t SECONDARY_ROAD_COLOR;
uint16_t TERTIARY_ROAD_COLOR;
uint16_t RESIDENTIAL_ROAD_COLOR;
uint16_t SERVICE_ROAD_COLOR;
uint16_t PATH_COLOR;
uint16_t FOOTWAY_COLOR;
uint16_t CYCLEWAY_COLOR;
uint16_t PEDESTRIAN_COLOR;
uint16_t OTHER_ROAD_COLOR;

// Specific road type colors
uint16_t ROAD_IMPORTANT_COLOR;
uint16_t ROAD_RAIL_COLOR;
uint16_t ROAD_FERRY_COLOR;
uint16_t ROAD_AERIALWAY_COLOR;
uint16_t ROAD_MINOR_COLOR;
uint16_t ROAD_PATH_CYCLE_STEPS_COLOR;

// Bridge Border Color
uint16_t BRIDGE_BORDER_COLOR;

// Other Map Feature Colors
uint16_t WATER_COLOR;
uint16_t BUILDING_COLOR;
uint16_t DEFAULT_FEATURE_COLOR;

// Landcover Specific Colors
uint16_t LANDCOVER_FARMLAND_GRASS_WOOD_FOREST_PARK_GARDEN_COLOR;
uint16_t LANDCOVER_WETLAND_COLOR;
uint16_t LANDCOVER_SAND_BEACH_COLOR;
uint16_t LANDCOVER_ICE_GLACIER_COLOR;
uint16_t LANDCOVER_ROCK_COLOR;
uint16_t LANDCOVER_DEFAULT_GREEN;

// Landuse Specific Colors
uint16_t LANDUSE_VEGETATION_COLOR;
uint16_t LANDUSE_RESIDENTIAL_COLOR;
uint16_t LANDUSE_COMMERCIAL_COLOR;
uint16_t LANDUSE_INDUSTRIAL_COLOR;
uint16_t LANDUSE_CEMETERY_COLOR;
uint16_t LANDUSE_RETAIL_COLOR;

// Place Specific Colors
uint16_t PLACE_COUNTRY_COLOR;
uint16_t PLACE_STATE_COLOR;
uint16_t PLACE_CITY_COLOR;
uint16_t PLACE_TOWN_COLOR;
uint16_t PLACE_VILLAGE_COLOR;
uint16_t PLACE_SMALL_SETTLEMENT_COLOR;
uint16_t PLACE_DEFAULT_COLOR;

// Other Layers
uint16_t BOUNDARY_COLOR;
uint16_t MOUNTAIN_PEAK_COLOR;
uint16_t AEROWAY_COLOR;

// =========================================================
// THEME LOGIC
// =========================================================

void setTheme(bool isDark) {
    if (isDark) {
        // =================================================
        // DARK MODE (Original Colors)
        // =================================================
        MAP_BACKGROUND_COLOR = 0x0882; // #0d1117 (Dark Blue/Grey)
        UI_BACKGROUND_COLOR = 0x0000;  // Black
        UI_TEXT_COLOR = 0xFFFF;        // White
        
        STATUS_BAR_COLOR = 0x0000;     // Black
        
        NAVIGATION_ARROW_COLOR = 0xFE2E; // #ffc777 (Warm Gold)
        ALTERNATE_ROUTE_COLOR = 0x659D;  // #64B5F6 (Light Blue)
        DESTINATION_MARKER_COLOR = 0x057E; // #01acf3 (Bright Cyan)
        WAYPOINT_MARKER_COLOR = 0xF5C0;    // #FBBC04 (Yellow-Orange)
        FUTURE_ROUTE_COLOR = 0x659D;       // #64B5F6
        
        // Roads
        ROAD_MAJOR_COLOR = 0xE526; // #e8a838 (Gold/Orange)
        ROAD_MEDIUM_COLOR = 0xCE99; // #d4d4d4 (Grey)
        ROAD_MINOR_PATH_COLOR = 0x4248; // #4a4a4a (Dark Grey)
        
        ROAD_IMPORTANT_COLOR = 0x8430; // #888888 
        ROAD_RAIL_COLOR = 0x07FF;     // Dark Cyan
        ROAD_FERRY_COLOR = 0x000F;    // Navy
        
        // Features
        WATER_COLOR = 0x1947;         // #1a2b3c
        BUILDING_COLOR = 0x6B4D;      // Medium Grey
        DEFAULT_FEATURE_COLOR = 0x8410; // Neutral Grey
        
        BRIDGE_BORDER_COLOR = 0x0000; // Black
        
        // Landcover (Dark Muted Greens)
        LANDCOVER_FARMLAND_GRASS_WOOD_FOREST_PARK_GARDEN_COLOR = 0x1963; 
        LANDCOVER_WETLAND_COLOR = 0x1947; 
        LANDCOVER_SAND_BEACH_COLOR = 0xC54F; 
        LANDCOVER_ICE_GLACIER_COLOR = 0xFFFF; 
        LANDCOVER_ROCK_COLOR = 0x7BEF; 
        LANDCOVER_DEFAULT_GREEN = 0x07E0; 

        // Landuse
        LANDUSE_VEGETATION_COLOR = 0x1963; 
        LANDUSE_RESIDENTIAL_COLOR = 0x18E4; 
        LANDUSE_COMMERCIAL_COLOR = 0x1904; 
        LANDUSE_INDUSTRIAL_COLOR = 0x18C4; 
        LANDUSE_CEMETERY_COLOR = 0x0400; 
        LANDUSE_RETAIL_COLOR = 0xF81F; 

        // Places (Light Text)
        PLACE_COUNTRY_COLOR = 0xFFFF; 
        PLACE_STATE_COLOR = 0xC618; 
        PLACE_CITY_COLOR = 0xFFE0; 
        PLACE_TOWN_COLOR = 0xFD20; 
        PLACE_VILLAGE_COLOR = 0xAFE5; 
        PLACE_SMALL_SETTLEMENT_COLOR = 0xAD55; 
        PLACE_DEFAULT_COLOR = 0xF81F; 

        BOUNDARY_COLOR = 0x8430; 
        MOUNTAIN_PEAK_COLOR = 0xFFE0; 
        AEROWAY_COLOR = 0x8430; 

    } else {
        // =================================================
        // LIGHT MODE
        // =================================================
        MAP_BACKGROUND_COLOR = 0xF7BE; // #f0f2f5 (Light Grey/White)
        UI_BACKGROUND_COLOR = 0xFFFF;  // White
        UI_TEXT_COLOR = 0x0000;        // Black
        
        STATUS_BAR_COLOR = 0xFFFF;     // White
        
        NAVIGATION_ARROW_COLOR = 0xFE2E; // #ffc777 (Keep same, pops on lighter bg too)
        ALTERNATE_ROUTE_COLOR = 0x659D;  // Keep same (Blue)
        DESTINATION_MARKER_COLOR = 0x057E; // Keep same
        WAYPOINT_MARKER_COLOR = 0xF5C0;    // Keep same
        FUTURE_ROUTE_COLOR = 0x659D;       
        
        // Roads (Simulating Google Maps Light Mode)
        ROAD_MAJOR_COLOR = 0xFD20; // Orange (TFT_ORANGE)
        ROAD_MEDIUM_COLOR = 0xFFFF; // White (TFT_WHITE)
        ROAD_MINOR_PATH_COLOR = 0xFFFF; // White (TFT_WHITE)
        
        ROAD_IMPORTANT_COLOR = 0xFDA0; // Light Orange
        ROAD_RAIL_COLOR = 0x7BEF;     // Grey
        ROAD_FERRY_COLOR = 0x001F;    // Blue
        
        // Features
        WATER_COLOR = 0x9E3F;         // #9bb7d4 (Light Blue)
        BUILDING_COLOR = 0xD69A;      // #d9d9d9 (Light Grey)
        DEFAULT_FEATURE_COLOR = 0xD69A; // Light Grey
        
        BRIDGE_BORDER_COLOR = 0xCE59; // Light Grey (shadow)
        
        // Landcover (Bright Greens)
        LANDCOVER_FARMLAND_GRASS_WOOD_FOREST_PARK_GARDEN_COLOR = 0xC6F0; // #c8eacc (Light Green)
        LANDCOVER_WETLAND_COLOR = 0x9E3F; // Light Blue
        LANDCOVER_SAND_BEACH_COLOR = 0xFF8F; // Pale Yellow
        LANDCOVER_ICE_GLACIER_COLOR = 0xFFFF; // White
        LANDCOVER_ROCK_COLOR = 0xCE59; // Light Grey
        LANDCOVER_DEFAULT_GREEN = 0xC6F0; 

        // Landuse
        LANDUSE_VEGETATION_COLOR = 0xC6F0; 
        LANDUSE_RESIDENTIAL_COLOR = 0xE71C; // #e0e0e0
        LANDUSE_COMMERCIAL_COLOR = 0xE73C; // #e6e6e6
        LANDUSE_INDUSTRIAL_COLOR = 0xDEFB; // #d0d0d0
        LANDUSE_CEMETERY_COLOR = 0xAEDC; // #aaccba
        LANDUSE_RETAIL_COLOR = 0xFFE0; // Light Yellow/Pinkish?
        
        // Places (Dark Text)
        PLACE_COUNTRY_COLOR = 0x0000; 
        PLACE_STATE_COLOR = 0x2104; // Dark Grey 
        PLACE_CITY_COLOR = 0x0000; 
        PLACE_TOWN_COLOR = 0x2104; 
        PLACE_VILLAGE_COLOR = 0x4208; // Dark Grey
        PLACE_SMALL_SETTLEMENT_COLOR = 0x8410; // Medium Grey
        PLACE_DEFAULT_COLOR = 0x0000; 

        BOUNDARY_COLOR = 0xA514; // Greyish
        MOUNTAIN_PEAK_COLOR = 0x7800; // Brownish
        AEROWAY_COLOR = 0xD69A; 
    }

    // Update Aliases (These are just copies of the values, so they need to be updated too)
    MOTORWAY_COLOR = ROAD_MAJOR_COLOR;
    TRUNK_COLOR = ROAD_MAJOR_COLOR;
    PRIMARY_ROAD_COLOR = ROAD_MAJOR_COLOR;
    SECONDARY_ROAD_COLOR = ROAD_MEDIUM_COLOR;
    TERTIARY_ROAD_COLOR = ROAD_MINOR_PATH_COLOR;
    RESIDENTIAL_ROAD_COLOR = ROAD_MEDIUM_COLOR;
    SERVICE_ROAD_COLOR = ROAD_MEDIUM_COLOR;
    PATH_COLOR = ROAD_MINOR_PATH_COLOR;
    FOOTWAY_COLOR = ROAD_MINOR_PATH_COLOR;
    CYCLEWAY_COLOR = ROAD_MINOR_PATH_COLOR;
    PEDESTRIAN_COLOR = ROAD_MINOR_PATH_COLOR;
    OTHER_ROAD_COLOR = ROAD_MINOR_PATH_COLOR;
    
    ROAD_AERIALWAY_COLOR = ROAD_MEDIUM_COLOR;
    ROAD_MINOR_COLOR = ROAD_MINOR_PATH_COLOR;
    ROAD_PATH_CYCLE_STEPS_COLOR = ROAD_MINOR_PATH_COLOR;
}
