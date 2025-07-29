// colors.h
#ifndef COLORS_H
#define COLORS_H

#include <stdint.h> // For uint16_t

// General UI Colors
const uint16_t MAP_BACKGROUND_COLOR = 0x31a6; // Dark blue-grey (#1a2632)
const uint16_t NAVIGATION_ARROW_COLOR = 0xFFFF; // White (TFT_WHITE)
const uint16_t STATUS_BAR_COLOR = 0x0000;     // Black (TFT_BLACK)

// Traffic Signal Colors
const uint16_t TRAFFIC_SIGNAL_RED = 0xF800;   // Red (TFT_RED)
const uint16_t TRAFFIC_SIGNAL_YELLOW = 0xFFE0; // Yellow (TFT_YELLOW)
const uint16_t TRAFFIC_SIGNAL_GREEN = 0x07E0; // Green (TFT_GREEN)
const uint16_t TRAFFIC_SIGNAL_BODY_COLOR = 0x7BEF; // Dark Grey (TFT_DARKGREY)

// Common Road Colors
const uint16_t ROAD_MAJOR_COLOR = 0xe60c; // For motorways, trunks, primary roads
const uint16_t ROAD_MEDIUM_COLOR = 0xc658; // For secondary, tertiary, residential, service, aerialway roads
const uint16_t ROAD_MINOR_PATH_COLOR = 0x7bcf; // For paths, footways, cycleways, pedestrian, minor, unclassified roads

// Road Colors - using common definitions
const uint16_t MOTORWAY_COLOR = ROAD_MAJOR_COLOR;
const uint16_t TRUNK_COLOR = ROAD_MAJOR_COLOR;
const uint16_t PRIMARY_ROAD_COLOR = ROAD_MAJOR_COLOR;
const uint16_t SECONDARY_ROAD_COLOR = ROAD_MEDIUM_COLOR;
const uint16_t TERTIARY_ROAD_COLOR = ROAD_MINOR_PATH_COLOR;
const uint16_t RESIDENTIAL_ROAD_COLOR = ROAD_MEDIUM_COLOR;
const uint16_t SERVICE_ROAD_COLOR = ROAD_MEDIUM_COLOR;
const uint16_t PATH_COLOR = ROAD_MINOR_PATH_COLOR;
const uint16_t FOOTWAY_COLOR = ROAD_MINOR_PATH_COLOR;
const uint16_t CYCLEWAY_COLOR = ROAD_MINOR_PATH_COLOR;
const uint16_t PEDESTRIAN_COLOR = ROAD_MINOR_PATH_COLOR;
const uint16_t OTHER_ROAD_COLOR = ROAD_MINOR_PATH_COLOR; // Default for unclassified roads

// Specific road type colors (from data_task.cpp logic)
const uint16_t ROAD_IMPORTANT_COLOR = ROAD_MAJOR_COLOR; // Custom color for primary, motorway, trunk roads (retained if different from ROAD_MAJOR_COLOR)
const uint16_t ROAD_RAIL_COLOR = 0x07FF;     // Dark Cyan (TFT_DARKCYAN) for rail
const uint16_t ROAD_FERRY_COLOR = 0x000F;    // Navy (TFT_NAVY) for ferry
const uint16_t ROAD_AERIALWAY_COLOR = ROAD_MEDIUM_COLOR; // Using common medium road color
const uint16_t ROAD_MINOR_COLOR = ROAD_MINOR_PATH_COLOR;    // Using common minor road color
const uint16_t ROAD_PATH_CYCLE_STEPS_COLOR = ROAD_MINOR_PATH_COLOR; // Using common minor path color

// Bridge Border Color (always black regardless of road type)
const uint16_t BRIDGE_BORDER_COLOR = 0x0000; // Black

// Other Map Feature Colors
const uint16_t WATER_COLOR = 0x001F;         // Blue
const uint16_t BUILDING_COLOR = 0x6B4D;      // Medium Grey
const uint16_t DEFAULT_FEATURE_COLOR = 0x8410; // A neutral grey for unclassified features

// POI Specific Colors (from getIconColor in data_task.cpp)
const uint16_t POI_SHELTER_COLOR = 0x07FF; // Dark Cyan (TFT_DARKCYAN)
const uint16_t POI_MOBILE_PHONE_COLOR = 0xF81F; // Violet (TFT_VIOLET)
const uint16_t POI_PAWNBROKER_COLOR = 0xA145; // Brown (TFT_BROWN)
const uint16_t POI_AGRARIAN_COLOR = 0x0400; // Dark Green (TFT_DARKGREEN)
const uint16_t POI_BICYCLE_SHOP_COLOR = 0x001F; // Blue (TFT_BLUE)
const uint16_t POI_SUPERMARKET_COLOR = 0xF800; // Red (TFT_RED)
const uint16_t POI_BAKERY_COLOR = 0xA145; // Brown (TFT_BROWN)
const uint16_t POI_ALCOHOL_SHOP_COLOR = 0x8010; // Purple (TFT_PURPLE)
const uint16_t POI_HOUSEWARE_SHOP_COLOR = 0xFEA0; // Gold (TFT_GOLD)
const uint16_t POI_GENERIC_SHOP_COLOR = 0xF81F; // Magenta (TFT_MAGENTA)
const uint16_t POI_BICYCLE_PARKING_COLOR = 0x0400; // Dark Green (TFT_DARKGREEN)
const uint16_t POI_BICYCLE_RENTAL_COLOR = 0x07E0; // Green (TFT_GREEN)
const uint16_t POI_TOILETS_COLOR = 0xFFFF; // White (TFT_WHITE)
const uint16_t POI_TAXI_COLOR = 0x7800; // Maroon (TFT_MAROON)
const uint16_t POI_PARKING_COLOR = 0x7BEF; // Dark Grey (TFT_DARKGREY)
const uint16_t POI_HOSPITAL_COLOR = 0xF800; // Red (TFT_RED)
const uint16_t POI_GENERIC_AMENITY_COLOR = 0xFFE0; // Yellow (TFT_YELLOW)
const uint16_t POI_ATTRACTION_COLOR = 0xF81F; // Pink (TFT_PINK)
const uint16_t POI_ACCOMMODATION_COLOR = 0xA145; // Brown (TFT_BROWN)
const uint16_t POI_CAMP_SITE_COLOR = 0x0400; // Dark Green (TFT_DARKGREEN)
const uint16_t POI_MUSEUM_GALLERY_COLOR = 0xFD20; // Orange (TFT_ORANGE)
const uint16_t POI_VIEWPOINT_COLOR = 0x07FF; // Cyan (TFT_CYAN)
const uint16_t POI_GENERIC_TOURISM_COLOR = 0x8010; // Purple (TFT_PURPLE)
const uint16_t POI_SPORT_COLOR = 0xF800; // Red (TFT_RED)
const uint16_t POI_HISTORIC_COLOR = 0xA145; // Brown (TFT_BROWN)
const uint16_t POI_AERODROME_COLOR = 0x07FF; // Cyan (TFT_CYAN)
const uint16_t POI_DEFAULT_COLOR = 0xAD55; // Light Grey

// Landcover Specific Colors
const uint16_t LANDCOVER_FARMLAND_GRASS_WOOD_FOREST_PARK_GARDEN_COLOR = 0x07E0; // Green
const uint16_t LANDCOVER_WETLAND_COLOR = 0x001F; // Blue
const uint16_t LANDCOVER_SAND_BEACH_COLOR = 0xE64E; // Sandy color
const uint16_t LANDCOVER_ICE_GLACIER_COLOR = 0xFFFF; // White
const uint16_t LANDCOVER_ROCK_COLOR = 0x7BEF; // Dark Grey
const uint16_t LANDCOVER_DEFAULT_GREEN = 0x07E0; // Default green for other landcover features

// Landuse Specific Colors
const uint16_t LANDUSE_VEGETATION_COLOR = 0x07E0; // Green (for forest, park, garden, grass, recreation_ground, village_green, orchard, vineyard, allotments)
const uint16_t LANDUSE_RESIDENTIAL_COLOR = 0xAD55; // Light Grey
const uint16_t LANDUSE_COMMERCIAL_COLOR = 0xFD20; // Orange
const uint16_t LANDUSE_INDUSTRIAL_COLOR = 0xA145; // Brown
const uint16_t LANDUSE_CEMETERY_COLOR = 0x0400; // Dark Green
const uint16_t LANDUSE_RETAIL_COLOR = 0xF81F; // Magenta

// Place Specific Colors
const uint16_t PLACE_COUNTRY_COLOR = 0xFFFF; // White
const uint16_t PLACE_STATE_COLOR = 0xC618; // Silver
const uint16_t PLACE_CITY_COLOR = 0xFFE0; // Yellow
const uint16_t PLACE_TOWN_COLOR = 0xFD20; // Orange
const uint16_t PLACE_VILLAGE_COLOR = 0xAFE5; // Greenyellow
const uint16_t PLACE_SMALL_SETTLEMENT_COLOR = 0xAD55; // Light Grey (hamlet, suburb, neighbourhood)
const uint16_t PLACE_DEFAULT_COLOR = 0xF81F; // Magenta

// Other Layers
const uint16_t BOUNDARY_COLOR = 0xFFFF; // White
const uint16_t MOUNTAIN_PEAK_COLOR = 0xFFE0; // Yellow
const uint16_t AEROWAY_COLOR = 0xAD55; // Light Grey

#endif // COLORS_H
