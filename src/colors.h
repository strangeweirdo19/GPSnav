// colors.h
#ifndef COLORS_H
#define COLORS_H

#include <stdint.h> // For uint16_t

// General UI Colors
extern uint16_t MAP_BACKGROUND_COLOR; 
extern uint16_t NAVIGATION_ARROW_COLOR;
extern uint16_t ALTERNATE_ROUTE_COLOR;
extern uint16_t DESTINATION_MARKER_COLOR;
extern uint16_t WAYPOINT_MARKER_COLOR;
extern uint16_t FUTURE_ROUTE_COLOR;
extern uint16_t STATUS_BAR_COLOR;

// New Dynamic UI Colors
extern uint16_t UI_TEXT_COLOR;
extern uint16_t UI_BACKGROUND_COLOR; // For startup screens etc.

// Traffic Signal Colors
const uint16_t TRAFFIC_SIGNAL_RED = 0xF800;   // Red (TFT_RED)
const uint16_t TRAFFIC_SIGNAL_YELLOW = 0xFFE0; // Yellow (TFT_YELLOW)
const uint16_t TRAFFIC_SIGNAL_GREEN = 0x07E0; // Green (TFT_GREEN)
const uint16_t TRAFFIC_SIGNAL_BODY_COLOR = 0x7BEF; // Dark Grey (TFT_DARKGREY)

// Common Road Colors
extern uint16_t ROAD_MAJOR_COLOR;
extern uint16_t ROAD_MEDIUM_COLOR;
extern uint16_t ROAD_MINOR_PATH_COLOR;

// Road Colors - using common definitions (These are just aliases, so they can remain const if they point to the externs? No, aliases to externs need to be defined safely or just use the externs directly. 
// Simpler: functions or macros? Or just externs for all?)
// Actually, `const uint16_t MOTORWAY_COLOR = ROAD_MAJOR_COLOR;` will error if ROAD_MAJOR_COLOR is not const. 
// So these must also be externs or #defines. Returns to #defines requires changing usage sites?
// Better: Make them externs too.

extern uint16_t MOTORWAY_COLOR;
extern uint16_t TRUNK_COLOR;
extern uint16_t PRIMARY_ROAD_COLOR;
extern uint16_t SECONDARY_ROAD_COLOR;
extern uint16_t TERTIARY_ROAD_COLOR;
extern uint16_t RESIDENTIAL_ROAD_COLOR;
extern uint16_t SERVICE_ROAD_COLOR;
extern uint16_t PATH_COLOR;
extern uint16_t FOOTWAY_COLOR;
extern uint16_t CYCLEWAY_COLOR;
extern uint16_t PEDESTRIAN_COLOR;
extern uint16_t OTHER_ROAD_COLOR;

// Specific road type colors
extern uint16_t ROAD_IMPORTANT_COLOR;
extern uint16_t ROAD_RAIL_COLOR;
extern uint16_t ROAD_FERRY_COLOR;
extern uint16_t ROAD_AERIALWAY_COLOR;
extern uint16_t ROAD_MINOR_COLOR;
extern uint16_t ROAD_PATH_CYCLE_STEPS_COLOR;

// Bridge Border Color
extern uint16_t BRIDGE_BORDER_COLOR;

// Other Map Feature Colors
extern uint16_t WATER_COLOR;
extern uint16_t BUILDING_COLOR;
extern uint16_t DEFAULT_FEATURE_COLOR;

// POI Specific Colors
// Keeping these CONST for now as they are iconic (e.g. McDonald's is always red/yellow)
// Unless we want "Dark Mode POIs"? Usually POI icons retain their brand colors.
const uint16_t POI_SHELTER_COLOR = 0x07FF; 
const uint16_t POI_MOBILE_PHONE_COLOR = 0xF81F; 
const uint16_t POI_PAWNBROKER_COLOR = 0xA145; 
const uint16_t POI_AGRARIAN_COLOR = 0x0400; 
const uint16_t POI_BICYCLE_SHOP_COLOR = 0x001F; 
const uint16_t POI_SUPERMARKET_COLOR = 0xF800; 
const uint16_t POI_BAKERY_COLOR = 0xA145; 
const uint16_t POI_ALCOHOL_SHOP_COLOR = 0x8010; 
const uint16_t POI_HOUSEWARE_SHOP_COLOR = 0xFEA0; 
const uint16_t POI_GENERIC_SHOP_COLOR = 0xF81F; 
const uint16_t POI_BICYCLE_PARKING_COLOR = 0x0400; 
const uint16_t POI_BICYCLE_RENTAL_COLOR = 0x07E0; 
const uint16_t POI_TOILETS_COLOR = 0xFFFF; 
const uint16_t POI_TAXI_COLOR = 0x7800; 
const uint16_t POI_PARKING_COLOR = 0x7BEF; 
const uint16_t POI_HOSPITAL_COLOR = 0xF800; 
const uint16_t POI_GENERIC_AMENITY_COLOR = 0xFFE0; 
const uint16_t POI_ATTRACTION_COLOR = 0xF81F; 
const uint16_t POI_ACCOMMODATION_COLOR = 0xA145; 
const uint16_t POI_CAMP_SITE_COLOR = 0x0400; 
const uint16_t POI_MUSEUM_GALLERY_COLOR = 0xFD20; 
const uint16_t POI_VIEWPOINT_COLOR = 0x07FF; 
const uint16_t POI_GENERIC_TOURISM_COLOR = 0x8010; 
const uint16_t POI_SPORT_COLOR = 0xF800; 
const uint16_t POI_HISTORIC_COLOR = 0xA145; 
const uint16_t POI_AERODROME_COLOR = 0x07FF; 
const uint16_t POI_DEFAULT_COLOR = 0xAD55; 

// Landcover Specific Colors (Dynamic for Day/Night)
extern uint16_t LANDCOVER_FARMLAND_GRASS_WOOD_FOREST_PARK_GARDEN_COLOR;
extern uint16_t LANDCOVER_WETLAND_COLOR;
extern uint16_t LANDCOVER_SAND_BEACH_COLOR;
extern uint16_t LANDCOVER_ICE_GLACIER_COLOR;
extern uint16_t LANDCOVER_ROCK_COLOR;
extern uint16_t LANDCOVER_DEFAULT_GREEN;

// Landuse Specific Colors
extern uint16_t LANDUSE_VEGETATION_COLOR;
extern uint16_t LANDUSE_RESIDENTIAL_COLOR;
extern uint16_t LANDUSE_COMMERCIAL_COLOR;
extern uint16_t LANDUSE_INDUSTRIAL_COLOR;
extern uint16_t LANDUSE_CEMETERY_COLOR;
extern uint16_t LANDUSE_RETAIL_COLOR;

// Place Specific Colors (Text Colors)
extern uint16_t PLACE_COUNTRY_COLOR;
extern uint16_t PLACE_STATE_COLOR;
extern uint16_t PLACE_CITY_COLOR;
extern uint16_t PLACE_TOWN_COLOR;
extern uint16_t PLACE_VILLAGE_COLOR;
extern uint16_t PLACE_SMALL_SETTLEMENT_COLOR;
extern uint16_t PLACE_DEFAULT_COLOR;

// Other Layers
extern uint16_t BOUNDARY_COLOR;
extern uint16_t MOUNTAIN_PEAK_COLOR;
extern uint16_t AEROWAY_COLOR;

// Theme Function
void setTheme(bool isDark);

#endif // COLORS_H
