-- Data processing based on openmaptiles.org schema
-- https://openmaptiles.org/schema/
-- Copyright (c) 2016, KlokanTech.com & OpenMapTiles contributors.
-- Used under CC-BY 4.0

--------
-- Alter these lines to control which languages are written for place/streetnames
--
-- Preferred language can be (for example) "en" for English, "de" for German, or nil to use OSM's name tag:
preferred_language = nil
-- This is written into the following vector tile attribute (usually "name:latin"):
preferred_language_attribute = "name:latin"
-- If OSM's name tag differs, then write it into this attribute (usually "name_int"):-
default_language_attribute = "name_int"
-- Also write these languages if they differ - for example, { "de", "fr" }
additional_languages = { }
--------

-- Enter/exit Tilemaker
function init_function(name,is_first)
end
function exit_function()
end

-- Implement Sets in tables
function Set(list)
	local set = {}
	for _, l in ipairs(list) do set[l] = true end
	return set
end

-- Meters per pixel if tile is 256x256
ZRES5  = 4891.97
ZRES6  = 2445.98
ZRES7  = 1222.99
ZRES8  = 611.5
ZRES9  = 305.7
ZRES10 = 152.9
ZRES11 = 76.4
ZRES12 = 38.2
ZRES13 = 19.1

-- The height of one floor, in meters
BUILDING_FLOOR_HEIGHT = 3.66
-- Used to express that a feature should not end up the vector tiles
INVALID_ZOOM = 99

-- Process node/way tags
aerodromeValues = Set { "international", "public", "regional", "military", "private" }
pavedValues = Set { "paved", "asphalt", "cobblestone", "concrete", "concrete:lanes", "concrete:plates", "metal", "paving_stones", "sett", "unhewn_cobblestone", "wood" }
unpavedValues = Set { "unpaved", "compacted", "dirt", "earth", "fine_gravel", "grass", "grass_paver", "gravel", "gravel_turf", "ground", "ice", "mud", "pebblestone", "salt", "sand", "snow", "woodchips" }

-- Process node tags

node_keys = {"aerialway","aeroway","amenity","barrier","highway","historic","leisure","natural","office","place","railway","shop","sport","tourism","waterway" }

-- Get admin level which the place node is capital of.
-- Returns nil in case of invalid capital and for places which are not capitals.
function capitalLevel(capital)
	local capital_al = tonumber(capital) or 0
	if capital == "yes" then
		capital_al = 2
	end
	if capital_al == 0 then
		return nil
	end
        return capital_al
end

-- Calculate rank for place nodes
-- place: value of place=*
-- popuplation: population as number
-- capital_al: result of capitalLevel()
function calcRank(place, population, capital_al)
	local rank = 0
	if capital_al and capital_al >= 2 and capital_al <= 4 then
		rank = capital_al
		if population > 3 * 10^6 then
			rank = rank - 2
		elseif population > 1 * 10^6 then
			rank = rank - 1
		elseif population < 100000 then
			rank = rank + 2
		elseif population < 50000 then
			rank = rank + 3
		end
		-- Safety measure to avoid place=village/farm/... appear early (as important capital) because a mapper added capital=yes/2/3/4
		if place ~= "city" then
			rank = rank + 3
			-- Decrease rank further if it is not even a town.
			if place ~= "town" then
				rank = rank + 2
			end
		end
		return rank
	end
	if place ~= "city" and place ~= "town" then
		return nil
        end
	if population > 3 * 10^6 then
		return 1
	elseif population > 1 * 10^6 then
		return 2
	elseif population > 500000 then
		return 3
	elseif population > 200000 then
		return 4
	elseif population > 100000 then
		return 5
	elseif population > 75000 then
		return 6
	elseif population > 50000 then
		return 7
	elseif population > 25000 then
		return 8
	elseif population > 10000 then
		return 9
	end
	return 10
end


function node_function()
	-- Write 'aerodrome_label'
	local aeroway = Find("aeroway")
	if aeroway == "aerodrome" then
		Layer("aerodrome_label", false)
		-- SetNameAttributes() -- Removed name setting for aerodrome_label
		Attribute("iata", Find("iata"))
		SetEleAttributes()
		Attribute("icao", Find("icao"))

		local aerodrome_value = Find("aeroway") -- Changed from Find("aerodrome") to Find("aeroway") as per common practice
		local class
		if aerodromeValues[aerodrome_value] then class = aerodrome_value else class = "other" end
		Attribute("class", class)
	end
	-- Write 'place'
	-- note that OpenMapTiles has a rank for countries (1-3), states (1-6) and cities (1-10+);
	--   we could potentially approximate it for cities based on the population tag
	local place = Find("place")
	if place ~= "" then
		local mz = 13
		local pop = tonumber(Find("population")) or 0
		local capital = capitalLevel(Find("capital"))
		local rank = calcRank(place, pop, capital)

		if     place == "continent"     then mz=0
		elseif place == "country"       then
			if     pop>50000000 then rank=1; mz=1
			elseif pop>20000000 then rank=2; mz=2
			else                     rank=3; mz=3 end
		elseif place == "state"         then mz=4
		elseif place == "province"         then mz=5
		elseif place == "city"          then mz=5
		elseif place == "town" and pop>8000 then mz=7
		elseif place == "town"          then mz=8
		elseif place == "village" and pop>2000 then mz=9
		elseif place == "village"       then mz=10
		elseif place == "borough"       then mz=10
		elseif place == "suburb"        then mz=11
		elseif place == "quarter"       then mz=12
		elseif place == "hamlet"        then mz=12
		elseif place == "neighbourhood" then mz=13
		elseif place == "isolated_dwelling" then mz=13
		elseif place == "locality"      then mz=13
		elseif place == "island"      then mz=12
		end

		Layer("place", false)
		Attribute("class", place)
		MinZoom(mz)
		if rank then AttributeNumeric("rank", rank) end
		if capital then AttributeNumeric("capital", capital) end
		if place=="country" then
			local iso_a2 = Find("ISO3166-1:alpha2")
			while iso_a2 == "" do
				local rel, role = NextRelation()
				if not rel then break end
				if role == 'label' then
					iso_a2 = FindInRelation("ISO3166-1:alpha2")
				end
			end
			Attribute("iso_a2", iso_a2)
		end
		SetNameAttributes() -- KEEP SetNameAttributes() for 'place'
		return
	end

	-- Write 'poi'
	local rank, class, subclass = GetPOIRank()
	if rank then WritePOI(class,subclass,rank) end

	-- Write 'mountain_peak' and 'water_name'
	local natural = Find("natural")
	if natural == "peak" or natural == "volcano" then
		Layer("mountain_peak", false)
		SetEleAttributes()
		AttributeNumeric("rank", 1)
		Attribute("class", natural)
		-- SetNameAttributes() -- Removed name setting for mountain_peak
		return
	end
	if natural == "bay" then
		Layer("water_name", false)
		-- SetNameAttributes() -- Removed name setting for water_name (bay)
		return
	end
end

-- Process way tags

majorRoadValues = Set { "motorway", "trunk", "primary" }
z9RoadValues  = Set { "secondary", "motorway_link", "trunk_link" }
z10RoadValues  = Set { "primary_link", "secondary_link" }
z11RoadValues   = Set { "tertiary", "tertiary_link", "busway", "bus_guideway" }
-- On zoom 12, various road classes are merged into "minor"
z12MinorRoadValues = Set { "unclassified", "residential", "road", "living_street" }
z12OtherRoadValues = Set { "raceway" }
z13RoadValues     = Set { "track", "service" }
manMadeRoadValues = Set { "pier", "bridge" }
pathValues      = Set { "footway", "cycleway", "bridleway", "path", "steps", "pedestrian", "platform" }
linkValues      = Set { "motorway_link", "trunk_link", "primary_link", "secondary_link", "tertiary_link" }
pavedValues     = Set { "paved", "asphalt", "cobblestone", "concrete", "concrete:lanes", "concrete:plates", "metal", "paving_stones", "sett", "unhewn_cobblestone", "wood" }
unpavedValues   = Set { "unpaved", "compacted", "dirt", "earth", "fine_gravel", "grass", "grass_paver", "gravel", "gravel_turf", "ground", "ice", "mud", "pebblestone", "salt", "sand", "snow", "woodchips" }
railwayClasses  = { rail="rail", narrow_gauge="rail", preserved="rail", funicular="rail", subway="transit", light_rail="transit", monorail="transit", tram="transit" }

aerowayBuildings= Set { "terminal", "gate", "tower" }
landcoverKeys   = {
    -- Natural features
    wood="wood", forest="wood",
    wetland="wetland",
    beach="sand", sand="sand", dune="sand",
    glacier="glacier", ice_shelf="ice",
    bare_rock="rock", scree="rock",
    fell="grass", grassland="grass", grass="grass", heath="grass", meadow="grass", tundra="grass",
    shrubbery="grass", -- Added natural=shrubbery
    water="water", -- Explicitly added natural=water

    -- Leisure features
    park="grass", garden="grass", golf_course="grass", recreation_ground="grass", village_green="grass",

    -- Landuse features
    farmland="farmland", farm="farmland", orchard="farmland", vineyard="farmland", plant_nursery="farmland",
    allotments="grass",
    cemetery="grass",
    greenfield="grass",
    conservation="conservation",
    forest="wood", -- landuse=forest, map to 'wood'
    grass="grass", -- landuse=grass, map to 'grass'
    scrub="grass", -- landuse=scrub, map to 'grass'
    flowerbed="grass", -- landuse=flowerbed
    
    -- Water-related landuse features
    salt_pond="water", -- landuse=salt_pond
    basin="water", -- landuse=basin
    reservoir="water", -- landuse=reservoir
    pond="water", -- landuse=pond
    
    -- Other landuse features that are often green/water related
    riverbank="grass", -- landuse=riverbank (often green)
    meadow="grass", -- landuse=meadow
    greenhouse_horticulture="farmland", -- landuse=greenhouse_horticulture, map to 'farmland'

    -- Landcover features (explicitly mapping if they are distinct or common as primary landcover tags)
    trees="wood", -- landcover=trees
    grass="grass", -- landcover=grass
    sand="sand", -- landcover=sand
    rock="rock", -- landcover=rock
    ice="ice", -- landcover=ice
    snow="snow", -- landcover=snow
    mud="wetland", -- landcover=mud (often associated with wetlands)
    crops="farmland", -- landcover=crops
    vineyard="farmland", -- landcover=vineyard
    orchard="farmland", -- landcover=orchard
    forest="wood", -- landcover=forest
    wetland="wetland", -- landcover=wetland
    bare_rock="rock", -- landcover=bare_rock
    glacier="glacier", -- landcover=glacier
    beach="sand", -- landcover=beach
    dune="sand", -- landcover=dune
    farmland="farmland", -- landcover=farmland
    water="water", -- landcover=water
}

-- POI key/value pairs: Filtered for Bike and Traveler related POIs
poiTags         = {
    aerialway = Set { "station" }, -- Traveler: transport
    amenity = Set {
        "bicycle_parking", "bicycle_rental", "bus_station", "ferry_terminal", "fuel",
        "parking", "shelter", "taxi", "toilets", "traffic_signals"
    }, -- Strictly filtered amenities for travelers/bikes
    building = Set { "dormitory" }, -- Traveler: lodging (This will be filtered out by the aggressive building filter)
    highway = Set { "bus_stop", "traffic_signals" }, -- Traveler: transport/infrastructure
    historic = Set { "castle", "ruins" }, -- Traveler: attractions
    leisure = Set {
        "marina", "sports_centre", "stadium", "swimming_area", "swimming_pool", "water_park"
    }, -- Traveler: leisure/attractions
    railway = Set { "halt", "station", "subway_entrance", "train_station_entrance", "tram_stop" }, -- Traveler: transport
    shop = Set {
        "alcohol", "bakery", "beverages", "bicycle", "books", "chemist", "convenience",
        "department_store", "general", "gift", "greengrocer", "ice_cream", "kiosk",
        "newsagent", "supermarket", "tobacco", "travel_agency"
    }, -- Traveler: essentials/supplies/bike
    sport = Set {
        "bmx", "cycling", "climbing", "climbing_adventure", "disc_golf", "diving",
        "free_flying", "golf", "model_aerodrome", "motocross", "motor", "multi",
        "orienteering", "paintball", "paragliding", "rc_car", "sailing", "scuba_diving",
        "shooting", "shooting_range", "skateboard", "skating", "skiing", "surfing",
        "swimming", "toboggan", "water_ski"
    }, -- Traveler: activities
    tourism = Set {
        "alpine_hut", "aquarium", "artwork", "attraction", "bed_and_breakfast", "camp_site",
        "caravan_site", "chalet", "gallery", "guest_house", "hostel", "hotel", "information",
        "motel", "museum", "picnic_site", "theme_park", "viewpoint", "zoo"
    }, -- Traveler: lodging/attractions/information
    waterway = Set { "dock" } -- Traveler: transport
}

-- POI "class" values: Filtered to match remaining POI classes
poiClasses      = {
    golf="golf", golf_course="golf", miniature_golf="golf",
    fast_food="fast_food", food_court="fast_food", -- These will be removed by amenity filter
    bus_stop="bus", bus_station="bus",
    subway_entrance="entrance", train_station_entrance="entrance",
    camp_site="campsite", caravan_site="campsite",
    laundry="laundry", dry_cleaning="laundry", -- These will be removed by amenity filter
    supermarket="grocery", deli="grocery", delicatessen="grocery", greengrocer="grocery", marketplace="grocery",
    books="library", library="library", -- These will be removed by amenity filter
    hotel="lodging", motel="lodging", bed_and_breakfast="lodging", guest_house="lodging", hostel="hostel", chalet="chalet", alpine_hut="alpine_hut", dormitory="dormitory",
    chocolate="ice_cream", confectionery="ice_cream", -- These will be removed by amenity filter
    post_box="post",  post_office="post", -- These will be removed by amenity filter
    cafe="cafe", -- This will be removed by amenity filter
    alcohol="alcohol_shop",  beverages="alcohol_shop",  wine="alcohol_shop",
    bar="bar", nightclub="bar", -- These will be removed by amenity filter
    marina="harbor", dock="harbor",
    car="car", car_repair="car", taxi="car",
    hospital="hospital", nursing_home="hospital",  clinic="clinic", -- These will be removed by amenity filter
    grave_yard="cemetery", cemetery="cemetery", -- These will be removed by amenity filter
    attraction="attraction", viewpoint="attraction",
    biergarten="beer", pub="beer", -- These will be removed by amenity filter
    traffic_signals="traffic_signals"
}
-- POI Class Ranks: Filtered to match remaining POI classes
poiClassRanks   = {
    hospital=1, -- Removed
    railway=2, bus=3, attraction=4, harbor=5,
    campsite=11, cemetery=12, -- Removed
    library=14, -- Removed
    post=16, -- Removed
    golf=17, shop=18,
    grocery=19, -- Removed
    fast_food=20, -- Removed
    bar=22, -- Removed
    traffic_signals=17,
    lodging=10,
}
waterClasses    = Set { "river", "riverbank", "stream", "canal", "drain", "ditch", "dock" }
waterwayClasses = Set { "stream", "river", "canal", "drain", "ditch" }

-- Scan relations for use in ways

function relation_scan_function()
	if Find("type")=="boundary" and Find("boundary")=="administrative" then
		Accept()
	end
end

function write_to_transportation_layer(minzoom, highway_class, subclass, ramp, service, is_rail, is_road, is_area)
	Layer("transportation", is_area)
	SetZOrder()
	Attribute("class", highway_class)
	if subclass and subclass ~= "" then
		Attribute("subclass", subclass)
	end
	AttributeNumeric("layer", tonumber(Find("layer")) or 0, accessMinzoom)
	SetBrunnelAttributes()
	-- We do not write any other attributes for areas.
	if is_area then
		SetMinZoomByAreaWithLimit(minzoom)
		return
	end
	MinZoom(minzoom)
	if ramp then AttributeNumeric("ramp",1) end

	-- Service
	if (is_rail or highway_class == "service") and (service and service ~="") then Attribute("service", service) end

	local accessMinzoom = 9
	if is_road then
		local oneway = Find("oneway")
		if oneway == "yes" or oneway == "1" then
			AttributeNumeric("oneway",1)
		end
		if oneway == "-1" then
			-- **** TODO
		end
		local surface = Find("surface")
		local surfaceMinzoom = 12
		if pavedValues[surface] then
			Attribute("surface", "paved", surfaceMinzoom)
		elseif unpavedValues[surface] then
			Attribute("surface", "unpaved", surfaceMinzoom)
		end
		if Holds("access") then Attribute("access", Find("access"), accessMinzoom) end
		if Holds("bicycle") then Attribute("bicycle", Find("bicycle"), accessMinzoom) end
		if Holds("foot") then Attribute("foot", Find("foot"), accessMinzoom) end
		if Holds("horse") then Attribute("horse", Find("horse"), accessMinzoom) end
		AttributeBoolean("toll", Find("toll") == "yes", accessMinzoom)
		if Find("expressway") == "yes" then AttributeBoolean("expressway", true, 7) end
		if Holds("mtb_scale") then Attribute("mtb:scale", Find("mtb:scale"), 10) end
	end
end

-- Process way tags

function way_function()
	local route    = Find("route")
	local highway  = Find("highway")
	local waterway = Find("waterway")
	local water    = Find("water")
	local building = Find("building")
	local natural  = Find("natural")
	local historic = Find("historic")
	local landuse  = Find("landuse")
	local leisure  = Find("leisure")
	local amenity  = Find("amenity")
	local aeroway  = Find("aeroway")
	local railway  = Find("railway")
	local service  = Find("service")
	local shop     = Find("shop")
	local tourism  = Find("tourism")
	local man_made = Find("man_made")
	local boundary = Find("boundary")
	local aerialway  = Find("aerialway")
	local public_transport  = Find("public_transport")
	local place = Find("place")
	local is_closed = IsClosed()
	local housenumber = Find("addr:housenumber") -- This variable is no longer used due to house number removal
	local write_name = false
	local construction = Find("construction")
	local is_highway_area = highway~="" and Find("area")=="yes" and is_closed

	-- Miscellaneous preprocessing
	if Find("disused") == "yes" then return end
	if boundary~="" and Find("protection_title")=="National Forest" and Find("operator")=="United States Forest Service" then return end
	if highway == "proposed" then return end
	if aerowayBuildings[aeroway] then building="yes"; aeroway="" end

	-- --- START: Aggressive Removal Filters (MODIFIED) ---
	-- Remove ALL building features (nodes, ways, relations)
	if building ~= "" then return end

	-- Removed the filtering for landuse tags to allow landuse-tagged greeneries to be processed.
	-- The following line was removed:
	-- if landuse ~= "" then return end

	-- Removed the filtering for natural/leisure tags to allow greeneries to be processed.
	-- The following lines were removed:
	-- if natural == "forest" or natural == "wood" or
	--    natural == "grassland" or natural == "grass" or natural == "scrub" or
	--    leisure == "park" or leisure == "garden" or leisure == "golf_course" or
	--    amenity == "university" or amenity == "school" or amenity == "college" or amenity == "zoo" then
	-- 	return
	-- end

	-- Remove ALL path features
	if pathValues[highway] then return end
	-- --- END: Aggressive Removal Filters (MODIFIED) ---

	-- Explicitly handle natural=coastline for the 'water' layer (sea/ocean)
	-- Removed direct area creation from coastline, relying on tilemaker's internal process
	-- if natural == "coastline" then
	--     Layer("water", true) -- Add to water layer as an area
	--     Attribute("class", "ocean") -- Classify as ocean
	--     MinZoom(0) -- Visible from zoom 0
	--     return -- Process no further for coastline
	-- end

	-- Removed specific building layer creation for dams
	-- elseif waterway == "dam" then Layer("building",is_closed)

	if place == "island" then
		LayerAsCentroid("place")
		Attribute("class", place)
		MinZoom(10)
		local pop = tonumber(Find("population")) or 0
		local capital = capitalLevel(Find("capital"))
		local rank = calcRank(place, pop, nil)
		if rank then AttributeNumeric("rank", rank) end
		SetNameAttributes() -- KEEP SetNameAttributes() for 'place' (island)
		return -- Return early after processing place islands
	end

	-- Boundaries within relations
	-- note that we process administrative boundaries as properties on ways, rather than as single relation geometries,
	--  because otherwise we get multiple renderings where boundaries are coterminous
	local admin_level = 11
	local isBoundary = false
	while true do
		local rel = NextRelation()
		if not rel then break end
		isBoundary = true
		admin_level = math.min(admin_level, tonumber(FindInRelation("admin_level")) or 11)
	end

	-- Boundaries in ways
	if boundary=="administrative" then
		local mz = 0
		if     admin_level>=3 and admin_level<5 then mz=4
		elseif admin_level>=5 and admin_level<7 then mz=8
		elseif admin_level==7 then mz=10
		elseif admin_level>=8 then mz=12
		end

		Layer("boundary",false)
		AttributeNumeric("admin_level", admin_level)
		MinZoom(mz)
		-- disputed status (0 or 1). some styles need to have the 0 to show it.
		local disputed = Find("disputed")
		if disputed=="yes" then
			AttributeNumeric("disputed", 1)
		else
			AttributeNumeric("disputed", 0)
		end
		-- SetNameAttributes() -- Removed name setting for boundaries
	end

	-- Aerialways ('transportation' and 'transportation_name')
	if aerialway ~= "" then
		write_to_transportation_layer(12, "aerialway", aerialway, false, nil, false, false, is_closed)
		-- Removed name setting for transportation_name (aerialway)
	end

	-- Roads ('transportation' and 'transportation_name')
	if highway ~= "" or public_transport == "platform" then
		local access = Find("access")
		local surface = Find("surface")

		local h = highway
		local is_road = true
		if h == "" then
			h = public_transport
			is_road = false
		end
		local subclass = nil
		local under_construction = false
		if highway == "construction" and construction ~= "" then
			h = construction
			under_construction = true
		end
		local minzoom = INVALID_ZOOM
		if majorRoadValues[h]        then minzoom = 4
		elseif h == "trunk"          then minzoom = 5
		elseif highway == "primary"  then minzoom = 7
		elseif z9RoadValues[h]       then minzoom = 9
		elseif z10RoadValues[h]      then minzoom = 10
		elseif z11RoadValues[h]      then minzoom = 11
		elseif z12MinorRoadValues[h] then
			minzoom = 12
			subclass = h
			h = "minor"
		elseif z12OtherRoadValues[h] then minzoom = 12
		elseif z13RoadValues[h]      then minzoom = 13
		-- Removed pathValues handling here as paths are filtered at the top
		end

		-- Links (ramp)
		local ramp=false
		if linkValues[h] then
			splitHighway = split(highway, "_")
			highway = splitHighway[1]; h = highway
			ramp = true
		end

		-- Construction
		if under_construction then
			h = h .. "_construction"
		end

		-- Drop underground platforms
		local layer = Find("layer")
		local layerNumeric = tonumber(layer)
		if not is_road and layerNumeric ~= nil and layerNumeric < 0 then
			minzoom = INVALID_ZOOM
		end

		-- Drop all areas except infrastructure for pedestrians handled above
		if is_highway_area and h ~= "path" then
			minzoom = INVALID_ZOOM
		end

		-- Write to layer
		if minzoom <= 14 then
			write_to_transportation_layer(minzoom, h, subclass, ramp, service, false, is_road, is_highway_area)

			-- Removed name setting for transportation_name (roads)
		end
	end

	-- Railways ('transportation' and 'transportation_name')
	if railway~="" then
		local class = railwayClasses[railway]
		if class then
			local minzoom = 14
			local usage = Find("usage")
			if railway == "rail" and service == "" then
				if usage == "main" then
					minzoom = 8
				else
					minzoom = 10
				end
			elseif railway == "narrow_gauge" and service == "" then
				minzoom = 10
			elseif railway == "light_rail" and service == "" then
				minzoom = 11
			end
			write_to_transportation_layer(minzoom, class, railway, false, service, true, false, is_closed)

			-- Removed name setting for transportation_name (railways)
		end
	end

	-- Pier
	if manMadeRoadValues[man_made] then
		write_to_transportation_layer(13, man_made, nil, false, nil, false, false, is_closed)
	end

	-- 'Ferry'
	if route=="ferry" then
		write_to_transportation_layer(9, "ferry", nil, false, nil, false, false, is_closed)

		-- Removed name setting for transportation_name (ferries)
	end

	-- 'Aeroway'
	if aeroway~="" then
		Layer("aeroway", is_closed)
		Attribute("class",aeroway)
		Attribute("ref",Find("ref"))
		-- write_name = true -- Removed this as names are generally suppressed
	end

	-- 'aerodrome_label'
	if aeroway=="aerodrome" then
	 	LayerAsCentroid("aerodrome_label")
	 	-- SetNameAttributes() -- Removed name setting for aerodrome_label
	 	Attribute("iata", Find("iata"))
  		SetEleAttributes()
 	 	Attribute("icao", Find("icao"))

 	 	local aerodrome = Find("aeroway") -- Corrected from Find(aeroway)
 	 	local class
 	 	if aerodromeValues[aerodrome] then class = aerodrome else class = "other" end
 	 	Attribute("class", class)
	end

	-- Set 'waterway' and associated
	if waterwayClasses[waterway] and not is_closed then
		if waterway == "river" and Holds("name") then
			Layer("waterway", false)
		else
			Layer("waterway_detail", false)
		end
		if Find("intermittent")=="yes" then AttributeNumeric("intermittent", 1) else AttributeNumeric("intermittent", 0) end
		Attribute("class", waterway)
		-- SetNameAttributes() -- Removed name setting for waterway
		SetBrunnelAttributes()
	-- Explicitly removed any landuse layer creation for "boatyard" and "fuel"
	elseif waterway == "boatyard"  then
		-- Removed Layer("landuse", is_closed);
		Attribute("class", "industrial"); MinZoom(12)
	elseif waterway == "fuel"      then
		-- Removed Layer("landuse", is_closed);
		Attribute("class", "industrial"); MinZoom(14)
	end
	-- Set names on rivers
	if waterwayClasses[waterway] and not is_closed then
		if waterway == "river" and Holds("name") then
			Layer("water_name", false)
		else
			Layer("water_name_detail", false)
			MinZoom(14)
		end
		Attribute("class", waterway)
		-- SetNameAttributes() -- Removed name setting for water_name (river)
	end

	-- Set 'water' (for lakes, rivers, and now ocean)
	-- Removed 'landuse' from this check as landuse is removed completely
	if natural=="water" or leisure=="swimming_pool" or waterClasses[waterway] then
		if Find("covered")=="yes" or not is_closed then return end
		local class="lake"; if waterway~="" then class="river" end
		if class=="lake" and Find("wikidata")=="Q192770" then return end
		Layer("water",true)
		SetMinZoomByArea(way)
		Attribute("class",class)

		if Find("intermittent")=="yes" then Attribute("intermittent",1) end
		-- Removed name setting for water_name_detail
	end

	-- Set 'landcover' (from natural, leisure, landuse, and now explicitly landcover)
	local l = natural
	if l=="" then l=landuse end -- Prioritize landuse over leisure for area types
	if l=="" then l=leisure end
	if l=="" then l=Find("landcover") end -- Add landcover as the last fallback for 'l'

	if landcoverKeys[l] then
		Layer("landcover", true); -- Re-enabled layer creation for landcover
		SetMinZoomByArea()
		Attribute("class", landcoverKeys[l])
		if l=="wetland" then Attribute("subclass", Find("wetland"))
		else Attribute("subclass", l) end
		-- write_name = true -- Names are generally suppressed for landcover
	end

	-- Parks
	-- Re-enabled processing for national parks and nature reserves.
	if     boundary=="national_park" then Layer("park",true); Attribute("class",boundary); -- SetNameAttributes() -- Names are generally suppressed for parks
	elseif leisure=="nature_reserve" then Layer("park",true); Attribute("class",leisure ); -- SetNameAttributes() -- Names are generally suppressed for nature_reserve
	end

	-- POIs ('poi' and 'poi_detail')
	local rank, class, subclass = GetPOIRank()
	if rank then WritePOI(class,subclass,rank); return end

	-- Catch-all (for named features that are not specific POIs, buildings, or landuse)
	if write_name and Holds("name") then -- This check is for original OSM names, but we won't write them
		LayerAsCentroid("poi_detail")
		AttributeNumeric("rank", 6) -- Default rank for named features without specific POI rank
		-- SetNameAttributes() -- Removed name setting for catch-all POIs
	end
end

-- Remap coastlines
function attribute_function(attr,layer)
	if attr["featurecla"]=="Glaciated areas" then
		return { subclass="glacier" }
	elseif attr["featurecla"]=="Antarctic Ice Shelf" then
		return { subclass="ice_shelf" }
	elseif layer=="ocean" then
		return { class="ocean" } -- Ensures 'ocean' layer from Tilemaker gets 'class=ocean' attribute
	else
		return attr
	end
end

-- ==========================================================
-- Common functions

-- Write a way centroid to POI layer
function WritePOI(class,subclass,rank)
	local layer = "poi"
	if rank>4 then layer="poi_detail" end
	LayerAsCentroid(layer)
	-- Conditionally set rank or class based on the type of POI
	if class == "shop" then -- For shops, remove names and only keep subclasses and poiClassRanks
		AttributeNumeric("rank", rank)
		Attribute("class", "shop") -- Explicitly set class to "shop"
		if subclass ~= "" then -- Only set subclass if it's not empty
			Attribute("subclass", subclass)
		end
		-- Explicitly remove name attributes for shops
		Attribute("name", "")
		Attribute("name:en", "")
		-- Add more language-specific name attributes if needed to ensure complete removal
		-- e.g., Attribute("name:ta", "") if 'name:ta' is common in your data
	elseif class == "amenity" and subclass == "fuel" then -- NEW: Add names for fuel stations
		SetNameAttributes()
		AttributeNumeric("rank", rank)
		Attribute("class", class)
		if subclass ~= "" then
			Attribute("subclass", subclass)
		end
	else
		-- Do NOT call SetNameAttributes() for any POIs (except 'place' which is handled in node_function/way_function)
		AttributeNumeric("rank", rank)
		Attribute("class", class)
		if subclass ~= "" then -- Only set subclass if it's not empty
			Attribute("subclass", subclass)
		end
	end
	-- layer defaults to 0
	AttributeNumeric("layer", tonumber(Find("layer")) or 0)
	-- indoor defaults to false
	AttributeBoolean("indoor", (Find("indoor") == "yes"))
	-- level has no default
	local level = tonumber(Find("level"))
	if level then
		AttributeNumeric("level", level)
	end
end

-- Check if there are name tags on the object
-- This function remains as is, but its calls will be removed where names are not desired.
function HasNames()
	if Holds("name") then return true end
	local iname
	local main_written = name
	if preferred_language and Holds("name:"..preferred_language) then return true end
	-- then set any additional languages
	for i,lang in ipairs(additional_languages) do
		if Holds("name:"..lang) then return true end
	end
	return false
end

-- Set name attributes on any object
-- This function will now only be called for 'place' features.
function SetNameAttributes()
	local name = Find("name"), iname
	local main_written = name
	-- if we have a preferred language, then write that (if available), and additionally write the base name tag
	if preferred_language and Holds("name:"..preferred_language) then
		iname = Find("name:"..preferred_language)
		Attribute(preferred_language_attribute, iname)
		if iname~=name and default_language_attribute then
			Attribute(default_language_attribute, name)
		else main_written = iname end
	else
		Attribute(preferred_language_attribute, name)
	end
	-- then set any additional languages
	for i,lang in ipairs(additional_languages) do
		iname = Find("name:"..lang)
		if iname=="" then iname=name end
		if iname~=main_written then Attribute("name:"..lang, iname) end
	end
end

-- Set ele and ele_ft on any object
function SetEleAttributes()
    local ele = Find("ele")
	if ele ~= "" then
		local meter = math.floor(tonumber(ele) or 0)
		local feet = math.floor(meter * 3.2808399)
		AttributeNumeric("ele", meter)
		AttributeNumeric("ele_ft", feet)
    end
end

function SetBrunnelAttributes()
	if Find("bridge") == "yes" or Find("man_made") == "bridge" then Attribute("brunnel", "bridge")
	elseif Find("tunnel") == "yes" then Attribute("brunnel", "tunnel")
	elseif Find("ford")   == "yes" then Attribute("brunnel", "ford")
	end
end

-- Set minimum zoom level by area
function SetMinZoomByArea()
	SetMinZoomByAreaWithLimit(0)
end

-- Set minimum zoom level by area but not below given minzoom
function SetMinZoomByAreaWithLimit(minzoom)
	local area=Area()
	if     minzoom <= 6 and area>ZRES5^2  then MinZoom(6)
	elseif minzoom <= 7 and area>ZRES6^2  then MinZoom(7)
	elseif minzoom <= 8 and area>ZRES7^2  then MinZoom(8)
	elseif minzoom <= 9 and area>ZRES9^2  then MinZoom(9)
	elseif minzoom <= 10 and area>ZRES10^2 then MinZoom(10)
	elseif minzoom <= 11 and area>ZRES11^2 then MinZoom(11)
	elseif minzoom <= 12 and area>ZRES12^2 then MinZoom(12)
	elseif minzoom <= 13 and area>ZRES13^2 then MinZoom(13)
	else                      MinZoom(14) end
end

-- Calculate POIs (typically rank 1-4 go to 'poi' z12-14, rank 5+ to 'poi_detail' z14)
-- returns rank, class, subclass
function GetPOIRank()
	local k,list,v,class,rank

	-- Can we find the tag?
	for k,list in pairs(poiTags) do
		if list[Find(k)] then
			v = Find(k)	-- k/v are the OSM tag pair
			class = poiClasses[v] or k
			rank  = poiClassRanks[class] or 25
			-- Defensive check for poiSubClasses being nil before indexing
			local subclassKey = nil
			if poiSubClasses then
				subclassKey = poiSubClasses[v]
			end

			if subclassKey then
				class = v
				v = Find(subclassKey)
			end
			return rank, class, v
		end
	end

	-- Catch-all for shops (modified to only keep subclass and poiClassRanks)
	local shop = Find("shop")
	if shop~="" then return poiClassRanks['shop'], "shop", shop end

	-- Nothing found
	return nil,nil,nil
end

-- Removed SetBuildingHeightAttributes function entirely as buildings are completely removed.

-- Implement z_order as calculated by Imposm
-- See https://imposm.org/docs/imposm3/latest/mapping.html#wayzorder for details.
function SetZOrder()
	local highway = Find("highway")
	local layer = tonumber(Find("layer"))
	local bridge = Find("bridge")
	local tunnel = Find("tunnel")
	local zOrder = 0
	if bridge ~= "" and bridge ~= "no" then
		zOrder = zOrder + 10
	elseif tunnel ~= "" and tunnel ~= "no" then
		zOrder = zOrder - 10
	end
	if not (layer == nil) then
		if layer > 7 then
			layer = 7
		elseif layer < -7 then
			layer = -7
		end
		zOrder = zOrder + layer * 10
	end
	local hwClass = 0
	-- See https://github.com/omniscale/imposm3/blob/53bb80726ca9456e4a0857b38803f9cc2fe8e33fd/mapping/columns.go#L251
	if highway == "motorway" then
		hwClass = 9
	elseif highway == "trunk" then
		hwClass = 8
	elseif highway == "primary" then
		hwClass = 6
	elseif highway == "secondary" then
		hwClass = 5
	elseif highway == "tertiary" then
		hwClass = 4
	else
		hwClass = 3
	end
	zOrder = zOrder + hwClass
	ZOrder(zOrder)
end

-- ==========================================================
-- Lua utility functions

function split(inputstr, sep) -- https://stackoverflow.com/a/7615129/4288232
	if sep == nil then
		sep = "%s"
	end
	local t={} ; i=1
	for str in string.gmatch(inputstr, "([^"..sep.."]+)") do
		t[i] = str
		i = i + 1
	end
	return t
end

-- vim: tabstop=2 shiftwidth=2 noexpandtab
