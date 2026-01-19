#include "polyline.h"
#include <Arduino.h>

bool decodePolyline(const std::string& encoded, std::vector<RouteNode, PSRAMAllocator<RouteNode>>& points) {
    size_t index = 0;
    size_t len = encoded.length();
    
    // Polyline decoding creates ABSOLUTE coordinates (1e6 scaled)
    int32_t currentLat1e6 = 0;
    int32_t currentLon1e6 = 0;
    
    // We need to track the PREVIOUS coordinate in 1e5 scale to calculate 1e5 deltas
    // for the RouteNode format.
    int32_t prevLat1e5 = 0;
    int32_t prevLon1e5 = 0;
    
    bool first = true;
    
    // Estimate capacity (conservative)
    points.clear();
    points.reserve(len / 3);

    while (index < len) {
        int32_t b, shift = 0, result = 0;

        // Decode Latitude
        do {
            if (index >= len) return false;
            b = encoded[index++] - 63;
            result |= (b & 0x1f) << shift;
            shift += 5;
        } while (b >= 0x20);
        int32_t dLat = ((result & 1) ? ~(result >> 1) : (result >> 1));
        currentLat1e6 += dLat;

        // Decode Longitude
        shift = 0; result = 0;
        do {
            if (index >= len) return false;
            b = encoded[index++] - 63;
            result |= (b & 0x1f) << shift;
            shift += 5;
        } while (b >= 0x20);
        int32_t dLon = ((result & 1) ? ~(result >> 1) : (result >> 1));
        currentLon1e6 += dLon;
        
        // --- Conversion to RouteNode (1e5 Deltas) ---
        
        int32_t lat1e5 = currentLat1e6 / 10;
        int32_t lon1e5 = currentLon1e6 / 10;
        
        if (first) {
            // First point: Set as Anchor (global) and add {0,0}
            activeRouteAnchor.lat = (double)currentLat1e6 / 1e6;
            activeRouteAnchor.lon = (double)currentLon1e6 / 1e6;
            
            RouteNode p = {0, 0};
            points.push_back(p);
            
            prevLat1e5 = lat1e5;
            prevLon1e5 = lon1e5;
            first = false;
        } else {
            // Calculate Delta relative to PREVIOUS point (1e5)
            // This matches the format expected by indexRoute / renderTask
            int32_t deltaLat = lat1e5 - prevLat1e5;
            int32_t deltaLon = lon1e5 - prevLon1e5;
            
            RouteNode p = {deltaLat, deltaLon};
            points.push_back(p);
            
            prevLat1e5 = lat1e5;
            prevLon1e5 = lon1e5;
        }
    }
    return true;
}

bool decodePolylineChunk(const std::string& chunk, std::vector<RouteNode, PSRAMAllocator<RouteNode>>& points, PolylineDecoderState& state, RouteAnchor* anchor) {
    std::string data = state.remainder + chunk;
    size_t len = data.length();
    size_t index = 0;
    size_t lastValidIndex = 0; // Index after last full point pair
    
    // Estimate capacity increase
    points.reserve(points.size() + len / 3);

    while (index < len) {
        // Save state backup for rollback
        int32_t tempLat = state.currentLat1e6;
        int32_t tempLon = state.currentLon1e6;
        size_t pointStartIndex = index;
        
        // --- Latitude ---
        int32_t b, shift = 0, result = 0;
        do {
            if (index >= len) { 
                // Incomplete Lat
                state.remainder = data.substr(pointStartIndex);
                return true; 
            }
            b = data[index++] - 63;
            result |= (b & 0x1f) << shift;
            shift += 5;
        } while (b >= 0x20);
        int32_t dLat = ((result & 1) ? ~(result >> 1) : (result >> 1));
        tempLat += dLat;

        // --- Longitude ---
        shift = 0; result = 0;
        do {
            if (index >= len) { 
                // Incomplete Lon -> Rollback entire point
                state.remainder = data.substr(pointStartIndex);
                return true; 
            }
            b = data[index++] - 63;
            result |= (b & 0x1f) << shift;
            shift += 5;
        } while (b >= 0x20);
        int32_t dLon = ((result & 1) ? ~(result >> 1) : (result >> 1));
        tempLon += dLon;
        
        // --- Commit ---
        state.currentLat1e6 = tempLat;
        state.currentLon1e6 = tempLon;
        lastValidIndex = index;
        
        // --- Conversion to RouteNode (1e5 Deltas) ---
        int32_t lat1e5 = state.currentLat1e6 / 10;
        int32_t lon1e5 = state.currentLon1e6 / 10;
        
        if (state.first) {
            // First point: Set Anchor (provided or global default)
            double lat = (double)state.currentLat1e6 / 1e6;
            double lon = (double)state.currentLon1e6 / 1e6;
            
            if (anchor != nullptr) {
                anchor->lat = lat;
                anchor->lon = lon;
            } else {
                activeRouteAnchor.lat = lat;
                activeRouteAnchor.lon = lon;
            }
            
            // Add {0,0} relative to anchor
            RouteNode p = {0, 0};
            points.push_back(p);
            
            state.prevLat1e5 = lat1e5;
            state.prevLon1e5 = lon1e5;
            state.first = false;
        } else {
            // Calculate Delta relative to PREVIOUS point (1e5)
            int32_t deltaLat = lat1e5 - state.prevLat1e5;
            int32_t deltaLon = lon1e5 - state.prevLon1e5;
            
            RouteNode p = {(int16_t)deltaLat, (int16_t)deltaLon};
            points.push_back(p);
            
            state.prevLat1e5 = lat1e5;
            state.prevLon1e5 = lon1e5;
        }
    }

    state.remainder = ""; // All consumed
    return true;
}
