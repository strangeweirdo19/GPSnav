#ifndef POLYLINE_H
#define POLYLINE_H

#include <vector>
#include <string>
#include "common.h"

// Decodes a Valhalla/OSRM encoded polyline string (precision 6) into a vector of RouteNodes (deltas)
// Updates activeRouteAnchor with the first point's coordinates.
// Stateful decoder for chunked updates
struct PolylineDecoderState {
    int32_t currentLat1e6 = 0;
    int32_t currentLon1e6 = 0;
    int32_t prevLat1e5 = 0;
    int32_t prevLon1e5 = 0;
    bool first = true;
    std::string remainder = "";
    
    void reset() {
        currentLat1e6 = 0;
        currentLon1e6 = 0;
        prevLat1e5 = 0;
        prevLon1e5 = 0;
        first = true;
        remainder = "";
    }
};

bool decodePolyline(const std::string& encoded, std::vector<RouteNode, PSRAMAllocator<RouteNode>>& points);
bool decodePolylineChunk(const std::string& chunk, std::vector<RouteNode, PSRAMAllocator<RouteNode>>& points, PolylineDecoderState& state, RouteAnchor* anchor = nullptr);

#endif // POLYLINE_H
