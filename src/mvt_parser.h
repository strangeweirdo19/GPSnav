// mvt_parser.h
#ifndef MVT_PARSER_H
#define MVT_PARSER_H

#include "common.h" // Include common definitions
#include <stdint.h> // For uint8_t, uint64_t

// Forward declarations for MVT decoding helper functions
inline uint64_t varint(const uint8_t *data, size_t &i, size_t dataSize);
inline int64_t zigzag(uint64_t n);
PSRAMString readPSRAMString(const uint8_t *data, size_t &i, size_t dataSize);

// Forward declaration for MVT layer parsing
ParsedLayer parseLayer(const uint8_t *data, size_t len);

// Forward declaration for the main MVT tile parsing function
void parseMVTForTile(const uint8_t *data_buffer, size_t data_len, const TileKey& key);

// Function to get the color for a given POI class/subclass (used in parsing)
uint16_t getIconColor(const PSRAMString& poiClass, const PSRAMString& poiSubclass);

#endif // MVT_PARSER_H
