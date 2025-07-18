// data_task.h
#ifndef DATA_TASK_H
#define DATA_TASK_H

#include "common.h" // Include common definitions

// Arduino and other hardware-specific includes needed by dataTask
#include <Arduino.h>
#include <SPI.h>
#include <FS.h>
#include "SD_MMC.h"
#include <sqlite3.h> // For SQLite DB operations

// Function declaration for the data task
void dataTask(void *pvParameters);

// Forward declaration for parseMVTForTile - FIXED: Added this declaration
void parseMVTForTile(const uint8_t *data_buffer, size_t data_len, const TileKey& key);

// Helper functions for tile coordinates (needed by dataTask to determine MBTiles file)
double tileXToLon(int x, int z);
double tileYToLat(int y, int z);

#endif // DATA_TASK_H
