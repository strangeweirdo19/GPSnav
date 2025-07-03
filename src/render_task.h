// render_task.h
#ifndef RENDER_TASK_H
#define RENDER_TASK_H

#include "common.h" // Include common definitions
#include <Arduino.h> // For basic Arduino types like uint16_t

// HMC5883L Compass includes (Moved from data_task.h)
#include <Wire.h> // Required for I2C communication
#include <Adafruit_Sensor.h> // Required for Adafruit unified sensor system
#include <Adafruit_HMC5883_U.h> // Required for Adafruit HMC5883L Unified Sensor

// Function declaration for the render task
void renderTask(void *pvParameters);

// Helper functions for tile coordinates (moved from data_task.cpp, now used by renderTask)
void latlonToTile(double lat, double lon, int z, int &x, int &y, int &ytms);
void latLonToMVTCoords(double lat, double lon, int z, int tileX, int tileY_TMS, int& mvtX, int& mvtY, int extent);

// Removed:
// double tileXToLon(int x, int z);
// double tileYToLat(int y, int z);

#endif // RENDER_TASK_H
