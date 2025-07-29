// render_task.h
#ifndef RENDER_TASK_H
#define RENDER_TASK_H

#include "common.h"      // Include common definitions
#include "map_renderer.h" // Include the new map renderer header
#include <Arduino.h>     // For basic Arduino types like uint16_t

// HMC5883L Compass includes (Moved from data_task.h)
#include <Wire.h> // Required for I2C communication
#include <Adafruit_Sensor.h> // Required for Adafruit unified sensor system
#include <Adafruit_HMC5883_U.h> // Required for Adafruit HMC5883L Unified Sensor

// Function declaration for the render task
void renderTask(void *pvParameters);

#endif // RENDER_TASK_H
