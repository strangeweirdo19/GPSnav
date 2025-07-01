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

// HMC5883L Compass includes
#include <Wire.h> // Required for I2C communication
#include <Adafruit_Sensor.h> // Required for Adafruit unified sensor system
#include <Adafruit_HMC5883_U.h> // Required for Adafruit HMC5883L Unified Sensor

// Function declaration for the data task
void dataTask(void *pvParameters);

#endif // DATA_TASK_H
