// config.h
// Configuration file for WiFi and other settings

#ifndef CONFIG_H
#define CONFIG_H

// =========================================================
// WiFi Configuration for OTA Updates
// =========================================================
// Replace these with your actual WiFi credentials
// These are used when performing OTA firmware or map updates via BLE

#define WIFI_SSID "YourWiFiSSID"
#define WIFI_PASSWORD "YourWiFiPassword"

// =========================================================
// Debug Configuration
// =========================================================
// Set to 1 to enable debug output, 0 to disable
#define DEBUG_SERIAL 0
#define DEBUG_GPS 0
#define DEBUG_BLE 0
#define DEBUG_ROUTE 0

// Debug print macros - only print when debug is enabled
#if DEBUG_SERIAL
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(...)
#endif

// Note: You can also use WiFiManager library for runtime configuration
// if you prefer not to hardcode credentials

#endif // CONFIG_H
