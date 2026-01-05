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

// Note: You can also use WiFiManager library for runtime configuration
// if you prefer not to hardcode credentials

#endif // CONFIG_H
