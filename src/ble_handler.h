// ble_handler.h
#ifndef BLE_HANDLER_H
#define BLE_HANDLER_H

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "config.h"  // WiFi credentials

// BLE Service and Characteristic UUIDs
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define RX_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"  // App -> ESP32
#define TX_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"  // ESP32 -> App

// Security settings
#define MAX_AUTH_ATTEMPTS 3
#define AUTH_LOCKOUT_TIME_MS 60000  // 60 seconds
#define SESSION_TIMEOUT_MS 300000   // 5 minutes
#define PAIRING_WINDOW_MS 60000     // 60 seconds - only allow new pairings in first minute
#define PAIRING_HEARTBEAT_TIMEOUT_MS 3500 // 3.5s timeout for pairing modal

// BLE Handler class
class BLEHandler {
public:
    BLEHandler();
    void init();
    void loop();  // Call this in main loop for session timeout check
    void sendNotification(const String& message);
    void updateAdvertising(bool pairingOpen); // New method
    bool isConnected();
    bool isDeviceAuthenticated();
    String getDeviceName();
    String getPIN();
    
    // Bonding management methods
    int getBondedDeviceCount();
    String getBondedDevicesList();  // Returns JSON array of bonded devices
    bool clearAllBonds();           // Clear all bonding data
    bool removeBond(const String& address);  // Remove specific device
    
    // Public methods for callbacks
    void resetAuthState();
    bool isWithinPairingWindow();
    
    // Callbacks for parsed commands (to be set by main application)
    void (*onCoordinatesReceived)(double lat, double lon) = nullptr;
    void (*onZoomReceived)(float zoom) = nullptr;
    void (*onOTAFirmwareReceived)(String url) = nullptr;
    void (*onOTAMapReceived)(String url) = nullptr;
    void (*onOTABLEProgress)(int percent) = nullptr;  // Called during BLE OTA transfer
    void (*onDeviceConnected)() = nullptr;   // Called when device connects
    void (*onAuthenticated)() = nullptr;     // Called when PIN verified
    void (*onDeviceDisconnected)() = nullptr; // Called when device disconnects
    
    // Public method for parsing received data (called by callbacks)
    void parseReceivedData(const String& data);
    
    // Public access to boot time for pairing window calculations
    unsigned long bootTime;  // Track when device booted for pairing window
    
    // Silent Auth State (Public for callbacks)
    bool waitingForToken;
    unsigned long authStartTime;
    unsigned long lastPairingHeartbeatTime; // Track heartbeats during pairing
    bool wifiOTAStartRequested; // Flag to trigger WiFi OTA from main loop

    // OTA State (Public for Global Handlers)
    bool otaActive;
    unsigned long otaStartTime;
    bool updateStarted;
    unsigned long clientConnectedTime;
    void handleOTA_WiFi_Start(); 
    void checkOTATimeout();
    
    // Static instance pointer for callback access
    static BLEHandler* instance;
    
    // Blacklist management (public for callback access)
    void addToBlacklist(NimBLEAddress addr);
    bool isBlacklisted(NimBLEAddress addr);

private:
    NimBLEServer* pServer;
    NimBLECharacteristic* pTxCharacteristic;
    NimBLECharacteristic* pRxCharacteristic;
    
    // Authentication state
    bool isAuthenticated;
    String devicePIN;
    String deviceName;
    int authAttempts;
    unsigned long lockoutUntil;
    unsigned long lastActivityTime;
    
    
    // Token buffer management
    bool addToken(const String& token);        // Add token to rolling buffer
    bool isValidToken(const String& token);    // Check if token is valid
    void removeToken(const String& token);     // Remove specific token
    void handleConnectionAuth(const String& token, bool hasToken);  // Centralized auth logic
    
    // Blacklist storage
    static const int MAX_BLACKLISTED = 10;
    NimBLEAddress blacklistedDevices[MAX_BLACKLISTED];
    int blacklistCount;
    
    void generatePIN();
    void generateDeviceName();
    bool validatePIN(const String& pin);
    bool checkSessionTimeout();
    bool validateInput(const String& data);
    
    String base64Encode(const String& input);
    void handleOTAFirmware(const String& url);
    void handleOTAMap(const String& url);
    bool connectToWiFi();
    
    // Exposing WiFi OTA methods to be private or public? 
    // Wait, handleOTA_WiFi_Start is already private but needed? 
    // Actually handleOTA_WiFi_Start is called by loop (public) so it can stay private if loop calls it?
    // But checkOTATimeout is called by loop so it can stay private.
    // However, updateStarted is accessed by global function handleUpdateUpload.
    // So updateStarted MUST be public.
    
    // Cleaning up private section:
    // (Removed OTA vars from here)
};

// Global BLE handler instance (defined in ble_handler.cpp)
extern BLEHandler bleHandler;

#endif // BLE_HANDLER_H
