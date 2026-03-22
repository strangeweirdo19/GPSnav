// ble_handler.h
#ifndef BLE_HANDLER_H
#define BLE_HANDLER_H

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "config.h"  // WiFi credentials
#include "polyline.h" // Include header for decoder struct

// BLE Service and Characteristic UUIDs
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define RX_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"  // App -> ESP32
#define TX_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"  // ESP32 -> App

// Security settings
#define MAX_AUTH_ATTEMPTS 3
#define AUTH_LOCKOUT_TIME_MS 60000  // 60 seconds
#define SESSION_TIMEOUT_MS 10000    // 10 seconds
#define PAIRING_WINDOW_MS 60000     // 60 seconds - only allow new pairings in first minute
#define PAIRING_HEARTBEAT_TIMEOUT_MS 3500 // 3.5s timeout for pairing modal

// Packet Structure Constants
#define PKT_MIN_SIZE 3  // Control Byte + State Byte + Checksum

// --- Control Byte Bitmasks ---
#define CTRL_BULK_MASK   0x03  // Bits 0 & 1
#define CTRL_ACK         0x04  // Bit 2
#define CTRL_NACK        0x08  // Bit 3
#define CTRL_RESET       0x10  // Bit 4
#define CTRL_REQ_STATUS  0x20  // Bit 5
#define CTRL_REQ_SYNC    0x40  // Bit 6
#define CTRL_HAS_GPS     0x80  // Bit 7

// Bulk Data Types (when CTRL_BULK_MASK == 0x03)
#define BULK_TYPE_AUTH   0x01
#define BULK_TYPE_ROUTE  0x02
#define BULK_TYPE_LOG    0x03

// --- State Byte Bitmasks ---
#define STATE_BRIGHTNESS_MASK 0x3F // Bits 0-5 (0-63)
#define STATE_THEME_MASK      0x40 // Bit 6
#define STATE_ROUTE_ACTIVE    0x80 // Bit 7: 1=Route Active (display), 0=Clear Route

// Packet Structure
struct Packet {
    uint8_t control;
    uint8_t state;
    uint8_t payload[255];
    size_t payloadLen; // Total length of payload buffer used
    
    // Helper to calculate checksum
    uint8_t calculateChecksum() const {
        uint8_t sum = control ^ state;
        for(size_t i=0; i<payloadLen; i++) sum ^= payload[i];
        return sum;
    }
};

// BLE Handler class
class BLEHandler {
public:
    BLEHandler();
    void init();
    void loop();  // Call this in main loop for session timeout check
    
    // Public API: all outgoing BLE data flows through txQueue
    void sendPacket(uint8_t control, uint8_t state, const uint8_t* data, size_t len);
    void sendBulkPacket(uint8_t bulkType, const uint8_t* data, size_t len, bool requestAck = false);
    void processOutgoingQueue(); // Call in loop()
    void notify(const String& message); // Public text notification (routes through txQueue)

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

    // Queued Token Operations (to avoid NVS writes in BLE callback)
    String pendingAddToken = "";
    String pendingTouchToken = "";
    bool needsAddToken = false;
    bool needsTouchToken = false;

    void processTokenQueue(); // Call in loop()
    
    // Callbacks for parsed commands (to be set by main application)
    void (*onCoordinatesReceived)(double lat, double lon) = nullptr;
    void (*onZoomReceived)(float zoom) = nullptr;
    void (*onOTAFirmwareReceived)(String url) = nullptr;
    void (*onOTAMapReceived)(String url) = nullptr;
    void (*onOTABLEProgress)(int percent) = nullptr;  // Called during BLE OTA transfer
    void (*onDeviceConnected)() = nullptr;   // Called when device connects
    void (*onAuthenticated)() = nullptr;     // Called when PIN verified
    void (*onDeviceDisconnected)() = nullptr; // Called when device disconnects
    void (*onZoomChange)(float zoom) = nullptr; // Called when auto-zoom triggers (speed based)
    
    // Public method for parsing received data (called by callbacks)
    void parseReceivedData(const String& data);
    void parseReceivedDataBinary(const std::string& data);  // For binary GPS (handles nulls)
    
    // Watchdog / Activity Management
    void updateActivityTimer(); 
    
    // New Binary Parser
    void parsePacket(const uint8_t* data, size_t len);

    // Public access to boot time for pairing window calculations
    unsigned long bootTime;  // Track when device booted for pairing window
    
    // Silent Auth State (Public for callbacks)
    volatile bool waitingForToken;
    unsigned long authStartTime;
    unsigned long lastPairingHeartbeatTime; // Track heartbeats during pairing
    unsigned long pendingSyncReqTime;       // Non-zero = send SYNC_REQ at this millis() time
    bool wifiOTAStartRequested; // Flag to trigger WiFi OTA from main loop

    // Relative Route Parsing State (Partial Decoding)
    PolylineDecoderState decoderState;


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
    
    // Tx Queue
    enum TxType { TX_BINARY, TX_TEXT };
    struct QueueItem {
        TxType  type;            // TX_BINARY or TX_TEXT
        Packet  packet;          // used when type == TX_BINARY
        char    text[256];       // used when type == TX_TEXT (null-terminated)
        unsigned long timestamp;
        uint8_t retries;
    };
    static const int TX_QUEUE_SIZE = 20;   // increased: text + binary share the queue
    QueueItem txQueue[TX_QUEUE_SIZE];
    int txHead = 0;
    int txTail = 0;
    int txCount = 0;
    bool isWaitingForAck = false;
    unsigned long lastTxTime = 0;

    // Internal send helpers — route through txQueue, do not call BLE directly
    void sendNotification(const String& message);

    // Rx Buffer for Fragmentation
    uint8_t rxBuffer[512];
    size_t rxBufferLen = 0;

    // Authentication state
    volatile bool isAuthenticated;
    String devicePIN;
    String deviceName;
    int authAttempts;
    unsigned long lockoutUntil;
    volatile unsigned long lastActivityTime;
    
    
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
    void handleGPSUpdate(double lat, double lon); // Centralized GPS logic
    
    // Route Hash State
    uint32_t activeRouteHash = 5381;
    void updateRouteHash(const char* data, size_t len);
    
    // Cleaning up private section:
    // (Removed OTA vars from here)
};

// Global BLE handler instance (defined in ble_handler.cpp)
extern BLEHandler bleHandler;

#endif // BLE_HANDLER_H
