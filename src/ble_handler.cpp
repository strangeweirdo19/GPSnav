#include "common.h"
// ble_handler.cpp
#include "ble_handler.h"
#include <WiFi.h>
#include <WebServer.h>

WebServer server(80);
#include <HTTPUpdate.h>
#include <HTTPClient.h>
#include <mbedtls/base64.h>
#include <mbedtls/md5.h>      // For MD5 verification
#include <Update.h>           // For OTA update application
#include <SD_MMC.h>
#include <FS.h>
#include <esp_task_wdt.h>
#include <Preferences.h>
#include <nvs_flash.h>
#include "polyline.h" // Include Polyline decoder

extern TaskHandle_t renderTaskHandle;

// Forward Declaration
void drawOTAProgress(int percent);

Preferences preferences;
String currentClientAddress = "";
uint16_t currentConnHandle = 0xFFFF; // Store current connection handle
String currentSessionToken = ""; // Store current session token for unpairing

// Polyline Buffer
// std::string polylineBuffer; // Removed for partial decoding
static PolylineDecoderState alternateDecoderStates[2]; // For alternate route streaming
static float currentTurnDistMeters = -1.0f; // Distance to next turn (for auto-zoom)

// Global instance
BLEHandler bleHandler;

// Waypoint Persistence for Multi-Stop Recovery
// Note: Waypoint struct is defined in common.h
std::vector<Waypoint> waypointBuffer;

// Initialize static instance pointer
BLEHandler* BLEHandler::instance = nullptr;

// Server callbacks for connection events
class ServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) {
//        Serial.println("");
//        Serial.println("================================================");
//        Serial.println("  BLE CLIENT CONNECTED!");
//        Serial.println("================================================");
        
        // Store current address and connection handle globally
        std::string addrStr = connInfo.getAddress().toString();
        currentClientAddress = String(addrStr.c_str());
        currentConnHandle = connInfo.getConnHandle();
//        Serial.print("DEBUG_CONNECT: Set currentClientAddress = '");
//        Serial.print(currentClientAddress);
//        Serial.print("' (length: ");
//        Serial.print(currentClientAddress.length());
//        Serial.println(")");
        
        // SECURITY: If pairing window closed, only allow whitelisted devices
        if (BLEHandler::instance && !BLEHandler::instance->isWithinPairingWindow()) {
            // Check blacklist
            if (BLEHandler::instance->isBlacklisted(connInfo.getAddress())) {
//                Serial.printf("[SECURITY] Pairing window closed - rejecting blacklisted device: %s\n", 
//                              connInfo.getAddress().toString().c_str());
                pServer->disconnect(connInfo.getConnHandle());
                return;
            }
            
//            Serial.printf("[SECURITY] Pairing window closed - allowing connection attempt from: %s (will verify token)\n",
//                          connInfo.getAddress().toString().c_str());
        }
        
        // ZERO TRUST MODEL: Always accept connection provisionally and wait for Token
//        Serial.println("  STATUS: Connection Accepted (Provisional)");
//        Serial.println("  Auth: Waiting for Token (1s)...");
        
        bleHandler.waitingForToken = true;
        bleHandler.authStartTime = millis();
        
        bleHandler.resetAuthState();
        bleHandler.sendNotification("AUTH_REQ");
        // Do NOT call onDeviceConnected yet (keeps PIN hidden)
    }

    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) {
//        Serial.println("BLE: Client disconnected");
//        Serial.println("DEBUG_DISCONNECT: Clearing currentClientAddress");
        currentClientAddress = "";
        currentConnHandle = 0xFFFF;
        phoneGpsActive = false; // Reset phone GPS status on disconnect
        bleHandler.resetAuthState();
        // Trigger callback to update status
        if (bleHandler.onDeviceDisconnected) {
            bleHandler.onDeviceDisconnected();
        }
        NimBLEDevice::startAdvertising();
//        Serial.println("BLE: Advertising restarted");
    }
};

// RX Characteristic callbacks for receiving data
class RxCallbacks: public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic) {
//        Serial.println("[DEBUG] RX Callback TRIGGERED!");  // First thing - proves callback fires
        
        std::string value = pCharacteristic->getValue();
//        Serial.print("[DEBUG] Raw length: ");
//        Serial.println(value.length());
        
        if (value.length() > 0) {
            // Check for BINARY GPS format: exactly 9 bytes, starts with 'G', might have null bytes
            if (value.length() == 9 && value[0] == 'G') {
                // Pass as raw std::string to handle binary data with embedded nulls
                bleHandler.parseReceivedDataBinary(value);
            } else {
                // Text command - safe to use String
                String receivedData = String(value.c_str());
                bleHandler.parseReceivedData(receivedData);
            }
        } else {
            // Serial.println("BLE RX: WARNING - Empty data received");
        }
    }
};

// Constructor
// Constructor
BLEHandler::BLEHandler() 
    : pServer(nullptr), pTxCharacteristic(nullptr), pRxCharacteristic(nullptr),
      isAuthenticated(false), authAttempts(0), lockoutUntil(0), lastActivityTime(0),
      bootTime(0), waitingForToken(false), authStartTime(0), blacklistCount(0),
      wifiOTAStartRequested(false),
      otaActive(false), otaStartTime(0),
      updateStarted(false), clientConnectedTime(0) {
    // Set static instance pointer for callback access
    instance = this;
}

// Generate unique device name from MAC address
void BLEHandler::generateDeviceName() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    char suffix[5];
    snprintf(suffix, sizeof(suffix), "%02X%02X", mac[4], mac[5]);
    deviceName = "Navion-HUD-" + String(suffix);
}

// Generate random 6-digit PIN
void BLEHandler::generatePIN() {
    devicePIN = "";
    for (int i = 0; i < 3; i++) {
        devicePIN += String(random(0, 10));
    }
//    Serial.println("===========================================");
//    Serial.printf("    BLE PIN: %s\n", devicePIN.c_str());
//    Serial.println("===========================================");
}

// ===== TOKEN BUFFER MANAGEMENT =====
#define MAX_TOKENS 10

bool BLEHandler::addToken(const String& token) {
    preferences.begin("ble-bonds", false);
    int count = preferences.getInt("token_count", 0);
    
    // Check if token already exists
    for (int i = 0; i < count; i++) {
        String key = "token_" + String(i);
        String existing = preferences.getString(key.c_str(), "");
        if (existing == token) {
            // Update timestamp for existing token
            String tsKey = "token_" + String(i) + "_ts";
            preferences.putULong(tsKey.c_str(), millis());
            preferences.end();
//            Serial.println("BLE: Token already exists, updated timestamp");
            return true;
        }
    }
    
    // If buffer full, evict oldest (LRU)
    if (count >= MAX_TOKENS) {
//        Serial.println("BLE: Token buffer full, evicting LRU");
        unsigned long oldestTime = ULONG_MAX;
        int oldestIndex = 0;
        
        for (int i = 0; i < count; i++) {
            String tsKey = "token_" + String(i) + "_ts";
            unsigned long ts = preferences.getULong(tsKey.c_str(), 0);
            if (ts < oldestTime) {
                oldestTime = ts;
                oldestIndex = i;
            }
        }
        
        // Overwrite oldest slot
        String key = "token_" + String(oldestIndex);
        String tsKey = "token_" + String(oldestIndex) + "_ts";
        preferences.putString(key.c_str(), token);
        preferences.putULong(tsKey.c_str(), millis());
    } else {
        // Add new token
        String key = "token_" + String(count);
        String tsKey = "token_" + String(count) + "_ts";
        preferences.putString(key.c_str(), token);
        preferences.putULong(tsKey.c_str(), millis());
        preferences.putInt("token_count", count + 1);
    }
    
    preferences.end();
//    Serial.printf("BLE: Token added to buffer (count: %d)\n", min(count + 1, MAX_TOKENS));
    return true;
}

bool BLEHandler::isValidToken(const String& token) {
    if (token.length() < 15) return false;
    
    preferences.begin("ble-bonds", true);
    int count = preferences.getInt("token_count", 0);
    
    for (int i = 0; i < count; i++) {
        String key = "token_" + String(i);
        String stored = preferences.getString(key.c_str(), "");
        if (stored == token) {
            // Update timestamp
            preferences.end();
            preferences.begin("ble-bonds", false);
            String tsKey = "token_" + String(i) + "_ts";
            preferences.putULong(tsKey.c_str(), millis());
            preferences.end();
            return true;
        }
    }
    
    preferences.end();
    return false;
}

void BLEHandler::removeToken(const String& token) {
    preferences.begin("ble-bonds", false);
    int count = preferences.getInt("token_count", 0);
    
    for (int i = 0; i < count; i++) {
        String key = "token_" + String(i);
        String stored = preferences.getString(key.c_str(), "");
        if (stored == token) {
            // Shift remaining tokens down
            for (int j = i; j < count - 1; j++) {
                String srcKey = "token_" + String(j + 1);
                String srcTsKey = "token_" + String(j + 1) + "_ts";
                String dstKey = "token_" + String(j);
                String dstTsKey = "token_" + String(j) + "_ts";
                
                String tokenVal = preferences.getString(srcKey.c_str(), "");
                unsigned long tsVal = preferences.getULong(srcTsKey.c_str(), 0);
                
                preferences.putString(dstKey.c_str(), tokenVal);
                preferences.putULong(dstTsKey.c_str(), tsVal);
            }
            
            // Remove last slot
            String lastKey = "token_" + String(count - 1);
            String lastTsKey = "token_" + String(count - 1) + "_ts";
            preferences.remove(lastKey.c_str());
            preferences.remove(lastTsKey.c_str());
            
            preferences.putInt("token_count", count - 1);
//            Serial.printf("BLE: Token removed from buffer (count: %d)\n", count - 1);
            break;
        }
    }
    
    preferences.end();
}

// Centralized connection auth decision
void BLEHandler::handleConnectionAuth(const String& token, bool hasToken) {
    if (hasToken && isValidToken(token)) {
        // Valid token -> Authenticate
        isAuthenticated = true;
        authAttempts = 0;
        waitingForToken = false;
        authStartTime = 0;
        currentSessionToken = token; // Track active token
//        Serial.println("BLE: Token Valid - Authenticated!");
        sendNotification("AUTH_OK");
        // Report GPS status
        sendNotification(gpsModulePresent ? "GPS_STATUS: 1" : "GPS_STATUS: 0");
        // Report route status for sync
        char statusMsg[64];
        // Format: ROUTE_STATUS,state,count,hash,selected,waypointCount
        snprintf(statusMsg, sizeof(statusMsg), "ROUTE_STATUS,%d,%d,%08X,%d,%d", 
                currentRouteState, routePointCount, routeHash, selectedRouteIndex, (int)waypointBuffer.size());
        sendNotification(statusMsg);
        if (onAuthenticated) onAuthenticated();
    } else if (hasToken && !isValidToken(token)) {
        // Invalid token -> Report to app and show PIN or disconnect
//        Serial.println("BLE: Invalid Token Detected");
        String tokenPrefix = token.substring(0, min(15, (int)token.length()));
        sendNotification("TOKEN_INVALID " + tokenPrefix);
        waitingForToken = false;
        
        if (isWithinPairingWindow()) {
//            Serial.println("BLE: Window Open -> Showing PIN");
            sendNotification("AUTH_FAIL");
            if (onDeviceConnected) onDeviceConnected();
        } else {
//            Serial.println("BLE: Window Closed -> Disconnecting");
            sendNotification("PAIRING_CLOSED");
            if (currentConnHandle != 0xFFFF && pServer) {
                pServer->disconnect(currentConnHandle);
            }
        }
    } else {
        // No token provided
        waitingForToken = false;
        
        // BIDIRECTIONAL SYNC: Check if we have saved tokens
        // If yes, it means the app unpaired us, so we should forget them too
        preferences.begin("ble-bonds", false);
        int tokenCount = preferences.getInt("token_count", 0);
        
        if (tokenCount > 0) {
//            Serial.printf("[SYNC] App sent AUTH_NONE but we have %d tokens. App likely unpaired us.\n", tokenCount);
//            Serial.println("[SYNC] Clearing our token buffer to stay in sync with app");
            
            // Clear the entire token buffer
            preferences.clear();
//            Serial.println("[SYNC] Token buffer cleared - both sides now unpaired");
        }
        preferences.end();
        
        if (isWithinPairingWindow()) {
//            Serial.println("BLE: No Token, Window Open -> Showing PIN");
            if (onDeviceConnected) onDeviceConnected();
        } else {
//            Serial.println("BLE: No Token, Window Closed -> Disconnecting");
            
            // Add to blacklist before disconnecting
            if (pServer && pServer->getConnectedCount() > 0) {
                NimBLEConnInfo connInfo = pServer->getPeerInfo(0);
                addToBlacklist(connInfo.getAddress());
            }
            
            sendNotification("PAIRING_CLOSED");
//            Serial.println("[PAIRING] Window CLOSED - only bonded devices can connect");
            if (currentConnHandle != 0xFFFF && pServer) {
                pServer->disconnect(currentConnHandle);
            }
        }
    }
}

// ===== BLACKLIST MANAGEMENT =====

void BLEHandler::addToBlacklist(NimBLEAddress addr) {
    // Check if already blacklisted
    for (int i = 0; i < blacklistCount; i++) {
        if (blacklistedDevices[i] == addr) {
//            Serial.println("[BLACKLIST] Device already blacklisted");
            return;
        }
    }
    
    // Add to blacklist (circular buffer)
    if (blacklistCount < MAX_BLACKLISTED) {
        blacklistedDevices[blacklistCount++] = addr;
    } else {
        // Overwrite oldest (index 0), shift array
        for (int i = 0; i < MAX_BLACKLISTED - 1; i++) {
            blacklistedDevices[i] = blacklistedDevices[i + 1];
        }
        blacklistedDevices[MAX_BLACKLISTED - 1] = addr;
    }
    
//    Serial.printf("[BLACKLIST] Added device: %s (count: %d)\n", 
//                  addr.toString().c_str(), blacklistCount);
}

bool BLEHandler::isBlacklisted(NimBLEAddress addr) {
    for (int i = 0; i < blacklistCount; i++) {
        if (blacklistedDevices[i] == addr) {
            return true;
        }
    }
    return false;
}

// Get device name
String BLEHandler::getDeviceName() {
    return deviceName;
}

// Get current PIN
String BLEHandler::getPIN() {
    return devicePIN;
}

// Validate PIN
bool BLEHandler::validatePIN(const String& pin) {
    return pin == devicePIN;
}

// Reset authentication state
void BLEHandler::resetAuthState() {
    isAuthenticated = false;
    authAttempts = 0;
    lastActivityTime = millis();
    waitingForToken = false;
    authStartTime = 0;
    currentSessionToken = ""; // Clear session token
    lastPairingHeartbeatTime = millis(); // Reset heartbeat on state reset
}

// Check session timeout
bool BLEHandler::checkSessionTimeout() {
    if (isAuthenticated && (millis() - lastActivityTime > SESSION_TIMEOUT_MS)) {
//        Serial.println("BLE: Session timeout (10s) - Disconnecting client");
        isAuthenticated = false;
        sendNotification("SESSION_TIMEOUT");
        
        // Force disconnect as requested
        if (currentConnHandle != 0xFFFF && pServer) {
            pServer->disconnect(currentConnHandle);
        }
        return true;
    }
    return false;
}

// Check if within pairing window (first 60 seconds after boot)
bool BLEHandler::isWithinPairingWindow() {
    return (millis() - bootTime) < PAIRING_WINDOW_MS;
}

// Get count of bonded devices
int BLEHandler::getBondedDeviceCount() {
    return NimBLEDevice::getNumBonds();
}

// Get list of bonded devices as JSON array
String BLEHandler::getBondedDevicesList() {
    int bondCount = NimBLEDevice::getNumBonds();
    
    if (bondCount == 0) {
        return "[]";
    }
    
    // Build JSON array by iterating through bonded addresses
    String json = "[";
    int addedCount = 0;
    
    // NimBLE doesn't have getBondedDevices, so we report count only
    // or iterate through peer info if available
    for (int i = 0; i < bondCount; i++) {
        if (addedCount > 0) json += ",";
        // Note: NimBLE doesn't expose easy access to bonded addresses list
        // We'll return simplified info for now
        json += "{\"index\":" + String(i) + "}";
        addedCount++;
    }
    json += "]";
    
    return json;
}

// Clear all bonding data
bool BLEHandler::clearAllBonds() {
//    Serial.println("BLE: Clearing all bonding data");
    NimBLEDevice::deleteAllBonds();
    
    // Clear custom whitelist
    preferences.begin("ble-bonds", false);
    preferences.clear();
    preferences.end();
    
//    Serial.println("BLE: All bonds cleared successfully");
    return true;
}

// Remove specific bonded device by address
bool BLEHandler::removeBond(const String& address) {
//    Serial.printf("BLE: Removing bond for device: %s\n", address.c_str());
    NimBLEAddress addr(address.c_str());
    int result = NimBLEDevice::deleteBond(addr);
    
    // Always remove from custom whitelist regardless of native bond result
    String macKey = address;
    macKey.replace(":", "");
    preferences.begin("ble-bonds", false);
    preferences.remove(macKey.c_str());
    preferences.end();
    
    if (result == 0 || true) { // Assume success if we cleared custom
//        Serial.println("BLE: Bond removed successfully");
        return true;
    } else {
        Serial.printf("BLE: Failed to remove bond, error: %d\n", result);
        return false;
    }
}

// Validate input to prevent buffer overflow and malicious data
bool BLEHandler::validateInput(const String& data) {
    if (data.length() > 512) {
        Serial.println("BLE: Input validation failed - too long");
        return false;
    }
    // Check for null bytes
    if (data.indexOf('\0') >= 0 && data.indexOf('\0') < data.length() - 1) {
        Serial.println("BLE: Input validation failed - null byte detected");
        return false;
    }
    return true;
}

// Initialize BLE
void BLEHandler::init() {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Debug: Dump stored tokens
    preferences.begin("ble-bonds", true); // Read-only
    int count = preferences.getInt("token_count", 0);
//    Serial.printf("BLE: Init - Found %d stored tokens in NVS\n", count);
    for(int i=0; i<count; i++) {
        String key = "token_" + String(i);
        String t = preferences.getString(key.c_str(), "MISSING");
//        Serial.printf("  Token[%d]: %s...\n", i, t.substring(0, 5).c_str());
    }
    preferences.end();

//    Serial.println("BLE: Initializing NimBLE...");
    
    // Record boot time for pairing window
    bootTime = millis();
    
    // Generate unique device name and PIN
    generateDeviceName();
    generatePIN();
    
    // Create the BLE Device
    NimBLEDevice::init(deviceName.c_str());
    NimBLEDevice::setMTU(512);

    // NUCLEAR CLEANUP: Delete all existing bonds to prevent legacy security
    NimBLEDevice::deleteAllBonds();
    
    // Security: Disable bonding and secure connections to prevent OS pairing dialogs
    // We are handling our own custom application-layer authentication
    NimBLEDevice::setSecurityAuth(false, false, false); 
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT); 
    // NUCLEAR OPTION for iOS: Explicitly say "We have NO keys to exchange"
    // This stops the phone from even asking "Would you like to pair?"
    NimBLEDevice::setSecurityInitKey(0);
    NimBLEDevice::setSecurityRespKey(0); 
    
//    Serial.println("BLE: Pairing window active for 60 seconds");
//    Serial.println("");
//    Serial.println("================================================");
    Serial.print("  PAIRING WINDOW: ");
    Serial.print(PAIRING_WINDOW_MS / 1000);
    Serial.println(" seconds from boot");
    Serial.println("  New devices can only pair during this time!");
//    Serial.println("================================================");
//    Serial.println("");
    
    // Create the BLE Server
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    
    // Create the BLE Service
    NimBLEService* pService = pServer->createService(SERVICE_UUID);
    
    // Create the TX Characteristic (ESP32 -> App, Notify)
    pTxCharacteristic = pService->createCharacteristic(
        TX_CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::INDICATE
    );
    
    // Create the RX Characteristic (App -> ESP32, Write)
    pRxCharacteristic = pService->createCharacteristic(
        RX_CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    pRxCharacteristic->setCallbacks(new RxCallbacks());
    
    // Start the service
    pService->start();
    
    // Start advertising
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMaxPreferred(0x12);
    
    // Start advertising with Pairing Open status
    updateAdvertising(true);
//    Serial.printf("BLE: Service started. Device name: %s\n", deviceName.c_str());
//    Serial.println("BLE: Waiting for client connection...");
}

// Update advertising data with pairing status
void BLEHandler::updateAdvertising(bool pairingOpen) {
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->stop(); // Stop before updating
    
    // Set Service Data: 0x01 = Pairing Open, 0x00 = Pairing Closed
    std::string serviceData;
    serviceData.push_back(pairingOpen ? 0x01 : 0x00);
    
    // We must set service data for our specific service UUID
    pAdvertising->setServiceData(NimBLEUUID(SERVICE_UUID), serviceData);
    
    pAdvertising->start();
//    Serial.printf("BLE: Advertising updated. Pairing Mode: %s\n", pairingOpen ? "OPEN" : "CLOSED");
}

// Loop function - call from main loop
void BLEHandler::loop() {
    // Feed watchdog at start of every loop iteration to prevent timeout
    esp_task_wdt_reset();
    
    server.handleClient();
    
    // Relative Route Parsing State
    bool isRouteStart;
    double lastLat;
    double lastLon;
    
    if (wifiOTAStartRequested) {
        wifiOTAStartRequested = false;
        handleOTA_WiFi_Start();
    }
    
    // Check OTA Timeout
    checkOTATimeout();
    
    static unsigned long lastStatusCheck = 0;
    static bool wasConnected = false;
    static bool wasPairingOpen = true; // Track previous state
    
    // Check for silent auth timeout
    if (waitingForToken) {
        if (millis() - authStartTime > 1000) { // 1 second timeout
//            Serial.println("BLE: Silent Auth Timeout - Using centralized logic");
            handleConnectionAuth("", false);
        }
    }
    
    // Check session timeout
    checkSessionTimeout();

    // Check Pairing Heartbeat Timeout (Only when waiting for token/PIN visible)
    // Check Pairing Heartbeat Timeout - REMOVED
    // We now rely on connection events or explicit cancellations
    /* 
    if (waitingForToken && millis() - lastPairingHeartbeatTime > PAIRING_HEARTBEAT_TIMEOUT_MS) {
        sendNotification("PAIRING_CLOSED");
        if (currentConnHandle != 0xFFFF && pServer) {
            pServer->disconnect(currentConnHandle);
        }
        waitingForToken = false;
    } 
    */
    
    // Check pairing window transition (approximate)
    bool isPairingOpen = isWithinPairingWindow();
    if (isPairingOpen != wasPairingOpen) {
        // State changed!
        updateAdvertising(isPairingOpen);
        wasPairingOpen = isPairingOpen;
    }
    
    // Check connection status frequently (every 500ms) for instant feedback
    if (millis() - lastStatusCheck >= 500) {
        lastStatusCheck = millis();

        bool connected = isConnected();
        
        // Report connection status change
        if (connected != wasConnected) {
            if (connected) {
//                Serial.println("\n[STATUS] BLE Link Established (Auth Required)");
                // Do NOT trigger callbacks here - let handshake complete first
            } else {
//                Serial.println("\n[STATUS] BLE device is DISCONNECTED - waiting for connection...");
                if (onDeviceDisconnected) {
                     onDeviceDisconnected();
                }
            }
            wasConnected = connected;
        }
        
        // Report pairing window status if not connected
        if (!connected) {
            unsigned long elapsed = (millis() - bootTime) / 1000;
            if (isWithinPairingWindow()) {
//                Serial.print("[PAIRING] Window OPEN - ");
                Serial.print((PAIRING_WINDOW_MS / 1000) - elapsed);
                Serial.println(" seconds remaining for new devices");
            } else {
//                Serial.println("[PAIRING] Window CLOSED - only bonded devices can connect");
            }
        }
        
        lastStatusCheck = millis();
    }
}

// Check if a device is connected
bool BLEHandler::isConnected() {
    return pServer->getConnectedCount() > 0;
}

// Check if device is authenticated
bool BLEHandler::isDeviceAuthenticated() {
    return isAuthenticated;
}

// Base64 encode function using mbedtls
String BLEHandler::base64Encode(const String& input) {
    size_t outputLen = 0;
    
    int ret = mbedtls_base64_encode(nullptr, 0, &outputLen, 
                                     (const unsigned char*)input.c_str(), 
                                     input.length());
    
    if (ret == MBEDTLS_ERR_BASE64_BUFFER_TOO_SMALL) {
        unsigned char* buffer = new unsigned char[outputLen];
        
        ret = mbedtls_base64_encode(buffer, outputLen, &outputLen,
                                    (const unsigned char*)input.c_str(),
                                    input.length());
        
        if (ret == 0) {
            String result = String((char*)buffer);
            delete[] buffer;
            return result;
        }
        delete[] buffer;
    }
    
    Serial.println("BLE: Base64 encoding failed");
    return "";
}

// Send notification to connected client (Plain Text)
void BLEHandler::sendNotification(const String& message) {
    if (isConnected()) {
        // Send plain text. Mobile app handles decoding from BLE bytes.
        // Prevent double-encoding or malloc issues.
        pTxCharacteristic->setValue((const uint8_t*)message.c_str(), message.length());
        pTxCharacteristic->notify();
        // Serial.printf("BLE TX: Sent: %s\n", message.c_str());
    } else {
        Serial.println("BLE TX: No device connected");
    }
}

// Parse binary GPS data (9 bytes: 'G' + lat_int32_le + lon_int32_le)
// This handles binary data with embedded null bytes that would break String
void BLEHandler::parseReceivedDataBinary(const std::string& data) {
    if (data.length() != 9 || data[0] != 'G') {
        sendNotification("E");
        return;
    }
    
    // Check authentication
    if (!isAuthenticated) {
        sendNotification("AUTH_REQUIRED");
        return;
    }
    
    // Parse binary GPS: 'G' + lat(4 bytes int32 LE) + lon(4 bytes int32 LE)
    const uint8_t* raw = (const uint8_t*)data.data();
    int32_t latInt = (int32_t)(raw[1] | (raw[2] << 8) | (raw[3] << 16) | (raw[4] << 24));
    int32_t lonInt = (int32_t)(raw[5] | (raw[6] << 8) | (raw[7] << 16) | (raw[8] << 24));
    double lat = latInt / 1000000.0;
    double lon = lonInt / 1000000.0;
    
    // Validate range
    if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
        sendNotification("E");
        return;
    }
    
    phoneGpsActive = true;
    lastPhoneCommandTime = millis();
    lastActivityTime = millis();
    
    // GPS Noise Filtering
    static double pendingLat = 0.0, pendingLon = 0.0;
    static int pendingConfirmCount = 0;
    static const float MAX_SPEED_MS = 111.0f;     // ~400 km/h max
    
    bool acceptPoint = true;
    
    if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        unsigned long now = millis();
        float dt = (now - gpsState.timestamp) / 1000.0f;
        
        if (dt > 0.01f && gpsState.timestamp > 0) {
            double dLat = lat - gpsState.lat;
            double dLon = lon - gpsState.lon;
            float distance = sqrt(dLat * dLat + dLon * dLon) * 111320.0f;
            float speed = distance / dt;
            
            if (speed > MAX_SPEED_MS) {
                // Speed too high - require confirmation
                if (pendingConfirmCount > 0) {
                    float pendingDist = sqrt(pow(lat - pendingLat, 2) + pow(lon - pendingLon, 2)) * 111320.0f;
                    if (pendingDist < 50.0f) {
                        pendingConfirmCount++;
                        if (pendingConfirmCount >= 2) {
                            pendingConfirmCount = 0;  // Confirmed
                        } else {
                            acceptPoint = false;
                        }
                    } else {
                        pendingLat = lat; pendingLon = lon;
                        pendingConfirmCount = 1;
                        acceptPoint = false;
                    }
                } else {
                    pendingLat = lat; pendingLon = lon;
                    pendingConfirmCount = 1;
                    acceptPoint = false;
                }
            } else {
                pendingConfirmCount = 0;
            }
            
            if (acceptPoint) {
                gpsState.speed = speed;
                gpsState.heading = atan2(dLon, dLat) * 180.0f / 3.14159f;
                if (gpsState.heading < 0) gpsState.heading += 360.0f;
                
                float instantRate = 1.0f / dt;
                gpsRate = (gpsRate == 0.0f) ? instantRate : gpsRate * 0.7f + instantRate * 0.3f;
            }
        }
        
        if (acceptPoint) {
            gpsState.lat = lat;
            gpsState.lon = lon;
            gpsState.timestamp = now;
        }
        xSemaphoreGive(gpsMutex);
    }
    
    // Arrival Detection (Check if near current target waypoint)
    if (acceptPoint) {
        if (xSemaphoreTake(routeMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            if (!waypointBuffer.empty() && currentWaypointIndex < waypointBuffer.size()) {
                double wpLat = waypointBuffer[currentWaypointIndex].lat;
                double wpLon = waypointBuffer[currentWaypointIndex].lon;
                
                // Approx distance in meters
                double dLat = (lat - wpLat) * 111320.0;
                double dLon = (lon - wpLon) * 111320.0 * cos(lat * 3.14159/180.0);
                double dist = sqrt(dLat*dLat + dLon*dLon);
                
                // Threshold: 25 meters
                if (dist < WAYPOINT_ARRIVAL_THRESHOLD_M) {
                    Serial.printf("Waypoint Arrived: index %d (dist: %.1fm)\n", currentWaypointIndex, dist);
                    
                    // Advance waypoint index
                    currentWaypointIndex++;
                    
                    // Update route coloring: make remaining route blue (single color)
                    routeLegSwitchIndex = 2147483647;
                    
                    // Notify App
                    sendNotification("ARRIVAL");
                    
                    // Remove reached waypoint from buffer (for backward compatibility)
                    waypointBuffer.erase(waypointBuffer.begin());
                    
                    // Since we erased, decrement currentWaypointIndex to stay aligned
                    if (currentWaypointIndex > 0) currentWaypointIndex--;
                }
            }
            xSemaphoreGive(routeMutex);
        }
    }
    
    // ---------------------------------------------------------
    // ROUTE TRIMMING (Progress Tracking)
    // Find closest point on route to current position to hide passed segments
    // ---------------------------------------------------------
    if (acceptPoint) {
        if (xSemaphoreTake(routeMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (!activeRoute.empty() && routeProgressIndex < activeRoute.size()) {
                // Initialize tracker if at start
                if (routeProgressIndex == 0 && currentRouteLat == 0.0) {
                    currentRouteLat = activeRouteAnchor.lat + (activeRoute[0].dLat / 100000.0);
                    currentRouteLon = activeRouteAnchor.lon + (activeRoute[0].dLon / 100000.0);
                }
                
                // Search window: check 50 points ahead of current progress
                int searchRadius = 50;
                int bestK = 0;
                double minDSq = (lat - currentRouteLat)*(lat - currentRouteLat) + (lon - currentRouteLon)*(lon - currentRouteLon);
                
                double traceLat = currentRouteLat;
                double traceLon = currentRouteLon;
                
                for (int k = 1; k < searchRadius; k++) {
                    int idx = routeProgressIndex + k;
                    if (idx >= activeRoute.size()) break;
                    
                    traceLat += activeRoute[idx].dLat / 100000.0;
                    traceLon += activeRoute[idx].dLon / 100000.0;
                    
                    double dSq = (lat - traceLat)*(lat - traceLat) + (lon - traceLon)*(lon - traceLon);
                    if (dSq < minDSq) {
                        minDSq = dSq;
                        bestK = k;
                    }
                }
                
                // Advance progress if we found a closer point ahead
                if (bestK > 0) {
                    // Update global tracker to the new best position
                    for (int k = 1; k <= bestK; k++) {
                        int idx = routeProgressIndex + k;
                        currentRouteLat += activeRoute[idx].dLat / 100000.0;
                        currentRouteLon += activeRoute[idx].dLon / 100000.0;
                    }
                    routeProgressIndex += bestK;
                }
            }
            xSemaphoreGive(routeMutex);
        }
    }
    
    if (acceptPoint && onCoordinatesReceived) {
        onCoordinatesReceived(lat, lon);
    }
}

// Parse received data and route to appropriate handlers
void BLEHandler::parseReceivedData(const String& data) {
    // Validate input
    if (!validateInput(data)) {
        Serial.println("BLE: Input validation FAILED");
        sendNotification("ERROR_INVALID_INPUT");
        return;
    }
    
    String trimmedData = data;
    trimmedData.trim();
    
//    Serial.print("BLE: Parsing command: '");
    Serial.print(trimmedData);
    Serial.println("'");
    
    // Update activity timestamp
    lastActivityTime = millis();
    
    // Check if locked out
    if (millis() < lockoutUntil) {
//        Serial.println("BLE: Device is LOCKED OUT");
        sendNotification("AUTH_LOCKED");
        return;
    }
    
    // Handle PIN authentication
    if (trimmedData.startsWith("PIN ")) {
        String pin = trimmedData.substring(4);
        pin.trim();
        
        // CRITICAL: Check pairing window before accepting PIN
        if (!isWithinPairingWindow()) {
//            Serial.println("BLE: PIN attempt REJECTED - Pairing window CLOSED");
            sendNotification("PAIRING_CLOSED");
            // Disconnect immediately
            if (currentConnHandle != 0xFFFF && pServer) {
                pServer->disconnect(currentConnHandle);
            }
            return;
        }
        
        if (validatePIN(pin)) {
            isAuthenticated = true;
            authAttempts = 0;
//            Serial.println("");
//            Serial.println("================================================");
            Serial.println("  ✓ AUTHENTICATION SUCCESSFUL");
            Serial.println("  Device is now authenticated and can send commands");
//            Serial.println("================================================");
//            Serial.println("");
            sendNotification("AUTH_OK");
            vTaskDelay(pdMS_TO_TICKS(20)); // Reduced delay (Optimized)
            
            // Report GPS status
            sendNotification(gpsModulePresent ? "GPS_STATUS: 1" : "GPS_STATUS: 0");
            vTaskDelay(pdMS_TO_TICKS(20)); // Reduced delay (Optimized)
            
            // DEBUG: Check client address
            Serial.print("  DEBUG: currentClientAddress = '");
            Serial.print(currentClientAddress);
            Serial.print("' (length: ");
            Serial.print(currentClientAddress.length());
            Serial.println(")");
            
            // Generate and Save Auth Token (Rolling Buffer)
            // NOTE: Always generate token regardless of address (address might be empty due to timing)
            
            // Generate simple random token
            String token = "";
            for(int i=0; i<16; i++) {
                token += String("0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz")[random(0, 62)];
            }
            
            // Add to rolling buffer (max 10, LRU eviction)
            addToken(token);
            
            // Track active token for unpairing
            currentSessionToken = token;
            
            Serial.println("  TRUSTED: Token added to buffer");
            
            // Send Token to Client (Use Indication for reliability)
            pTxCharacteristic->setValue("TOKEN " + token);
            pTxCharacteristic->notify(true); // true = Indication (Ack required)
            Serial.println("BLE TX: Sent: TOKEN " + token); // Manual Log since we bypassed sendNotification
            vTaskDelay(pdMS_TO_TICKS(10)); // Reduced (Optimized)

            // Trigger callback to clear PIN display
            if (onAuthenticated) {
                onAuthenticated();
            }
            return;
        } else {
            authAttempts++;
//            Serial.println("");
//            Serial.println("================================================");
            Serial.println("  ✗ AUTHENTICATION FAILED");
            Serial.printf("  Wrong PIN! Attempt %d of %d\n", authAttempts, MAX_AUTH_ATTEMPTS);
            Serial.print("  Correct PIN: ");
            Serial.println(getPIN());
//            Serial.println("================================================");
//            Serial.println("");
            
            if (authAttempts >= MAX_AUTH_ATTEMPTS) {
                lockoutUntil = millis() + AUTH_LOCKOUT_TIME_MS;
//                Serial.println("BLE: Max auth attempts reached - locking out");
                sendNotification("AUTH_LOCKED");
                // Disconnect the client - DISABLED to allow UI feedback
                // pServer->disconnect(pServer->getPeerInfo(0).getConnHandle());
            } else {
                sendNotification("AUTH_FAIL");
            }
        }
        return;
    }

    // Handle Token Authentication - Use centralized logic
    if (trimmedData.startsWith("AUTH_TOKEN ")) {
        String token = trimmedData.substring(11);
        token.trim();
        handleConnectionAuth(token, true);
        return;
    }

    // Handle AUTH_NONE (Client has no token) - Use centralized logic
    if (trimmedData == "AUTH_NONE") {
//         Serial.println("BLE: Client sent AUTH_NONE");
         handleConnectionAuth("", false);
         return;
    }
    
    // Check for FW_MD5 (Allowed without Auth)
    if (trimmedData == "FW_MD5") {
//        Serial.println("");
//        Serial.println("================================================");
        Serial.println("FW_MD5: Command received");
        String md5 = ESP.getSketchMD5();
        Serial.print("FW_MD5: Current firmware MD5: ");
        Serial.println(md5);
        Serial.print("FW_MD5: Sending response... ");
        sendNotification("FW_MD5 " + md5);
        Serial.println("OK");
//        Serial.println("================================================");
//        Serial.println("");
        return;
    }

    // Check for OTA Firmware command (Allowed without Auth for specific update flow)
    if (trimmedData.startsWith("OTA_FW ")) {
        String url = trimmedData.substring(7);
        url.trim();
        
        // Validate URL
        if (!url.startsWith("http://") && !url.startsWith("https://")) {
//            Serial.println("BLE: Invalid OTA URL protocol");
            sendNotification("ERROR_INVALID_URL");
            return;
        }
        
//        Serial.printf("BLE: OTA Firmware request: %s\n", url.c_str());
        handleOTAFirmware(url);
        if (onOTAFirmwareReceived) {
            onOTAFirmwareReceived(url);
        }
        return;
    }
    
    // =========================================================
    // OTA VIA BLE (No WiFi required) - Allowed without Auth
    // =========================================================
    
    // OTA WiFi Start
    if (trimmedData == "OTA_WIFI_START") {
        wifiOTAStartRequested = true;
        return;
    }
    

    
    // All other commands require authentication
    if (!isAuthenticated) {
//        Serial.println("BLE: Command rejected - not authenticated");
        sendNotification("AUTH_REQ");
        return;
    }
    
    // Check for OTA Map command
    if (trimmedData.startsWith("OTA_MAP ")) {
        String url = trimmedData.substring(8);
        url.trim();
        
        // Validate URL
        if (!url.startsWith("http://") && !url.startsWith("https://")) {
//            Serial.println("BLE: Invalid OTA URL protocol");
            sendNotification("ERROR_INVALID_URL");
            return;
        }
        
//        Serial.printf("BLE: OTA Map request: %s\n", url.c_str());
        handleOTAMap(url);
        if (onOTAMapReceived) {
            onOTAMapReceived(url);
        }
        return;
    }
    
    // Check for ZOOM command
    if (trimmedData.startsWith("ZOOM ")) {
        String zoomStr = trimmedData.substring(5);
        zoomStr.trim();
        float zoomValue = zoomStr.toFloat();
        
        if (zoomValue > 0.0) {
//            Serial.printf("BLE: Received zoom level: %.2f\n", zoomValue);
            if (onZoomReceived) {
                onZoomReceived(zoomValue);
            }
        } else {
            sendNotification("ERROR_INVALID_ZOOM");
        }
        return;
    }

    if (trimmedData == "UNPAIR") {
//        Serial.println("BLE: Client requested unpair (forget me)");
        
        // Remove the specific token used for this session
        if (currentSessionToken.length() > 0) {
            removeToken(currentSessionToken);
//            Serial.println("BLE: Removed session token from buffer");
            currentSessionToken = "";
        }
        
        // Also remove legacy bond checks if any
        if (currentClientAddress.length() > 0) {
            removeBond(currentClientAddress);
        }
        
        sendNotification("UNPAIRED");
        delay(500); // Wait for notification to send
        
        // Disconnect to enforce removal
        if (currentConnHandle != 0xFFFF && pServer) {
             pServer->disconnect(currentConnHandle);
        }
        return;
    }
    
    // Bonding management commands
    if (trimmedData == "BONDS_LIST") {
//        Serial.println("BLE: Listing bonded devices");
        String bondsList = getBondedDevicesList();
        sendNotification("BONDS_LIST " + bondsList);
        return;
    }
    
    if (trimmedData == "BONDS_COUNT") {
        int count = getBondedDeviceCount();
//        Serial.printf("BLE: Bonded device count: %d\n", count);
        sendNotification("BONDS_COUNT " + String(count));
        return;
    }


    
    if (trimmedData == "BONDS_CLEAR") {
//        Serial.println("BLE: Clear all bonds requested");
        if (clearAllBonds()) {
            sendNotification("BONDS_CLEARED");
        } else {
            sendNotification("BONDS_CLEAR_FAILED");
        }
        return;
    }
    
    if (trimmedData.startsWith("BOND_REMOVE ")) {
        String address = trimmedData.substring(12);
        address.trim();
//        Serial.printf("BLE: Remove bond requested for: %s\n", address.c_str());
        if (removeBond(address)) {
            sendNotification("BOND_REMOVED");
        } else {
            sendNotification("BOND_REMOVE_FAILED");
        }
        return;
    }
    
    if (trimmedData.startsWith("CMD_RESET")) {
//        Serial.println("BLE: Reset command received");
        sendNotification("RESET_OK");
        delay(100);  // Give time for notification to send
        ESP.restart();
        return;
    }

    // =========================================================
    // ROUTE OVERLAY COMMANDS (POLYLINE ENCODED)
    // =========================================================

    if (trimmedData.startsWith("ROUTE_START_POLYLINE")) {
        Serial.println("BLE: Start Polyline Route (Partial Decoding)");
        
        // Parse Total Size if available (Format: ROUTE_START_POLYLINE:12345)
        routeTotalBytes = 0;
        int colonIdx = trimmedData.indexOf(':');
        if (colonIdx > 0) {
            routeTotalBytes = trimmedData.substring(colonIdx + 1).toInt();
            Serial.printf("BLE: Expecting %d bytes\n", routeTotalBytes);
        }
        
        // Reset Decoder State
        decoderState.reset();
        
        // Clear existing route
        if (xSemaphoreTake(routeMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            activeRoute.clear();
            activeRouteAnchor = {0.0, 0.0}; // Reset anchor to prevent stale destination marker
            
            // Clear alternates on new route
            alternateRoutes[0].clear();
            alternateRoutes[1].clear();
            alternateAnchors[0] = {0.0, 0.0};
            alternateAnchors[1] = {0.0, 0.0};
            // waypointBuffer.clear(); // REMOVED: Keep pre-synced waypoints
            selectedRouteIndex = -1;
            
            routeProgressIndex = 0;
            routeLegSwitchIndex = 2147483647; // Reset to default (single-color route)
            currentWaypointIndex = 0; // Reset to first waypoint
            currentRouteLat = 0.0;
            currentRouteLon = 0.0;
            if (routeTotalBytes > 0) {
                // Approximate 5 bytes per point? Conservative reserve.
                activeRoute.reserve(routeTotalBytes / 4);
            }
            routeAvailable = false;
            xSemaphoreGive(routeMutex);
        }
        isRouteSyncing = true; // Start overlay
        routeSyncProgressBytes = 0; // Reset progress
        xTaskNotify(renderTaskHandle, 0x01, eSetBits); // Force screen update
        sendNotification("A_POLY_START");
        return;
    }

    if (trimmedData == "ROUTE_CANCEL") {
        Serial.println("BLE: Route Cancelled by App");
        
        // Clear Route
        if (xSemaphoreTake(routeMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            activeRoute.clear();
            routeAvailable = false;
            decoderState.reset();
            xSemaphoreGive(routeMutex);
        }
        
        // Stop UI Overlay
        isRouteSyncing = false; 
        routeSyncProgressBytes = 0;
        
        // Force Render Update
        xTaskNotify(renderTaskHandle, 0x01, eSetBits);
        
        sendNotification("A_CANCEL_OK");
        return;
    }


    if (trimmedData.startsWith("ROUTE_APPEND_POLYLINE,")) {
        // ROUTE_APPEND_POLYLINE,<string>
        const char* p = trimmedData.c_str() + 22; // Skip prefix
        size_t len = strlen(p);
        routeSyncProgressBytes += len; // Update progress
        
        // Decode Chunk immediately
        if (xSemaphoreTake(routeMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
             decodePolylineChunk(p, activeRoute, decoderState);
             
             // Index IMMEDIATELY for real-time rendering
             // Note: indexRouteLocked is fast enough (O(N)) for moderate routes.
             // If latencies increase, we might throttle this (e.g. index every 5th chunk).
             indexRouteLocked();
             
             xSemaphoreGive(routeMutex);
        }
        
        // Send ACK with current decoded count to confirm receipt/progress
        sendNotification("A_POLY_APPEND " + String(routeSyncProgressBytes));
        // Force update to show progress
        xTaskNotify(renderTaskHandle, 0x01, eSetBits);
        return;
    }

    if (trimmedData == "ROUTE_END_POLYLINE") {
        Serial.printf("BLE: End. Total Points: %d\n", activeRoute.size());
        
        if (xSemaphoreTake(routeMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
             if (!decoderState.remainder.empty()) {
                 Serial.println("BLE: Warning - Remainder bytes ignored at end of stream.");
             }
             
             Serial.printf("BLE: Polyline Decoded Successfully! Points: %d\n", activeRoute.size());
             routeAvailable = true;
             
             // Trigger Indexing (Final)
             indexRouteLocked(); 
             
             // Trigger Render
             xTaskNotify(renderTaskHandle, 0x01, eSetBits);
             
             sendNotification("A_POLY_END Success," + String(activeRoute.size()));
             xSemaphoreGive(routeMutex);
        } else {
             sendNotification("E_ROUTE_MUTEX");
        }
        
        isRouteSyncing = false; // End overlay
        xTaskNotify(renderTaskHandle, 0x01, eSetBits); // Force screen update
        
        return;
    }

    // =========================================================
    // ALTERNATE ROUTE COMMANDS (for auto-selection feature)
    // =========================================================
    
    // ALT_START:idx,len - Start alternate route sync
    if (trimmedData.startsWith("ALT_START:")) {
        String params = trimmedData.substring(10);
        int commaIdx = params.indexOf(',');
        if (commaIdx > 0) {
            int idx = params.substring(0, commaIdx).toInt();
            int totalLen = params.substring(commaIdx + 1).toInt();
            
            if (idx >= 0 && idx <= 1) {
                Serial.printf("BLE: Start Alternate Route %d (len=%d)\n", idx, totalLen);
                
                // Reset decoder state for this alternate
                alternateDecoderStates[idx].reset();
                
                // Clear existing alternate route
                if (xSemaphoreTake(routeMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    alternateRoutes[idx].clear();
                    if (totalLen > 0) {
                        alternateRoutes[idx].reserve(totalLen / 4);
                    }
                    xSemaphoreGive(routeMutex);
                }
                
                sendNotification("A_ALT_START:" + String(idx));
                return;
            }
        }
        sendNotification("E_ALT_START");
        return;
    }
    
    // ALT_APPEND:idx,chunk - Append to alternate route
    if (trimmedData.startsWith("ALT_APPEND:")) {
        int firstComma = trimmedData.indexOf(',');
        if (firstComma > 11) {
            int idx = trimmedData.substring(11, firstComma).toInt();
            const char* chunk = trimmedData.c_str() + firstComma + 1;
            
            if (idx >= 0 && idx <= 1) {
                if (xSemaphoreTake(routeMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    decodePolylineChunk(chunk, alternateRoutes[idx], alternateDecoderStates[idx], &alternateAnchors[idx]);
                    xSemaphoreGive(routeMutex);
                }
                sendNotification("A_ALT_APPEND:" + String(idx));
                return;
            }
        }
        sendNotification("E_ALT_APPEND");
        return;
    }
    
    // ALT_END:idx - Finish alternate route sync
    if (trimmedData.startsWith("ALT_END:")) {
        int idx = trimmedData.substring(8).toInt();
        if (idx >= 0 && idx <= 1) {
            if (xSemaphoreTake(routeMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                // Anchor is already set by decodePolylineChunk during ALT_APPEND
                // No need to recalculate or overwrite it here.
                xSemaphoreGive(routeMutex);
            }
            Serial.printf("BLE: Alternate %d complete. Points: %d\n", idx, alternateRoutes[idx].size());
            sendNotification("A_ALT_END:" + String(idx) + "," + String(alternateRoutes[idx].size()));
            xTaskNotify(renderTaskHandle, 0x01, eSetBits); // Force redraw to show alternates
            return;
        }
        sendNotification("E_ALT_END");
        return;
    }
    
    // ROUTE_SELECT:idx - Force select a route (-1=main, 0=alt0, 1=alt1) to be ACTIVE
    if (trimmedData.startsWith("ROUTE_SELECT:")) {
        int idx = trimmedData.substring(13).toInt();
        
        if (idx >= 0 && idx <= 1) {
             if (xSemaphoreTake(routeMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                 if (!alternateRoutes[idx].empty()) {
                     Serial.printf("BLE: Swapping Active Route with Alternate %d\n", idx);
                     
                     // 1. Swap Vectors (O(1) pointer swap)
                     activeRoute.swap(alternateRoutes[idx]);
                     
                     // 2. Swap Anchors (Start points)
                     RouteAnchor tempAnchor = activeRouteAnchor;
                     activeRouteAnchor = alternateAnchors[idx];
                     alternateAnchors[idx] = tempAnchor;
                     
                     // 3. Swap Decoder States (to maintain stream continuity if needed)
                     PolylineDecoderState tempState = bleHandler.decoderState;
                     bleHandler.decoderState = alternateDecoderStates[idx];
                     alternateDecoderStates[idx] = tempState;
                     
                     // 4. Reset Active Route State
                     routeAvailable = true;
                     routePointCount = activeRoute.size();
                     
                     // 5. Re-Index the NEW active route for rendering/navigation
                     indexRouteLocked();
                     
                     // 6. Reset Selection (new active is implicitly default)
                     selectedRouteIndex = -1;
                     
                     sendNotification("A_ROUTE_SELECT:SWAPPED," + String(idx));
                     xTaskNotify(renderTaskHandle, 0x01, eSetBits);
                 } else {
                     Serial.printf("BLE: Alternate %d is empty, cannot swap.\n", idx);
                     sendNotification("E_ROUTE_SELECT:EMPTY");
                 }
                 xSemaphoreGive(routeMutex);
             }
             return;
        } else if (idx == -1) {
             // Already Active, do nothing
             return;
        }
        sendNotification("E_ROUTE_SELECT");
        return;
    }

    // =========================================================
    // ROUTE OVERLAY COMMANDS (LEGACY / CHUNKED)
    // =========================================================
    

    

    
    // Route end - mark route as ready
    // Format: ROUTE_END,count,hash (or just ROUTE_END for backwards compatibility)

    
    // Route clear - remove overlay
    if (trimmedData == "ROUTE_CLEAR") {
        if (xSemaphoreTake(routeMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            activeRoute.clear();
            routeAvailable = false;
            currentTurnType = TurnType::NONE; // Clear turn indicator
            
            // Reset route state
            currentRouteState = ROUTE_NONE;
            routePointCount = 0;
            routeHash = 0;
            
            // Also clear waypoints on full route clear
            waypointBuffer.clear();
            
            Serial.println("BLE: Route cleared");
            routeProgressIndex = 0;
            routeLegSwitchIndex = 2147483647; // Reset to infinite (single color)
            currentWaypointIndex = 0; // Reset waypoint progression
            xSemaphoreGive(routeMutex);
        }
        sendNotification("ROUTE_CLEAR_OK");
        return;
    }

    // ROUTE_LEG_INDEX:123
    if (trimmedData.startsWith("ROUTE_LEG_INDEX:")) {
        int idx = trimmedData.substring(16).toInt();
        if (xSemaphoreTake(routeMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            routeLegSwitchIndex = idx;
            xSemaphoreGive(routeMutex);
            Serial.printf("DEBUG: routeLegSwitchIndex set to %d\n", routeLegSwitchIndex);
        }
        return;
    }
    
    // Route status query - for sync verification
    if (trimmedData == "ROUTE_STATUS" || trimmedData == "ROUTE_STATUS?") {
        char statusMsg[64];
        snprintf(statusMsg, sizeof(statusMsg), "ROUTE_STATUS,%d,%d,%08X,%d", 
                currentRouteState, routePointCount, routeHash, selectedRouteIndex);
        sendNotification(statusMsg);
        return;
    }

    // Route resume check
    if (trimmedData == "GET_DESTINATION" || trimmedData == "GET_DESTINATION?") {
        if (xSemaphoreTake(routeMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            bool hasRoute = false;
            double destLat = 0, destLon = 0;

            if (selectedRouteIndex == -1 && !activeRoute.empty()) {
                hasRoute = true;
                // Use decoder state if available (it holds last point)
                destLat = bleHandler.decoderState.currentLat1e6 / 1000000.0;
                destLon = bleHandler.decoderState.currentLon1e6 / 1000000.0;
            } else if (selectedRouteIndex >= 0 && selectedRouteIndex <= 1 && !alternateRoutes[selectedRouteIndex].empty()) {
                hasRoute = true;
                destLat = alternateDecoderStates[selectedRouteIndex].currentLat1e6 / 1000000.0;
                destLon = alternateDecoderStates[selectedRouteIndex].currentLon1e6 / 1000000.0;
            }

            xSemaphoreGive(routeMutex);

            if (hasRoute) {
                char msg[64];
                snprintf(msg, sizeof(msg), "DESTINATION,%.6f,%.6f", destLat, destLon);
                sendNotification(msg);
            } else {
                sendNotification("E_NO_ROUTE");
            }
        }
        return;
    }





    // ===================================
    // WAYPOINT RECOVERY
    // ===================================
    
    // WAYPOINTS_SET:lat1,lon1;lat2,lon2;...
    if (trimmedData.startsWith("WAYPOINT_SET:") || trimmedData.startsWith("WAYPOINTS_SET:")) {
        if (xSemaphoreTake(routeMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            waypointBuffer.clear(); // Auto-clear existing waypoints
            
            String pointsStr;
            if (trimmedData.startsWith("WAYPOINT_SET:")) {
                pointsStr = trimmedData.substring(13);
            } else {
                pointsStr = trimmedData.substring(14);
            }
            
            // Parse semicolon-separated list
            int startIdx = 0;
            while (startIdx < pointsStr.length()) {
                int semiIdx = pointsStr.indexOf(';', startIdx);
                if (semiIdx == -1) semiIdx = pointsStr.length();
                
                String pair = pointsStr.substring(startIdx, semiIdx);
                int commaIdx = pair.indexOf(',');
                if (commaIdx > 0) {
                    Waypoint wp;
                    wp.lat = pair.substring(0, commaIdx).toDouble();
                    wp.lon = pair.substring(commaIdx + 1).toDouble();
                    
                    if (waypointBuffer.size() < MAX_WAYPOINTS) {
                        waypointBuffer.push_back(wp);
                    }
                }
                startIdx = semiIdx + 1;
            }
            
            // Force redraw/update if needed (optional)
            // xTaskNotify(renderTaskHandle, 0x01, eSetBits);
            
            xSemaphoreGive(routeMutex);
            sendNotification("WAYPOINTS_SET_OK");
        }
        return;
    }

    // WAYPOINT_CLEAR
    if (trimmedData == "WAYPOINT_CLEAR") {
        if (xSemaphoreTake(routeMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            waypointBuffer.clear();
            xSemaphoreGive(routeMutex);
        }
        return;
    }

    // WAYPOINT_ADD:lat,lon
    if (trimmedData.startsWith("WAYPOINT_ADD:")) {
        int commaIdx = trimmedData.indexOf(',');
        if (commaIdx > 0) {
            String latStr = trimmedData.substring(13, commaIdx);
            String lonStr = trimmedData.substring(commaIdx + 1);
            
            Waypoint newWp;
            newWp.lat = latStr.toDouble();
            newWp.lon = lonStr.toDouble();
            
            if (xSemaphoreTake(routeMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                if (waypointBuffer.size() < MAX_WAYPOINTS) {
                    waypointBuffer.push_back(newWp);
                }
                xSemaphoreGive(routeMutex);
            }
        }
        return;
    }

    // GET_WAYPOINTS
    if (trimmedData == "GET_WAYPOINTS" || trimmedData == "GET_WAYPOINTS?") {
        String resp = "WAYPOINTS:";
        if (xSemaphoreTake(routeMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            for (int i = 0; i < waypointBuffer.size(); i++) {
                resp += String(waypointBuffer[i].lat, 6) + "," + String(waypointBuffer[i].lon, 6);
                if (i < waypointBuffer.size() - 1) resp += ";";
            }
            xSemaphoreGive(routeMutex);
        }
        sendNotification(resp);
        return;
    }

    // Turn direction command: TURN <type>
    if (trimmedData.startsWith("TURN ")) {
        String turnStr = trimmedData.substring(5);
        turnStr.trim();
        turnStr.toUpperCase();
        
        if (turnStr == "STRAIGHT") {
            currentTurnType = TurnType::STRAIGHT;
        } else if (turnStr == "LEFT") {
            currentTurnType = TurnType::LEFT;
        } else if (turnStr == "RIGHT") {
            currentTurnType = TurnType::RIGHT;
        } else if (turnStr == "SLIGHT_LEFT") {
            currentTurnType = TurnType::SLIGHT_LEFT;
        } else if (turnStr == "SLIGHT_RIGHT") {
            currentTurnType = TurnType::SLIGHT_RIGHT;
        } else if (turnStr == "UTURN") {
            currentTurnType = TurnType::UTURN;
        } else {
            currentTurnType = TurnType::NONE;
        }
        Serial.printf("BLE: Turn direction set to %s\n", turnStr.c_str());
        return;
        return;
    }
    
    // Turn Distance command: TURN_DIST <meters>
    if (trimmedData.startsWith("TURN_DIST ")) {
        float d = trimmedData.substring(10).toFloat();
        if (d >= 0) currentTurnDistMeters = d;
        // Serial.printf("BLE: Turn dist %.1fm\n", currentTurnDistMeters);
        return;
    }

    // PAIRING_HEARTBEAT handler removed
    /*
    if (trimmedData == "PAIRING_HEARTBEAT") {
        lastPairingHeartbeatTime = millis();
        return;
    }
    */
    
    // ==================================================
    // HIGH-SPEED GPS MODE: G prefix
    // Binary Format (preferred): G + lat(int32_le) + lon(int32_le) = 9 bytes, scale x1,000,000
    // String Format (legacy): Glat,lon (e.g., G12.82730,80.21930)
    // Response: Single byte 'K' (ACK)
    // ==================================================
    if (trimmedData.startsWith("G") && trimmedData.length() > 1) {
        double lat = 0.0, lon = 0.0;
        bool validParse = false;
        
        // Check for BINARY format: exactly 9 bytes, no comma
        if (trimmedData.length() == 9 && trimmedData.indexOf(',') == -1) {
            // Binary format: G (1 byte) + lat (4 bytes int32 LE) + lon (4 bytes int32 LE)
            const uint8_t* raw = (const uint8_t*)trimmedData.c_str();
            int32_t latInt = (int32_t)(raw[1] | (raw[2] << 8) | (raw[3] << 16) | (raw[4] << 24));
            int32_t lonInt = (int32_t)(raw[5] | (raw[6] << 8) | (raw[7] << 16) | (raw[8] << 24));
            lat = latInt / 1000000.0;
            lon = lonInt / 1000000.0;
            validParse = true;
            // Serial.printf("GPS Binary: lat=%f, lon=%f\n", lat, lon);
        } else {
            // String format fallback: Glat,lon
            String gpsData = trimmedData.substring(1); // Remove 'G' prefix
            int commaIdx = gpsData.indexOf(',');
            if (commaIdx > 0) {
                lat = gpsData.substring(0, commaIdx).toDouble();
                lon = gpsData.substring(commaIdx + 1).toDouble();
                validParse = true;
            }
        }
        
        if (validParse && lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0) {
            phoneGpsActive = true;
            lastPhoneCommandTime = millis();
            
            // =====================================================
            // GPS NOISE FILTERING
            // Reject erratic readings that are physically impossible
            // =====================================================
            static double pendingLat = 0.0, pendingLon = 0.0;
            static int pendingConfirmCount = 0;
            static const float MAX_SPEED_MS = 111.0f;     // ~400 km/h max
            static const float BIG_JUMP_METERS = 100.0f;  // Require confirmation for jumps > 100m
            
            bool acceptPoint = true;
            
            // Update GPS state for dead reckoning interpolation
            if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                unsigned long now = millis();
                float dt = (now - gpsState.timestamp) / 1000.0f; // seconds
                
                if (dt > 0.01f && gpsState.timestamp > 0) {
                    // Calculate distance and heading from last position
                    double dLat = lat - gpsState.lat;
                    double dLon = lon - gpsState.lon;
                    float distance = sqrt(dLat * dLat + dLon * dLon) * 111320.0f; // approx meters
                    float speed = distance / dt;
                    
                    // FILTER 1: Reject impossible speed (> 200 km/h)
                    if (speed > MAX_SPEED_MS) {
                        // Check if this matches pending point (confirmation)
                        if (pendingConfirmCount > 0) {
                            float pendingDist = sqrt(pow(lat - pendingLat, 2) + pow(lon - pendingLon, 2)) * 111320.0f;
                            if (pendingDist < 50.0f) {
                                pendingConfirmCount++;
                                if (pendingConfirmCount >= 2) {
                                    // Confirmed - this is a real teleport (user in vehicle, etc)
                                    // Accept and reset pending
                                    pendingConfirmCount = 0;
                                    // Serial.println("[GPS] Big jump CONFIRMED after 2 readings");
                                } else {
                                    acceptPoint = false; // Wait for more confirmations
                                }
                            } else {
                                // New different point, restart confirmation
                                pendingLat = lat;
                                pendingLon = lon;
                                pendingConfirmCount = 1;
                                acceptPoint = false;
                                // Serial.printf("[GPS] Rejected: impossible speed %.1f m/s (waiting for confirm)\n", speed);
                            }
                        } else {
                            // First big jump, start confirmation process
                            pendingLat = lat;
                            pendingLon = lon;
                            pendingConfirmCount = 1;
                            acceptPoint = false;
                            // Serial.printf("[GPS] Rejected: impossible speed %.1f m/s (need confirmation)\n", speed);
                        }
                    } else {
                        // Speed is reasonable - accept point
                        pendingConfirmCount = 0; // Reset confirmation counter
                    }
                    
                    if (acceptPoint) {
                gpsState.speed = speed;
                gpsState.heading = atan2(dLon, dLat) * 180.0f / 3.14159f;
                if (gpsState.heading < 0) gpsState.heading += 360.0f;
                
                // ---------------------------------------------------------
                // AUTO-ZOOM (Speed-Based)
                // < 30 km/h (8.3 m/s) -> Zoom 4 (Closest)
                // 30-60 km/h (8.3-16.6) -> Zoom 3
                // 60-90 km/h (16.6-25.0) -> Zoom 2
                // > 90 km/h (25.0) -> Zoom 1 (Furthest)
                // ---------------------------------------------------------
                static float lastZoomSpeed = 0.0f;
                static int currentAutoZoom = 1;
                float hysteresis = 2.0f; // 2 m/s buffer to avoid jitter
                
                int targetZoom = currentAutoZoom;
                if (speed < (8.3f - hysteresis)) targetZoom = 4.0f;
                else if (speed > (8.3f + hysteresis) && speed < (16.6f - hysteresis)) targetZoom = 3.0f;
                else if (speed > (16.6f + hysteresis) && speed < (25.0f - hysteresis)) targetZoom = 2.0f;
                else if (speed > (25.0f + hysteresis)) targetZoom = 1.0f;
                
                // Turn Modifier: Zoom IN (increase level) if nearing a turn
                if (currentTurnDistMeters >= 0 && currentTurnDistMeters < 200.0f) {
                   if (targetZoom < 4) targetZoom++;
                }
                
                // Only change if significantly different to prevent rapid toggling
                if (targetZoom != currentAutoZoom && onZoomChange) {
                     currentAutoZoom = targetZoom;
                     onZoomChange((float)targetZoom);
                     // Serial.printf("Auto-Zoom: Speed %.1f m/s -> Level %d\n", speed, targetZoom);
                }

                float instantRate = 1.0f / dt;
                gpsRate = (gpsRate == 0.0f) ? instantRate : gpsRate * 0.7f + instantRate * 0.3f;
            }
                } else if (gpsState.timestamp == 0) {
                    // First fix - always accept
                    // Serial.println("GPS: First fix received");
                }

                if (acceptPoint) {
                    gpsState.lat = lat;
                    gpsState.lon = lon;
                    gpsState.timestamp = now;
                }
                xSemaphoreGive(gpsMutex);
            }
            
            if (acceptPoint && onCoordinatesReceived) {
                onCoordinatesReceived(lat, lon);
            }
            // K ACK is sent by onCoordinatesReceived callback (only for accepted points)
            return;
        }

        // Invalid G command - send error
        sendNotification("E");
        return;
    }
    
    // Check for coordinates (contains comma) - legacy format
    int commaIndex = trimmedData.indexOf(',');
    if (commaIndex > 0) {
        String latStr = trimmedData.substring(0, commaIndex);
        String lonStr = trimmedData.substring(commaIndex + 1);
        
        double lat = latStr.toDouble();
        double lon = lonStr.toDouble();
        
        // Basic validation
        if (lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0) {
//            Serial.printf("BLE: Received coordinates - Lat: %.6f, Lon: %.6f\n", lat, lon);
            phoneGpsActive = true; // Mark phone GPS as active
            lastPhoneCommandTime = millis(); // Track last command time
            if (onCoordinatesReceived) {
                onCoordinatesReceived(lat, lon);
            }
        } else {
            sendNotification("ERROR_INVALID_COORDS");
        }
        return;
    }
    
//    Serial.printf("BLE: Unknown command format: %s\n", trimmedData.c_str());
    sendNotification("ERROR_UNKNOWN_CMD");
}

// Connect to WiFi
bool BLEHandler::connectToWiFi() {
    if (WiFi.status() == WL_CONNECTED) {
//        Serial.println("WiFi: Already connected");
        return true;
    }
    
//    Serial.printf("WiFi: Connecting to %s...\n", WIFI_SSID);
    sendNotification("WIFI_CONNECTING");
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
        esp_task_wdt_reset();  // Reset watchdog during WiFi connection
    }
//    Serial.println();
    
    if (WiFi.status() == WL_CONNECTED) {
//        Serial.printf("WiFi: Connected! IP: %s\n", WiFi.localIP().toString().c_str());
        sendNotification("WIFI_CONNECTED");
        return true;
    } else {
        Serial.println("WiFi: Connection failed");
        sendNotification("WIFI_FAILED");
        return false;
    }
}

// Handle OTA Firmware update
void BLEHandler::handleOTAFirmware(const String& url) {
    sendNotification("OTA_FW_START");

    // UI Update: Start Firmware OTA
    globalOTAState.active = true;
    strncpy(globalOTAState.type, "FIRMWARE", sizeof(globalOTAState.type) - 1);
    globalOTAState.percent = 0;
    
    if (!connectToWiFi()) {
        sendNotification("OTA_FW_FAILED");
        return;
    }
    
//    Serial.printf("OTA: Starting firmware update from: %s\n", url.c_str());
    
    WiFiClient client;
    
    // Set up update callbacks for progress
    static int lastPercent = -1; // Static to retain value across calls
    httpUpdate.onProgress([&](int current, int total) {
        int percent = (current * 100) / total;
        if (percent != lastPercent) {
            Serial.printf("OTA Progress: %d%%\n", percent);
            lastPercent = percent;
            bleHandler.sendNotification("OTA_PROGRESS " + String(percent));
            
            // UI Update: Progress
            globalOTAState.percent = percent;
            esp_task_wdt_reset(); // Keep watchdog happy
        }
        esp_task_wdt_reset();  // Reset watchdog during OTA
    });
    
    t_httpUpdate_return ret = httpUpdate.update(client, url);
    
    switch (ret) {
        case HTTP_UPDATE_FAILED:
            Serial.printf("OTA: Firmware update failed. Error (%d): %s\n", 
                         httpUpdate.getLastError(), 
                         httpUpdate.getLastErrorString().c_str());
            sendNotification("OTA_FW_FAILED");
            globalOTAState.active = false; // UI Update: Failed
            break;
            
        case HTTP_UPDATE_NO_UPDATES:
//            Serial.println("OTA: No updates available");
            sendNotification("OTA_FW_NO_UPDATE");
            globalOTAState.active = false; // UI Update: Done
            break;
            
        case HTTP_UPDATE_OK:
//            Serial.println("OTA: Firmware update successful! Rebooting...");
            sendNotification("OTA_FW_SUCCESS");
            globalOTAState.percent = 100; // UI Update: 100%
            delay(1000); // Give UI time to show 100%
            ESP.restart();
            break;
    }
}

// Handle OTA Map update
void BLEHandler::handleOTAMap(const String& url) {
    sendNotification("OTA_MAP_START");
    
    // UI Update: Start Map OTA
    globalOTAState.active = true;
    strncpy(globalOTAState.type, "MAP DATA", sizeof(globalOTAState.type) - 1);
    globalOTAState.percent = 0;
    
    if (!connectToWiFi()) {
        sendNotification("OTA_MAP_FAILED");
        globalOTAState.active = false; // UI Update: Failed
        return;
    }
    
//    Serial.printf("OTA: Starting map download from: %s\n", url.c_str());
    
    // Extract filename from URL
    int lastSlash = url.lastIndexOf('/');
    String filename = "/";
    if (lastSlash >= 0) {
        filename += url.substring(lastSlash + 1);
    } else {
        filename += "map_update.mbtiles";
    }
    
//    Serial.printf("OTA: Saving to SD: %s\n", filename.c_str());
    
    WiFiClient client;
    HTTPClient http;
    
    http.begin(client, url);
    http.setTimeout(30000);  // 30 second timeout
    int httpCode = http.GET();
    
    if (httpCode == HTTP_CODE_OK) {
        int contentLength = http.getSize();
//        Serial.printf("OTA: Download size: %d bytes\n", contentLength);
        
        File file = SD_MMC.open(filename, FILE_WRITE);
        if (!file) {
            Serial.println("OTA: Failed to open file for writing");
            sendNotification("OTA_MAP_FAILED");
            globalOTAState.active = false; // UI Update: Failed
            http.end();
            return;
        }
        
        WiFiClient* stream = http.getStreamPtr();
        uint8_t buffer[4096];
        int totalWritten = 0; // Renamed from totalRead for clarity with new code
        int lastPercent = 0;
        
        while (http.connected() && (contentLength > 0 || contentLength == -1)) {
            size_t size = stream->available();
            if (size) {
                int c = stream->readBytes(buffer, min((size_t)sizeof(buffer), size));
                file.write(buffer, c);
                totalWritten += c; // Use totalWritten
                
                if (contentLength > 0) {
                    // Recalculate percent based on totalWritten and original contentLength
                    int percent = (int)((totalWritten * 100) / http.getSize()); // Use http.getSize() for total
                    if (percent != lastPercent) {
                        lastPercent = percent;
//                        Serial.printf("OTA: Downloaded %d%%\n", percent);
                        sendNotification("OTA_PROGRESS " + String(percent));
                        
                        // UI Update: Progress
                        globalOTAState.percent = percent;
                        esp_task_wdt_reset();
                    }
                }
                esp_task_wdt_reset();  // Reset watchdog during download
            }
            delay(1);
        }
        
        file.close();
        http.end(); // End HTTP client here
        
        if (totalWritten == http.getSize()) { // Compare totalWritten with actual total size
//            Serial.printf("OTA: Map download complete. Total: %d bytes\n", totalWritten);
            sendNotification("OTA_MAP_SUCCESS");
            
            // UI Update: Success
            globalOTAState.percent = 100;
            delay(1000); 
            globalOTAState.active = false; // Done
            
            // Notify Render Task to reload
            bool success = true;
            xQueueSend(tileParsedNotificationQueue, &success, 0); 

        } else {
            Serial.printf("OTA: Download incomplete. Expected: %d, Got: %d\n", http.getSize(), totalWritten);
            sendNotification("OTA_MAP_FAILED");
            globalOTAState.active = false; // UI Update: Failed
        }

    } else {
        Serial.printf("OTA: HTTP error: %d\n", httpCode);
        sendNotification("OTA_MAP_FAILED");
        globalOTAState.active = false; // UI Update: Failed
    }
    
    http.end(); // Ensure http client is ended even on error
}

// =========================================================
// OTA VIA WIFI (SoftAP) IMPLEMENTATION
// =========================================================

// Helpers for WebServer
// Global tracking for upload stall detection
static unsigned long lastDataReceivedTime = 0;
static const unsigned long UPLOAD_STALL_TIMEOUT_MS = 5000; // 5 seconds
static size_t otaRequestTotalSize = 0;

void handleUpdateUpload() {
    HTTPUpload& upload = server.upload();
    // Shared progress trackers across upload phases
    static size_t bytesReceived = 0;
    static size_t lastLoggedBytes = 0;
    static bool firstChunk = true;
    
    // Track data reception time
    lastDataReceivedTime = millis();
    
    // Mark update as started to prevent connection timeout
    if (!bleHandler.updateStarted) {
        bleHandler.updateStarted = true;
        Serial.println("OTA: Update process started!");
    }

    if (upload.status == UPLOAD_FILE_START) {
        // Suspend Render Task immediately to prevent freeze artifacts
        if (renderTaskHandle != NULL) {
            vTaskSuspend(renderTaskHandle);
        }
        
        // Reset per-upload trackers
        bytesReceived = 0;
        lastLoggedBytes = 0;
        firstChunk = true;

        // Get generic Content-Length if available
        if (server.hasHeader("Content-Length")) {
            otaRequestTotalSize = server.header("Content-Length").toInt();
        } else {
            otaRequestTotalSize = upload.totalSize; // Fallback
        }

        // Enable OTA Screen
        globalOTAState.active = true;
        strncpy(globalOTAState.type, "FIRMWARE", 15);
        globalOTAState.percent = 0;
        
        drawOTAProgress(0); // Immediate feedback
        Serial.printf("OTA: Upload START - Filename: %s\n", upload.filename.c_str());
        Serial.printf("OTA: Calling Update.begin(UPDATE_SIZE_UNKNOWN)...\n");

        // If we already know the total size, pass it to Update to improve progress accounting
        size_t beginSize = (otaRequestTotalSize > 0) ? otaRequestTotalSize : UPDATE_SIZE_UNKNOWN;
        if (!Update.begin(beginSize)) {
            Update.printError(Serial);
            Serial.println("OTA: ERROR - Update.begin() FAILED! Rebooting.");
            server.client().stop();
            WiFi.softAPdisconnect(true);
            delay(100);
            ESP.restart();
        }
        Serial.println("OTA: Update.begin() SUCCESS");
        
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        // Track first chunk for special handling

        uint8_t* dataPtr = upload.buf;
        size_t dataSize = upload.currentSize;
        
        if (firstChunk) {
            Serial.printf("OTA: First WRITE chunk - size: %u bytes\n", upload.currentSize);
            Serial.printf("OTA: First 16 bytes (hex): ");
            for (int i = 0; i < min(16, (int)upload.currentSize); i++) {
                Serial.printf("%02X ", upload.buf[i]);
            }
            Serial.println();
            
            // iOS multipart adds CRLF (0D 0A) prefix - skip it!
            // Look for E9 magic byte and skip any leading garbage
            int skipBytes = 0;
            for (int i = 0; i < min(10, (int)dataSize); i++) {
                if (dataPtr[i] == 0xE9) {
                    skipBytes = i;
                    break;
                }
            }
            
            if (skipBytes > 0) {
                Serial.printf("OTA: Skipping %d leading bytes to find magic byte E9\n", skipBytes);
                dataPtr += skipBytes;
                dataSize -= skipBytes;
                Serial.printf("OTA: Adjusted first bytes (hex): ");
                for (int i = 0; i < min(8, (int)dataSize); i++) {
                    Serial.printf("%02X ", dataPtr[i]);
                }
                Serial.println();
            }
            
            firstChunk = false;
        }
        
        // Flashing firmware to ESP
        if (Update.write(dataPtr, dataSize) != dataSize) {
            Update.printError(Serial);
            Serial.println("OTA: ERROR - Write FAILED! Aborting.");
            Update.abort();
            server.client().stop();
            WiFi.softAPdisconnect(true);
            delay(100);
            ESP.restart();
        }
        
        // Update progress using total bytes received instead of chunk size
        bytesReceived += dataSize;
        size_t totalForPercent = otaRequestTotalSize;
        if (totalForPercent == 0) {
            totalForPercent = Update.size(); // Might be set if begin(size) succeeded
        }
        if (totalForPercent == 0) {
            totalForPercent = upload.totalSize; // Fallback to cumulative size seen so far
        }

        if (totalForPercent > 0) {
            int p = (int)((uint64_t)bytesReceived * 100 / totalForPercent);
            if (p > 100) p = 100; // Defensive clamp
            if (p != globalOTAState.percent) {
                globalOTAState.percent = p;
                drawOTAProgress(p);
            }
        } else if (bytesReceived > 0 && globalOTAState.percent == 0) {
            // Unknown total size: at least show something after first data arrives
            globalOTAState.percent = 1;
            drawOTAProgress(1);
        }

        esp_task_wdt_reset();

        // Progress logging every 10%
        if ((bytesReceived / (1024 * 100)) != (lastLoggedBytes / (1024 * 100))) {
            Serial.printf("OTA: Progress - %u KB written\n", bytesReceived / 1024);
            lastLoggedBytes = bytesReceived;
        }
        
    } else if (upload.status == UPLOAD_FILE_END) {
        Serial.println("========================================");
        Serial.printf("OTA: Upload END - Total: %u bytes\n", upload.totalSize);
        Serial.println("========================================");
        
        if (Update.end(true)) {
            Serial.printf("OTA: SUCCESS! %u bytes uploaded\n", upload.totalSize);
            globalOTAState.percent = 100;
            drawOTAProgress(100);
        } else {
            Update.printError(Serial);
            Serial.println("OTA: ERROR - Update.end() FAILED! Rebooting.");
            WiFi.softAPdisconnect(true);
            delay(100);
            ESP.restart();
        }
    }
}

// Direct rendering helper for OTA (bypasses render_task)
void drawOTAProgress(int percent) {
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    
    // Debug logging
    // Serial.printf("OTA DRAW: %d%%\n", percent); 

    sprite.fillScreen(TFT_BLACK);
    
    // Title
    sprite.setTextColor(TFT_WHITE, TFT_BLACK);
    sprite.setTextSize(1); // Reduced from 2 to fit screen
    sprite.setTextDatum(MC_DATUM);
    sprite.drawString("UPDATING...", screenW / 2, 40);
    
    // Subtitle
    sprite.setTextSize(1);
    sprite.setTextColor(TFT_CYAN, TFT_BLACK); 
    sprite.drawString("FIRMWARE", screenW / 2, 60);

    // Progress Bar
    int barWidth = screenW - 40;
    int barHeight = 10;
    int barX = 20;
    int barY = 80;
    
    sprite.drawRect(barX, barY, barWidth, barHeight, TFT_WHITE);
    int fillWidth = (barWidth - 4) * percent / 100;
    
    // Safety check for fillWidth
    if (fillWidth < 0) fillWidth = 0;
    if (fillWidth > (barWidth - 4)) fillWidth = (barWidth - 4);
    
    if (fillWidth > 0) {
        sprite.fillRect(barX + 2, barY + 2, fillWidth, barHeight - 4, TFT_GREEN);
    }

    // Text
    sprite.setTextSize(1);
    sprite.setTextColor(TFT_WHITE, TFT_BLACK);
    String pStr = String(percent) + "%";
    sprite.drawString(pStr, screenW / 2, 100);

    // Disclaimer at bottom
    sprite.setTextSize(1);
    sprite.setTextColor(TFT_YELLOW, TFT_BLACK);
    sprite.setTextDatum(MC_DATUM);
    sprite.drawString("Device will restart", screenW / 2, 130);
    sprite.drawString("Do not turn off", screenW / 2, 145);

    sprite.pushSprite(0, 0);
}

// Handler for RAW binary upload (no multipart, just raw POST body)
void handleRawUpdate() {
    Serial.println("\n========================================");
    Serial.println("OTA: RAW UPDATE HANDLER CALLED");
    Serial.println("========================================");
    
    // WDT already disabled when WiFi OTA started
    bleHandler.updateStarted = true;
    
    // Enable OTA Screen
    globalOTAState.active = true;
    strncpy(globalOTAState.type, "FIRMWARE", 15);
    globalOTAState.percent = 0;
    
    // Suspend Render Task to take over display
    if (renderTaskHandle != NULL) {
        vTaskSuspend(renderTaskHandle);
    }
    
    // Initial Draw
    drawOTAProgress(0);

    WiFiClient client = server.client();
    Serial.printf("OTA: Client connected: %s\n", client.connected() ? "YES" : "NO");
    
    // Get Content-Length from headers
    size_t contentLength = 0;
    if (server.hasHeader("Content-Length")) {
        contentLength = server.header("Content-Length").toInt();
        Serial.printf("OTA: Content-Length header: %u bytes\n", contentLength);
    } else {
        Serial.println("OTA: ERROR - No Content-Length header!");
    }
    
    // Log all headers for debugging
    Serial.println("OTA: Request headers:");
    for (int i = 0; i < server.headers(); i++) {
        Serial.printf("  %s: %s\n", server.headerName(i).c_str(), server.header(i).c_str());
    }
    
    if (contentLength == 0 || contentLength > 10000000) { // Max 10MB
        Serial.printf("OTA: ERROR - Invalid content length: %u\n", contentLength);
        server.send(400, "text/plain", "Invalid Content-Length");
        return;
    }
    
    Serial.println("OTA: Calling Update.begin()...");
    if (!Update.begin(contentLength)) {
        Serial.println("OTA: ERROR - Update.begin() failed!");
        Update.printError(Serial);
        server.send(500, "text/plain", "Update.begin failed");
        return;
    }
    Serial.println("OTA: Update.begin() SUCCESS");
    
    // Read and flash in chunks - use Stream.readBytes with timeout
    uint8_t buf[512]; // Smaller buffer for better responsiveness
    size_t remaining = contentLength;
    size_t totalWritten = 0;
    client.setTimeout(5000); // 5s timeout for each read operation
    
    Serial.println("OTA: Starting to read data stream...");
    Serial.printf("OTA: Will read %u bytes total\n", contentLength);
    
    int chunkCount = 0;
    while (remaining > 0 && client.connected()) {
        yield(); // Allow other tasks to run

        // Update Progress
        if (contentLength > 0) {
            int p = (int)((uint64_t)(contentLength - remaining) * 100 / contentLength);
            if (p != globalOTAState.percent) {
                 globalOTAState.percent = p;
                 drawOTAProgress(p); // Direct Draw
            }
        }
        
        size_t toRead = min(remaining, sizeof(buf));
        
        // readBytes will block until data arrives or timeout
        if (chunkCount == 0) {
            Serial.printf("OTA: Reading first chunk (%u bytes)...\n", toRead);
        }
        
        size_t bytesRead = client.readBytes(buf, toRead);
        
        if (bytesRead == 0) {
            Serial.printf("OTA: ERROR - Read failed! Got 0 bytes (remaining: %u, chunk: %d)\n", remaining, chunkCount);
            Serial.printf("OTA: Client still connected: %s\n", client.connected() ? "YES" : "NO");
            Update.abort();
            server.send(500, "text/plain", "Stream read timeout");
            return;
        }
        
        if (chunkCount == 0) {
            Serial.printf("OTA: First chunk received! %u bytes\n", bytesRead);
            Serial.printf("OTA: First 16 bytes (hex): ");
            for (int i = 0; i < min(16, (int)bytesRead); i++) {
                Serial.printf("%02X ", buf[i]);
            }
            Serial.println();
        }
        
        if (Update.write(buf, bytesRead) != bytesRead) {
            Serial.println("OTA: ERROR - Flash write failed!");
            Update.printError(Serial);
            Update.abort();
            server.send(500, "text/plain", "Flash write failed");
            return;
        }
        
        remaining -= bytesRead;
        totalWritten += bytesRead;
        lastDataReceivedTime = millis();
        chunkCount++;
        
        // Progress every 10%
        int currentPercent = (totalWritten * 100 / contentLength);
        int prevPercent = ((totalWritten - bytesRead) * 100 / contentLength);
        if ((currentPercent / 10) != (prevPercent / 10)) {
            Serial.printf("OTA: Progress %d%% (%u / %u bytes, %d chunks)\n", 
                         currentPercent, totalWritten, contentLength, chunkCount);
        }
    }
    
    Serial.printf("OTA: Read loop complete. Total chunks: %d, Total bytes: %u\n", chunkCount, totalWritten);
    
    if (!Update.end(true)) {
        Serial.println("OTA: ERROR - Update.end() failed!");
        Update.printError(Serial);
        server.send(500, "text/plain", "Update.end failed");
        return;
    }
    
    Serial.println("========================================");
    Serial.printf("OTA: SUCCESS! Upload complete! %u bytes\n", totalWritten);
    Serial.println("========================================");
    
    // Turn off WiFi immediately
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    bleHandler.otaActive = false;
    
    server.send(200, "text/plain", "OK");
    
    Serial.println("OTA: Rebooting in 500ms...");
    delay(500);
    ESP.restart();
}

void handleUpdateEnd() {
    // IMMEDIATELY turn off WiFi before responding
    Serial.println("OTA: Transfer complete. Shutting down WiFi...");
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    bleHandler.otaActive = false;
    
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    
    Serial.println("OTA: Rebooting in 500ms...");
    delay(500);
    ESP.restart();
}

void BLEHandler::handleOTA_WiFi_Start() {
    Serial.println("OTA: Switching to WiFi SoftAP mode");
    
    // Check if already active
    if (otaActive) {
        Serial.println("OTA: WiFi SoftAP already active. Resetting timer and resending creds.");
        otaStartTime = millis();
        // Resend creds below...
    } else {
        // Free up RAM and stop other tasks
        prepareForWifiOTA();
    }

    // Calculate Password: Navion + Last 6 MAC chars (Derived from serial)
    // ESP_MAC_BT gives 6 bytes.
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    char macSuffix[13];
    // Format all 6 bytes (12 chars)
    snprintf(macSuffix, sizeof(macSuffix), "%02X%02X%02X%02X%02X%02X", 
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
    // Use last 6 characters (indices 6 to 11)
    String fullMac = String(macSuffix);
    String derivedPart = fullMac.substring(6); 
    String password = "Navion" + derivedPart;
    
    Serial.printf("OTA: Setting SoftAP Creds -> SSID: %s, PASS: %s\n", deviceName.c_str(), password.c_str());
    
    // Start SoftAP (Only if not active)
    if (!otaActive) {
        WiFi.mode(WIFI_AP);
        if (WiFi.softAP(deviceName.c_str(), password.c_str())) {
            
            delay(100); // Wait for AP
            IPAddress IP = WiFi.softAPIP();
            Serial.print("OTA: SoftAP IP: ");
            Serial.println(IP);
            
            // Setup Server
            server.on("/status", HTTP_GET, []() {
                server.send(200, "text/plain", "READY");
                // Reset timeout on activity
                bleHandler.otaStartTime = millis(); 
            });
            
            server.on("/update", HTTP_POST, handleUpdateEnd, handleUpdateUpload);
            server.on("/update_raw", HTTP_POST, handleRawUpdate); // Raw binary upload
            
            // Collect all headers for proper multipart parsing
            const char* headerKeys[] = {"Content-Type", "Content-Length", "Content-Disposition"};
            server.collectHeaders(headerKeys, 3);
            
            server.begin();
            Serial.println("OTA: WebServer started (headers collection enabled)");
            
            otaActive = true;
            otaStartTime = millis();
            updateStarted = false; // Reset 
            clientConnectedTime = 0; // Reset
            
            // CRITICAL: Disable WDT for loopTask immediately when WiFi OTA starts
            // WiFi stack operations can block the loop, causing WDT timeout
            esp_task_wdt_reset();
            esp_task_wdt_delete(NULL); // NULL = current task (loopTask)
            Serial.println("OTA: WDT disabled for loopTask during WiFi operation");

            String msg = "OTA_WIFI_CRED," + deviceName + "," + password + "," + IP.toString();
            sendNotification(msg);
        } else {
            Serial.println("OTA: SoftAP Failed!");
            sendNotification("OTA_WIFI_FAIL");
        }
    } else {
        // Already active - Just resend credentials
        IPAddress IP = WiFi.softAPIP();
        String msg = "OTA_WIFI_CRED," + deviceName + "," + password + "," + IP.toString();
        sendNotification(msg);
    }
}

void BLEHandler::checkOTATimeout() {
    if (otaActive) {
        unsigned long now = millis();
        int stations = WiFi.softAPgetStationNum();
        
        // 1. Connection Event Logic
        // If stations > 0 but clientConnectedTime is 0, set it
        if (stations > 0 && clientConnectedTime == 0) {
            clientConnectedTime = now;
            Serial.println("OTA: Client Connected! Waiting for data...");
        }
        // If stations == 0, reset
        if (stations == 0) {
            clientConnectedTime = 0;
        }
        
        // 2. TIMEOUT 1: Global Timeout (2 mins, if no update started)
        if (!updateStarted && (now - otaStartTime > 120000)) {
            Serial.println("OTA: Global Timeout (2 mins). Turning off.");
            WiFi.softAPdisconnect(true);
            WiFi.mode(WIFI_OFF);
            otaActive = false;
        }

        // 3. TIMEOUT 2: Aggressive "Connected but Idle" Timeout (15s)
        // If client connected, but no update started within 15 seconds -> KILL
        if (stations > 0 && !updateStarted && clientConnectedTime > 0) {
            if (now - clientConnectedTime > 15000) {
                Serial.println("OTA: Client Connected but NO DATA for 15s. Aggressive Timeout. Rebooting.");
                WiFi.softAPdisconnect(true);
                delay(100);
                ESP.restart();
            }
        }
        
        // 4. TIMEOUT 3: Upload Stall Detection (5s during active transfer)
        // If update HAS started, check if data is stalling
        if (updateStarted && lastDataReceivedTime > 0) {
            if (now - lastDataReceivedTime > UPLOAD_STALL_TIMEOUT_MS) {
                Serial.println("OTA: UPLOAD STALL DETECTED! No data for 5s. Aborting transfer.");
                Update.abort();
                WiFi.softAPdisconnect(true);
                delay(100);
                ESP.restart();
            }
        }
    }
}




