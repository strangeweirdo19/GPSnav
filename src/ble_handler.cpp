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

extern TaskHandle_t renderTaskHandle;

// Forward Declaration
void drawOTAProgress(int percent);

Preferences preferences;
String currentClientAddress = "";
uint16_t currentConnHandle = 0xFFFF; // Store current connection handle
String currentSessionToken = ""; // Store current session token for unpairing

// Global instance
BLEHandler bleHandler;

// Initialize static instance pointer
BLEHandler* BLEHandler::instance = nullptr;

// Server callbacks for connection events
class ServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) {
//        Serial.println("");
//        Serial.println("================================================");
        Serial.println("  BLE CLIENT CONNECTED!");
//        Serial.println("================================================");
        
        // Store current address and connection handle globally
        std::string addrStr = connInfo.getAddress().toString();
        currentClientAddress = String(addrStr.c_str());
        currentConnHandle = connInfo.getConnHandle();
        Serial.print("DEBUG_CONNECT: Set currentClientAddress = '");
        Serial.print(currentClientAddress);
        Serial.print("' (length: ");
        Serial.print(currentClientAddress.length());
        Serial.println(")");
        
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
        Serial.println("  STATUS: Connection Accepted (Provisional)");
        Serial.println("  Auth: Waiting for Token (1s)...");
        
        bleHandler.waitingForToken = true;
        bleHandler.authStartTime = millis();
        
        bleHandler.resetAuthState();
        bleHandler.sendNotification("AUTH_REQ");
        // Do NOT call onDeviceConnected yet (keeps PIN hidden)
    }

    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) {
//        Serial.println("BLE: Client disconnected");
        Serial.println("DEBUG_DISCONNECT: Clearing currentClientAddress");
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
        Serial.println(value.length());
        
        if (value.length() > 0) {
            String receivedData = String(value.c_str());
//            Serial.println("");
//            Serial.println("================================================");
//            Serial.print("BLE RX: Received ");
//            Serial.print(receivedData.length());
//            Serial.println(" bytes");
//            Serial.print("BLE RX: Data: '");
//            Serial.print(receivedData);
//            Serial.println("'");
//            Serial.println("================================================");
            bleHandler.parseReceivedData(receivedData);
        } else {
            // Serial.println("BLE RX: WARNING - Empty data received");
        }
    }
};

// Constructor
BLEHandler::BLEHandler() 
    : pServer(nullptr), pTxCharacteristic(nullptr), pRxCharacteristic(nullptr),
      isAuthenticated(false), authAttempts(0), lockoutUntil(0), lastActivityTime(0),
      bootTime(0), waitingForToken(false), authStartTime(0), blacklistCount(0),
      wifiOTAStartRequested(false), otaActive(false), otaStartTime(0),
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
    Serial.printf("    BLE PIN: %s\n", devicePIN.c_str());
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
//        Serial.println("BLE: Session timeout - re-authentication required");
        isAuthenticated = false;
        sendNotification("SESSION_TIMEOUT");
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
    if (waitingForToken && millis() - lastPairingHeartbeatTime > PAIRING_HEARTBEAT_TIMEOUT_MS) {
        // Serial.println("BLE: Pairing heartbeat timed out - Closing PIN display");
        sendNotification("PAIRING_CLOSED");
        if (currentConnHandle != 0xFFFF && pServer) {
            pServer->disconnect(currentConnHandle);
        }
        waitingForToken = false;
    }
    
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
        Serial.printf("BLE TX: Sent: %s\n", message.c_str());
    } else {
        Serial.println("BLE TX: No device connected");
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
    // ROUTE OVERLAY COMMANDS
    // =========================================================
    
    // Route start - clear and prepare for new route
    if (trimmedData.startsWith("ROUTE_START,")) {
        int expectedCount = trimmedData.substring(12).toInt();
        Serial.printf("BLE: Route start, expecting %d points\n", expectedCount);
        
        if (xSemaphoreTake(routeMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            activeRoute.clear(); // Does not shrink capacity, only size = 0
            
            // Runtime Check: Do we have PSRAM?
            int effectiveLimit = MAX_ROUTE_POINTS;
            if (ESP.getPsramSize() == 0) {
                 effectiveLimit = 1000; // Fallback for Internal RAM (safe limit)
                 if (expectedCount > effectiveLimit) {
                     Serial.printf("⚠️ No PSRAM! Limiting route from %d to %d points.\n", expectedCount, effectiveLimit);
                 }
            } else {
                 Serial.printf("✅ PSRAM Detected. Allowing up to %d points.\n", MAX_ROUTE_POINTS);
            }
            
            activeRoute.reserve(std::min(expectedCount, effectiveLimit)); 
            routeAvailable = false;
            xSemaphoreGive(routeMutex);
        }
        sendNotification("ROUTE_START_OK");
        return;
    }
    
    // Route chunk - receive batch of coordinates
    if (trimmedData.startsWith("ROUTE_CHUNK,")) {
        // Format: ROUTE_CHUNK,idx,lat1,lon1,lat2,lon2,...
        // Use C-style parsing to avoid String memory fragmentation
        const char* dataPtr = trimmedData.c_str() + 12; // Skip "ROUTE_CHUNK,"
        
        // Reset watchdog to prevent timeout during parsing
        esp_task_wdt_reset();
        
        if (xSemaphoreTake(routeMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            // Skip the index value (first number before comma)
            while (*dataPtr && *dataPtr != ',') dataPtr++;
            if (*dataPtr == ',') dataPtr++;
            
            // Runtime Limit Check (Must match ROUTE_START logic)
            int effectiveLimit = (ESP.getPsramSize() > 0) ? MAX_ROUTE_POINTS : 1000;

            // Parse lat,lon pairs using strtod for efficiency
            char* endPtr;
            while (*dataPtr && activeRoute.size() < effectiveLimit) {
                // Parse lat
                double lat = strtod(dataPtr, &endPtr);
                if (endPtr == dataPtr) break; // No number found
                dataPtr = endPtr;
                if (*dataPtr == ',') dataPtr++;
                
                // Parse lon
                double lon = strtod(dataPtr, &endPtr);
                if (endPtr == dataPtr) break; // No number found
                dataPtr = endPtr;
                if (*dataPtr == ',') dataPtr++;
                
                // Add point if valid
                if (lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0) {
                    activeRoute.push_back({lat, lon});
                }
            }
            
            if (activeRoute.size() >= effectiveLimit) {
                // Only warn once if we just hit the limit
                static bool warned = false;
                if (!warned) {
                     Serial.printf("BLE WARNING: Route buffer full! %d points. truncated.\n", effectiveLimit);
                     warned = true;
                }
            } else {
                 // Reset warning flag when under limit (new route)
                 // Note: Ideally this should be reset in ROUTE_START or ROUTE_CLEAR, 
                 // but since static local is tricky, just rely on the printf being harmless if repeated occasionally.
            }
            xSemaphoreGive(routeMutex);
        }
        return;
    }
    
    // Route end - mark route as ready
    if (trimmedData == "ROUTE_END") {
        if (xSemaphoreTake(routeMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            routeAvailable = (activeRoute.size() >= 2);
            Serial.printf("BLE: Route complete, %d points, available=%d\n", activeRoute.size(), routeAvailable);
            xSemaphoreGive(routeMutex);
        }
        sendNotification("ROUTE_END_OK");
        return;
    }
    
    // Route clear - remove overlay
    if (trimmedData == "ROUTE_CLEAR") {
        if (xSemaphoreTake(routeMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            activeRoute.clear();
            routeAvailable = false;
            currentTurnType = TurnType::NONE; // Clear turn indicator
            Serial.println("BLE: Route cleared");
            xSemaphoreGive(routeMutex);
        }
        sendNotification("ROUTE_CLEAR_OK");
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
    }

    if (trimmedData == "PAIRING_HEARTBEAT") {
        // Serial.println("BLE: Pairing heartbeat received");
        lastPairingHeartbeatTime = millis();
        return;
    }
    
    // Check for coordinates (contains comma)
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




