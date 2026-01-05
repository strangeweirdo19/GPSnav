// BLEService.ts - FIXED VERSION WITH BASE64 DECODING

import { NativeModules, NativeEventEmitter } from 'react-native';
const BleManager = NativeModules.BleManager;
const bleManagerEmitter = new NativeEventEmitter(BleManager);

export class BLEService {
  private rxCharacteristicUUID = 'beb5483e-36e1-4688-b7f5-ea07361b26a8';
  private txCharacteristicUUID = 'beb5483e-36e1-4688-b7f5-ea07361b26a9';
  private serviceUUID = '4fafc201-1fb5-459e-8fcc-c5c9c331914b';
  private deviceId: string = '';
  private isAuthenticated: boolean = false;
  private notificationListeners: Map<string, (value: string) => void> = new Map();

  /**
   * Base64 Decode function
   */
  private base64Decode(encodedString: string): string {
    try {
      console.log(`[Base64] Decoding: "${encodedString}"`);
      const decoded = Buffer.from(encodedString, 'base64').toString('utf-8');
      console.log(`[Base64] Decoded result: "${decoded}"`);
      return decoded;
    } catch (error) {
      console.error('[Base64] Decode error:', error);
      return encodedString; // Return original if decode fails
    }
  }

  /**
   * Base64 Encode function
   */
  private base64Encode(plainString: string): string {
    try {
      const encoded = Buffer.from(plainString, 'utf-8').toString('base64');
      console.log(`[Base64] Encoding "${plainString}" → "${encoded}"`);
      return encoded;
    } catch (error) {
      console.error('[Base64] Encode error:', error);
      return plainString;
    }
  }

  /**
   * Connect to BLE device
   */
  async connect(deviceId: string): Promise<void> {
    this.deviceId = deviceId;
    console.log(`\n[BLE] ========== CONNECTING ==========`);
    console.log(`[BLE] Device ID: ${deviceId}`);

    try {
      await BleManager.connect(deviceId);
      console.log(`[BLE] ✓ Connected`);
      
      // Start listening for characteristic notifications
      this.setupNotificationListener();
      
      console.log(`[BLE] ========== CONNECTED ==========\n`);
    } catch (error) {
      console.error(`[BLE] ✗ Connection failed:`, error);
      throw error;
    }
  }

  /**
   * Setup listener for TX characteristic notifications
   */
  private setupNotificationListener(): void {
    console.log(`[BLE] Setting up notification listener...`);
    
    bleManagerEmitter.addListener(
      'BleManagerDidUpdateValueForCharacteristic',
      ({ value, peripheral, characteristic, service }: any) => {
        // Check if this is our TX characteristic
        if (characteristic.toLowerCase() === this.txCharacteristicUUID.toLowerCase()) {
          console.log(`\n[BLE RX] ========== NOTIFICATION RECEIVED ==========`);
          
          // Value comes as array of bytes
          let decodedString = '';
          
          if (Array.isArray(value)) {
            // Convert byte array to string
            decodedString = String.fromCharCode(...value);
            console.log(`[BLE RX] Raw bytes: [${value.join(', ')}]`);
            console.log(`[BLE RX] As ASCII: "${decodedString}"`);
          } else if (typeof value === 'string') {
            decodedString = value;
            console.log(`[BLE RX] Received string: "${decodedString}"`);
          }
          
          // Decode Base64
          const decoded = this.base64Decode(decodedString);
          console.log(`[BLE RX] Final decoded: "${decoded}"`);
          console.log(`[BLE RX] ========================================\n`);
          
          // Trigger any waiting listeners
          this.notificationListeners.forEach((callback, prefix) => {
            if (decoded.startsWith(prefix)) {
              console.log(`[BLE] ✓ Matched listener for: "${prefix}"`);
              callback(decoded);
            }
          });
        }
      }
    );
    
    console.log(`[BLE] ✓ Notification listener ready`);
  }

  /**
   * Send command to RX characteristic
   */
  async sendCommand(command: string): Promise<boolean> {
    console.log(`\n[BLE TX] ========== SENDING COMMAND ==========`);
    console.log(`[BLE TX] Plain text: "${command}"`);
    
    try {
      // Encode as Base64
      const encoded = this.base64Encode(command);
      
      // Convert string to byte array for BLE write
      const bytes = [];
      for (let i = 0; i < encoded.length; i++) {
        bytes.push(encoded.charCodeAt(i));
      }
      
      console.log(`[BLE TX] Byte array: [${bytes.join(', ')}]`);
      console.log(`[BLE TX] Writing to RX characteristic...`);
      
      // Write to RX characteristic
      await BleManager.writeWithoutResponse(
        this.deviceId,
        this.serviceUUID,
        this.rxCharacteristicUUID,
        bytes
      );
      
      console.log(`[BLE TX] ✓ Command sent successfully`);
      console.log(`[BLE TX] =====================================\n`);
      return true;
      
    } catch (error) {
      console.error(`[BLE TX] ✗ Write failed:`, error);
      console.log(`[BLE TX] =====================================\n`);
      return false;
    }
  }

  /**
   * Authenticate with PIN
   */
  async authenticate(pin: string): Promise<boolean> {
    console.log(`\n[AUTH] ========== AUTHENTICATION START ==========`);
    console.log(`[AUTH] PIN: ${pin}`);
    
    try {
      const command = `PIN ${pin}`;
      const sent = await this.sendCommand(command);
      
      if (!sent) {
        console.log(`[AUTH] ✗ Failed to send PIN command`);
        return false;
      }
      
      console.log(`[AUTH] Waiting for AUTH_OK response (5s timeout)...`);
      const result = await this.waitForNotification('AUTH_OK', 5000);
      
      if (result) {
        this.isAuthenticated = true;
        console.log(`[AUTH] ✓ AUTHENTICATED`);
        console.log(`[AUTH] =======================================\n`);
        return true;
      } else {
        console.log(`[AUTH] ✗ No response or timeout`);
        console.log(`[AUTH] =======================================\n`);
        return false;
      }
    } catch (error) {
      console.error(`[AUTH] ✗ Error:`, error);
      console.log(`[AUTH] =======================================\n`);
      return false;
    }
  }

  /**
   * Get firmware MD5 from device
   */
  async getFirmwareMD5(): Promise<string | null> {
    console.log(`\n[FW_MD5] ========== REQUESTING DEVICE MD5 ==========`);
    
    if (!this.isAuthenticated) {
      console.log(`[FW_MD5] ✗ Not authenticated!`);
      return null;
    }

    try {
      const startTime = Date.now();
      console.log(`[FW_MD5] Sending FW_MD5 command...`);
      
      const sent = await this.sendCommand('FW_MD5');
      if (!sent) {
        console.log(`[FW_MD5] ✗ Failed to send command`);
        return null;
      }
      
      console.log(`[FW_MD5] Waiting for response (10s timeout)...`);
      const response = await this.waitForNotification('FW_MD5', 10000);
      
      if (response) {
        const elapsed = Date.now() - startTime;
        console.log(`[FW_MD5] ✓ Response received in ${elapsed}ms`);
        console.log(`[FW_MD5] Full response: "${response}"`);
        
        // Parse: "FW_MD5 <hash>"
        const parts = response.split(' ');
        if (parts.length >= 2) {
          const md5 = parts[1];
          console.log(`[FW_MD5] Extracted MD5: ${md5}`);
          console.log(`[FW_MD5] ==========================================\n`);
          return md5;
        }
      } else {
        console.log(`[FW_MD5] ✗ Timeout or no response`);
      }
      
      console.log(`[FW_MD5] ==========================================\n`);
      return null;
    } catch (error) {
      console.error(`[FW_MD5] ✗ Error:`, error);
      console.log(`[FW_MD5] ==========================================\n`);
      return null;
    }
  }

  /**
   * Check for firmware updates
   */
  async checkForUpdates(serverMD5Url: string): Promise<boolean> {
    console.log(`\n[UPDATE] ========== CHECKING FOR UPDATES ==========`);
    console.log(`[UPDATE] Server MD5 URL: ${serverMD5Url}`);
    
    try {
      // Step 1: Get server MD5
      console.log(`[UPDATE] Step 1: Fetching server MD5...`);
      const serverResponse = await fetch(serverMD5Url, {
        timeout: 5000
      });
      const serverMD5 = (await serverResponse.text()).trim();
      console.log(`[UPDATE] Server MD5: ${serverMD5}`);
      
      // Step 2: Get device MD5
      console.log(`[UPDATE] Step 2: Getting device MD5...`);
      const deviceMD5 = await this.getFirmwareMD5();
      if (!deviceMD5) {
        console.log(`[UPDATE] ✗ Failed to get device MD5`);
        console.log(`[UPDATE] ==========================================\n`);
        return false;
      }
      
      // Step 3: Compare
      console.log(`[UPDATE] Step 3: Comparing hashes...`);
      console.log(`[UPDATE]   Server: ${serverMD5}`);
      console.log(`[UPDATE]   Device: ${deviceMD5}`);
      
      if (serverMD5 === deviceMD5) {
        console.log(`[UPDATE] ✓ FIRMWARE IS UP-TO-DATE`);
        console.log(`[UPDATE] ==========================================\n`);
        return false;
      } else {
        console.log(`[UPDATE] ⚠ NEW FIRMWARE AVAILABLE`);
        console.log(`[UPDATE] ==========================================\n`);
        return true;
      }
    } catch (error) {
      console.error(`[UPDATE] ✗ Error:`, error);
      console.log(`[UPDATE] ==========================================\n`);
      return false;
    }
  }

  /**
   * Wait for specific notification
   */
  private waitForNotification(
    expectedPrefix: string,
    timeoutMs: number
  ): Promise<string | null> {
    return new Promise((resolve) => {
      const timeout = setTimeout(() => {
        console.log(`[BLE] ✗ Timeout waiting for "${expectedPrefix}" (${timeoutMs}ms)`);
        this.notificationListeners.delete(expectedPrefix);
        resolve(null);
      }, timeoutMs);

      this.notificationListeners.set(expectedPrefix, (value: string) => {
        clearTimeout(timeout);
        this.notificationListeners.delete(expectedPrefix);
        resolve(value);
      });
    });
  }

  /**
   * Disconnect
   */
  async disconnect(): Promise<void> {
    console.log(`\n[BLE] Disconnecting...`);
    try {
      await BleManager.disconnect(this.deviceId);
      this.isAuthenticated = false;
      this.notificationListeners.clear();
      console.log(`[BLE] ✓ Disconnected\n`);
    } catch (error) {
      console.error(`[BLE] Disconnect error:`, error);
    }
  }
}
