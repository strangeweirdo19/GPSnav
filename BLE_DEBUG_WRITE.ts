// Debug/Test BLE Write - Add this to your app for testing

async testBLEWrite() {
  const serviceUUID = '4fafc201-1fb5-459e-8fcc-c5c9c331914b';
  const rxCharUUID = 'beb5483e-36e1-4688-b7f5-ea07361b26a8';
  const deviceId = 'Navion-HUD-EE66';

  console.log('\n=== BLE WRITE DEBUG TEST ===\n');

  try {
    // Test 1: Simple text write (NO Base64, just plain text)
    console.log('[TEST 1] Writing plain text "HELLO" to RX...');
    const bytes = [];
    for (let i = 0; i < 'HELLO'.length; i++) {
      bytes.push('HELLO'.charCodeAt(i));
    }
    console.log(`[TEST 1] Byte array: [${bytes.join(', ')}]`);
    
    await BleManager.writeWithoutResponse(
      deviceId,
      serviceUUID,
      rxCharUUID,
      bytes
    );
    console.log('[TEST 1] ✓ Write sent\n');

    // Wait 2 seconds
    await new Promise(resolve => setTimeout(resolve, 2000));

    // Test 2: Write authenticated command (PIN first)
    console.log('[TEST 2] Writing "PIN 842403"...');
    const pinCmd = 'PIN 842403';
    const pinBytes = [];
    for (let i = 0; i < pinCmd.length; i++) {
      pinBytes.push(pinCmd.charCodeAt(i));
    }
    console.log(`[TEST 2] Byte array: [${pinBytes.join(', ')}]`);
    
    await BleManager.writeWithoutResponse(
      deviceId,
      serviceUUID,
      rxCharUUID,
      pinBytes
    );
    console.log('[TEST 2] ✓ Write sent\n');

    // Wait 2 seconds
    await new Promise(resolve => setTimeout(resolve, 2000));

    // Test 3: Write FW_MD5 command
    console.log('[TEST 3] Writing "FW_MD5"...');
    const fwCmd = 'FW_MD5';
    const fwBytes = [];
    for (let i = 0; i < fwCmd.length; i++) {
      fwBytes.push(fwCmd.charCodeAt(i));
    }
    console.log(`[TEST 3] Byte array: [${fwBytes.join(', ')}]`);
    
    await BleManager.writeWithoutResponse(
      deviceId,
      serviceUUID,
      rxCharUUID,
      fwBytes
    );
    console.log('[TEST 3] ✓ Write sent\n');

    console.log('=== CHECK ESP32 SERIAL LOGS FOR RX MESSAGES ===\n');

  } catch (error) {
    console.error('[ERROR]', error);
  }
}

// Call this to debug:
// this.testBLEWrite();
