#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_HMC5883_U.h>
#include <PNGdec.h>
#include <vector>
#include <ArduinoJson.h>
#include <SD_MMC.h>  // SD card via SDIO
#include "esp_heap_caps.h"

// Smoothing
#define AVERAGE_WINDOW 10
#define MAX_IMAGE_WIDTH 256

TFT_eSPI tft = TFT_eSPI(); // TFT instance
TFT_eSprite sprite = TFT_eSprite(&tft); // DMA-accelerated sprite

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Display constants
const int16_t centerX = 60;
const int16_t centerY = 64;
const int16_t radius = 50;
const int16_t triSize = 6;
const int16_t offset = 290;

// Mode control (0 = Compass, 1 = Speed, 2 = Navigator)
int mode = 0;
PNG png;                   // PNG decoder instance

// Global buffer
uint16_t* imgBuffer = nullptr;
uint16_t imgWidth = 0, imgHeight = 0;

// PNG draw callback — called for each decoded row
void pngDraw(PNGDRAW *pDraw) {
  if (!imgBuffer) return;

  static uint16_t lineBuffer[MAX_IMAGE_WIDTH];  // Ensure this is large enough for your max image width

  // Decode this row to RGB565 format (big-endian matches ESP32/TFT_eSPI)
  png.getLineAsRGB565(pDraw, lineBuffer, PNG_RGB565_LITTLE_ENDIAN, 0xFFFFFFFF);

  // Copy decoded RGB565 line to the destination image buffer
  uint16_t *dest = imgBuffer + (pDraw->y * imgWidth);
  memcpy(dest, lineBuffer, pDraw->iWidth * sizeof(uint16_t));
}

// Decode PNG in memory
bool decodePNGToBuffer(uint8_t* pngData, uint32_t pngSize) {
  if (imgBuffer) {
    free(imgBuffer);
    imgBuffer = nullptr;
  }

  int rc = png.openRAM(pngData, pngSize, pngDraw);
  if (rc != PNG_SUCCESS) {
    Serial.println("Failed to open PNG");
    return false;
  }

  imgWidth = png.getWidth();
  imgHeight = png.getHeight();
  Serial.printf("Decoded PNG: %dx%d\n", imgWidth, imgHeight);

  imgBuffer = (uint16_t*)ps_malloc(imgWidth * imgHeight * sizeof(uint16_t));
  if (!imgBuffer) {
    Serial.println("ps_malloc failed");
    png.close();
    return false;
  }

  memset(imgBuffer, 0, imgWidth * imgHeight * sizeof(uint16_t));

  rc = png.decode(NULL, 0);
  png.close();

  if (rc != PNG_SUCCESS) {
    Serial.println("PNG decode error");
    free(imgBuffer);
    imgBuffer = nullptr;
    return false;
  }

  return true;
}


void latlon_to_tile(double lat, double lon, int *x_tile, int *y_tile) {
  const int n = 65536; // 2^16
  *x_tile = (int)((lon + 180.0) / 360.0 * n);

  double lat_rad = lat * M_PI / 180.0;
  *y_tile = (int)((1.0 - log(tan(lat_rad) + 1.0 / cos(lat_rad)) / M_PI) / 2.0 * n);
}


void renderTileFromBin(int x, int y) {
  String binPath = "/" + String(x) + "/" + String((y / 25) * 25) + ".bin"; // y snapped to 25-multiple
  String targetFilename = String(y) + ".png";

  Serial.println("Loading bin file: " + binPath);
  File binFile = SD_MMC.open(binPath, FILE_READ);
  if (!binFile) {
    Serial.println("Failed to open bin file.");
    return;
  }

  uint32_t startTime = millis(); // Start measuring time

  // --- Read Version (2 bytes) ---
  uint16_t version;
  binFile.read((uint8_t*)&version, 2);
  if (version != 0x0001) {
    Serial.println("Unsupported bin version!");
    binFile.close();
    return;
  }

  // --- Read Index Size (4 bytes) ---
  uint32_t indexSize;
  binFile.read((uint8_t*)&indexSize, 4);

  // --- Read JSON Index ---
  std::vector<char> indexData(indexSize + 1, 0); // +1 for null terminator
  binFile.read((uint8_t*)indexData.data(), indexSize);
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, indexData.data());
  if (err) {
    Serial.println("Failed to parse JSON index!");
    binFile.close();
    return;
  }

  // --- Find Target Entry ---
  bool found = false;
  uint32_t fileOffset = 0;
  uint32_t fileSize = 0;
  for (JsonObject file : doc.as<JsonArray>()) {
    if (file["filename"] == targetFilename) {
      fileOffset = file["offset"];
      fileSize = file["size"];
      found = true;
      break;
    }
  }

  if (!found) {
    Serial.println("Tile not found in bin index.");
    binFile.close();
    return;
  }

  Serial.printf("Found tile at offset %lu, size %lu bytes\n", fileOffset, fileSize);

  // --- Seek and Read PNG Data ---
  if (!binFile.seek(2 + 4 + indexSize + fileOffset)) { // 2 bytes version + 4 bytes index size + index + file offset
    Serial.println("Failed to seek to PNG data!");
    binFile.close();
    return;
  }

  std::vector<uint8_t> pngData(fileSize);
  binFile.read(pngData.data(), fileSize);
  binFile.close();

  uint32_t endTime = millis();
  Serial.printf("Time to load tile: %lu ms\n", endTime - startTime);

  // --- Decode PNG into Buffer ---
  if (decodePNGToBuffer(pngData.data(), fileSize)) {
    // --- Render to TFT ---
    tft.pushImage(0, 0, imgWidth, imgHeight, imgBuffer);
    Serial.println("Tile rendered successfully.");
  } else {
    Serial.println("Failed to decode/render PNG.");
  }
}

float headingBuffer[AVERAGE_WINDOW];
int headingIndex = 0;
float currentHeading = 0;

float smoothHeading(float newHeading) {
  headingBuffer[headingIndex] = newHeading;
  headingIndex = (headingIndex + 1) % AVERAGE_WINDOW;

  float sumSin = 0, sumCos = 0;
  for (int i = 0; i < AVERAGE_WINDOW; i++) {
    float rad = headingBuffer[i] * DEG_TO_RAD;
    sumSin += sin(rad);
    sumCos += cos(rad);
  }

  float avg = atan2(sumSin, sumCos) * 180.0 / PI;
  if (avg < 0) avg += 360;
  return avg;
}

String getCardinalDirection(int deg) {
  const char* directions[] = {
    "N", "NE", "E", "SE", "S", "SW", "W", "NW"
  };
  return directions[((deg + 22) % 360) / 45];
}

void drawTick(int angleDeg, uint16_t color, bool thick = false) {
  float angle = (angleDeg - 90) * DEG_TO_RAD;
  int16_t x0 = centerX + radius * cos(angle);
  int16_t y0 = centerY + radius * sin(angle);
  int16_t x1 = centerX + (radius + 5) * cos(angle);
  int16_t y1 = centerY + (radius + 5) * sin(angle);
  sprite.drawLine(x0, y0, x1, y1, color);
  if (thick) {
    sprite.drawLine(x0 + 1, y0 + 1, x1 + 1, y1 + 1, color);
  }
}

void drawLabel(const char* text, int angleDeg, int offset, uint8_t size = 1) {
  float angle = (angleDeg - 90) * DEG_TO_RAD;
  int16_t x = centerX + (radius - offset) * cos(angle);
  int16_t y = centerY + (radius - offset) * sin(angle);
  sprite.setTextSize(size);
  sprite.setTextDatum(MC_DATUM);
  sprite.drawString(text, x, y);
}

void drawCircle(int x, int y, int radius, uint16_t color) {
  sprite.drawCircle(x, y, radius, color);
}

void drawCompass(float headingDeg) {
  sprite.fillSprite(TFT_BLACK);

  // Draw ticks and labels
  for (int i = 0; i < 360; i += 10) {
    int rotated = (i + 360 - (int)headingDeg) % 360;
    uint16_t color = (i % 30 == 0) ? TFT_WHITE : 0x7BEF; // light gray
    if (i == 0) color = TFT_RED;

    bool thick = (i % 30 == 0);
    drawTick(rotated, color, thick);

    if (i % 90 == 0) {
      const char* label = (i == 0) ? "N" : (i == 90) ? "E" : (i == 180) ? "S" : "W";
      drawLabel(label, rotated, 16, 2);
      drawTick((rotated + 1) % 360, color);
      drawTick((rotated - 1 + 360) % 360, color);
    } else if (i % 30 == 0) {
      char buf[4];
      sprintf(buf, "%d", i);
      drawLabel(buf, rotated, 12, 1);
    }
  }

  // Red triangle heading pointer
  int16_t apexX = centerX + 1;
  int16_t apexY = centerY - radius + 30;
  sprite.fillTriangle(
    apexX, apexY - triSize,
    apexX - triSize, apexY + triSize,
    apexX + triSize, apexY + triSize,
    TFT_RED
  );

  // Display heading text
  char degText[8];
  int intHeading = round(headingDeg);
  sprintf(degText, "%d", intHeading);

  sprite.setTextSize(2);
  sprite.setTextDatum(TL_DATUM);
  sprite.drawString(degText, 115, 10);  // Move the heading to the left of the circle

  // Circle after the number (small circle instead of degree symbol)
  drawCircle(115 + sprite.textWidth(degText) + 5, 10, 2, TFT_WHITE);  // Draw the circle after the number

  String dir = getCardinalDirection(intHeading);
  sprite.setTextSize(2);
  sprite.drawString(dir, 125, 30);

  sprite.pushSprite(0, 0);
}

void drawSpeed(float speed) {
  sprite.fillSprite(TFT_BLACK);
  sprite.setTextSize(2);
  sprite.setTextDatum(MC_DATUM);
  sprite.drawString("Speed: ", 80, 40);
  sprite.setTextSize(3);
  char speedText[10];
  sprintf(speedText, "%.2f km/h", speed);
  sprite.drawString(speedText, 80, 80);
  sprite.pushSprite(0, 0);
}

void drawNavigator(void) {
  sprite.fillSprite(TFT_BLACK);
  sprite.pushImage(0, 0, imgWidth, imgHeight, imgBuffer);
  sprite.pushSprite(0, 0);
}

// Task to continuously read the compass data on Core 0
void readCompassTask(void* parameter) {
  while (true) {
    sensors_event_t event;
    mag.getEvent(&event);

    // Calculate heading from the magnetometer
    float heading = atan2(event.magnetic.y, event.magnetic.x) * 180.0 / PI;
    if (heading < 0) heading += 360;

    // Adjusting for the offset of 220 degrees (North = 220°)
    heading = 360 - (heading + offset); 
    if (heading < 0) 
      heading += 360; // Ensuring heading stays between 0-360

    // Smooth the heading
    currentHeading = smoothHeading(heading);

    delay(50); // Adjust reading frequency (in ms)
  }
}

// Task to update the display based on the compass data on Core 1
void displayTask(void* parameter) {
  while (true) {
    // Handle mode switching here (you can integrate button or input-based mode changes)
    if (mode == 0) {
      drawCompass(currentHeading);
    }
    else if (mode == 1) {
      float speed = 60.5; // Placeholder for speed value from GPS
      drawSpeed(speed);
    }
    else if (mode == 2) {
      drawNavigator();
    }

    delay(50); // Adjust display refresh rate
  }
}

void latlon_to_tile_z16(double lat, double lon, int *x_tile, int *y_tile) {
    const int n = 65536; // 2^16
    *x_tile = (int)((lon + 180.0) / 360.0 * n);

    double lat_rad = lat * M_PI / 180.0;
    *y_tile = (int)((1.0 - log(tan(lat_rad) + 1.0 / cos(lat_rad)) / M_PI) / 2.0 * n);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  // Initialize TFT
  tft.init();
  tft.setRotation(1); // Landscape
  tft.fillScreen(TFT_BLACK);

  // Initialize Sprite
  sprite.setColorDepth(16);
  sprite.createSprite(160, 128); // Size matches the ST7735 display
  sprite.setSwapBytes(true);     // Required for DMA drawing

  // Initialize Compass
  if (!mag.begin()) {
    Serial.println("Failed to initialize HMC5883L!");
    while (1) delay(10);
  }

  // Initialize SD Card
  if (!SD_MMC.begin()) {
    Serial.println("SD_MMC initialization failed!");
    while (1) delay(10);
  } else {
    Serial.println("SD_MMC initialized.");
  }
  if (psramFound()) {
    Serial.printf("Total PSRAM: %u bytes\n", ESP.getPsramSize());
  } else {
    Serial.println("PSRAM not available.");
  }
  // Create Tasks
  xTaskCreatePinnedToCore(readCompassTask, "CompassTask", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(displayTask, "DisplayTask", 4096, NULL, 1, NULL, 1);

  // Initial fill for heading buffer
  for (int i = 0; i < AVERAGE_WINDOW; i++) {
    headingBuffer[i] = 0;
  }
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n'); // Read the input string

    // Extract latitude and longitude
    float lat = 0.0, lon = 0.0;
    int commaIndex = input.indexOf(',');
    if (commaIndex != -1) {
      lat = input.substring(0, commaIndex).toFloat();
      lon = input.substring(commaIndex + 1).toFloat();

      // Convert to tile coordinates
      int x_tile, y_tile;
      latlon_to_tile(lat, lon, &x_tile, &y_tile);

      // Render the tile from the .bin file
      renderTileFromBin(x_tile, y_tile);
      
      Serial.printf("GPS Coordinates: %.6f, %.6f\n", lat, lon);
      Serial.printf("Tile Coordinates: X=%d, Y=%d\n", x_tile, y_tile);
      mode = 2;
    }
    else if (input.equalsIgnoreCase("reset")) {
      Serial.println("Resetting ESP32...");
      delay(100);  // Allow message to flush
      ESP.restart();
    }

    else {
      Serial.println("Invalid input format. Please enter coordinates in the format 'lat, lon'");
    }
  }
}