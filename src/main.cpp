#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_HMC5883_U.h>
#include <PNGdec.h>
#include <vector>
#include <ArduinoJson.h>
#include <PNGdec.h>
#include <SD_MMC.h>  // SD card via SDIO


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
int imgWidth = 0;
int imgHeight = 0;
bool decodePNGToBuffer(uint8_t* pngData, uint32_t pngSize) {
  int rc = png.openRAM(pngData, pngSize, [](PNGDRAW* pDraw) {
    // Called for each line decoded
    if (!imgBuffer) return; // No buffer available

    uint16_t* lineBuf = (uint16_t*)pDraw->pPixels;
    memcpy(imgBuffer + (pDraw->y * imgWidth), lineBuf, pDraw->iWidth * sizeof(uint16_t));
  });

  if (rc != PNG_SUCCESS) {
    Serial.println("PNG decode failed!");
    return false;
  }

  imgWidth = png.getWidth();
  imgHeight = png.getHeight();
  Serial.printf("Decoded PNG: %d x %d\n", imgWidth, imgHeight);

  if (imgBuffer) free(imgBuffer); // Free if already allocated

  imgBuffer = (uint16_t*)malloc(imgWidth * imgHeight * sizeof(uint16_t));
  if (!imgBuffer) {
    Serial.println("Failed to allocate buffer!");
    png.close();
    return false;
  }

  // Start decoding
  rc = png.decode(NULL, 0);
  png.close();

  if (rc != PNG_SUCCESS) {
    Serial.println("PNG decode process failed!");
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
// Smoothing
#define AVERAGE_WINDOW 10
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

void drawNavigator(String destination) {
  sprite.fillSprite(TFT_BLACK);
  sprite.setTextSize(2);
  sprite.setTextDatum(MC_DATUM);
  sprite.drawString("Navigate to:", 80, 40);
  sprite.setTextSize(3);
  sprite.drawString(destination, 80, 80);
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
      String destination = "New York"; // Placeholder for navigation destination
      drawNavigator(destination);
    }

    delay(50); // Adjust display refresh rate
  }
}

void listFiles(const char* dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = SD_MMC.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("DIR : ");
      Serial.println(file.name());
      if (levels) {
        listFiles(file.name(), levels - 1);
      }
    } else {
      Serial.print("FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
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

  if (!mag.begin()) {
    Serial.println("Could not find a valid HMC5883L sensor.");
    while (1);
  }

  if (!SD_MMC.begin()) {
    Serial.println("Card Mount Failed");
    while (1);
  }
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    while (1);
  }

  Serial.println("SD card initialized successfully.");

  listFiles("/", 5); // List everything up to 5 levels deep


  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);

  sprite.setColorDepth(8);
  sprite.createSprite(160, 128);
  sprite.fillSprite(TFT_BLACK);

  // Create tasks
  xTaskCreatePinnedToCore(readCompassTask, "ReadCompass", 10000, NULL, 1, NULL, 0); // Run on Core 0
  xTaskCreatePinnedToCore(displayTask, "Display", 10000, NULL, 1, NULL, 1); // Run on Core 1
  
}


void loop() {
  // Nothing to do in loop, as tasks are handled by FreeRTOS
}
