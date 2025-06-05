#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_HMC5883_U.h>
#include <PNGdec.h>
#include <vector>
#include <map>
#include <ArduinoJson.h>
#include <SD_MMC.h> // SD card via SDIO
#include "esp_heap_caps.h"

// Smoothing
#define AVERAGE_WINDOW 10
#define MAX_IMAGE_WIDTH 256

TFT_eSPI tft = TFT_eSPI();              // TFT instance
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
PNG png; // PNG decoder instance

// Global buffer
uint16_t *imgCropped = nullptr;
uint16_t *imgRotated = nullptr;
uint16_t *displayBuffer = nullptr;
uint16_t *imgBuffer = nullptr;
uint16_t *imgCache[3][3] = {nullptr}; // Initializes all 9 pointers to nullptr
uint16_t imgWidth = 256, imgHeight = 256;
uint16_t tftWidth = 160, tftHeight = 128;
uint16_t rotWidth = 128, rotHeight = 128;
uint16_t cropWidth = 182, cropHeight = 182;

float headingBuffer[AVERAGE_WINDOW];
int headingIndex = 0;
float lat = 0, lon = 0;
float currentHeading = 0;

// PNG draw callback — called for each decoded row
void pngDraw(PNGDRAW *pDraw)
{
  if (!imgBuffer)
    return;

  static uint16_t lineBuffer[MAX_IMAGE_WIDTH]; // Ensure this is large enough for your max image width

  // Decode this row to RGB565 format (big-endian matches ESP32/TFT_eSPI)
  png.getLineAsRGB565(pDraw, lineBuffer, PNG_RGB565_LITTLE_ENDIAN, 0xFFFFFFFF);

  // Copy decoded RGB565 line to the destination image buffer
  uint16_t *dest = imgBuffer + (pDraw->y * imgWidth);
  memcpy(dest, lineBuffer, pDraw->iWidth * sizeof(uint16_t));
}
// Decode PNG in memory
bool decodePNGToBuffer(uint8_t *pngData, uint32_t pngSize)
{
  if (imgBuffer)
  {
    memset(imgBuffer, TFT_BLACK, imgWidth * imgHeight * sizeof(uint16_t));
  }

  int rc = png.openRAM(pngData, pngSize, pngDraw);
  if (rc != PNG_SUCCESS)
  {
    Serial.println("Failed to open PNG");
    return false;
  }

  imgWidth = png.getWidth();
  imgHeight = png.getHeight();

  rc = png.decode(NULL, 0);
  png.close();

  if (rc != PNG_SUCCESS)
  {
    Serial.println("PNG decode error");
    return false;
  }

  return true;
}

void renderTileFromBin(int x, int y, int countX)
{
  const int tileYs[3] = {y - 1, y, y + 1};
  int countY = 1;
  std::map<int, std::vector<int>> binToTiles;

  // Map each needed tile to its corresponding bin
  for (int i = 0; i < 3; i++)
  {
    int tileY = tileYs[i];
    if (tileY < 0)
      continue; // Skip invalid tiles

    int binBaseY = (tileY / 25) * 25;
    binToTiles[binBaseY].push_back(tileY);
  }

  // Load each bin only once and extract all needed tiles
  for (auto &[binBaseY, tileList] : binToTiles)
  {
    String binPath = "/" + String(x) + "/" + String(binBaseY) + ".bin";

    File binFile = SD_MMC.open(binPath, FILE_READ);
    if (!binFile)
    {
      Serial.println("Failed to open bin file.");
      continue;
    }

    uint16_t version;
    binFile.read((uint8_t *)&version, 2);
    if (version != 0x0001)
    {
      Serial.println("Unsupported bin version!");
      binFile.close();
      continue;
    }

    uint32_t indexSize;
    binFile.read((uint8_t *)&indexSize, 4);

    std::vector<char> indexData(indexSize + 1, 0);
    binFile.read((uint8_t *)indexData.data(), indexSize);

    StaticJsonDocument<4096> doc; // Increase size if needed
    if (deserializeJson(doc, indexData.data()))
    {
      Serial.println("Failed to parse bin index.");
      binFile.close();
      continue;
    }

    JsonArray indexArray = doc.as<JsonArray>();

    // Lookup table for quick access
    std::map<String, std::pair<uint32_t, uint32_t>> tileInfoMap;
    for (JsonObject file : indexArray)
    {
      String filename = file["filename"].as<String>();
      uint32_t offset = file["offset"];
      uint32_t size = file["size"];
      tileInfoMap[filename] = {offset, size};
    }

    // Load each tile in this bin
    for (int tileY : tileList)
    {
      String filename = String(tileY) + ".png";
      uint32_t offset = tileInfoMap[filename].first;
      uint32_t size = tileInfoMap[filename].second;

      if (!binFile.seek(2 + 4 + indexSize + offset) | !tileInfoMap.count(filename))
      {
        Serial.println("Failed to seek to tile: " + filename);
        memset(imgCache[countX - 1][countY - 1], TFT_BLACK, imgWidth * imgHeight * sizeof(uint16_t));
        countY++;
        continue;
      }

      std::vector<uint8_t> pngData(size);
      binFile.read(pngData.data(), size);
      if (decodePNGToBuffer(pngData.data(), size))
      {
      }
      else
      {
        Serial.println("Failed to decode: " + filename);
      }
      memcpy(imgCache[countY - 1][countX - 1], imgBuffer, imgWidth * imgHeight * sizeof(uint16_t));
      countY++;
      yield();       // Allows background tasks to run
      vTaskDelay(1); // Optional short delay to prevent WDT
    }

    binFile.close();
  }
}

float smoothHeading(float newHeading)
{
  headingBuffer[headingIndex] = newHeading;
  headingIndex = (headingIndex + 1) % AVERAGE_WINDOW;

  float sumSin = 0, sumCos = 0;
  for (int i = 0; i < AVERAGE_WINDOW; i++)
  {
    float rad = headingBuffer[i] * DEG_TO_RAD;
    sumSin += sin(rad);
    sumCos += cos(rad);
  }

  float avg = atan2(sumSin, sumCos) * 180.0 / PI;
  if (avg < 0)
    avg += 360;
  return avg;
}

String getCardinalDirection(int deg)
{
  const char *directions[] = {
      "N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  return directions[((deg + 22) % 360) / 45];
}

void drawTick(int angleDeg, uint16_t color, bool thick = false)
{
  float angle = (angleDeg - 90) * DEG_TO_RAD;
  int16_t x0 = centerX + radius * cos(angle);
  int16_t y0 = centerY + radius * sin(angle);
  int16_t x1 = centerX + (radius + 5) * cos(angle);
  int16_t y1 = centerY + (radius + 5) * sin(angle);
  sprite.drawLine(x0, y0, x1, y1, color);
  if (thick)
  {
    sprite.drawLine(x0 + 1, y0 + 1, x1 + 1, y1 + 1, color);
  }
}

void drawLabel(const char *text, int angleDeg, int offset, uint8_t size = 1)
{
  float angle = (angleDeg - 90) * DEG_TO_RAD;
  int16_t x = centerX + (radius - offset) * cos(angle);
  int16_t y = centerY + (radius - offset) * sin(angle);
  sprite.setTextSize(size);
  sprite.setTextDatum(MC_DATUM);
  sprite.drawString(text, x, y);
}

void drawCircle(int x, int y, int radius, uint16_t color)
{
  sprite.drawCircle(x, y, radius, color);
}

void drawCompass(float headingDeg)
{
  sprite.fillSprite(TFT_BLACK);

  // Draw ticks and labels
  for (int i = 0; i < 360; i += 10)
  {
    int rotatedAngle = (i + 360 - (int)headingDeg) % 360;
    uint16_t color = (i % 30 == 0) ? TFT_WHITE : 0x7BEF; // light gray
    if (i == 0)
      color = TFT_RED;

    bool thick = (i % 30 == 0);
    drawTick(rotatedAngle, color, thick);

    if (i % 90 == 0)
    {
      const char *label = (i == 0) ? "N" : (i == 90) ? "E"
                                       : (i == 180)  ? "S"
                                                     : "W";
      drawLabel(label, rotatedAngle, 16, 2);
      drawTick((rotatedAngle + 1) % 360, color);
      drawTick((rotatedAngle - 1 + 360) % 360, color);
    }
    else if (i % 30 == 0)
    {
      char buf[4];
      sprintf(buf, "%d", i);
      drawLabel(buf, rotatedAngle, 12, 1);
    }
  }

  // Red triangle heading pointer
  int16_t apexX = centerX + 1;
  int16_t apexY = centerY - radius + 30;
  sprite.fillTriangle(
      apexX, apexY - triSize,
      apexX - triSize, apexY + triSize,
      apexX + triSize, apexY + triSize,
      TFT_RED);

  // Display heading text
  char degText[8];
  int intHeading = round(headingDeg);
  sprintf(degText, "%d", intHeading);

  sprite.setTextSize(2);
  sprite.setTextDatum(TL_DATUM);
  sprite.drawString(degText, 115, 10); // Move the heading to the left of the circle

  // Circle after the number (small circle instead of degree symbol)
  drawCircle(115 + sprite.textWidth(degText) + 5, 10, 2, TFT_WHITE); // Draw the circle after the number

  String dir = getCardinalDirection(intHeading);
  sprite.setTextSize(2);
  sprite.drawString(dir, 125, 30);

  sprite.pushSprite(0, 0);
}

void drawSpeed(float speed)
{
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

void drawNavigator(void)
{
  sprite.fillSprite(TFT_BLACK);
  sprite.pushImage(0, 0, tftWidth, tftHeight, displayBuffer);
  sprite.pushSprite(0, 0);
}

void renderCenteredMap(int px, int py)
{
  const int TILE_SIZE = 256;
  const int MAP_SIZE = 182;

  int centerTileX = px / TILE_SIZE;
  int centerTileY = py / TILE_SIZE;

  // Helper function to perform floor division
  auto floorDiv = [](int a, int b)
  {
    return (a < 0) ? ((a - b + 1) / b) : (a / b);
  };

  for (int y = 0; y < MAP_SIZE; y++)
  {
    for (int x = 0; x < MAP_SIZE; x++)
    {
      int gx = px - MAP_SIZE / 2 + x;
      int gy = py - MAP_SIZE / 2 + y;

      // Correct tile lookup using floor division
      int tileX = floorDiv(gx, TILE_SIZE);
      int tileY = floorDiv(gy, TILE_SIZE);

      int localX = gx % TILE_SIZE;
      int localY = gy % TILE_SIZE;
      if (localX < 0)
        localX += TILE_SIZE;
      if (localY < 0)
        localY += TILE_SIZE;

      int cacheX = tileX - centerTileX + 1;
      int cacheY = tileY - centerTileY + 1;

      // Debug for top-left and bottom-right corners
      if ((x == 0 && y == 0) || (x == MAP_SIZE - 1 && y == MAP_SIZE - 1))
      {
      }

      uint16_t color = TFT_BLACK;
      if (cacheX >= 0 && cacheX < 3 && cacheY >= 0 && cacheY < 3)
      {
        if (imgCache[cacheY][cacheX] != nullptr)
        {
          color = imgCache[cacheY][cacheX][localY * TILE_SIZE + localX];
        }
      }

      imgRotated[y * cropWidth + x] = color;
    }
  }
}
// Task to continuously read the compass data on Core 0
void readCompassTask(void *parameter)
{
  while (true)
  {
    sensors_event_t event;
    mag.getEvent(&event);

    // Calculate heading from the magnetometer
    float heading = atan2(event.magnetic.y, event.magnetic.x) * 180.0 / PI;
    if (heading < 0)
      heading += 360;

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
void displayTask(void *parameter)
{
  while (true)
  {
    // Handle mode switching here (you can integrate button or input-based mode changes)
    if (mode == 0)
    {
      drawCompass(currentHeading);
    }
    else if (mode == 1)
    {
      float speed = 60.5; // Placeholder for speed value from GPS
      drawSpeed(speed);
    }
    else if (mode == 2)
    {
      drawNavigator();
    }

    delay(50); // Adjust display refresh rate
  }
}

void getTileAndOffsetFromLatLon(int *tileX, int *tileY, int *px, int *py)
{
  const int mapSize = 256 << 16; // 256 * 2^16 = 16,777,216

  // Convert lon to normalized X
  double x = (lon + 180.0) / 360.0;

  // Convert lat to Mercator Y
  double sinLat = sin(lat * DEG_TO_RAD);
  double y = 0.5 - log((1 + sinLat) / (1 - sinLat)) / (4.0 * PI);

  // Convert to global pixel coordinates at zoom 16
  int pixelX = (int)(x * mapSize);
  int pixelY = (int)(y * mapSize);

  // Get tile indices and pixel offset within the tile
  *tileX = pixelX / 256;
  *tileY = pixelY / 256;
  *px = pixelX % 256;
  *py = pixelY % 256;
}
// Assumes imgRotated is 182x182, imgCropped is 128x128, angle is in degrees
void rotateMap(float angleDeg)
{
  const int SRC_SIZE = 182;
  const int DST_SIZE = 128;

  const float angleRad = angleDeg * 3.14159265f / 180.0f;
  const float cosA = cosf(angleRad);
  const float sinA = sinf(angleRad);

  // Center of source and destination images
  const float srcCenter = SRC_SIZE / 2.0f;
  const float dstCenter = DST_SIZE / 2.0f;

  for (int y = 0; y < DST_SIZE; y++)
  {
    for (int x = 0; x < DST_SIZE; x++)
    {
      // Destination coordinates relative to center
      float dx = x - dstCenter;
      float dy = y - dstCenter;

      // Rotate coordinates (inverse rotation)
      float srcX = dx * cosA + dy * sinA + srcCenter;
      float srcY = -dx * sinA + dy * cosA + srcCenter;

      int sx = roundf(srcX);
      int sy = roundf(srcY);

      uint16_t color = TFT_BLACK; // fallback color
      if (sx >= 0 && sx < SRC_SIZE && sy >= 0 && sy < SRC_SIZE)
      {
        color = imgRotated[sy * SRC_SIZE + sx];
      }

      imgCropped[y * DST_SIZE + x] = color;
    }
  }
}
// Copies 128x128 imgCropped into 128x160 displayBuffer with 32-pixel black strip on top
void copyToDisplayBuffer()
{
  if (!imgCropped || !displayBuffer) return;

  const int stripWidth = 32;
  const int mapWidth = rotWidth;   // 128
  const int mapHeight = rotHeight; // 128

  // Fill left strip (32 pixels wide) with black
  for (int y = 0; y < mapHeight; y++) {
    for (int x = 0; x < stripWidth; x++) {
      displayBuffer[y * tftWidth + x] = TFT_BLACK;
    }
  }

  // Copy 128x128 cropped map into the right side
  for (int y = 0; y < mapHeight; y++) {
    for (int x = 0; x < mapWidth; x++) {
      displayBuffer[y * tftWidth + (x + stripWidth)] = imgCropped[y * mapWidth + x];
    }
  }
}

void handleGPS(void)
{
  unsigned long startTime = millis(); // Start timing
  // Convert to tile coordinates
  int tileX, tileY, px, py;
  getTileAndOffsetFromLatLon(&tileX, &tileY, &px, &py);

  // Render the tile from the .bin file
  for (int dx = -1; dx <= 1; dx++)
  {
    renderTileFromBin(tileX + dx, tileY, dx + 2); // index: 1 for -1, 2 for 0, 3 for +1
    yield();                                      // Allows background tasks to run
    vTaskDelay(1);                                // Optional short delay to prevent WDT
  }
  // 12.788129, 80.222655
  renderCenteredMap(px, py);
  rotateMap(currentHeading);
  copyToDisplayBuffer();
  unsigned long endTime = millis(); // End timing
  Serial.printf("handleGPS() took %lu ms\n", endTime - startTime);
  Serial.printf("GPS Coordinates: %.6f, %.6f\n", lat, lon);
  Serial.printf("Tile Coordinates: X=%d, Y=%d\n", tileX, tileY);
}

void setup()
{
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
  displayBuffer = (uint16_t *)ps_malloc(tftWidth * tftHeight * sizeof(uint16_t));
  imgCropped = (uint16_t *)ps_malloc(cropWidth * cropHeight * sizeof(uint16_t));
  imgRotated = (uint16_t *)ps_malloc(rotWidth * rotHeight * sizeof(uint16_t));
  imgBuffer = (uint16_t *)ps_malloc(imgWidth * imgHeight * sizeof(uint16_t));
  // Allocate PSRAM for tile buffers
  for (int y = 0; y < 3; y++)
  {
    for (int x = 0; x < 3; x++)
    {
      imgCache[x][y] = (uint16_t *)ps_malloc(256 * 256 * sizeof(uint16_t));
      if (!imgCache[x][y])
      {
        Serial.printf("Allocation failed for tile [%d][%d]\n", y, x);
      }
    }
  }

  // Initialize Compass
  if (!mag.begin())
  {
    Serial.println("Failed to initialize HMC5883L!");
    while (1)
      delay(10);
  }

  // Initialize SD Card
  if (!SD_MMC.begin())
  {
    Serial.println("SD_MMC initialization failed!");
    while (1)
      delay(10);
  }
  else
  {
    Serial.println("SD_MMC initialized.");
  }
  if (psramFound())
  {
    Serial.printf("Total PSRAM: %u bytes\n", ESP.getPsramSize());
  }
  else
  {
    Serial.println("PSRAM not available.");
  }
  // Create Tasks
  xTaskCreatePinnedToCore(readCompassTask, "CompassTask", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(displayTask, "DisplayTask", 4096, NULL, 1, NULL, 1);

  // Initial fill for heading buffer
  for (int i = 0; i < AVERAGE_WINDOW; i++)
  {
    headingBuffer[i] = 0;
  }
}

void loop()
{
  if (Serial.available())
  {
    String input = Serial.readStringUntil('\n'); // Read the input string

    // Extract latitude and longitude
    int commaIndex = input.indexOf(',');
    if (commaIndex != -1)
    {
      lat = input.substring(0, commaIndex).toFloat();
      lon = input.substring(commaIndex + 1).toFloat();
      handleGPS();
      mode = 2;
    }
    else if (input.equalsIgnoreCase("reset"))
    {
      Serial.println("Resetting ESP32...");
      delay(100); // Allow message to flush
      ESP.restart();
    }

    else
    {
      Serial.println("Invalid input format. Please enter coordinates in the format 'lat, lon'");
    }
  }
}
