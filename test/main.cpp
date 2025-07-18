/*
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
uint16_t *imgCache[3][3] = {nullptr};   // Initializes all 9 pointers to nullptr
int tileCoords[3][3][2] = {{{-1, -1}}}; // To track (tileX, tileY) for each cache slot
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
  const int TILE_COUNT_Y = 3;
  const int tileYs[TILE_COUNT_Y] = { y - 1, y, y + 1 };
  const int cacheX = countX - 1;

  // 1. Map of binBaseY → list of tileYs
  std::map<int, std::vector<int>> binToTiles;

  // 2. Needed tiles for this column
  std::vector<int> neededTileYs;

  for (int i = 0; i < TILE_COUNT_Y; i++)
  {
    int tileY = tileYs[i];
    int relativeY = tileY - (y - 1); // 0, 1, 2
    if (tileY < 0 || relativeY < 0 || relativeY >= 3)
      continue;

    bool alreadyInCorrectSlot = (tileCoords[relativeY][cacheX][0] == x && tileCoords[relativeY][cacheX][1] == tileY);
    if (alreadyInCorrectSlot)
      continue; // ✅ already in place

    // 3. Check if this tile exists elsewhere in the grid
    bool foundElsewhere = false;
    for (int sy = 0; sy < 3 && !foundElsewhere; sy++)
    {
      for (int sx = 0; sx < 3 && !foundElsewhere; sx++)
      {
        if (tileCoords[sy][sx][0] == x && tileCoords[sy][sx][1] == tileY)
        {
          // 🔁 Copy tile from [sy][sx] to [relativeY][cacheX]
          memcpy(imgCache[relativeY][cacheX], imgCache[sy][sx], imgWidth * imgHeight * sizeof(uint16_t));
          tileCoords[relativeY][cacheX][0] = x;
          tileCoords[relativeY][cacheX][1] = tileY;
          foundElsewhere = true;
        }
      }
    }

    if (!foundElsewhere)
    {
      // Not loaded yet → need to decode
      neededTileYs.push_back(tileY);
      int binBaseY = (tileY / 25) * 25;
      binToTiles[binBaseY].push_back(tileY);
    }
  }

  // 4. Load tiles from bins
  for (auto &[binBaseY, tileList] : binToTiles)
  {
    String binPath = "/" + String(x) + "/" + String(binBaseY) + ".bin";
    File binFile = SD_MMC.open(binPath, FILE_READ);
    if (!binFile)
    {
      Serial.println("❌ Failed to open bin: " + binPath);
      // fill black for all tileYs in this bin
      for (int tileY : tileList)
      {
        int relativeY = tileY - (y - 1);
        memset(imgCache[relativeY][cacheX], 0, imgWidth * imgHeight* sizeof(uint16_t));
        tileCoords[relativeY][cacheX][0] = -1;
        tileCoords[relativeY][cacheX][1] = -1;
      }
      continue;
    }

    uint16_t version;
    binFile.read((uint8_t *)&version, 2);
    if (version != 0x0001)
    {
      Serial.println("❌ Unsupported bin version!");
      binFile.close();
      continue;
    }

    uint32_t indexSize;
    binFile.read((uint8_t *)&indexSize, 4);
    std::vector<char> indexData(indexSize + 1, 0);
    binFile.read((uint8_t *)indexData.data(), indexSize);

    StaticJsonDocument<4096> doc;
    if (deserializeJson(doc, indexData.data()))
    {
      Serial.println("❌ Failed to parse bin index.");
      binFile.close();
      continue;
    }

    JsonArray indexArray = doc.as<JsonArray>();
    std::map<String, std::pair<uint32_t, uint32_t>> tileInfoMap;
    for (JsonObject file : indexArray)
    {
      String filename = file["filename"].as<String>();
      uint32_t offset = file["offset"];
      uint32_t size = file["size"];
      tileInfoMap[filename] = {offset, size};
    }

    for (int tileY : tileList)
    {
      int relativeY = tileY - (y - 1);
      if (relativeY < 0 || relativeY > 2) continue;

      String filename = String(tileY) + ".png";
      if (!tileInfoMap.count(filename))
      {
        Serial.println("❌ Tile not in index: " + filename);
        memset(imgCache[relativeY][cacheX], 0, imgWidth * imgHeight * sizeof(uint16_t));
        tileCoords[relativeY][cacheX][0] = -1;
        tileCoords[relativeY][cacheX][1] = -1;
        continue;
      }

      auto [offset, size] = tileInfoMap[filename];
      if (!binFile.seek(2 + 4 + indexSize + offset) || size == 0)
      {
        Serial.println("❌ Seek/size error for tile: " + filename);
        memset(imgCache[relativeY][cacheX], 0, imgWidth * imgHeight * sizeof(uint16_t));
        tileCoords[relativeY][cacheX][0] = -1;
        tileCoords[relativeY][cacheX][1] = -1;
        continue;
      }

      std::vector<uint8_t> pngData(size);
      binFile.read(pngData.data(), size);

      if (decodePNGToBuffer(pngData.data(), size))
      {
        memcpy(imgCache[relativeY][cacheX], imgBuffer, imgWidth * imgHeight * sizeof(uint16_t));
        tileCoords[relativeY][cacheX][0] = x;
        tileCoords[relativeY][cacheX][1] = tileY;
      }
      else
      {
        Serial.println("❌ Decode failed: " + filename);
        memset(imgCache[relativeY][cacheX], 0, imgWidth * imgHeight * sizeof(uint16_t));
        tileCoords[relativeY][cacheX][0] = -1;
        tileCoords[relativeY][cacheX][1] = -1;
      }

      yield();
      vTaskDelay(1);
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
  constexpr int TILE_SIZE = 256;
  constexpr int CROP_SIZE = 182;  // cropWidth == cropHeight
  constexpr int CENTER_OFFSET = CROP_SIZE / 2;

  int centerTileX = px / TILE_SIZE;
  int centerTileY = py / TILE_SIZE;

  // Precompute horizontal lookups
  int tileXLookup[CROP_SIZE];
  int localXLookup[CROP_SIZE];
  int cacheXLookup[CROP_SIZE];

  for (int x = 0; x < CROP_SIZE; ++x)
  {
    int gx = px - CENTER_OFFSET + x;
    int tileX = (gx < 0) ? ((gx - TILE_SIZE + 1) / TILE_SIZE) : (gx / TILE_SIZE);
    int localX = gx % TILE_SIZE;
    if (localX < 0) localX += TILE_SIZE;
    int cacheX = tileX - centerTileX + 1;

    tileXLookup[x] = tileX;
    localXLookup[x] = localX;
    cacheXLookup[x] = cacheX;
  }

  for (int y = 0; y < CROP_SIZE; ++y)
  {
    int gy = py - CENTER_OFFSET + y;
    int tileY = (gy < 0) ? ((gy - TILE_SIZE + 1) / TILE_SIZE) : (gy / TILE_SIZE);
    int localY = gy % TILE_SIZE;
    if (localY < 0) localY += TILE_SIZE;

    int cacheY = tileY - centerTileY + 1;

    uint16_t* dstRow = &imgRotated[y * CROP_SIZE];

    if (cacheY < 0 || cacheY >= 3)
    {
      memset(dstRow, 0, CROP_SIZE * sizeof(uint16_t));
      continue;
    }

    for (int x = 0; x < CROP_SIZE; ++x)
    {
      uint16_t color = TFT_BLACK;
      int cacheX = cacheXLookup[x];

      if (cacheX >= 0 && cacheX < 3)
      {
        uint16_t* tile = imgCache[cacheY][cacheX];
        if (tile)
        {
          color = tile[localY * TILE_SIZE + localXLookup[x]];
        }
      }

      dstRow[x] = color;
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
void rotateMap(void)
{
  constexpr int SRC_SIZE = 182;
  constexpr int DST_SIZE = 128;

  const float angleRad = currentHeading * 3.14159265f / 180.0f;
  const float cosA = cosf(angleRad);
  const float sinA = sinf(angleRad);
  const float srcCenter = SRC_SIZE / 2.0f;
  const float dstCenter = DST_SIZE / 2.0f;

  // Precompute dx for all x
  float dxs[DST_SIZE];
  for (int x = 0; x < DST_SIZE; ++x)
  {
    dxs[x] = x - dstCenter;
  }

  for (int y = 0; y < DST_SIZE; ++y)
  {
    float dy = y - dstCenter;

    float sinA_dy = dy * sinA;
    float cosA_dy = dy * cosA;

    for (int x = 0; x < DST_SIZE; ++x)
    {
      float dx = dxs[x];

      float srcX = dx * cosA + sinA_dy + srcCenter;
      float srcY = -dx * sinA + cosA_dy + srcCenter;

      // Use int cast instead of roundf for speed
      int sx = (int)(srcX + 0.5f);
      int sy = (int)(srcY + 0.5f);

      uint16_t color = TFT_BLACK;
      if (sx >= 0 && sx < SRC_SIZE && sy >= 0 && sy < SRC_SIZE)
      {
        color = imgRotated[sy * SRC_SIZE + sx];
      }

      imgCropped[y * DST_SIZE + x] = color;
    }
  }
}
// Copies 128x128 imgCropped into 128x160 displayBuffer with 32-pixel black strip on top
void copyToDisplayBuffer(void)
{
  if (!imgCropped || !displayBuffer)
    return;

  const int stripWidth = tftWidth - rotWidth; // e.g., 160 - 128 = 32
  const int totalWidth = tftWidth;
  const int mapWidth = rotWidth;
  const int mapHeight = rotHeight;

  // Static buffer reused for every row (left strip = black, right = map)
  static uint16_t rowBuffer[160] = {0};  // ensure it's large enough

  for (int y = 0; y < mapHeight; y++)
  {
    // Copy cropped map (128 pixels) into right side of rowBuffer
    memcpy(&rowBuffer[stripWidth], &imgCropped[y * mapWidth], mapWidth * sizeof(uint16_t));

    // Copy entire rowBuffer to displayBuffer
    memcpy(&displayBuffer[y * totalWidth], rowBuffer, totalWidth * sizeof(uint16_t));
  }
}

void handleGPS(void *parameter)
{
  while (true)
  {
    if (lat != 0 && lon != 0)
    {
      unsigned long overallStart = millis();

      unsigned long t0 = millis();
      int tileX, tileY, px, py;
      getTileAndOffsetFromLatLon(&tileX, &tileY, &px, &py);
      unsigned long t1 = millis();
      Serial.printf("⏱ getTileAndOffsetFromLatLon() took %lu ms\n", t1 - t0);

      t0 = millis();
      for (int dx = -1; dx <= 1; dx++)
      {
        renderTileFromBin(tileX + dx, tileY, dx + 2); // dx: -1, 0, 1 => index: 1, 2, 3
        yield();
        vTaskDelay(1);
      }
      t1 = millis();
      Serial.printf("⏱ renderTileFromBin() total took %lu ms\n", t1 - t0);

      t0 = millis();
      renderCenteredMap(px, py);
      t1 = millis();
      Serial.printf("⏱ renderCenteredMap() took %lu ms\n", t1 - t0);

      t0 = millis();
      rotateMap();
      t1 = millis();
      Serial.printf("⏱ rotateMap() took %lu ms\n", t1 - t0);

      t0 = millis();
      copyToDisplayBuffer();
      t1 = millis();
      Serial.printf("⏱ copyToDisplayBuffer() took %lu ms\n", t1 - t0);

      unsigned long overallEnd = millis();
      Serial.printf("✅ handleGPS() total time: %lu ms\n", overallEnd - overallStart);
      Serial.printf("📍 GPS Coordinates: %.6f, %.6f\n", lat, lon);
      Serial.printf("🧭 Tile Coordinates: X=%d, Y=%d\n", tileX, tileY);
    }

    yield();       // Allows background tasks to run
    vTaskDelay(1); // Optional short delay to prevent WDT
  }
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
  xTaskCreatePinnedToCore(handleGPS, "GPSTask", 4096, NULL, 1, NULL, 0);
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


*/
