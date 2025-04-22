#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_HMC5883_U.h>

TFT_eSPI tft = TFT_eSPI(); // TFT instance
TFT_eSprite sprite = TFT_eSprite(&tft); // DMA-accelerated sprite

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Display constants
const int16_t centerX = 60;
const int16_t centerY = 64;
const int16_t radius = 50;
const int16_t triSize = 6;

const int16_t offset = 290;

// Smoothing
#define AVERAGE_WINDOW 10
float headingBuffer[AVERAGE_WINDOW];
int headingIndex = 0;

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

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!mag.begin()) {
    Serial.println("Could not find a valid HMC5883L sensor.");
    while (1);
  }

  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);

  sprite.setColorDepth(8);
  sprite.createSprite(160, 128);
  sprite.fillSprite(TFT_BLACK);
}

void loop() {
  sensors_event_t event;
  mag.getEvent(&event);

  float heading = atan2(event.magnetic.y, event.magnetic.x) * 180.0 / PI;
  if (heading < 0) heading += 360;

  // Adjusting for the offset of 220 degrees (North = 220°)
  heading = 360 - (heading + offset); 
  if (heading < 0) 
    heading += 360; // Ensuring heading stays between 0-360

  float smoothed = smoothHeading(heading);
  drawCompass(smoothed);
  delay(50);
}
