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

// Triangle settings
const int16_t triSize = 6;

// Setup
void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!mag.begin()) {
    Serial.println("Could not find a valid HMC5883L sensor.");
    while (1);
  }

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  sprite.setColorDepth(8);  // Set color depth for sprite
  sprite.createSprite(128, 160); // Create sprite buffer
  sprite.fillSprite(TFT_BLACK);
  sprite.setTextDatum(MC_DATUM);
  sprite.setTextColor(TFT_WHITE, TFT_BLACK);
}

// Draw a single tick mark
void drawTick(int angleDeg, uint16_t color) {
  float angle = (angleDeg - 90) * DEG_TO_RAD;
  int16_t x0 = centerX + radius * cos(angle);
  int16_t y0 = centerY + radius * sin(angle);
  int16_t x1 = centerX + (radius + 5) * cos(angle);
  int16_t y1 = centerY + (radius + 5) * sin(angle);
  sprite.drawLine(x0, y0, x1, y1, color);
}

// Draw a text label
void drawLabel(const char* text, int angleDeg, int offset, uint8_t size = 1) {
  float angle = (angleDeg - 90) * DEG_TO_RAD;
  int16_t x = centerX + (radius - offset) * cos(angle);
  int16_t y = centerY + (radius - offset) * sin(angle);
  sprite.setTextSize(size);
  sprite.drawString(text, x, y);
}

// Draw the rotating compass
void drawCompass(float headingDeg) {
  sprite.fillSprite(TFT_BLACK);

  // Draw ticks and labels
  for (int i = 0; i < 360; i += 10) {
    int rotated = (i + 360 - (int)headingDeg) % 360;
    uint16_t color = (i % 30 == 0) ? TFT_WHITE : 0x7BEF; // Gray for minor
    if (i == 0) color = TFT_RED;  // Top line red
    drawTick(rotated, color);

    if (i % 90 == 0) {
      const char* label = (i == 0) ? "N" : (i == 90) ? "E" : (i == 180) ? "S" : "W";
      drawLabel(label, rotated, 16, 2);
      drawTick((rotated+1)%360, color);
      drawTick((rotated-1)%360, color);
    } else if (i % 30 == 0) {
      char buf[4];
      sprintf(buf, "%d", i);
      drawLabel(buf, rotated, 12, 1);
    }
  }

  // Red triangle pointing up
  int16_t apexX = centerX + 1;
  int16_t apexY = centerY - radius + 30;

  sprite.fillTriangle(
    apexX, apexY - triSize,
    apexX - triSize, apexY + triSize,
    apexX + triSize, apexY + triSize,
    TFT_RED
  );

  sprite.pushSprite(0, 0); // Fast blit with DMA
}

void loop() {
  sensors_event_t event;
  mag.getEvent(&event);

  float heading = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI;
  if (heading < 0) heading += 360;

  drawCompass(heading);
  delay(100);
}
