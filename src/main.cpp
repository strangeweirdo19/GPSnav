#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

#define TFT_DC 26
#define TFT_CS 25
#define TFT_MOSI 23
#define TFT_RST 19
#define TFT_SCLK 18
#define TFT_BACKLIGHT 5

// Select your display here:
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
// Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

float p = 3.1415926;

unsigned long minFrameTime = ULONG_MAX;
unsigned long maxFrameTime = 0;
unsigned long totalFrameTime = 0;
unsigned int frameCount = 0;

// ========== Drawing functions copied from your working code ==========

void testlines(uint16_t color) {
  tft.fillScreen(ST77XX_BLACK);
  for (int16_t x = 0; x < tft.width(); x += 6)
    tft.drawLine(0, 0, x, tft.height() - 1, color);
  for (int16_t y = 0; y < tft.height(); y += 6)
    tft.drawLine(0, 0, tft.width() - 1, y, color);
}

void testfastlines(uint16_t color1, uint16_t color2) {
  tft.fillScreen(ST77XX_BLACK);
  for (int16_t y = 0; y < tft.height(); y += 5)
    tft.drawFastHLine(0, y, tft.width(), color1);
  for (int16_t x = 0; x < tft.width(); x += 5)
    tft.drawFastVLine(x, 0, tft.height(), color2);
}

void testdrawrects(uint16_t color) {
  tft.fillScreen(ST77XX_BLACK);
  for (int16_t x = 0; x < tft.width(); x += 6)
    tft.drawRect(tft.width() / 2 - x / 2, tft.height() / 2 - x / 2, x, x, color);
}

void testfillrects(uint16_t color1, uint16_t color2) {
  tft.fillScreen(ST77XX_BLACK);
  for (int16_t x = tft.width() - 1; x > 6; x -= 6) {
    tft.fillRect(tft.width() / 2 - x / 2, tft.height() / 2 - x / 2, x, x, color1);
    tft.drawRect(tft.width() / 2 - x / 2, tft.height() / 2 - x / 2, x, x, color2);
  }
}

void testfillcircles(uint8_t radius, uint16_t color) {
  for (int16_t x = radius; x < tft.width(); x += radius * 2) {
    for (int16_t y = radius; y < tft.height(); y += radius * 2) {
      tft.fillCircle(x, y, radius, color);
    }
  }
}

void testdrawcircles(uint8_t radius, uint16_t color) {
  for (int16_t x = 0; x < tft.width() + radius; x += radius * 2) {
    for (int16_t y = 0; y < tft.height() + radius; y += radius * 2) {
      tft.drawCircle(x, y, radius, color);
    }
  }
}

void testtriangles() {
  tft.fillScreen(ST77XX_BLACK);
  uint16_t color = 0xF800;
  int t, w = tft.width() / 2;
  int x = tft.height() - 1, y = 0, z = tft.width();
  for (t = 0; t <= 15; t++) {
    tft.drawTriangle(w, y, y, x, z, x, color);
    x -= 4; y += 4; z -= 4; color += 100;
  }
}

void testroundrects() {
  tft.fillScreen(ST77XX_BLACK);
  uint16_t color = 100;
  for (int t = 0; t <= 4; t++) {
    int x = 0, y = 0;
    int w = tft.width() - 2;
    int h = tft.height() - 2;
    for (int i = 0; i <= 16; i++) {
      tft.drawRoundRect(x, y, w, h, 5, color);
      x += 2; y += 3; w -= 4; h -= 6; color += 1100;
    }
    color += 100;
  }
}

void mediabuttons() {
  tft.fillScreen(ST77XX_BLACK);
  tft.fillRoundRect(25, 10, 78, 60, 8, ST77XX_WHITE);
  tft.fillTriangle(42, 20, 42, 60, 90, 40, ST77XX_RED);
  tft.fillRoundRect(25, 90, 78, 60, 8, ST77XX_WHITE);
  tft.fillRoundRect(39, 98, 20, 45, 5, ST77XX_GREEN);
  tft.fillRoundRect(69, 98, 20, 45, 5, ST77XX_GREEN);
}


void logFrameTime(unsigned long frameTime) {
  if (frameTime < minFrameTime) minFrameTime = frameTime;
  if (frameTime > maxFrameTime) maxFrameTime = frameTime;
  totalFrameTime += frameTime;
  frameCount++;
}

void printStats() {
  Serial.println("============== Frame Stats ==============");
  Serial.print("Total Frames: "); Serial.println(frameCount);
  Serial.print("Min Frame Time: "); Serial.print(minFrameTime); Serial.println(" ms");
  Serial.print("Max Frame Time: "); Serial.print(maxFrameTime); Serial.println(" ms");
  Serial.print("Average Frame Time: "); Serial.print(totalFrameTime / frameCount); Serial.println(" ms");
  Serial.print("Estimated FPS: "); Serial.println(1000.0 / (totalFrameTime / frameCount));
  Serial.println("=========================================");
}

void testFrame() {
  unsigned long start = millis();

  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.println("Measuring frame draw time...");
  delay(100);

  testlines(ST77XX_YELLOW);
  testfastlines(ST77XX_RED, ST77XX_BLUE);
  testdrawrects(ST77XX_GREEN);
  testfillrects(ST77XX_YELLOW, ST77XX_MAGENTA);
  testfillcircles(10, ST77XX_BLUE);
  testdrawcircles(10, ST77XX_WHITE);
  testroundrects();
  testtriangles();
  mediabuttons();

  unsigned long end = millis();
  unsigned long frameTime = end - start;
  Serial.print("Frame Time: "); Serial.print(frameTime); Serial.println(" ms");
  logFrameTime(frameTime);
}

void setup(void) {
  Serial.begin(115200);
  delay(1000);
  Serial.println("TFT FPS Benchmark Starting...");

  tft.initR(INITR_BLACKTAB); // Adjust as needed for your display
  tft.setRotation(0);
  tft.fillScreen(ST77XX_BLACK);
  delay(500);
}

void loop() {
  testFrame();

  if (frameCount >= 10) { // Run for 10 frames, then report
    printStats();
    while (true); // Halt after printing
  }
}
