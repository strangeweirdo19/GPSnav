// boot_screen.h
// Simple boot screen for ESP32
#ifndef BOOT_SCREEN_H
#define BOOT_SCREEN_H

#include <TFT_eSPI.h>

class BootScreen {
private:
    TFT_eSPI& tft;
    int screenW;
    int screenH;
    int currentProgress;
    int maxProgress;
    
    const uint16_t BG_COLOR = TFT_BLACK;
    const uint16_t TEXT_COLOR = TFT_WHITE;
    const uint16_t PROGRESS_BG = 0x2945;
    const uint16_t PROGRESS_FG = TFT_WHITE;
    
    // Progress bar geometry (computed at runtime for proper centering)
    int barX = 0;
    int barY = 0;
    int barW = 0;
    int barH = 0;
    int barRadius = 0;
    
public:
    BootScreen(TFT_eSPI& display, int w, int h, int totalSteps) 
        : tft(display), screenW(w), screenH(h), currentProgress(0), maxProgress(totalSteps) {
    }
    
    void show() {
        tft.fillScreen(BG_COLOR);
        // Draw "NAVION" text (centered) and compute progress bar placement
        tft.setTextColor(TEXT_COLOR, BG_COLOR);
        tft.setTextDatum(MC_DATUM);
        tft.setTextSize(2);
        const String title = "NAVION";

        // Draw title and position bar a fixed distance below it
        tft.drawString(title, screenW / 2, screenH / 2 - 8);

        // Compute bar geometry: width scales with screen, leaves margins
        barW = screenW - 80;
        if (barW < 120) barW = 120;
        if (barW > screenW - 40) barW = screenW - 40;
        barH = screenH / 40; // reasonable thickness for most displays
        if (barH < 6) barH = 6;
        if (barH > 18) barH = 18;
        barRadius = barH / 2;
        barX = (screenW - barW) / 2;
        // place the bar a bit below the title
        barY = (screenH / 2 - 8) + 20;

        // Draw empty progress bar background (rounded)
        tft.fillRoundRect(barX, barY, barW, barH, barRadius, PROGRESS_BG);
    }
    
    void updateProgress(int step) {
        currentProgress = step;
        // Ensure geometry is valid (in case updateProgress is called before show)
        if (barW <= 0 || barH <= 0) {
            // Fallback to sensible defaults
            barW = (screenW > 0) ? max(120, screenW - 80) : 100;
            barH = max(6, screenH / 40);
            barRadius = barH / 2;
            barX = (screenW - barW) / 2;
            barY = screenH / 2 + 24;
        }

        float percentage = (float)currentProgress / (float)maxProgress;
        if (percentage < 0.0f) percentage = 0.0f;
        if (percentage > 1.0f) percentage = 1.0f;
        int fillWidth = (int)(barW * percentage);

        // Redraw background to clear prior fill, then draw the new fill portion
        tft.fillRoundRect(barX, barY, barW, barH, barRadius, PROGRESS_BG);
        if (fillWidth > 0) {
            tft.fillRoundRect(barX, barY, fillWidth, barH, barRadius, PROGRESS_FG);
        }
    }
    
    void complete() {
        updateProgress(maxProgress);
        delay(200);
        // Don't clear screen - render task will handle it after tiles load
    }
};

#endif // BOOT_SCREEN_H
