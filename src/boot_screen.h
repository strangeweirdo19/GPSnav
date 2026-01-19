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
    
    const int PROGRESS_BAR_WIDTH = 100;
    const int PROGRESS_BAR_HEIGHT = 2;
    
public:
    BootScreen(TFT_eSPI& display, int w, int h, int totalSteps) 
        : tft(display), screenW(w), screenH(h), currentProgress(0), maxProgress(totalSteps) {
    }
    
    void show() {
        tft.fillScreen(BG_COLOR);
        
        // Draw "NAVION" text (smaller, centered)
        tft.setTextColor(TEXT_COLOR, BG_COLOR);
        tft.setTextDatum(MC_DATUM);
        tft.setTextSize(2);
        tft.drawString("NAVION", screenW / 2, screenH / 2 - 10);
        
        // Draw empty progress bar
        int barX = (screenW - PROGRESS_BAR_WIDTH) / 2;
        int barY = screenH / 2 + 15;
        tft.fillRoundRect(barX, barY, PROGRESS_BAR_WIDTH, PROGRESS_BAR_HEIGHT, 1, PROGRESS_BG);
    }
    
    void updateProgress(int step) {
        currentProgress = step;
        
        float percentage = (float)currentProgress / (float)maxProgress;
        int fillWidth = (int)(PROGRESS_BAR_WIDTH * percentage);
        
        int barX = (screenW - PROGRESS_BAR_WIDTH) / 2;
        int barY = screenH / 2 + 15;
        
        if (fillWidth > 0) {
            tft.fillRoundRect(barX, barY, fillWidth, PROGRESS_BAR_HEIGHT, 1, PROGRESS_FG);
        }
    }
    
    void complete() {
        updateProgress(maxProgress);
        delay(200);
        // Don't clear screen - render task will handle it after tiles load
    }
};

#endif // BOOT_SCREEN_H
