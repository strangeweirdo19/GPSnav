// render_task.cpp
#include "render_task.h"
#include "common.h"
#include "ble_handler.h"
#include "map_renderer.h" // Include the new map renderer header
#include "colors.h"       // Include the new colors header
#include "qr_code.h"      // Smart Startup QR Code
#include <cfloat>         // Required for FLT_MAX and FLT_MIN
#include "DFRobot_QMC5883.h"  // QMC5883L + HMC5883 compass support

// HMC5883L compass object - changed to Adafruit_HMC5883_Unified
Adafruit_HMC5883_Unified hmc5883l = Adafruit_HMC5883_Unified(12345); // Using a sensor ID, common for unified sensors

// QMC5883L compass object for clone compass support
DFRobot_QMC5883 qmc5883;

// Compass type tracking
enum CompassType {
    COMPASS_NONE = 0,
    COMPASS_HMC5883L = 1,
    COMPASS_QMC5883L = 2,
    COMPASS_HMC_COMPAT = 3,
    COMPASS_QMC_2C_OFFSET = 4,
    COMPASS_QMC6310 = 5
};
static CompassType activeCompassType = COMPASS_NONE;
static uint8_t activeCompassI2CAddress = 0;
static int16_t lastQmcX = 0;
static int16_t lastQmcY = 0;
static uint16_t sameQmcSampleCount = 0;
static int16_t lastHmcCompatX = 0;
static int16_t lastHmcCompatY = 0;
static uint16_t sameHmcCompatSampleCount = 0;
static uint16_t consecutiveCompassReadFailures = 0;
static unsigned long lastCompassRecoveryMs = 0;
static uint8_t clone2CRecoveryStep = 0;

// Compass state flag - set true only when a supported compass is found on I2C bus
static bool compassEnabled = false;

// =========================================================
// I2C BUS SCANNER - Detect all connected I2C devices
// Returns the first device address found or 0 if nothing found
// =========================================================
static uint8_t scanI2CBus(bool printAll = true) {
    uint8_t firstFound = 0;
    int deviceCount = 0;
    Serial.println("Render: Scanning I2C bus...");
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission();
        if (err == 0) {
            deviceCount++;
            if (firstFound == 0) firstFound = addr;
            if (printAll) {
                Serial.printf("Render: I2C device found at 0x%02X", addr);
                // Annotate known addresses
                if (addr == 0x1E) Serial.print(" [HMC5883L / HMC5883 Compass]");
                else if (addr == 0x0C || addr == 0x0D || addr == 0x2C) Serial.print(" [QMC5883L Clone Compass]");
                else if (addr == 0x68 || addr == 0x69) Serial.print(" [MPU6050/MPU9250 IMU]");
                else if (addr == 0x3C || addr == 0x3D) Serial.print(" [SSD1306/SH1106 OLED]");
                Serial.println();
            }
        }
    }
    if (deviceCount == 0) {
        Serial.println("Render: No I2C devices found on bus");
    } else {
        Serial.printf("Render: I2C scan complete - %d device(s) found\n", deviceCount);
    }
    return firstFound;
}

// Global variables for compass heading filtering (moving average)
// Changed from std::vector to std::array for fixed-size circular buffer
std::array<float, COMPASS_FILTER_WINDOW_SIZE> headingReadings_buffer; // Fixed-size array
size_t headingReadings_index = 0; // Current write position
bool headingReadings_full = false; // Flag to indicate buffer is filled once
float lastSmoothedRotationAngle = 0.0f; // For exponential smoothing

// Periodic compass auto-calibration (hard-iron offset only)
struct CompassAutoCalState {
    bool active = false;
    bool hasOffset = false;
    unsigned long startMs = 0;
    unsigned long nextStartMs = 0;
    int sampleCount = 0;
    float minX = FLT_MAX;
    float maxX = -FLT_MAX;
    float minY = FLT_MAX;
    float maxY = -FLT_MAX;
    float offsetX = 0.0f;
    float offsetY = 0.0f;
};

static CompassAutoCalState compassAutoCal;
static const unsigned long COMPASS_AUTO_CAL_INTERVAL_MS = 5UL * 60UL * 1000UL; // every 5 min
static const unsigned long COMPASS_AUTO_CAL_DURATION_MS = 20UL * 1000UL;        // sample for 20s
static const int COMPASS_AUTO_CAL_MIN_SAMPLES = 40;
static const float COMPASS_AUTO_CAL_MIN_SPAN_UT = 8.0f;
static float startupPhoneRotationAngle = 0.0f;

static void startCompassAutoCalibration() {
    compassAutoCal.active = true;
    compassAutoCal.startMs = millis();
    compassAutoCal.sampleCount = 0;
    compassAutoCal.minX = FLT_MAX;
    compassAutoCal.maxX = -FLT_MAX;
    compassAutoCal.minY = FLT_MAX;
    compassAutoCal.maxY = -FLT_MAX;
    Serial.println("Render: Compass auto-calibration started (rotate device slowly)");
}

static void finishCompassAutoCalibration() {
    const float spanX = compassAutoCal.maxX - compassAutoCal.minX;
    const float spanY = compassAutoCal.maxY - compassAutoCal.minY;

    if (compassAutoCal.sampleCount >= COMPASS_AUTO_CAL_MIN_SAMPLES &&
        spanX >= COMPASS_AUTO_CAL_MIN_SPAN_UT &&
        spanY >= COMPASS_AUTO_CAL_MIN_SPAN_UT) {
        compassAutoCal.offsetX = (compassAutoCal.maxX + compassAutoCal.minX) * 0.5f;
        compassAutoCal.offsetY = (compassAutoCal.maxY + compassAutoCal.minY) * 0.5f;
        compassAutoCal.hasOffset = true;
        Serial.printf("Render: Compass auto-calibration updated offsets X=%.2f Y=%.2f (samples=%d, spanX=%.2f, spanY=%.2f)\n",
                      compassAutoCal.offsetX, compassAutoCal.offsetY,
                      compassAutoCal.sampleCount, spanX, spanY);
    } else {
        Serial.printf("Render: Compass auto-calibration skipped (insufficient motion/samples: samples=%d, spanX=%.2f, spanY=%.2f)\n",
                      compassAutoCal.sampleCount, spanX, spanY);
    }

    compassAutoCal.active = false;
    compassAutoCal.nextStartMs = millis() + COMPASS_AUTO_CAL_INTERVAL_MS;
}

static float normalizeCompassAngle(float angle) {
    angle = fmodf(angle, 360.0f);
    if (angle < 0.0f) angle += 360.0f;
    return angle;
}

static float shortestCompassAngleDelta(float fromAngle, float toAngle) {
    return fmodf((toAngle - fromAngle) + 540.0f, 360.0f) - 180.0f;
}

struct CompassGpsAlignState {
    float dynamicOffsetDeg = 0.0f;
    float lastGpsHeadingDeg = 0.0f;
    float filteredErrorDeg = 0.0f;
    uint16_t trustedSampleStreak = 0;
    bool hasLastGpsHeading = false;
    unsigned long lastLogMs = 0;
};

static CompassGpsAlignState compassGpsAlign;
static const float COMPASS_GPS_ALIGN_MIN_SPEED_MS = 4.0f;
static const float COMPASS_GPS_ALIGN_MAX_TURN_RATE_DEG = 8.0f;
static const float COMPASS_GPS_ALIGN_MAX_ERROR_DEG = 60.0f;
static const float COMPASS_GPS_ALIGN_GAIN = 0.015f;
static const float COMPASS_GPS_ALIGN_MAX_STEP_DEG = 0.35f;
static const float COMPASS_GPS_ALIGN_MAX_DYNAMIC_OFFSET_DEG = 35.0f;
static const uint16_t COMPASS_GPS_ALIGN_MIN_STREAK = 8;
static const unsigned long COMPASS_GPS_ALIGN_MAX_GPS_AGE_MS = 1500UL;

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// =========================================================
// 2D KALMAN POSITION FILTER
// State: [lat, lon, vLat, vLon]  (velocities in deg/s)
// Process noise: accounts for acceleration changes
// Measurement noise: models GPS position uncertainty
// =========================================================
struct KalmanPosFilter {
    bool initialized = false;
    double lat  = 0.0;   // estimated latitude
    double lon  = 0.0;   // estimated longitude
    double vLat = 0.0;   // estimated lat velocity (deg/s)
    double vLon = 0.0;   // estimated lon velocity (deg/s)
    // Diagonal error covariance P (only diagonal tracked for speed)
    float pLat  = 1e-6f;
    float pLon  = 1e-6f;
    float pVLat = 1e-8f;
    float pVLon = 1e-8f;
    unsigned long lastMs = 0;
};

static KalmanPosFilter kalmanPos;
static const float POS_MAX_SNAP_DIST_LOW_SPEED_M  = 10.0f;
static const float POS_MAX_SNAP_DIST_HIGH_SPEED_M = 22.0f;
static const float POS_STOP_SPEED_ENTER_MS = 0.55f;
static const float POS_STOP_SPEED_EXIT_MS  = 1.20f;

// Q: process noise per second squared.
// R: GPS measurement noise variance.
static const float KAL_Q_POS  = 2e-9f;   // ~5m/s^2 max lateral accel at road scale
static const float KAL_Q_VEL  = 4e-9f;
static const float KAL_R_POS_SLOW = 8e-8f;  // ~3m GPS sigma at low speed
static const float KAL_R_POS_FAST = 3e-8f;  // ~2m GPS sigma at speed

static void applyKalmanPositionFilter(double rawLat,
                                       double rawLon,
                                       float speedMps,
                                       unsigned long nowMs,
                                       double &filteredLat,
                                       double &filteredLon) {
    if (!kalmanPos.initialized || rawLat == 0.0) {
        kalmanPos.initialized = true;
        kalmanPos.lat  = rawLat;   kalmanPos.lon  = rawLon;
        kalmanPos.vLat = 0.0;      kalmanPos.vLon = 0.0;
        kalmanPos.pLat = 1e-6f;    kalmanPos.pLon = 1e-6f;
        kalmanPos.pVLat= 1e-8f;    kalmanPos.pVLon= 1e-8f;
        kalmanPos.lastMs = nowMs;
        filteredLat = rawLat;      filteredLon = rawLon;
        return;
    }

    float dt = clampf((float)(nowMs - kalmanPos.lastMs) / 1000.0f, 0.0f, 3.0f);
    kalmanPos.lastMs = nowMs;
    if (dt < 0.005f) { filteredLat = kalmanPos.lat; filteredLon = kalmanPos.lon; return; }

    // --- Predict step ---
    double predLat  = kalmanPos.lat  + kalmanPos.vLat * dt;
    double predLon  = kalmanPos.lon  + kalmanPos.vLon * dt;
    float  predPLat = kalmanPos.pLat + kalmanPos.pVLat * dt * dt + KAL_Q_POS * dt;
    float  predPLon = kalmanPos.pLon + kalmanPos.pVLon * dt * dt + KAL_Q_POS * dt;
    float  predPVLat= kalmanPos.pVLat + KAL_Q_VEL * dt;
    float  predPVLon= kalmanPos.pVLon + KAL_Q_VEL * dt;

    // Measurement noise: tighter at speed (better SNR, straighter roads)
    float rPos = (speedMps > 5.0f) ? KAL_R_POS_FAST : KAL_R_POS_SLOW;
    // Inflate R when nearly stationary to resist GPS shimmer
    if (speedMps < 1.0f) rPos *= 4.0f;

    // --- Update step (lat) ---
    float kLat  = predPLat  / (predPLat  + rPos);
    float kVLat = predPVLat / (predPVLat + rPos);
    double innovLat = rawLat - predLat;
    kalmanPos.lat   = predLat  + kLat  * innovLat;
    kalmanPos.vLat  = kalmanPos.vLat + kVLat * innovLat / fmaxf(dt, 0.01f);
    kalmanPos.pLat  = (1.0f - kLat)  * predPLat;
    kalmanPos.pVLat = (1.0f - kVLat) * predPVLat;

    // --- Update step (lon) ---
    float kLon  = predPLon  / (predPLon  + rPos);
    float kVLon = predPVLon / (predPVLon + rPos);
    double innovLon = rawLon - predLon;
    kalmanPos.lon   = predLon  + kLon  * innovLon;
    kalmanPos.vLon  = kalmanPos.vLon + kVLon * innovLon / fmaxf(dt, 0.01f);
    kalmanPos.pLon  = (1.0f - kLon)  * predPLon;
    kalmanPos.pVLon = (1.0f - kVLon) * predPVLon;

    filteredLat = kalmanPos.lat;
    filteredLon = kalmanPos.lon;
}

struct SnapLockState {
    bool active = false;
    double lat = 0.0;
    double lon = 0.0;
    int routeIdx = 0;
    float routeFrac = 0.0f;
    unsigned long lastUpdateMs = 0;
};

static SnapLockState routeSnapLock;

static float shortestBearingDeltaDeg(float a, float b) {
    float d = fmodf((a - b) + 540.0f, 360.0f) - 180.0f;
    return fabsf(d);
}


static int lastSnappedIdx = -1;

static bool snapPositionToRouteCenterline(double queryLat,
                                          double queryLon,
                                          float headingDeg,
                                          float speedMps,
                                          double &snappedLat,
                                          double &snappedLon,
                                          float &distanceToRouteM) {
    if (activeRoute.size() < 2) return false;
    if (routeProgressIndex < 0 || routeProgressIndex >= (int)activeRoute.size()) return false;

    double routeLat = currentRouteLat;
    double routeLon = currentRouteLon;
    int centerIdx = routeProgressIndex;

    // Fallback if route progress absolute point is not yet initialized.
    if ((routeLat == 0.0 && routeLon == 0.0) || centerIdx == 0) {
        routeLat = activeRouteAnchor.lat;
        routeLon = activeRouteAnchor.lon;
        for (int i = 1; i <= centerIdx && i < (int)activeRoute.size(); ++i) {
            routeLat += activeRoute[i].dLat / ROUTE_SCALE;
            routeLon += activeRoute[i].dLon / ROUTE_SCALE;
        }
    }

    const int backWindow = (speedMps > 5.0f) ? 2 : 5;
    const int fwdWindow = 25;
    int startIdx = centerIdx - backWindow;
    if (startIdx < 0) startIdx = 0;
    int endIdx = centerIdx + fwdWindow;
    if (endIdx > (int)activeRoute.size() - 1) endIdx = (int)activeRoute.size() - 1;

    // Move routeLat/Lon back to startIdx absolute position.
    for (int i = centerIdx; i > startIdx; --i) {
        routeLat -= activeRoute[i].dLat / ROUTE_SCALE;
        routeLon -= activeRoute[i].dLon / ROUTE_SCALE;
    }

    const float cosRef = cosf((float)queryLat * (3.14159f / 180.0f));
    const float mPerDegLon = 111320.0f * ((fabsf(cosRef) < 0.01f) ? 0.01f : cosRef);
    const float mPerDegLat = 111320.0f;

    bool found = false;
    float bestScore = FLT_MAX;
    float bestDist = FLT_MAX;
    double bestLat = queryLat;
    double bestLon = queryLon;
    int bestIdx = -1;

    double aLat = routeLat;
    double aLon = routeLon;

    const float PERSISTENCE_BONUS_M = 15.0f;
    const float JUNCTION_THRESHOLD_M = 15.0f;

    for (int i = startIdx; i < endIdx; ++i) {
        double bLat = aLat + activeRoute[i + 1].dLat / ROUTE_SCALE;
        double bLon = aLon + activeRoute[i + 1].dLon / ROUTE_SCALE;

        float ax = (float)((aLon - queryLon) * mPerDegLon);
        float ay = (float)((aLat - queryLat) * mPerDegLat);
        float bx = (float)((bLon - queryLon) * mPerDegLon);
        float by = (float)((bLat - queryLat) * mPerDegLat);

        float abx = bx - ax;
        float aby = by - ay;
        float ab2 = abx * abx + aby * aby;
        float segLen = sqrtf(ab2);

        if (segLen > 0.01f) {
            float segBearing = atan2f(abx, aby) * (180.0f / 3.14159f);
            if (segBearing < 0.0f) segBearing += 360.0f;
            float hDelta = shortestBearingDeltaDeg(headingDeg, segBearing);

            // 1. Heading Gating (at driving speed)
            if (speedMps > 4.0f && hDelta > 75.0f) {
                aLat = bLat; aLon = bLon; continue;
            }

            float t = (-(ax * abx + ay * aby)) / ab2;
            t = clampf(t, 0.0f, 1.0f);

            float px = ax + t * abx;
            float py = ay + t * aby;
            float dist = sqrtf(px * px + py * py);

            // 2. Probabilistic Scoring
            float score = dist;

            // Heading penalty: quadratically increase cost for misalignment
            if (speedMps > 1.0f) {
                float hPenalty = (hDelta / 45.0f) * (hDelta / 45.0f) * 10.0f;
                score += hPenalty;
            }

            // Persistence: favor the current road segment unless jumping at a junction
            if (i == lastSnappedIdx) {
                float distToStart = t * segLen;
                float distToEnd = (1.0f - t) * segLen;
                bool nearJunction = (distToStart < JUNCTION_THRESHOLD_M || distToEnd < JUNCTION_THRESHOLD_M);
                
                if (!nearJunction) {
                    score -= PERSISTENCE_BONUS_M;
                }
            }

            // 3. Favor forward progress (don't snap behind current point at speed)
            if (speedMps > 5.0f && i < centerIdx - 1) {
                score += 30.0f;
            }

            if (score < bestScore) {
                bestScore = score;
                bestDist = dist;
                bestIdx = i;
                bestLon = queryLon + (px / mPerDegLon);
                bestLat = queryLat + (py / mPerDegLat);
                found = true;
            }
        }

        aLat = bLat;
        aLon = bLon;
    }

    if (!found) return false;

    float maxSnapDist = (speedMps < 3.0f) ? 35.0f : 20.0f;
    if (bestDist > maxSnapDist) return false;

    lastSnappedIdx = bestIdx;
    snappedLat = bestLat;
    snappedLon = bestLon;
    distanceToRouteM = bestDist;
    return true;
}

static bool writeI2CRegister(uint8_t addr, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}

static bool readI2CRegisters(uint8_t addr, uint8_t startReg, uint8_t* data, size_t len) {
    Wire.beginTransmission(addr);
    Wire.write(startReg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    size_t received = Wire.requestFrom((int)addr, (int)len);
    if (received != len) {
        return false;
    }

    for (size_t i = 0; i < len; i++) {
        data[i] = Wire.read();
    }
    return true;
}

static bool readI2CRegister(uint8_t addr, uint8_t reg, uint8_t &value) {
    return readI2CRegisters(addr, reg, &value, 1);
}

// QMC6310 register map (detected on some 0x2C boards):
// CID @0x00 = 0x80, XYZ @0x01..0x06, STATUS @0x09, CTRL1 @0x0A, CTRL2 @0x0B.
static bool initQMC6310AtAddress(uint8_t addr) {
    // Enter suspend first.
    if (!writeI2CRegister(addr, 0x0A, 0x00)) {
        return false;
    }
    delay(5);

    // Axis orientation: X north, Y west, Z up.
    writeI2CRegister(addr, 0x29, 0x06);

    // CTRL2: field-range 2G + set/reset enabled.
    if (!writeI2CRegister(addr, 0x0B, 0x0C)) {
        return false;
    }

    // CTRL1: NORMAL mode + ODR 50Hz + OSR/DSR defaults.
    if (!writeI2CRegister(addr, 0x0A, 0x15)) {
        return false;
    }

    uint8_t ctrl1 = 0;
    uint8_t ctrl2 = 0;
    if (!readI2CRegister(addr, 0x0A, ctrl1)) return false;
    if (!readI2CRegister(addr, 0x0B, ctrl2)) return false;

    // Validate that mode bits are non-zero and control register reads back.
    if ((ctrl1 & 0x03) == 0x00) {
        return false;
    }
    if ((ctrl2 & 0x0C) == 0x00) {
        return false;
    }

    return true;
}

static bool readQMC6310RawAtAddress(uint8_t addr, float &magX, float &magY) {
    uint8_t raw[6] = {0};
    if (!readI2CRegisters(addr, 0x01, raw, sizeof(raw))) {
        return false;
    }

    int16_t x = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t y = (int16_t)((raw[3] << 8) | raw[2]);

    // Reject frozen/stuck invalid frame.
    if (x == 0 && y == 0) {
        return false;
    }

    if (x == lastQmcX && y == lastQmcY) {
        if (sameQmcSampleCount < 65535) sameQmcSampleCount++;
    } else {
        sameQmcSampleCount = 0;
    }
    lastQmcX = x;
    lastQmcY = y;

    if (sameQmcSampleCount > 30) {
        // Re-enter normal mode if stream freezes.
        writeI2CRegister(addr, 0x0A, 0x00);
        delay(2);
        writeI2CRegister(addr, 0x0B, 0x0C);
        writeI2CRegister(addr, 0x0A, 0x15);
        sameQmcSampleCount = 0;
        Serial.println("Render: QMC6310 stream stuck, re-arming normal mode");
        return false;
    }

    magX = (float)x;
    magY = (float)y;
    return true;
}

// Initialize QMC5883/clone at explicit detected address (0x0C/0x0D/0x2C)
static bool initQMC5883AtAddress(uint8_t addr) {
    // 0x2C modules in the field often behave better with Mecha/QMC style init.
    if (addr == 0x2C) {
        // Soft reset first, then continuous mode (200Hz, 8G, OSR 512).
        writeI2CRegister(addr, 0x0A, 0x80);
        delay(10);
        if (!writeI2CRegister(addr, 0x0B, 0x01)) {
            return false;
        }
        if (!writeI2CRegister(addr, 0x09, 0x1D)) {
            return false;
        }
    } else {
        // DFRobot library writes identity registers before configuring QMC mode.
        // Some clones require this sequence to start producing real data.
        if (!writeI2CRegister(addr, 0x0B, 0x01)) {
            return false;
        }
        if (!writeI2CRegister(addr, 0x20, 0x40)) {
            return false;
        }
        if (!writeI2CRegister(addr, 0x21, 0x01)) {
            return false;
        }

        // Continuous mode, 50Hz ODR, 8G range, 512 oversampling (0x1D)
        if (!writeI2CRegister(addr, 0x09, 0x1D)) {
            return false;
        }
        // QMC config2 / range register: 8G range is 0x10 in the DFRobot driver.
        if (!writeI2CRegister(addr, 0x0A, 0x10)) {
            return false;
        }
    }

    // Validate that data registers are readable
    uint8_t raw[6] = {0};
    return readI2CRegisters(addr, 0x00, raw, sizeof(raw));
}

static void dumpCompassRegisters(uint8_t addr) {
    uint8_t regs[13] = {0};
    if (readI2CRegisters(addr, 0x00, regs, sizeof(regs))) {
        Serial.printf("Render: Compass reg dump @0x%02X", addr);
        for (size_t i = 0; i < sizeof(regs); i++) {
            Serial.printf(" %02X", regs[i]);
        }
        Serial.println();
    } else {
        Serial.printf("Render: Compass reg dump failed @0x%02X\n", addr);
    }
}

// Initialize HMC5883-compatible clones on non-standard addresses (e.g., 0x2C)
static bool initHMCCompatAtAddress(uint8_t addr) {
    // Config A: 8 samples averaged, 15Hz, normal measurement
    if (!writeI2CRegister(addr, 0x00, 0x70)) {
        return false;
    }
    // Config B: gain = 1.3Ga
    if (!writeI2CRegister(addr, 0x01, 0x20)) {
        return false;
    }
    // Mode: continuous conversion
    if (!writeI2CRegister(addr, 0x02, 0x00)) {
        return false;
    }

    // Verify data registers are readable
    uint8_t raw[6] = {0};
    return readI2CRegisters(addr, 0x03, raw, sizeof(raw));
}

static bool readQMC5883RawAtAddress(uint8_t addr, float &magX, float &magY) {
    uint8_t raw[6] = {0};
    if (!readI2CRegisters(addr, 0x00, raw, sizeof(raw))) {
        return false;
    }

    int16_t x = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t y = (int16_t)((raw[3] << 8) | raw[2]);

    // Some clones at 0x2C return this fixed sentinel when QMC map is wrong.
    if ((x == 128 || x == -128) && y == 0) {
        return false;
    }

    if (x == lastQmcX && y == lastQmcY) {
        if (sameQmcSampleCount < 65535) sameQmcSampleCount++;
    } else {
        sameQmcSampleCount = 0;
    }
    lastQmcX = x;
    lastQmcY = y;

    // If QMC samples are frozen for too long, force fallback to alternate map.
    if (sameQmcSampleCount > 40) {
        return false;
    }

    // Reject clearly invalid all-zero samples that cause heading to stick at 0
    if (x == 0 && y == 0) {
        return false;
    }

    magX = (float)x;
    magY = (float)y;
    return true;
}

// Some clones expose big-endian XY pairs at 0x00..0x03.
static bool readQMC5883BigEndianRawAtAddress(uint8_t addr, float &magX, float &magY) {
    uint8_t raw[6] = {0};
    if (!readI2CRegisters(addr, 0x00, raw, sizeof(raw))) {
        return false;
    }

    int16_t x = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t y = (int16_t)((raw[2] << 8) | raw[3]);

    if (x == 0 && y == 0) {
        return false;
    }

    // Reject a common stuck frame seen on incompatible maps.
    if (x == (int16_t)0x8000 && y == 0) {
        return false;
    }

    magX = (float)x;
    magY = (float)y;
    return true;
}

// Some 0x2C clones expose status at 0x00 and XYZ data from 0x01 onward.
static bool readQMC5883_2COffsetRawAtAddress(uint8_t addr, float &magX, float &magY) {
    uint8_t raw[6] = {0};
    if (!readI2CRegisters(addr, 0x01, raw, sizeof(raw))) {
        return false;
    }

    int16_t x = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t y = (int16_t)((raw[3] << 8) | raw[2]);

    if (x == 0 && y == 0) {
        return false;
    }

    // Filter obvious stuck readings that can still appear on bad register map matches.
    if ((x == 128 || x == -128) && y == 0) {
        return false;
    }

    // If this clone gets stuck outputting one frame forever, re-arm conversion mode.
    if (x == lastQmcX && y == lastQmcY) {
        if (sameQmcSampleCount < 65535) sameQmcSampleCount++;
    } else {
        sameQmcSampleCount = 0;
    }
    lastQmcX = x;
    lastQmcY = y;

    if (sameQmcSampleCount > 30) {
        writeI2CRegister(addr, 0x0A, 0x80); // soft reset
        delay(2);
        writeI2CRegister(addr, 0x0B, 0x01);
        writeI2CRegister(addr, 0x09, 0x1D);
        sameQmcSampleCount = 0;
        Serial.println("Render: 0x2C QMC-2C stream stuck, re-arming sensor mode");
        return false;
    }

    magX = (float)x;
    magY = (float)y;
    return true;
}

static bool readHMCCompatRawAtAddress(uint8_t addr, float &magX, float &magY) {
    uint8_t raw[6] = {0};
    if (!readI2CRegisters(addr, 0x03, raw, sizeof(raw))) {
        return false;
    }

    // HMC5883 register order: X_MSB, X_LSB, Z_MSB, Z_LSB, Y_MSB, Y_LSB
    int16_t x = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t y = (int16_t)((raw[4] << 8) | raw[5]);

    if (x == lastHmcCompatX && y == lastHmcCompatY) {
        if (sameHmcCompatSampleCount < 65535) sameHmcCompatSampleCount++;
    } else {
        sameHmcCompatSampleCount = 0;
    }
    lastHmcCompatX = x;
    lastHmcCompatY = y;

    // If also frozen for long periods, treat as invalid so caller can retry other map.
    if (sameHmcCompatSampleCount > 40) {
        return false;
    }

    if (x == 0 && y == 0) {
        return false;
    }

    magX = (float)x;
    magY = (float)y;
    return true;
}

static bool initQMC5883AtAddressAlt(uint8_t addr) {
    // Alternate profile used by stubborn 0x2C clones.
    if (!writeI2CRegister(addr, 0x0B, 0x01)) return false;
    if (!writeI2CRegister(addr, 0x20, 0x40)) return false;
    if (!writeI2CRegister(addr, 0x21, 0x01)) return false;
    if (!writeI2CRegister(addr, 0x09, 0x18)) return false;
    if (!writeI2CRegister(addr, 0x0A, 0x01)) return false;

    uint8_t raw[6] = {0};
    return readI2CRegisters(addr, 0x00, raw, sizeof(raw));
}

static void tryRecover2CClone() {
    if (activeCompassI2CAddress != 0x2C) return;

    unsigned long now = millis();
    if (consecutiveCompassReadFailures < 25) return;
    if (now - lastCompassRecoveryMs < 1200) return;

    lastCompassRecoveryMs = now;
    clone2CRecoveryStep = (uint8_t)((clone2CRecoveryStep + 1) % 3);

    if (clone2CRecoveryStep == 0) {
        if (initQMC5883AtAddress(0x2C)) {
            activeCompassType = COMPASS_QMC_2C_OFFSET;
            Serial.println("Render: 0x2C recovery -> QMC standard profile");
        }
    } else if (clone2CRecoveryStep == 1) {
        if (initHMCCompatAtAddress(0x2C)) {
            activeCompassType = COMPASS_HMC_COMPAT;
            Serial.println("Render: 0x2C recovery -> HMC-compatible profile");
        }
    } else {
        if (initQMC5883AtAddressAlt(0x2C)) {
            activeCompassType = COMPASS_QMC5883L;
            Serial.println("Render: 0x2C recovery -> QMC alternate profile");
        }
    }

    consecutiveCompassReadFailures = 0;
}

static void drawPixelAlphaOnSprite(TFT_eSprite &target, int x, int y, uint16_t color, float alpha) {
    if (x < 0 || y < 0 || x >= target.width() || y >= target.height()) return;
    uint16_t existingColor = target.readPixel(x, y);
    uint16_t blendedColor = blendColors(existingColor, color, alpha);
    target.drawPixel(x, y, blendedColor);
}

// Same AA thick-line approach used by road rendering, adapted for any sprite target.
static void drawThickLineAAOnSprite(TFT_eSprite &target,
                                    float x0,
                                    float y0,
                                    float x1,
                                    float y1,
                                    float width,
                                    uint16_t color) {
    float dx = x1 - x0;
    float dy = y1 - y0;
    float len = sqrtf(dx * dx + dy * dy);
    if (len <= 0.0f) return;

    float ux = width * 0.5f * (dy / len);
    float uy = width * 0.5f * (-dx / len);

    float minXf = min(min(x0 + ux, x1 + ux), min(x1 - ux, x0 - ux));
    float maxXf = max(max(x0 + ux, x1 + ux), max(x1 - ux, x0 - ux));
    float minYf = min(min(y0 + uy, y1 + uy), min(y1 - uy, y0 - uy));
    float maxYf = max(max(y0 + uy, y1 + uy), max(y1 - uy, y0 - uy));

    int minX = max(0, (int)floorf(minXf) - 1);
    int minY = max(0, (int)floorf(minYf) - 1);
    int maxX = min(target.width() - 1, (int)ceilf(maxXf) + 1);
    int maxY = min(target.height() - 1, (int)ceilf(maxYf) + 1);

    float bax = x1 - x0;
    float bay = y1 - y0;
    float lenSq = bax * bax + bay * bay;
    if (lenSq <= 0.0f) return;

    float halfWidth = width * 0.5f;

    for (int y = minY; y <= maxY; y++) {
        for (int x = minX; x <= maxX; x++) {
            float px = x + 0.5f;
            float py = y + 0.5f;

            float t = ((px - x0) * bax + (py - y0) * bay) / lenSq;
            float cx;
            float cy;
            if (t < 0.0f) {
                cx = x0;
                cy = y0;
            } else if (t > 1.0f) {
                cx = x1;
                cy = y1;
            } else {
                cx = x0 + t * bax;
                cy = y0 + t * bay;
            }

            float dist = sqrtf((px - cx) * (px - cx) + (py - cy) * (py - cy));
            float alpha = 0.0f;
            if (dist < halfWidth - 0.5f) {
                alpha = 1.0f;
            } else if (dist < halfWidth + 0.5f) {
                alpha = 1.0f - (dist - (halfWidth - 0.5f));
            }

            if (alpha > 0.0f) {
                drawPixelAlphaOnSprite(target, x, y, color, alpha);
            }
        }
    }
}

static void drawCenteredN(TFT_eSprite &target, int centerX, int centerY, int halfWidth, int halfHeight, uint16_t color) {
    const int leftX = centerX - halfWidth;
    const int rightX = centerX + halfWidth;
    const int topY = centerY - halfHeight;
    const int bottomY = centerY + halfHeight;

    drawThickLineAAOnSprite(target, (float)leftX, (float)bottomY, (float)leftX, (float)topY, 1.5f, color);
    drawThickLineAAOnSprite(target, (float)leftX, (float)topY, (float)rightX, (float)bottomY, 1.5f, color);
    drawThickLineAAOnSprite(target, (float)rightX, (float)bottomY, (float)rightX, (float)topY, 1.5f, color);
}

// Read compass data from either HMC5883L or QMC5883L
static bool readCompassData(float &magX, float &magY) {
    if (!compassEnabled) return false;
    
    if (activeCompassType == COMPASS_HMC5883L) {
        // HMC5883L requires DRDY pin check
        if (digitalRead(DRDY_PIN) != HIGH) return false;
        sensors_event_t event;
        hmc5883l.getEvent(&event);
        if (isnan(event.magnetic.x) || isnan(event.magnetic.y)) return false;
        magX = event.magnetic.x;
        magY = event.magnetic.y;
        return true;
    }
    else if (activeCompassType == COMPASS_QMC5883L) {
        // Use explicit detected I2C address to support clone variants (including 0x2C)
        if (activeCompassI2CAddress == 0) return false;

        // 0x2C clones frequently use offset map: status at 0x00, XYZ at 0x01
        if (activeCompassI2CAddress == 0x2C && readQMC5883_2COffsetRawAtAddress(activeCompassI2CAddress, magX, magY)) {
            consecutiveCompassReadFailures = 0;
            activeCompassType = COMPASS_QMC_2C_OFFSET;
            Serial.printf("Render: Compass 0x%02X switched to QMC-2C offset mode\n", activeCompassI2CAddress);
            return true;
        }

        if (readQMC5883RawAtAddress(activeCompassI2CAddress, magX, magY)) {
            consecutiveCompassReadFailures = 0;
            return true;
        }

        if (readQMC5883BigEndianRawAtAddress(activeCompassI2CAddress, magX, magY)) {
            consecutiveCompassReadFailures = 0;
            return true;
        }

        // Some 0x2C clones are HMC-compatible despite QMC-like labeling.
        if (readHMCCompatRawAtAddress(activeCompassI2CAddress, magX, magY)) {
            consecutiveCompassReadFailures = 0;
            activeCompassType = COMPASS_HMC_COMPAT;
            Serial.printf("Render: Compass 0x%02X switched to HMC-compatible raw mode\n", activeCompassI2CAddress);
            return true;
        }

        consecutiveCompassReadFailures++;
        tryRecover2CClone();
        return false;
    }
    else if (activeCompassType == COMPASS_QMC6310) {
        if (activeCompassI2CAddress == 0) return false;
        if (readQMC6310RawAtAddress(activeCompassI2CAddress, magX, magY)) {
            consecutiveCompassReadFailures = 0;
            return true;
        }

        consecutiveCompassReadFailures++;
        return false;
    }
    else if (activeCompassType == COMPASS_QMC_2C_OFFSET) {
        if (activeCompassI2CAddress == 0) return false;
        if (readQMC5883_2COffsetRawAtAddress(activeCompassI2CAddress, magX, magY)) {
            consecutiveCompassReadFailures = 0;
            return true;
        }

        // Fallback if this interpretation fails later.
        if (readQMC5883RawAtAddress(activeCompassI2CAddress, magX, magY)) {
            consecutiveCompassReadFailures = 0;
            activeCompassType = COMPASS_QMC5883L;
            Serial.printf("Render: Compass 0x%02X switched from QMC-2C offset to QMC mode\n", activeCompassI2CAddress);
            return true;
        }
        if (readQMC5883BigEndianRawAtAddress(activeCompassI2CAddress, magX, magY)) {
            consecutiveCompassReadFailures = 0;
            activeCompassType = COMPASS_QMC5883L;
            Serial.printf("Render: Compass 0x%02X switched from QMC-2C offset to QMC-BE mode\n", activeCompassI2CAddress);
            return true;
        }
        if (readHMCCompatRawAtAddress(activeCompassI2CAddress, magX, magY)) {
            consecutiveCompassReadFailures = 0;
            activeCompassType = COMPASS_HMC_COMPAT;
            Serial.printf("Render: Compass 0x%02X switched from QMC-2C offset to HMC mode\n", activeCompassI2CAddress);
            return true;
        }

        consecutiveCompassReadFailures++;
        tryRecover2CClone();
        return false;
    }
    else if (activeCompassType == COMPASS_HMC_COMPAT) {
        if (activeCompassI2CAddress == 0) return false;
        if (readHMCCompatRawAtAddress(activeCompassI2CAddress, magX, magY)) {
            consecutiveCompassReadFailures = 0;
            return true;
        }

        // If HMC-compatible read fails, retry QMC format in case chip changed behavior.
        if (readQMC5883RawAtAddress(activeCompassI2CAddress, magX, magY)) {
            consecutiveCompassReadFailures = 0;
            activeCompassType = COMPASS_QMC5883L;
            Serial.printf("Render: Compass 0x%02X switched back to QMC raw mode\n", activeCompassI2CAddress);
            return true;
        }
        if (readQMC5883BigEndianRawAtAddress(activeCompassI2CAddress, magX, magY)) {
            consecutiveCompassReadFailures = 0;
            activeCompassType = COMPASS_QMC5883L;
            Serial.printf("Render: Compass 0x%02X switched back to QMC-BE raw mode\n", activeCompassI2CAddress);
            return true;
        }

        consecutiveCompassReadFailures++;
        tryRecover2CClone();
        return false;
    }
    return false;
}

static void drawRotatedPhoneIcon(int centerX, int centerY, float headingDeg, bool showTriangle, bool showN) {
    TFT_eSprite phoneSprite(&tft);
    const int iconW = 56;
    const int iconH = 78;
    phoneSprite.setColorDepth(16);
    phoneSprite.createSprite(iconW, iconH);
    phoneSprite.fillSprite(UI_BACKGROUND_COLOR);
    phoneSprite.setTextDatum(MC_DATUM);

    const int bodyX = 10;
    const int bodyY = 6;
    const int bodyW = 36;
    const int bodyH = 62;
    const int bodyR = 7;

    phoneSprite.fillRoundRect(bodyX, bodyY, bodyW, bodyH, bodyR, UI_TEXT_COLOR);
    phoneSprite.fillRoundRect(bodyX + 2, bodyY + 2, bodyW - 4, bodyH - 4, bodyR - 1, UI_BACKGROUND_COLOR);
    phoneSprite.drawRoundRect(bodyX, bodyY, bodyW, bodyH, bodyR, UI_TEXT_COLOR);

    // Small speaker notch and home button to keep the phone shape recognizable.
    drawThickLineAAOnSprite(phoneSprite,
                            (float)(bodyX + 10),
                            (float)(bodyY + 6),
                            (float)(bodyX + bodyW - 10),
                            (float)(bodyY + 6),
                            1.3f,
                            UI_TEXT_COLOR);
    phoneSprite.drawCircle(bodyX + bodyW / 2, bodyY + bodyH - 8, 2, UI_TEXT_COLOR);

    if (showTriangle) {
        int triCx = bodyX + bodyW / 2;
        int triTop = bodyY + 14;
        phoneSprite.fillTriangle(triCx, triTop,
                                 triCx - 5, triTop + 9,
                                 triCx + 5, triTop + 9,
                                 TFT_RED);
        drawThickLineAAOnSprite(phoneSprite,
                    (float)triCx, (float)triTop,
                    (float)(triCx - 5), (float)(triTop + 9),
                    1.5f,
                    TFT_RED);
        drawThickLineAAOnSprite(phoneSprite,
                    (float)triCx, (float)triTop,
                    (float)(triCx + 5), (float)(triTop + 9),
                    1.5f,
                    TFT_RED);
    }

    if (showN) {
        drawCenteredN(phoneSprite, bodyX + bodyW / 2, bodyY + bodyH / 2 + 2, 3, 5, TFT_WHITE);
    }

    phoneSprite.setPivot(iconW / 2, iconH / 2);
    sprite.setPivot(centerX, centerY);
    phoneSprite.pushRotated(&sprite, (int16_t)roundf(headingDeg), UI_BACKGROUND_COLOR);
    sprite.setPivot(0, 0);
    phoneSprite.deleteSprite();
}


// =========================================================
// QR CODE DRAWING HELPER
// =========================================================
void drawQRCode(int x, int y) {
    // 100x100 bitmap packed as 1 bit per pixel
    int byteIdx = 0;
    int bitShift = 7;
    
    for (int r = 0; r < QR_CODE_HEIGHT; r++) {
         for (int c = 0; c < QR_CODE_WIDTH; c++) {
              uint8_t val = pgm_read_byte(&qr_code_bitmap[byteIdx]);
              bool isBlack = (val >> bitShift) & 1;
              
              if (isBlack) {
                  sprite.drawPixel(x + c, y + r, UI_TEXT_COLOR);
              } else {
                  sprite.drawPixel(x + c, y + r, UI_BACKGROUND_COLOR);
              }
              
              bitShift--;
              if (bitShift < 0) {
                  bitShift = 7;
                  byteIdx++;
              }
         }
    }
}

// =========================================================
// STARTUP SCREEN RENDERING
// =========================================================
void renderStartupScreen(const ControlParams& controlParams) {
    sprite.fillScreen(UI_BACKGROUND_COLOR); 
    
    sprite.setTextColor(UI_TEXT_COLOR, UI_BACKGROUND_COLOR);
    sprite.setTextDatum(MC_DATUM);
    sprite.setTextSize(2);
    
    if (currentStartupState == STARTUP_WAITING_GPS) {
        sprite.drawString("Waiting for", screenW/2, screenH/2 - 10);
        sprite.drawString("GPS...", screenW/2, screenH/2 + 15);
        
    } else if (currentStartupState == STARTUP_WAITING_BLE) {
        // Draw a rotated phone icon using the current compass heading.
        const int phoneW = 56;
        const int phoneH = 78;
        const int phoneX = (screenW - phoneW) / 2;
        const int phoneY = (screenH - phoneH) / 2 - 14;
        drawRotatedPhoneIcon(screenW / 2, phoneY + phoneH / 2, startupPhoneRotationAngle, true, true);

        // Show PIN if pairing is initiated (use showPIN from controlParams as primary source,
        // fall back to bleHandler flag for the first instants before queue is processed)
        if (controlParams.showPIN || bleHandler.waitingForToken) {
            String pinStr = (controlParams.pinCode[0] != '\0')
                            ? String(controlParams.pinCode)
                            : bleHandler.getPIN();
            sprite.setTextSize(3);
            sprite.setTextColor(TFT_WHITE, UI_BACKGROUND_COLOR);
            sprite.drawString(pinStr, screenW/2, phoneY + phoneH + 17);
            
            sprite.setTextSize(1);
            sprite.setTextColor(TFT_CYAN, UI_BACKGROUND_COLOR);
            sprite.drawString("Enter PIN in App", screenW/2, phoneY + phoneH + 46);
        } else if (bleHandler.isConnected() && bleHandler.isDeviceAuthenticated()) {
            // Phone is connected and authenticated — waiting for map data
            static unsigned long lastDotTime = 0;
            static uint8_t dotCount = 0;
            if (millis() - lastDotTime > 500) {
                dotCount = (dotCount + 1) % 4;
                lastDotTime = millis();
            }
            String dots = "";
            for (uint8_t d = 0; d < dotCount; d++) dots += ".";

            sprite.setTextSize(1);
            sprite.setTextColor(TFT_GREEN, UI_BACKGROUND_COLOR);
            sprite.drawString("Connected", screenW/2, phoneY + phoneH + 16);
        } else {
            // Not connected yet — prompt user
            sprite.setTextSize(1);
            sprite.setTextColor(UI_TEXT_COLOR, UI_BACKGROUND_COLOR);
            sprite.drawString("Open App", screenW/2, phoneY + phoneH + 20);
            sprite.drawString("to Connect", screenW/2, phoneY + phoneH + 34);
        }

    } else if (currentStartupState == STARTUP_QR_CODE) {
        // Draw QR Centered
        int qrX = (screenW - QR_CODE_WIDTH) / 2;
        int qrY = (screenH - QR_CODE_HEIGHT) / 2 + 10; // Shift down slightly
        
        drawQRCode(qrX, qrY); 
        
        sprite.setTextSize(1);
        sprite.drawString("Scan to App", screenW/2, qrY - 10);
    }
    
    sprite.setTextDatum(TL_DATUM); // Reset
}

// =========================================================
// RENDER TASK (Core 1)
// Handles display updates, sensor readings, and tile requests
// =========================================================
// Forward declarations
struct RenderParams;
static bool latLonToScreen(double lat, double lon, const RenderParams& rp, float cosT, float sinT, int pivotY, int& sx, int& sy);

static void drawManeuverIndicator(TFT_eSprite &target) {
    if (activeManeuverList.empty() || bleHandler.currentManeuverIndex >= activeManeuverList.size()) return;

    const int barY = 11; // Below 10px status bar
    const int margin = 6;
    const int iconSize = 20;
    const int textStartX = margin + iconSize + 10;
    const int availableW = screenW - textStartX - margin;
    
    Maneuver& m = activeManeuverList[bleHandler.currentManeuverIndex];
    String street = String(m.street_name);
    if (street.length() == 0) street = "Proceed to route";

    String distStr = "";
    if (currentTurnDistMeters >= 0) {
        if (currentTurnDistMeters < 1000) {
            distStr = String((int)currentTurnDistMeters) + "m: ";
        } else {
            distStr = String(currentTurnDistMeters / 1000.0f, 1) + "km: ";
        }
    }
    String fullText = distStr + street;

    target.setTextSize(1);
    
    std::vector<String> lines;
    String currentLine = "";
    int spaceWidth = target.textWidth(" ");
    int cursorX = 0;

    char buf[64]; 
    strncpy(buf, fullText.c_str() ? fullText.c_str() : "", sizeof(buf)-1);
    buf[sizeof(buf)-1] = '\0';
    
    char* word = strtok(buf, " ");
    while (word != NULL) {
        int wordWidth = target.textWidth(word);
        if (cursorX + wordWidth > availableW) {
            lines.push_back(currentLine);
            currentLine = String(word);
            cursorX = wordWidth;
        } else {
            if (currentLine.length() > 0) {
                currentLine += " ";
                cursorX += spaceWidth;
            }
            currentLine += word;
            cursorX += wordWidth;
        }
        word = strtok(NULL, " ");
    }
    if (currentLine.length() > 0) lines.push_back(currentLine);

    const int lineHeight = 12;
    int barH = lines.size() * lineHeight + 10;
    if (barH < 28) barH = 28;

    // 2. Render Semi-transparent background
    for (int y = barY; y < barY + barH; y++) {
        if (y >= 128) break;
        for (int x = 0; x < 160; x++) {
            uint16_t bg = target.readPixel(x, y);
            target.drawPixel(x, y, blendColors(bg, TFT_BLACK, 192));
        }
    }

    // 3. Draw Turn Icon (Left Side)
    int iconX = margin + iconSize/2;
    int iconY = barY + 14; 
    uint16_t iconColor = TFT_WHITE;
    const float w = 3.0f;
    
    switch (currentTurnType) {
        case TurnType::LEFT:
        case TurnType::SLIGHT_LEFT:
            drawThickLineAAOnSprite(target, iconX + 6, iconY + 8, iconX + 6, iconY - 4, w, iconColor);
            drawThickLineAAOnSprite(target, iconX + 6, iconY - 4, iconX - 6, iconY - 4, w, iconColor);
            drawThickLineAAOnSprite(target, iconX - 6, iconY - 4, iconX - 2, iconY - 8, w, iconColor);
            drawThickLineAAOnSprite(target, iconX - 6, iconY - 4, iconX - 2, iconY, w, iconColor);
            break;
        case TurnType::RIGHT:
        case TurnType::SLIGHT_RIGHT:
            drawThickLineAAOnSprite(target, iconX - 6, iconY + 8, iconX - 6, iconY - 4, w, iconColor);
            drawThickLineAAOnSprite(target, iconX - 6, iconY - 4, iconX + 6, iconY - 4, w, iconColor);
            drawThickLineAAOnSprite(target, iconX + 6, iconY - 4, iconX + 2, iconY - 8, w, iconColor);
            drawThickLineAAOnSprite(target, iconX + 6, iconY - 4, iconX + 2, iconY, w, iconColor);
            break;
        case TurnType::STRAIGHT:
            drawThickLineAAOnSprite(target, iconX, iconY + 8, iconX, iconY - 8, w, iconColor);
            drawThickLineAAOnSprite(target, iconX, iconY - 8, iconX - 5, iconY - 3, w, iconColor);
            drawThickLineAAOnSprite(target, iconX, iconY - 8, iconX + 5, iconY - 3, w, iconColor);
            break;
        case TurnType::UTURN:
            drawThickLineAAOnSprite(target, iconX + 6, iconY + 8, iconX + 6, iconY - 1, w, iconColor);
            drawThickLineAAOnSprite(target, iconX + 6, iconY - 1, iconX + 4, iconY - 5, w, iconColor);
            drawThickLineAAOnSprite(target, iconX + 4, iconY - 5, iconX - 4, iconY - 5, w, iconColor);
            drawThickLineAAOnSprite(target, iconX - 4, iconY - 5, iconX - 6, iconY - 1, w, iconColor);
            drawThickLineAAOnSprite(target, iconX - 6, iconY - 1, iconX - 6, iconY + 6, w, iconColor);
            drawThickLineAAOnSprite(target, iconX - 6, iconY + 6, iconX - 10, iconY + 2, w, iconColor);
            drawThickLineAAOnSprite(target, iconX - 6, iconY + 6, iconX - 2, iconY + 2, w, iconColor);
            break;
        default: break;
    }

    // 4. Draw Wrapped Text
    target.setTextColor(TFT_WHITE);
    target.setTextDatum(TL_DATUM);
    for (size_t i = 0; i < lines.size(); i++) {
        target.drawString(lines[i], textStartX, barY + 6 + (i * lineHeight));
    }

    // 5. Render Vertical Guidance Bubble (Sky Blue / White) next to text
    if (m.road_level == 1 || m.road_level == 2 || m.road_level == 3) {
        int bubbleX = screenW - 15;
        int bubbleY = barY + barH/2;
        
        uint16_t bColor = (m.road_level == 3) ? TFT_WHITE : 0x87CEEB; // White for ground, Sky Blue for bridge/tunnel
        target.fillSmoothCircle(bubbleX, bubbleY, 8, bColor);
        target.setTextColor((m.road_level == 3) ? TFT_BLACK : TFT_WHITE);
        
        const char* hint = "=";
        if (m.road_level == 1) hint = "^";
        else if (m.road_level == 2) hint = "v";
        
        target.drawCentreString(hint, bubbleX, bubbleY - 4, 1);
    }
}

static void drawElevationBubbleOnMap(TFT_eSprite &target,
                                     const RenderParams &rp,
                                     float cosT, float sinT,
                                     int pivotY) {
    if (activeManeuverList.empty()) return;

    for (size_t mi = 0; mi < activeManeuverList.size(); mi++) {
        const Maneuver &m = activeManeuverList[mi];
        RoadLevel rl = static_cast<RoadLevel>(m.road_level);
        if (rl == RoadLevel::GROUND) continue;
        
        uint32_t si = m.shape_index;
        if (si == 0 || si + 1 >= activeRoute.size()) continue;

        double ptLat = activeRouteAnchor.lat;
        double ptLon = activeRouteAnchor.lon;
        for (uint32_t i = 1; i <= si && i < activeRoute.size(); ++i) {
            ptLat += activeRoute[i].dLat / ROUTE_SCALE;
            ptLon += activeRoute[i].dLon / ROUTE_SCALE;
        }

        int bx, by;
        if (!latLonToScreen(ptLat, ptLon, rp, cosT, sinT, pivotY, bx, by)) continue;

        int distSq = (bx - 80)*(bx - 80) + (by - 64)*(by - 64);
        if (distSq > 10000) continue;

        double nextLat = ptLat + activeRoute[si + 1].dLat / ROUTE_SCALE;
        double nextLon = ptLon + activeRoute[si + 1].dLon / ROUTE_SCALE;
        int nx, ny;
        latLonToScreen(nextLat, nextLon, rp, cosT, sinT, pivotY, nx, ny);

        float dx = (float)(nx - bx);
        float dy = (float)(ny - by);
        float len = sqrtf(dx * dx + dy * dy);
        if (len < 0.5f) continue;

        float perpLx = -dy / len;  float perpLy =  dx / len;
        float perpRx =  dy / len;  float perpRy = -dx / len;
        const int OFFSET = 20;
        int lx = bx + (int)(perpLx * OFFSET);  int ly = by + (int)(perpLy * OFFSET);
        int rx = bx + (int)(perpRx * OFFSET);  int ry = by + (int)(perpRy * OFFSET);

        auto edgeClearance = [&](int cx, int cy) -> int {
            int d = min(cx, screenW - cx);
            d = min(d, min(cy, screenH - cy));
            return d;
        };

        bool useLeft = edgeClearance(lx, ly) >= edgeClearance(rx, ry);
        int bubX = useLeft ? lx : rx;
        int bubY = useLeft ? ly : ry;

        target.fillSmoothCircle(bubX, bubY, 8, 0x87CEEB); // Sky Blue
        target.setTextColor(TFT_WHITE);
        target.drawCentreString((rl == RoadLevel::BRIDGE) ? "^" : "v", bubX, bubY - 4, 1);
    }
}

// Helper: project a lat/lon to screen (x,y) using the current render params.
// Returns false if the point is outside the screen viewport (±50px margin).
static bool latLonToScreen(double lat, double lon,
                            const RenderParams &rp,
                            float cosT, float sinT,
                            int pivotY,
                            int &outX, int &outY) {
    int tileX, tileY_std, tileY_TMS;
    latlonToTile(lat, lon, currentTileZ, tileX, tileY_std, tileY_TMS);

    int mvtX, mvtY;
    latLonToMVTCoords(lat, lon, currentTileZ, tileX, tileY_TMS, mvtX, mvtY, rp.layerExtent);

    float trw = (float)screenW * rp.zoomScaleFactor;
    float trh = (float)screenH * rp.zoomScaleFactor;
    float offX = (float)(tileX - rp.centralTileX) * trw;
    float offY = (float)(rp.centralTileY_TMS - tileY_TMS) * trh;
    float scX = trw / rp.layerExtent;
    float scY = trh / rp.layerExtent;

    float sx = (float)mvtX * scX + offX - rp.displayOffsetX;
    float sy = (float)mvtY * scY + offY - rp.displayOffsetY;

    float tx = sx - (float)(screenW / 2);
    float ty = sy - (float)pivotY;
    outX = (int)roundf(tx * cosT - ty * sinT + (float)(screenW / 2));
    outY = (int)roundf(tx * sinT + ty * cosT + (float)pivotY);

    return (outX > -50 && outX < screenW + 50 && outY > -50 && outY < screenH + 50);
}

void renderTask(void *pvParameters) {
#ifndef I2C_SDA
#define I2C_SDA 21
#endif
#ifndef I2C_SCL
#define I2C_SCL 22
#endif
#ifndef DRDY_PIN
#define DRDY_PIN 35
#endif

    // Boot screen still showing - don't touch display yet
    // Just initialize sprite
    sprite.setColorDepth(16);
    sprite.createSprite(screenW, screenH);

    // Initialize compass with I2C auto-detection
    // Initialize I2C bus with board-configurable pins
    Wire.begin(I2C_SDA, I2C_SCL);
    delay(50); // Let bus settle

    uint8_t compassAddr = scanI2CBus(true);

    if (compassAddr != 0) {
        // Try HMC5883L first (common address 0x1E)
        Serial.printf("Render: I2C compass device detected at 0x%02X - initializing...\n", compassAddr);
        if (compassAddr == 0x1E && hmc5883l.begin()) {
            pinMode(DRDY_PIN, INPUT);
            hmc5883l.setMagGain(HMC5883_MAGGAIN_1_3);
            compassAutoCal.nextStartMs = millis() + 10000;
            Serial.printf("Render: Compass DRDY pin configured on GPIO%d\n", DRDY_PIN);
            Serial.println("✅ Render Task: HMC5883L compass initialized successfully");
            activeCompassType = COMPASS_HMC5883L;
            activeCompassI2CAddress = compassAddr;
            compassEnabled = true;
        }
        // Try QMC5883L family (0x0C, 0x0D, 0x2C clones)
        else if (compassAddr == 0x2C) {
            uint8_t cid = 0x00;
            bool hasCid = readI2CRegister(compassAddr, 0x00, cid);

            // QMC6310-compatible path for 0x2C clones that report CID=0x80.
            if (hasCid && cid == 0x80 && initQMC6310AtAddress(compassAddr)) {
                compassAutoCal.nextStartMs = millis() + 10000;
                Serial.printf("✅ Render Task: QMC6310-compatible compass initialized at 0x%02X\n", compassAddr);
                dumpCompassRegisters(compassAddr);
                activeCompassType = COMPASS_QMC6310;
                activeCompassI2CAddress = compassAddr;
                compassEnabled = true;
            } else if (initQMC5883AtAddress(compassAddr)) {
                compassAutoCal.nextStartMs = millis() + 10000;
                Serial.printf("✅ Render Task: QMC5883L (clone) compass initialized at 0x%02X\n", compassAddr);
                dumpCompassRegisters(compassAddr);
                activeCompassType = COMPASS_QMC_2C_OFFSET;
                activeCompassI2CAddress = compassAddr;
                compassEnabled = true;
            }

            if (!compassEnabled) {
                Serial.printf("⚠️  Render Task: 0x2C specific init failed (CID read=%s, CID=0x%02X), trying generic fallback\n",
                              hasCid ? "ok" : "failed",
                              cid);
                if (initHMCCompatAtAddress(compassAddr)) {
                    compassAutoCal.nextStartMs = millis() + 10000;
                    activeCompassType = COMPASS_HMC_COMPAT;
                    activeCompassI2CAddress = compassAddr;
                    compassEnabled = true;
                    Serial.printf("✅ Render Task: Generic HMC-compatible init successful at 0x%02X\n", compassAddr);
                    dumpCompassRegisters(compassAddr);
                }
            }
        }
        // Try QMC5883L family (0x0C, 0x0D clones)
        else if ((compassAddr == 0x0C || compassAddr == 0x0D) && initQMC5883AtAddress(compassAddr)) {
            compassAutoCal.nextStartMs = millis() + 10000;
            Serial.printf("✅ Render Task: QMC5883L (clone) compass initialized at 0x%02X\n", compassAddr);
            dumpCompassRegisters(compassAddr);
            activeCompassType = COMPASS_QMC5883L;
            activeCompassI2CAddress = compassAddr;
            compassEnabled = true;
        }
        // Fallback: assume it's a compass
        else {
            Serial.printf("⚠️  Render Task: Compass at 0x%02X - generic init\n", compassAddr);
            Serial.println("   Will attempt both QMC and HMC-compatible clone init");
            if (initHMCCompatAtAddress(compassAddr)) {
                compassAutoCal.nextStartMs = millis() + 10000;
                activeCompassType = COMPASS_HMC_COMPAT;
                activeCompassI2CAddress = compassAddr;
                compassEnabled = true;
                Serial.printf("✅ Render Task: Generic HMC-compatible init successful at 0x%02X\n", compassAddr);
                dumpCompassRegisters(compassAddr);
            } else if (initQMC5883AtAddress(compassAddr)) {
                compassAutoCal.nextStartMs = millis() + 10000;
                activeCompassType = COMPASS_QMC5883L;
                activeCompassI2CAddress = compassAddr;
                compassEnabled = true;
                Serial.printf("✅ Render Task: Generic QMC init successful at 0x%02X\n", compassAddr);
                dumpCompassRegisters(compassAddr);
            } else {
                activeCompassType = COMPASS_NONE;
                activeCompassI2CAddress = 0;
                compassEnabled = false;
                Serial.printf("❌ Render Task: Compass init failed at 0x%02X - disabled\n", compassAddr);
            }
        }
    } else {
        Serial.println("⚠️  Render Task: No I2C devices found - compass disabled (map will use GPS heading only)");
        activeCompassType = COMPASS_NONE;
        activeCompassI2CAddress = 0;
        compassEnabled = false;
    }

    // Initialize currentControlParams with default values
    ControlParams currentControlParams = {
        .targetLat = 0.0,  // No hardcoded position - waits for first GPS
        .targetLon = 0.0,
        .zoomFactor = 1.0,
        .cullingBufferPercentageLeft = DEFAULT_CULLING_BUFFER_PERCENTAGE_LEFT,
        .cullingBufferPercentageRight = DEFAULT_CULLING_BUFFER_PERCENTAGE_RIGHT,
        .cullingBufferPercentageTop = DEFAULT_CULLING_BUFFER_PERCENTAGE_TOP,
        .cullingBufferPercentageBottom = DEFAULT_CULLING_BUFFER_PERCENTAGE_BOTTOM,
        .bleIconMode = 1,
        .showPIN = false,
        .pinCode = {0},
        .deviceName = {0}
    };
    RenderParams currentRenderParams;

    // State variables for loading management
    TileKey currentRequestedCenterTile = {-1, -1, -1};
    TileKey currentlyLoadedCenterTile = {-2, -2, -2};

    float internalCurrentRotationAngle = 0.0f;
    float lastSentRotationAngle = 0.0f;
    float lastSentZoomFactor = 0.0f;
    float compassTargetRotationAngle = 0.0f;
    bool compassRotationInitialized = false;
    unsigned long lastCompassTargetUpdateMs = 0;
    unsigned long lastCompassInterpolationMs = 0;

    const int STATUS_BAR_HEIGHT = 10;
    const int ARROW_MARGIN_ABOVE_STATUS_BAR = 2;
    const float STATUS_BAR_ALPHA = 0.5f;
    
    bool bootScreenCleared = false; // Track if we've cleared the boot screen
    const int TILES_NEEDED_FOR_BOOT = 20; // Show boot screen until this many tiles load
    bool gpsReady = false; // True once we receive a real GPS coordinate (non-zero)
    
    // Animated Zoom State
    float animatedZoomFactor = 1.0f; // Current animated zoom level
    const float ZOOM_LERP_SPEED = 0.25f; // 25% per frame (snappier than 0.15f)
    const unsigned long RENDER_FRAME_MIN_MS = 1000UL / 60UL; // Cap render loop to 60 FPS

    StartupState lastStartupState = STARTUP_BOOT_ANIMATION;

    while (true) {
        bool screenNeedsUpdate = false;
        const unsigned long now = millis();
        
        // Force update on state change
        if (currentStartupState != lastStartupState) {
            lastStartupState = currentStartupState;
            screenNeedsUpdate = true;
        }
        
        // Check if we should clear boot screen and show map
        if (!bootScreenCleared && tilesLoadedCount >= TILES_NEEDED_FOR_BOOT) {
            tft.fillScreen(MAP_BACKGROUND_COLOR);
            bootScreenCleared = true;
            screenNeedsUpdate = true;
            Serial.println("Boot screen cleared - showing map");
        }
        
        // Update boot screen progress bar based on tile loading
        // Use a dynamically-sized, centered bar so it lines up correctly on different displays
        if (!bootScreenCleared && tilesLoadedCount > 0) {
            int barW = screenW - 80;
            if (barW < 120) barW = 120;
            if (barW > screenW - 40) barW = screenW - 40;
            int barH = screenH / 40;
            if (barH < 6) barH = 6;
            if (barH > 18) barH = 18;
            int barRadius = barH / 2;
            int barX = (screenW - barW) / 2;
            int barY = screenH / 2 + 60; // reasonable offset below the center/phone icon area

            int progress = (tilesLoadedCount * 100) / TILES_NEEDED_FOR_BOOT;
            if (progress < 0) progress = 0;
            if (progress > 100) progress = 100;

            int fillWidth = (barW * progress) / 100;

            // draw background and fill portion
            tft.fillRoundRect(barX, barY, barW, barH, barRadius, TFT_DARKGREY);
            if (fillWidth > 0) {
                tft.fillRoundRect(barX, barY, fillWidth, barH, barRadius, TFT_WHITE);
            }
        }

        // 1. Check for new control parameters from the main loop (user input)
        ControlParams receivedControlParams;
        bool gotNewControlParams = false;
        while (xQueueReceive(controlParamsQueue, &receivedControlParams, 0) == pdPASS) {
            currentControlParams = receivedControlParams;
            gotNewControlParams = true;
        }
        if (gotNewControlParams) {
            screenNeedsUpdate = true; // New control params always mean screen update
        }

        // Logic for blinking BLE Connected icon (Mode 0)
        static unsigned long lastIconBlinkTime = 0;
        static bool iconBlinkVisible = true;
        if (currentControlParams.bleIconMode == 0) { // Blink Red
             if (millis() - lastIconBlinkTime > 500) {
                 iconBlinkVisible = !iconBlinkVisible;
                 lastIconBlinkTime = millis();
                 screenNeedsUpdate = true; // Trigger redraw for blink update
             }
        } else {
            iconBlinkVisible = true; // Always visible in other modes
        }

        // 2. Read compass data and update rotation (rate-limited for FPS)
        static unsigned long lastCompassRead = 0;
        const unsigned long COMPASS_READ_INTERVAL_MS = 50; // Read compass at 20Hz max
        
        float magX = 0.0f, magY = 0.0f;
        bool compassReadThisFrame = false;
        
        if (compassEnabled && millis() - lastCompassRead >= COMPASS_READ_INTERVAL_MS) {
            if (readCompassData(magX, magY)) {
                lastCompassRead = millis();
                compassReadThisFrame = true;
            }
        }
        
        // Only update heading if compass was read and data is valid
        if (compassReadThisFrame) {
            if (!compassAutoCal.active && millis() >= compassAutoCal.nextStartMs) {
                startCompassAutoCalibration();
            }

            if (compassAutoCal.active) {
                compassAutoCal.sampleCount++;
                if (magX < compassAutoCal.minX) compassAutoCal.minX = magX;
                if (magX > compassAutoCal.maxX) compassAutoCal.maxX = magX;
                if (magY < compassAutoCal.minY) compassAutoCal.minY = magY;
                if (magY > compassAutoCal.maxY) compassAutoCal.maxY = magY;

                if (millis() - compassAutoCal.startMs >= COMPASS_AUTO_CAL_DURATION_MS) {
                    finishCompassAutoCalibration();
                }
            }

            const float correctedX = compassAutoCal.hasOffset ? (magX - compassAutoCal.offsetX) : magX;
            const float correctedY = compassAutoCal.hasOffset ? (magY - compassAutoCal.offsetY) : magY;

            float rawHeading = atan2(correctedY, correctedX);
            rawHeading = rawHeading * 180 / PI;

            // Auto-adjust compass heading offset against GPS course when motion is reliable.
            float headingOffsetDeg = COMPASS_HEADING_OFFSET_DEG + compassGpsAlign.dynamicOffsetDeg;
            float adjustedHeading = normalizeCompassAngle(rawHeading + headingOffsetDeg);

            float gpsHeadingDeg = 0.0f;
            float gpsSpeedMs = 0.0f;
            unsigned long gpsTimestamp = 0;
            bool haveGpsState = false;
            if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                gpsHeadingDeg = gpsState.heading;
                gpsSpeedMs = gpsState.speed;
                gpsTimestamp = gpsState.timestamp;
                haveGpsState = (gpsTimestamp > 0);
                xSemaphoreGive(gpsMutex);
            }

            bool gpsFresh = haveGpsState && ((now - gpsTimestamp) <= COMPASS_GPS_ALIGN_MAX_GPS_AGE_MS);
            bool gpsFastEnough = gpsSpeedMs >= COMPASS_GPS_ALIGN_MIN_SPEED_MS;
            bool gpsHeadingStable = true;
            if (compassGpsAlign.hasLastGpsHeading) {
                float gpsTurnDelta = fabs(shortestCompassAngleDelta(compassGpsAlign.lastGpsHeadingDeg, gpsHeadingDeg));
                gpsHeadingStable = gpsTurnDelta <= COMPASS_GPS_ALIGN_MAX_TURN_RATE_DEG;
            }

            if (gpsFresh && gpsFastEnough && gpsHeadingStable) {
                float headingError = shortestCompassAngleDelta(adjustedHeading, gpsHeadingDeg);
                if (fabs(headingError) <= COMPASS_GPS_ALIGN_MAX_ERROR_DEG) {
                    // IIR filter + trust streak before integrating offset to prevent runaway drift.
                    compassGpsAlign.filteredErrorDeg =
                        (compassGpsAlign.filteredErrorDeg * 0.85f) + (headingError * 0.15f);

                    if (compassGpsAlign.trustedSampleStreak < 65535) {
                        compassGpsAlign.trustedSampleStreak++;
                    }

                    if (compassGpsAlign.trustedSampleStreak >= COMPASS_GPS_ALIGN_MIN_STREAK) {
                        float stepDeg = clampf(
                            compassGpsAlign.filteredErrorDeg * COMPASS_GPS_ALIGN_GAIN,
                            -COMPASS_GPS_ALIGN_MAX_STEP_DEG,
                            COMPASS_GPS_ALIGN_MAX_STEP_DEG
                        );

                        compassGpsAlign.dynamicOffsetDeg += stepDeg;
                        compassGpsAlign.dynamicOffsetDeg = clampf(
                            compassGpsAlign.dynamicOffsetDeg,
                            -COMPASS_GPS_ALIGN_MAX_DYNAMIC_OFFSET_DEG,
                            COMPASS_GPS_ALIGN_MAX_DYNAMIC_OFFSET_DEG
                        );
                    }

                    headingOffsetDeg = COMPASS_HEADING_OFFSET_DEG + compassGpsAlign.dynamicOffsetDeg;
                    adjustedHeading = normalizeCompassAngle(rawHeading + headingOffsetDeg);

                    if (now - compassGpsAlign.lastLogMs > 5000UL) {
                        compassGpsAlign.lastLogMs = now;
                        float gpsSpeedKmh = gpsSpeedMs * 3.6f;
                        Serial.printf(
                            "Render: Compass GPS align offset=%.2f (err=%.2f filt=%.2f streak=%u gps=%.1f speed=%.1fkm/h)\n",
                            compassGpsAlign.dynamicOffsetDeg,
                            headingError,
                            compassGpsAlign.filteredErrorDeg,
                            compassGpsAlign.trustedSampleStreak,
                            gpsHeadingDeg,
                            gpsSpeedKmh
                        );
                    }
                } else {
                    compassGpsAlign.trustedSampleStreak = 0;
                }
            } else {
                compassGpsAlign.trustedSampleStreak = 0;
            }

            if (haveGpsState) {
                compassGpsAlign.lastGpsHeadingDeg = gpsHeadingDeg;
                compassGpsAlign.hasLastGpsHeading = true;
            }

            rawHeading = adjustedHeading;

            // Add new reading to circular buffer
            headingReadings_buffer[headingReadings_index] = rawHeading;
            headingReadings_index = (headingReadings_index + 1) % COMPASS_FILTER_WINDOW_SIZE;
            if (!headingReadings_full && headingReadings_index == 0) {
                headingReadings_full = true; // Buffer has been filled at least once
            }

            // Calculate moving average using circular statistics
            float sumSin = 0.0f;
            float sumCos = 0.0f;
            size_t count = headingReadings_full ? COMPASS_FILTER_WINDOW_SIZE : headingReadings_index;
            
            for (size_t i = 0; i < count; ++i) {
                sumSin += sin(radians(headingReadings_buffer[i]));
                sumCos += cos(radians(headingReadings_buffer[i]));
            }
            
            // Avoid division by zero if count is 0 (shouldn't happen if filter window is > 0)
            if (count > 0) {
                float averagedHeading = degrees(atan2(sumSin / count, sumCos / count));
                if (averagedHeading < 0) averagedHeading += 360;

                // Smooth across 0/360 by using shortest angular difference
                float angleDelta = shortestCompassAngleDelta(lastSmoothedRotationAngle, averagedHeading);
                internalCurrentRotationAngle = lastSmoothedRotationAngle + (COMPASS_EXPONENTIAL_SMOOTHING_ALPHA * angleDelta);
                internalCurrentRotationAngle = normalizeCompassAngle(internalCurrentRotationAngle);
                lastSmoothedRotationAngle = internalCurrentRotationAngle;

                if (!compassRotationInitialized) {
                    lastSentRotationAngle = internalCurrentRotationAngle;
                    compassTargetRotationAngle = internalCurrentRotationAngle;
                    compassRotationInitialized = true;
                    lastCompassTargetUpdateMs = now;
                    lastCompassInterpolationMs = now;
                    screenNeedsUpdate = true;
                } else {
                    float targetDelta = shortestCompassAngleDelta(compassTargetRotationAngle, internalCurrentRotationAngle);
                    if (fabs(targetDelta) >= COMPASS_ROTATION_THRESHOLD_DEG ||
                        (now - lastCompassTargetUpdateMs) >= COMPASS_ROTATION_HOLD_MS) {
                        compassTargetRotationAngle = internalCurrentRotationAngle;
                        lastCompassTargetUpdateMs = now;
                    }
                }
            } else {
                // Keep last value instead of snapping to 0 when sensor data is briefly invalid
                internalCurrentRotationAngle = lastSmoothedRotationAngle;
            }
        }

        if (compassRotationInitialized) {
            float elapsedMs = (float)(now - lastCompassInterpolationMs);
            lastCompassInterpolationMs = now;

            float rotationToTarget = shortestCompassAngleDelta(lastSentRotationAngle, compassTargetRotationAngle);
            if (fabs(rotationToTarget) > 0.01f) {
                float interpolationAlpha = elapsedMs / (float)COMPASS_ROTATION_INTERPOLATION_MS;
                if (interpolationAlpha > 1.0f) interpolationAlpha = 1.0f;
                if (interpolationAlpha < 0.0f) interpolationAlpha = 0.0f;

                lastSentRotationAngle = normalizeCompassAngle(lastSentRotationAngle + (rotationToTarget * interpolationAlpha));
                if (fabs(shortestCompassAngleDelta(lastSentRotationAngle, compassTargetRotationAngle)) > 0.01f) {
                    screenNeedsUpdate = true;
                } else {
                    lastSentRotationAngle = compassTargetRotationAngle;
                }
            }
        }

        startupPhoneRotationAngle = compassRotationInitialized ? lastSentRotationAngle : 0.0f;


        // Check if rotation has changed significantly
        float rotationDelta = shortestCompassAngleDelta(lastSentRotationAngle, compassTargetRotationAngle);
        if (fabs(rotationDelta) >= COMPASS_ROTATION_THRESHOLD_DEG) {
            screenNeedsUpdate = true;
        }

        // =========================================================
        // GPS INTERPOLATION (Dead Reckoning) for smooth animation
        // =========================================================
        // The app sends GPS at 3Hz (~333ms) when driving fast, 1Hz (~1000ms) when slow.
        // gpsRate (in ble_handler) tracks the exponential moving average of the BLE packet rate.
        // We use that measured rate to perfectly calibrate the dead-reckoning window so the
        // extrapolated position lands exactly at the next real packet — no jumps, no lag.
        // =========================================================
        double smoothLat = currentControlParams.targetLat;
        double smoothLon = currentControlParams.targetLon;
        float gpsSpeedForFilter = 0.0f;
        float gpsHeadingForFilter = 0.0f;
        static bool motionActive = false;
        
        if (phoneGpsActive && xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            float elapsed = (now - gpsState.timestamp) / 1000.0f; // seconds since last GPS
            gpsSpeedForFilter = gpsState.speed;
            gpsHeadingForFilter = gpsState.heading;
            
            // Dynamically derive the expected packet interval from the measured rate.
            // gpsRate is packets/sec (EMA-smoothed). Clamp to sane range [0.25, 5.0] Hz.
            float measuredRateHz = (gpsRate > 0.25f && gpsRate < 5.0f) ? gpsRate : 1.0f;
            float packetIntervalSec = 1.0f / measuredRateHz; // e.g. 0.33s at 3Hz, 1.0s at 1Hz

            // Extrapolation window = 1 full packet interval. 
            // Brake window = 0.5 packet intervals (in case signal is briefly lost).
            float extrapTime = packetIntervalSec * 1.2f; // +20% safety margin
            float brakeTime  = packetIntervalSec * 0.5f;

            float effectiveHeadingRad = gpsState.heading * (3.14159f / 180.0f);

            // Route bearing snap: if heading is within 30° of upcoming road segment, lock onto road
            if (!activeRoute.empty() && routeProgressIndex < activeRoute.size() - 1) {
                double dLat = activeRoute[routeProgressIndex + 1].dLat;
                double dLon = activeRoute[routeProgressIndex + 1].dLon;
                float routeBearing = atan2((float)dLon, (float)dLat) * (180.0f / 3.14159f);
                if (routeBearing < 0) routeBearing += 360.0f;
                float diff = fmod(fabs(gpsState.heading - routeBearing), 360.0f);
                if (diff > 180.0f) diff = 360.0f - diff;
                if (diff < 30.0f) {
                    effectiveHeadingRad = routeBearing * (3.14159f / 180.0f);
                }
            }
            
            // Motion hysteresis prevents repeated move/stop flapping near zero speed.
            if (motionActive) {
                if (gpsState.speed < POS_STOP_SPEED_ENTER_MS) motionActive = false;
            } else {
                if (gpsState.speed > POS_STOP_SPEED_EXIT_MS) motionActive = true;
            }

            if (motionActive && gpsState.speed > 0.0f && elapsed > 0.0f) {
                float v0 = gpsState.speed;
                float distance = 0.0f;
                
                if (elapsed <= extrapTime) {
                    // Smooth interpolation: ease-in slightly to avoid tiny start/stop jerks.
                    float t = clampf(elapsed / extrapTime, 0.0f, 1.0f);
                    float smoothT = t * t * (3.0f - 2.0f * t);
                    distance = v0 * elapsed * (0.80f + 0.20f * smoothT);
                    screenNeedsUpdate = true;
                } else if (elapsed < extrapTime + brakeTime) {
                    // Soft brake: decelerates to zero over half a packet interval
                    float t = elapsed - extrapTime;
                    distance = (v0 * extrapTime) + v0 * (t - (0.5f * t * t / brakeTime));
                    screenNeedsUpdate = true;
                } else {
                    // Frozen: signal lost, hold last extrapolated position
                    distance = (v0 * extrapTime) + v0 * (brakeTime * 0.5f);
                }
                
                double cosLat = cos(gpsState.lat * 3.14159 / 180.0);
                double dLat = (distance * cos(effectiveHeadingRad)) / 111320.0;
                double dLon = (distance * sin(effectiveHeadingRad)) / (111320.0 * cosLat);
                
                smoothLat = gpsState.lat + dLat;
                smoothLon = gpsState.lon + dLon;
            } else if (gpsState.lat != 0.0 && gpsState.lon != 0.0) {
                smoothLat = gpsState.lat;
                smoothLon = gpsState.lon;
            }
            xSemaphoreGive(gpsMutex);
        } else if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            // Hardware GPS path: no BLE-rate dead reckoning, but still use filtered position pipeline.
            if (gpsState.lat != 0.0 || gpsState.lon != 0.0) {
                smoothLat = gpsState.lat;
                smoothLon = gpsState.lon;
                gpsSpeedForFilter = gpsState.speed;
                gpsHeadingForFilter = gpsState.heading;
            }
            xSemaphoreGive(gpsMutex);
        }

        // Apply adaptive position smoothing to reduce low-speed jitter and camera shimmer.
        if (smoothLat != 0.0 || smoothLon != 0.0) {
            applyKalmanPositionFilter(smoothLat, smoothLon, gpsSpeedForFilter, now, smoothLat, smoothLon);
        }

        // When a route is active and we are close enough, pin camera location toward road centerline.
        // This is confidence-gated by route distance and heading alignment in snapPositionToRouteCenterline().
        if ((smoothLat != 0.0 || smoothLon != 0.0) &&
            xSemaphoreTake(routeMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            double snapLat = smoothLat;
            double snapLon = smoothLon;
            float snapDistM = 0.0f;
            bool snapped = snapPositionToRouteCenterline(
                smoothLat,
                smoothLon,
                gpsHeadingForFilter,
                gpsSpeedForFilter,
                snapLat,
                snapLon,
                snapDistM
            );
            xSemaphoreGive(routeMutex);

            if (snapped) {
                float maxSnapDist = (gpsSpeedForFilter < 3.0f)
                    ? POS_MAX_SNAP_DIST_LOW_SPEED_M
                    : POS_MAX_SNAP_DIST_HIGH_SPEED_M;
                float closeness = 1.0f - clampf(snapDistM / maxSnapDist, 0.0f, 1.0f);
                float snapBlend = clampf(0.35f + 0.50f * closeness, 0.35f, 0.85f);

                // Hysteresis lock: keep snap stable when stop/go near intersections.
                if (routeSnapLock.active) {
                    float cosRef = cosf((float)smoothLat * (3.14159f / 180.0f));
                    float mPerDegLon = 111320.0f * ((fabsf(cosRef) < 0.01f) ? 0.01f : cosRef);
                    float mPerDegLat = 111320.0f;
                    float dLockX = (float)((routeSnapLock.lon - snapLon) * mPerDegLon);
                    float dLockY = (float)((routeSnapLock.lat - snapLat) * mPerDegLat);
                    float lockDeltaM = sqrtf(dLockX * dLockX + dLockY * dLockY);

                    // If new snap is close to locked one, favor lock for stability.
                    if (lockDeltaM < 6.0f) {
                        snapLat = routeSnapLock.lat + (snapLat - routeSnapLock.lat) * 0.25f;
                        snapLon = routeSnapLock.lon + (snapLon - routeSnapLock.lon) * 0.25f;
                        snapBlend = fmaxf(snapBlend, 0.55f);
                    }
                }

                smoothLat = smoothLat + (snapLat - smoothLat) * snapBlend;
                smoothLon = smoothLon + (snapLon - smoothLon) * snapBlend;

                routeSnapLock.active = true;
                routeSnapLock.lat = smoothLat;
                routeSnapLock.lon = smoothLon;
                routeSnapLock.lastUpdateMs = now;
            } else if (routeSnapLock.active && (now - routeSnapLock.lastUpdateMs) < 900UL && gpsSpeedForFilter < 1.8f) {
                // Briefly hold last good snap when stationary to avoid back-and-forth flutter.
                smoothLat = routeSnapLock.lat;
                smoothLon = routeSnapLock.lon;
            } else {
                routeSnapLock.active = false;
            }
        }



        // Mark GPS as ready once we receive a real, non-zero coordinate
        // (either from BLE phone GPS or the physical NEO-6M module)
        if (!gpsReady && (smoothLat != 0.0 || smoothLon != 0.0)) {
            gpsReady = true;
            Serial.println("GPS Ready: First real coordinate received - starting tile requests");
        }

        // 3. Calculate current central tile and target MVT point (using interpolated position)
        // Skip tile requests until a real GPS coordinate arrives.
        // Show the existing "Open App / to Connect" phone screen (STARTUP_WAITING_BLE).
        if (!gpsReady) {
            if (currentStartupState != STARTUP_WAITING_BLE) {
                currentStartupState = STARTUP_WAITING_BLE;
                screenNeedsUpdate = true;
            }
            if (screenNeedsUpdate) {
                renderStartupScreen(currentControlParams);
                sprite.pushSprite(0, 0);
                screenNeedsUpdate = false;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        int newCentralTileX, newCentralTileY_std, newCentralTileY_TMS;
        latlonToTile(smoothLat, smoothLon, currentTileZ,
                     newCentralTileX, newCentralTileY_std, newCentralTileY_TMS);

        // --- TILE HYSTERESIS ---
        // Only switch the central tile if the coordinate moves significantly into a new tile.
        // This prevents jitter/flicker and constant queue resets when near a tile boundary.
        static TileKey stableCenterTile = {0, 0, 0};
        const double HYSTERESIS_PIXELS = 15.0; // 15-pixel buffer (~2-5 meters at Z16)
        
        bool tileSwitchNeeded = false;
        if (stableCenterTile.z != currentTileZ) {
            tileSwitchNeeded = true; // Zoom changed
        } else {
            // Calculate distance from stable center in pixels
            int sMvtX, sMvtY;
            latLonToMVTCoords(smoothLat, smoothLon, currentTileZ, 
                              stableCenterTile.x, stableCenterTile.y_tms, 
                              sMvtX, sMvtY, currentLayerExtent);
            
            float scale = (float)256.0f / currentLayerExtent; // Standard 256px tile logic
            float pX = sMvtX * scale;
            float pY = sMvtY * scale;
            
            // If we are outside [0, 256] by more than HYSTERESIS_PIXELS, we must switch
            if (pX < -HYSTERESIS_PIXELS || pX > 256.0f + HYSTERESIS_PIXELS ||
                pY < -HYSTERESIS_PIXELS || pY > 256.0f + HYSTERESIS_PIXELS) {
                tileSwitchNeeded = true;
            }
        }

        if (tileSwitchNeeded) {
            stableCenterTile = {currentTileZ, newCentralTileX, newCentralTileY_TMS};
        }

        TileKey newRequestedCenterTile = stableCenterTile;
        
        // Recalculate everything relative to the stable center
        int activeX = stableCenterTile.x;
        int activeY_TMS = stableCenterTile.y_tms;

        int internalTargetPointMVT_X, internalTargetPointMVT_Y;
        latLonToMVTCoords(smoothLat, smoothLon, currentTileZ, activeX, activeY_TMS,
                          internalTargetPointMVT_X, internalTargetPointMVT_Y, currentLayerExtent);


        // 4. Update currentRenderParams
        currentRenderParams.centralTileX = activeX;
        currentRenderParams.centralTileY_TMS = activeY_TMS;
        currentRenderParams.targetPointMVT_X = internalTargetPointMVT_X;
        currentRenderParams.targetPointMVT_Y = internalTargetPointMVT_Y;
        currentRenderParams.layerExtent = currentLayerExtent;
        
        // Determine Target Zoom (Auto-Zoom Logic)
        float targetZoom = currentControlParams.zoomFactor;

        // AUTO-ZOOM: boost zoom at low speeds, return to neutral at highway speeds.
        // Only positive modifiers — zoom never goes below the user's base level.
        static float smoothedSpeed = 0.0f;
        float rawSpeed = (gpsState.speed > 0) ? gpsState.speed : 0.0f; // m/s
        smoothedSpeed = (0.03f * rawSpeed) + (0.97f * smoothedSpeed); // heavy smoothing

        float autoZoomModifier = 0.0f;
        if (smoothedSpeed < 1.4f) {
            // < 5 km/h: stationary / walking — zoom in close
            autoZoomModifier = 2.5f;
        } else if (smoothedSpeed < 8.3f) {
            // 5–30 km/h: ramp from +2.5 down to 0
            float t = (smoothedSpeed - 1.4f) / (8.3f - 1.4f);
            autoZoomModifier = 2.5f * (1.0f - t);
        }
        // >= 30 km/h: neutral (modifier = 0), user's zoom applies as-is

        targetZoom += autoZoomModifier;

        // Hard limits: minimum 1.0× (no sub-1x zoom), maximum 4.5×
        if (targetZoom < 1.0f) targetZoom = 1.0f;
        if (targetZoom > 4.5f) targetZoom = 4.5f;

        if (fabs(animatedZoomFactor - targetZoom) > 0.001f) {
            animatedZoomFactor += (targetZoom - animatedZoomFactor) * ZOOM_LERP_SPEED;
            screenNeedsUpdate = true;
        } else {
            animatedZoomFactor = targetZoom;
        }
        currentRenderParams.zoomScaleFactor = animatedZoomFactor;
        currentRenderParams.mapRotationDegrees = lastSentRotationAngle; // Use STABLE angle
        currentRenderParams.cullingBufferPercentageLeft = currentControlParams.cullingBufferPercentageLeft;
        currentRenderParams.cullingBufferPercentageRight = currentControlParams.cullingBufferPercentageRight;
        currentRenderParams.cullingBufferPercentageTop = currentControlParams.cullingBufferPercentageTop;
        currentRenderParams.cullingBufferPercentageBottom = currentControlParams.cullingBufferPercentageBottom;

        // Define arrow properties
        int arrowSize = NAVIGATION_ARROW_SIZE;
        int arrowHalfSize = arrowSize / 2;

        // Calculate the Y coordinate for the center of the arrow's base,
        // positioning it above the status bar with a margin.
        // The lowest point of the arrow is centerY + halfSize.
        // We want this lowest point to be at (screenH - STATUS_BAR_HEIGHT - ARROW_MARGIN_ABOVE_STATUS_BAR).
        int arrowBaseCenterY = (screenH - STATUS_BAR_HEIGHT) - ARROW_MARGIN_ABOVE_STATUS_BAR - arrowHalfSize;

        // The pivotY for map rotation is the tip of the arrow.
        currentRenderParams.pivotY = arrowBaseCenterY - arrowSize;

        // Calculate display offsets. The map's target point should align with the arrow's tip.
        float scaledPointX_global = (float)currentRenderParams.targetPointMVT_X * (screenW * currentRenderParams.zoomScaleFactor / currentRenderParams.layerExtent);
        float scaledPointY_global = (float)currentRenderParams.targetPointMVT_Y * (screenH * currentRenderParams.zoomScaleFactor / currentRenderParams.layerExtent);
        
        // Adjust display offsets to correct point positioning.
        // A positive offset here shifts the map content to the left (for X) or upwards (for Y).
        // If the displayed point is to the right of actual, increase displayOffsetX.
        // If the displayed point is below actual, increase displayOffsetY.
        currentRenderParams.displayOffsetX = round(scaledPointX_global - (screenW / 2.0f) -2); // Increased offset to +5
        currentRenderParams.displayOffsetY = round(scaledPointY_global - currentRenderParams.pivotY +1); // Increased offset to +5


        // 5. Request necessary tiles from Data Task based on the new loading strategy
        if (!(newRequestedCenterTile == currentlyLoadedCenterTile)) {

            // Clear any pending requests for old center tile if the central tile has changed
            if (!(newRequestedCenterTile == currentlyLoadedCenterTile)) {
                Serial.printf("Render Task: Central tile changed to Z:%d X:%d Y:%d. Resetting tile request queue.\n",
                              newRequestedCenterTile.z, newRequestedCenterTile.x, newRequestedCenterTile.y_tms);
                xQueueReset(tileRequestQueue);
            }

            currentlyLoadedCenterTile = newRequestedCenterTile;
            lastSentZoomFactor = currentControlParams.zoomFactor;
            screenNeedsUpdate = true; // New tile requests always mean screen update

            std::set<TileKey> requestedTilesInCycle; // To track what's sent this cycle

            auto sendTileRequest = [&](const TileKey& key) -> bool {
                // Check if already requested in this cycle or already loaded
                if (requestedTilesInCycle.find(key) == requestedTilesInCycle.end()) {
                    bool alreadyLoaded = false;
                    if (xSemaphoreTake(loadedTilesDataMutex, 0) == pdTRUE) {
                        alreadyLoaded = (loadedTilesData.find(key) != loadedTilesData.end());
                        xSemaphoreGive(loadedTilesDataMutex);
                    }
                    if (!alreadyLoaded) {
                        // Attempt to send the request (short timeout to avoid blocking render)
                        if (xQueueSend(tileRequestQueue, &key, pdMS_TO_TICKS(10)) != pdPASS) {
                            Serial.printf("❌ Render Task: Failed to send tile request Z:%d X:%d Y:%d. Queue full? (Timeout)\n",
                                          key.z, key.x, key.y_tms);
                            return false;
                        }
                        requestedTilesInCycle.insert(key); // Mark as sent
                        return true;
                    }
                }
                return false; // Already requested or loaded
            };

            // Implementing "9+16" tile loading strategy:
            // Ring 0: Central tile (1 tile)
            sendTileRequest(newRequestedCenterTile);

            // Ring 1: 3x3 grid (excluding center) = 8 tiles
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) { // Corrected: Added 'dy <= 1' to the loop condition
                    if (dx == 0 && dy == 0) continue; // Skip central tile
                    TileKey neighborKey = {currentTileZ, newRequestedCenterTile.x + dx, newRequestedCenterTile.y_tms + dy};
                    sendTileRequest(neighborKey);
                }
            }

            // Ring 2: 5x5 grid (excluding 3x3 inner grid) = 16 tiles
            for (int dx = -2; dx <= 2; ++dx) {
                for (int dy = -2; dy <= 2; ++dy) { // Corrected: Added 'dy <= 2' to the loop condition
                    // Skip the inner 3x3 grid (already handled in Ring 0 and Ring 1)
                    if (abs(dx) <= 1 && abs(dy) <= 1) continue;
                    TileKey neighborKey = {currentTileZ, newRequestedCenterTile.x + dx, newRequestedCenterTile.y_tms + dy};
                    sendTileRequest(neighborKey);
                }
            }
        }

        // 6. Check for tile parsed notifications (optional, but good for responsiveness)
        bool notification;
        if (xQueueReceive(tileParsedNotificationQueue, &notification, 0) == pdPASS) {
            if (notification) {
                screenNeedsUpdate = true; // A tile was successfully parsed, so the map data has changed.
            }
        }

        // 7. Render the map (only if needed AND boot screen is cleared)
        if ((screenNeedsUpdate || globalOTAState.active) && bootScreenCleared) {
            
            // =========================================================
            // OTA UPDATE SCREEN
            // =========================================================
            if (globalOTAState.active) {
                // ... (Logic continues below, no change needed)

                // Background
                sprite.fillScreen(TFT_BLACK);
                
                // Title "UPDATING..."
                sprite.setTextColor(TFT_WHITE, TFT_BLACK);
                sprite.setTextSize(2);
                sprite.setTextDatum(MC_DATUM); // Middle Center
                sprite.drawString("UPDATING...", screenW / 2, 40);
                
                // Type "FIRMWARE" or "MAP DATA"
                sprite.setTextSize(1);
                sprite.setTextColor(TFT_CYAN, TFT_BLACK);
                sprite.drawString(globalOTAState.type, screenW / 2, 70);
                
                // Progress Bar
                int barWidth = screenW - 40;
                int barHeight = 10;
                int barX = 20;
                int barY = 90;
                
                // Draw Empty Bar
                sprite.drawRect(barX, barY, barWidth, barHeight, TFT_WHITE);
                
                // Draw Filled Bar
                int fillWidth = (barWidth - 4) * globalOTAState.percent / 100;
                if (fillWidth > 0) {
                    sprite.fillRect(barX + 2, barY + 2, fillWidth, barHeight - 4, TFT_GREEN);
                }
                
                // Percentage Text
                sprite.setTextColor(TFT_WHITE, TFT_BLACK);
                sprite.setTextSize(2);
                String percentStr = String(globalOTAState.percent) + "%";
                sprite.drawString(percentStr, screenW / 2, 120);
                
                sprite.pushSprite(0, 0);
                vTaskDelay(pdMS_TO_TICKS(50)); // Limit refresh rate
                continue; // Skip map rendering
            }

            // ===================================
            // SMART STARTUP LOGIC
            // ===================================
            if (currentStartupState != STARTUP_MAPPING) {
                 renderStartupScreen(currentControlParams);
            } else {
                 sprite.fillScreen(MAP_BACKGROUND_COLOR);
                 if (xSemaphoreTake(loadedTilesDataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                // Eviction logic: Remove tiles that are no longer within the 5x5 grid
                // This aligns with the "9+16" loading strategy, keeping a 5x5 buffer of loaded tiles.
                int minX_5x5 = newRequestedCenterTile.x - 2;
                int maxX_5x5 = newRequestedCenterTile.x + 2;
                int minY_5x5 = newRequestedCenterTile.y_tms - 2;
                int maxY_5x5 = newRequestedCenterTile.y_tms + 2;

                for (auto it = loadedTilesData.begin(); it != loadedTilesData.end(); ) {
                    const TileKey& tileKey = it->first;
                    if (tileKey.z == currentTileZ &&
                        (tileKey.x < minX_5x5 || tileKey.x > maxX_5x5 ||
                         tileKey.y_tms < minY_5x5 || tileKey.y_tms > maxY_5x5)) {
                        it = loadedTilesData.erase(it);
                    } else {
                        ++it;
                    }
                }

                if (!loadedTilesData.empty()) {
                    for (auto it = loadedTilesData.begin(); it != loadedTilesData.end(); ++it) {
                        const TileKey& tileKey = it->first;
                        std::vector<ParsedLayer, PSRAMAllocator<ParsedLayer>>& layers = it->second; // Use non-const reference

                        // Sort layers by drawOrder before rendering
                        std::sort(layers.begin(), layers.end(), [](const ParsedLayer& a, const ParsedLayer& b) {
                            return a.drawOrder < b.drawOrder;
                        });

                        for (const auto& layer : layers) {
                            for (const auto& feature : layer.features) {
                                // SKIP bridges on this pass - they'll be drawn in a second pass
                                // to ensure they always appear above water layers
                                if (!(feature.hasBridge && !feature.hasTunnel)) {
                                    drawParsedFeature(feature, layer.extent, tileKey, currentRenderParams);
                                }
                            }
                        }
                    }

                    // =========================================================
                    // SECOND PASS: Draw all bridges (ensuring they're above water)
                    // =========================================================
                    for (auto it = loadedTilesData.begin(); it != loadedTilesData.end(); ++it) {
                        const TileKey& tileKey = it->first;
                        std::vector<ParsedLayer, PSRAMAllocator<ParsedLayer>>& layers = it->second;

                        for (const auto& layer : layers) {
                            for (const auto& feature : layer.features) {
                                // ONLY draw bridges in this pass
                                if (feature.hasBridge && !feature.hasTunnel) {
                                    drawParsedFeature(feature, layer.extent, tileKey, currentRenderParams);
                                }
                            }
                        }
                    }
                } else {
                    Serial.println("Render Task: loadedTilesData is empty, no tiles to draw."); // Added debug print
                }
                xSemaphoreGive(loadedTilesDataMutex);
            } else {
                Serial.println("❌ Render Task: Failed to acquire mutex for loadedTilesData during drawing (timeout).");
            }

            // =========================================================
            // DRAW ROUTE OVERLAY (Blue polyline on top of map)
            // =========================================================
            if ((routeAvailable || (isRouteSyncing && !activeRoute.empty())) && xSemaphoreTake(routeMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (!activeRoute.empty()) {
                    // Re-index trigger
                    if (lastIndexedZoom != currentTileZ) {
                        indexRouteLocked();
                    }

                    // Calculate rotation params (common for all)
                    float cosTheta = cos(radians(currentRenderParams.mapRotationDegrees));
                    float sinTheta = sin(radians(currentRenderParams.mapRotationDegrees));
                    int centerX = screenW / 2;
                    int centerY = currentRenderParams.pivotY;

                    // Rotate points around the map pivot without perspective warping.
                    auto rotatePoint = [&](float sx, float sy, int &outX, int &outY) {
                        float tx = sx - centerX, ty = sy - centerY;
                        float rx = tx * cosTheta - ty * sinTheta;
                        float ry = tx * sinTheta + ty * cosTheta;
                        outX = (int)roundf(rx + centerX);
                        outY = (int)roundf(ry + centerY);
                    };

                    // ===========================================
                    // RENDER ALTERNATES (Light Gray)
                    // ===========================================
                    for (int rIdx = 0; rIdx < 2; rIdx++) {
                        if (!alternateRoutes[rIdx].empty()) {
                            // Use Anchor + Delta for first point
                            double altLat = alternateAnchors[rIdx].lat + alternateRoutes[rIdx][0].dLat / 100000.0;
                            double altLon = alternateAnchors[rIdx].lon + alternateRoutes[rIdx][0].dLon / 100000.0;
                            int lastX = -1, lastY = -1;
                            
                            for (size_t k = 0; k < alternateRoutes[rIdx].size(); k++) {
                                if (k > 0) {
                                    altLat += alternateRoutes[rIdx][k].dLat / 100000.0;
                                    altLon += alternateRoutes[rIdx][k].dLon / 100000.0;
                                }
                                
                                // Check bounds optimization
                                // if (abs(altLat - currentControlParams.targetLat) > 0.05) continue; // Rough check
                                
                                int tileX, tileY_std, tileY_TMS;
                                latlonToTile(altLat, altLon, currentTileZ, tileX, tileY_std, tileY_TMS);
                                int mvtX, mvtY;
                                latLonToMVTCoords(altLat, altLon, currentTileZ, tileX, tileY_TMS, mvtX, mvtY, currentRenderParams.layerExtent);

                                float tileRenderWidth = (float)screenW * currentRenderParams.zoomScaleFactor;
                                float tileRenderHeight = (float)screenH * currentRenderParams.zoomScaleFactor;
                                float tileOffsetX = (float)(tileX - currentRenderParams.centralTileX) * tileRenderWidth;
                                float tileOffsetY = (float)(currentRenderParams.centralTileY_TMS - tileY_TMS) * tileRenderHeight;
                                float scaleX = (float)screenW * currentRenderParams.zoomScaleFactor / currentRenderParams.layerExtent;
                                float scaleY = (float)screenH * currentRenderParams.zoomScaleFactor / currentRenderParams.layerExtent;
                                float screenX = (float)mvtX * scaleX + tileOffsetX - currentRenderParams.displayOffsetX;
                                float screenY = (float)mvtY * scaleY + tileOffsetY - currentRenderParams.displayOffsetY;

                                int finalX, finalY;
                                rotatePoint(screenX, screenY, finalX, finalY);

                                if (lastX != -1) {
                                    sprite.drawLine(lastX, lastY, finalX, finalY, ALTERNATE_ROUTE_COLOR);
                                }
                                lastX = finalX;
                                lastY = finalY;
                            }
                        }
                    }

                    // Refine trim point from live location so route overlay follows current position strictly.
                    int effectiveRouteProgressIndex = routeProgressIndex;
                    float effectiveRouteProgressFrac = routeProgressFrac;
                    if (activeRoute.size() >= 2 && (smoothLat != 0.0 || smoothLon != 0.0)) {
                        int baseIdx = routeProgressIndex;
                        if (baseIdx < 1) baseIdx = 1;
                        if (baseIdx > (int)activeRoute.size() - 2) baseIdx = (int)activeRoute.size() - 2;

                        // Keep the trim search forward-only so the visible route stays clipped behind the marker.
                        int startIdx = baseIdx;
                        if (startIdx < 1) startIdx = 1;
                        int endIdx = baseIdx + ((gpsSpeedForFilter > 4.0f) ? 18 : 12);
                        if (endIdx > (int)activeRoute.size() - 2) endIdx = (int)activeRoute.size() - 2;

                        // Build absolute position at startIdx by rewinding from current route progress point.
                        double segLat = currentRouteLat;
                        double segLon = currentRouteLon;
                        if (segLat == 0.0 && segLon == 0.0) {
                            segLat = activeRouteAnchor.lat;
                            segLon = activeRouteAnchor.lon;
                            for (int i = 1; i <= startIdx; ++i) {
                                segLat += activeRoute[i].dLat / ROUTE_SCALE;
                                segLon += activeRoute[i].dLon / ROUTE_SCALE;
                            }
                        } else if (startIdx < routeProgressIndex) {
                            for (int i = routeProgressIndex; i > startIdx; --i) {
                                segLat -= activeRoute[i].dLat / ROUTE_SCALE;
                                segLon -= activeRoute[i].dLon / ROUTE_SCALE;
                            }
                        } else if (startIdx > routeProgressIndex) {
                            for (int i = routeProgressIndex + 1; i <= startIdx; ++i) {
                                segLat += activeRoute[i].dLat / ROUTE_SCALE;
                                segLon += activeRoute[i].dLon / ROUTE_SCALE;
                            }
                        }

                        const float cosRef = cosf((float)smoothLat * (3.14159f / 180.0f));
                        const float mPerDegLon = 111320.0f * ((fabsf(cosRef) < 0.01f) ? 0.01f : cosRef);
                        const float mPerDegLat = 111320.0f;

                        float bestDistM = FLT_MAX;
                        int bestIdx = routeProgressIndex;
                        float bestFrac = routeProgressFrac;

                        for (int i = startIdx; i <= endIdx; ++i) {
                            double nextLat = segLat + activeRoute[i + 1].dLat / ROUTE_SCALE;
                            double nextLon = segLon + activeRoute[i + 1].dLon / ROUTE_SCALE;

                            float ax = (float)((segLon - smoothLon) * mPerDegLon);
                            float ay = (float)((segLat - smoothLat) * mPerDegLat);
                            float bx = (float)((nextLon - smoothLon) * mPerDegLon);
                            float by = (float)((nextLat - smoothLat) * mPerDegLat);

                            float abx = bx - ax;
                            float aby = by - ay;
                            float ab2 = abx * abx + aby * aby;

                            if (ab2 > 0.0001f) {
                                // Heading gate at driving speed to avoid snapping to wrong nearby segment.
                                if (gpsSpeedForFilter > 2.0f) {
                                    float segBearing = atan2f(abx, aby) * (180.0f / 3.14159f);
                                    if (segBearing < 0.0f) segBearing += 360.0f;
                                    float hdiff = shortestBearingDeltaDeg(gpsHeadingForFilter, segBearing);
                                    if (hdiff > 80.0f) {
                                        segLat = nextLat;
                                        segLon = nextLon;
                                        continue;
                                    }
                                }

                                float t = (-(ax * abx + ay * aby)) / ab2;
                                t = clampf(t, 0.0f, 1.0f);

                                float px = ax + t * abx;
                                float py = ay + t * aby;
                                float distM = sqrtf(px * px + py * py);

                                if (distM < bestDistM) {
                                    bestDistM = distM;
                                    bestIdx = i;
                                    bestFrac = t;
                                }
                            }

                            segLat = nextLat;
                            segLon = nextLon;
                        }

                        float maxTrimSnapM = (gpsSpeedForFilter < 3.0f) ? 7.5f : 14.0f;
                        bool forwardEnough = (bestIdx > routeProgressIndex) ||
                                             (bestIdx == routeProgressIndex && bestFrac >= routeProgressFrac + 0.02f);
                        if (bestDistM <= maxTrimSnapM && forwardEnough) {
                            effectiveRouteProgressIndex = bestIdx;
                            effectiveRouteProgressFrac = bestFrac;
                            // Persist forward progress so trimming does not bounce backward across frames.
                            routeProgressIndex = effectiveRouteProgressIndex;
                            routeProgressFrac = effectiveRouteProgressFrac;
                        }
                    }
                    
                    // Decouple route rendering from loaded map tiles
                    // Iterate directly over the route buckets
                    // xSemaphoreTake(loadedTilesDataMutex, pdMS_TO_TICKS(50)); // No longer needed for route

                    for (const auto& item : routeTileIndex) {
                        const TileKey& tKey = item.first;
                        
                        // Visibility Check optimization (Manhattan distance)
                        // Only draw tiles that are reasonably close to the center
                        // 2 tile radius is usually enough for 320x240 screen at standard zooms
                        if (abs(tKey.x - currentRenderParams.centralTileX) > 2 || 
                            abs(tKey.y_tms - currentRenderParams.centralTileY_TMS) > 2) {
                            continue;
                        }

                        // Draw all segments in this tile
                        for (const auto& seg : item.second) {
                            double currentLat = seg.startLat;
                            double currentLon = seg.startLon;
                                
                                // Render Loop for Segment
                                for (size_t k = 0; k < seg.count; k++) {
                                    size_t idx = seg.startIndex + k;
                                    
                                    // ROUTE TRIMMING: Skip points already passed
                                    if (idx < routeProgressIndex) {
                                        // Still need to update coordinates for next iteration!
                                        if (k < seg.count - 1) {
                                            size_t nextIdx = idx + 1;
                                            if (nextIdx < activeRoute.size()) {
                                                currentLat += activeRoute[nextIdx].dLat / 100000.0;
                                                currentLon += activeRoute[nextIdx].dLon / 100000.0;
                                            }
                                        }
                                        continue; 
                                    }

                                    // If this is the FIRST visible point (idx == routeProgressIndex),
                                    // force it to NOT draw a line from the previous (hidden) point.
                                    // This is handled automatically because we skipped the previous iteration,
                                    // so 'prevScreenX' logic which depends on k-1 flow needs check.
                                    // But wait, the loop calculates prevScreenX by backtracking if k==0.
                                    // If k > 0 but we skipped k-1, we need to ensure we don't use stale prevScreenX?
                                    // Actually, I should just set a flag or let the logic handle it.
                                    
                                    // Calculate current point screen position
                                    // ------------------------------------------
                                    // Reuse existing projection logic helper? 
                                    // Or inline it as before. Inlining for now to match style.
                                    
                                    int tileX, tileY_std, tileY_TMS;
                                    latlonToTile(currentLat, currentLon, currentTileZ, tileX, tileY_std, tileY_TMS);
                                    
                                    int mvtX, mvtY;
                                    latLonToMVTCoords(currentLat, currentLon, currentTileZ, tileX, tileY_TMS, mvtX, mvtY, currentRenderParams.layerExtent);
                                    
                                    float tileRenderWidth = (float)screenW * currentRenderParams.zoomScaleFactor;
                                    float tileRenderHeight = (float)screenH * currentRenderParams.zoomScaleFactor;
                                    float tileOffsetX = (float)(tileX - currentRenderParams.centralTileX) * tileRenderWidth;
                                    float tileOffsetY = (float)(currentRenderParams.centralTileY_TMS - tileY_TMS) * tileRenderHeight;
                                    
                                    float scaleX = (float)screenW * currentRenderParams.zoomScaleFactor / currentRenderParams.layerExtent;
                                    float scaleY = (float)screenH * currentRenderParams.zoomScaleFactor / currentRenderParams.layerExtent;
                                    
                                    float screenX = (float)mvtX * scaleX + tileOffsetX - currentRenderParams.displayOffsetX;
                                    float screenY = (float)mvtY * scaleY + tileOffsetY - currentRenderParams.displayOffsetY;
                                    
                                    // Rotation
                                    float translatedX = screenX - centerX;
                                    float translatedY = screenY - centerY;
                                    float rotatedX = translatedX * cosTheta - translatedY * sinTheta;
                                    float rotatedY = translatedX * sinTheta + translatedY * cosTheta;
                                    int finalX = round(rotatedX + centerX);
                                    int finalY = round(rotatedY + centerY);
                                    
                                    // Flat map mode: no perspective transform.
                                    
                                    // Determine Previous Point for pairing
                                    // ------------------------------------
                                    int prevScreenX = -1, prevScreenY = -1;
                                    
                                    if (k == 0) {
                                        // First point of segment.
                                        // If this is NOT the absolute start of route (idx 1 is first real delta point),
                                        // then back-calculate the PREVIOUS point from the delta.
                                        // activeRoute[idx] stores (current - prev).
                                        // So prev = current - delta.
                                        
                                        if (idx > 1) { // Index 0 is dummy, Index 1 is start. So if idx > 1, there is a previous point.
                                            double prevLat = currentLat - (activeRoute[idx].dLat / ROUTE_SCALE);
                                            double prevLon = currentLon - (activeRoute[idx].dLon / ROUTE_SCALE);
                                            
                                            // Project Previous Point
                                            int pTileX, pTileY_std, pTileY_TMS;
                                            latlonToTile(prevLat, prevLon, currentTileZ, pTileX, pTileY_std, pTileY_TMS);
                                            int pMvtX, pMvtY;
                                            latLonToMVTCoords(prevLat, prevLon, currentTileZ, pTileX, pTileY_TMS, pMvtX, pMvtY, currentRenderParams.layerExtent);
                                            
                                            float pTileOffsetX = (float)(pTileX - currentRenderParams.centralTileX) * tileRenderWidth;
                                            float pTileOffsetY = (float)(currentRenderParams.centralTileY_TMS - pTileY_TMS) * tileRenderHeight;
                                            
                                            float pScreenX = (float)pMvtX * scaleX + pTileOffsetX - currentRenderParams.displayOffsetX;
                                            float pScreenY = (float)pMvtY * scaleY + pTileOffsetY - currentRenderParams.displayOffsetY;
                                            
                                            float pTransX = pScreenX - centerX;
                                            float pTransY = pScreenY - centerY;
                                            float pRotX = pTransX * cosTheta - pTransY * sinTheta;
                                            float pRotY = pTransX * sinTheta + pTransY * cosTheta;
                                            
                                            int pFinalX = round(pRotX + centerX);
                                            int pFinalY = round(pRotY + centerY);
                                            
                                            // Perspective removed
                                            prevScreenX = pFinalX;
                                            prevScreenY = pFinalY;
                                        }
                                    } else {
                                        // Not first point, we could cache the previous finalX/Y from previous iteration?
                                        // But here we are re-calculating for simplicity of the loop structure inside block.
                                        // Actually optimization: current point becomes prev for next k.
                                        // But loop design here is "Calculate Current, Find Prev". 
                                        // Let's optimize: Store 'lastFinalX/Y' outside k loop.
                                        // But handling the k=0 case is the special logic.
                                        // Okay, let's keep it safe. k=0 handles boundary crossing.
                                        // k > 0 handles internal connections.
                                        // Need to ensure k>0 connects to k-1.
                                        // I will defer k>0 logic to the `prevScreenX` variable if I hoist it out.
                                    }
                                    
                                    // DRAW LINE
                                    // ---------
                                    // Issues with the above "k > 0" thought:
                                    // If I iterate k, I want to draw line (k-1 -> k).
                                    // For k=0, I draw line (boundary -> 0).
                                    // So I always draw a line ending at 'current'.
                                    
                                    // Wait, if I use the hoist method:
                                    // int lastX, lastY;
                                    // if k=0: calculate lastX/Y using backtrack.
                                    // else: lastX/Y = valid from previous iter.
                                    // Then draw last -> current.
                                    // Then last = current.
                                    
                                    // Refined Loop:
                                    // 1. Calculate Current Point Screen Coords (finalX, finalY)
                                    // 2. Identify Prev Screen Coords (prevScreenX, prevScreenY)
                                    //    If k==0: Backtrack calculation.
                                    //    If k>0:  Use stored value from previous iteration.
                                    
                                    // But wait, my manual backtrack code calculates prevScreenX/Y.
                                    // So:
                                    
                                    /* 
                                     int lastScreenX = -1, lastScreenY = -1;
                                     // ... inside k loop...
                                     // Calculate current finalX, finalY
                                     
                                     if (k == 0) {
                                        // Backtrack logic -> sets lastScreenX/Y
                                     } 
                                     
                                     if (lastScreenX != -1) {
                                        drawLine(lastScreenX, lastScreenY, finalX, finalY)
                                     }
                                     
                                     lastScreenX = finalX;
                                     lastScreenY = finalY;
                                     
                                     // Update lat/lon for next k
                                     if (k + 1 < seg.count) {
                                        currentLat += delta...
                                        currentLon += delta...
                                     }
                                    */
                                     
                                    // This looks cleaner!
                                } // end k loop
                                
                                // IMPLEMENTING THE CLEANER LOOP NOW
                                // Lambda to prevent code duplication for two-pass rendering
                                auto drawRoutePass = [&](bool drawFuture) {
                                    int lastScreenX = -9999, lastScreenY = -9999;
                                    double iterLat = seg.startLat;
                                    double iterLon = seg.startLon;
                                    
                                    for (size_t k = 0; k < seg.count; k++) {
                                        size_t idx = seg.startIndex + k;
                                        bool startFromCutPoint = false;
                                        
                                        // ROUTE TRIMMING: Skip points already passed
                                        if (idx < effectiveRouteProgressIndex) {
                                            if (k < seg.count - 1) {
                                                size_t nextIdx = idx + 1;
                                                if (nextIdx < activeRoute.size()) {
                                                    iterLat += activeRoute[nextIdx].dLat / 100000.0;
                                                    iterLon += activeRoute[nextIdx].dLon / 100000.0;
                                                }
                                            }
                                            continue; 
                                        }

                                        if (idx == effectiveRouteProgressIndex) {
                                            // Interpolate the exact cut-point on this segment
                                            // using routeProgressFrac so the line starts precisely
                                            // where the GPS position lies — not at the nearest vertex.
                                            int nxtIdx = (int)idx + 1;
                                            if (effectiveRouteProgressFrac > 0.0f && nxtIdx < (int)activeRoute.size()) {
                                                double nxtLat = iterLat + (activeRoute[nxtIdx].dLat / ROUTE_SCALE);
                                                double nxtLon = iterLon + (activeRoute[nxtIdx].dLon / ROUTE_SCALE);
                                                double cutLat = iterLat + effectiveRouteProgressFrac * (nxtLat - iterLat);
                                                double cutLon = iterLon + effectiveRouteProgressFrac * (nxtLon - iterLon);

                                                int cTileX, cTileY_std, cTileY_TMS;
                                                latlonToTile(cutLat, cutLon, currentTileZ, cTileX, cTileY_std, cTileY_TMS);
                                                int cMvtX, cMvtY;
                                                latLonToMVTCoords(cutLat, cutLon, currentTileZ, cTileX, cTileY_TMS, cMvtX, cMvtY, currentRenderParams.layerExtent);
                                                float cTileRenderWidth  = (float)screenW * currentRenderParams.zoomScaleFactor;
                                                float cTileRenderHeight = (float)screenH * currentRenderParams.zoomScaleFactor;
                                                float cTileOffX = (float)(cTileX - currentRenderParams.centralTileX) * cTileRenderWidth;
                                                float cTileOffY = (float)(currentRenderParams.centralTileY_TMS - cTileY_TMS) * cTileRenderHeight;
                                                float cScaleX = (float)screenW * currentRenderParams.zoomScaleFactor / currentRenderParams.layerExtent;
                                                float cScaleY = (float)screenH * currentRenderParams.zoomScaleFactor / currentRenderParams.layerExtent;
                                                float cSX = (float)cMvtX * cScaleX + cTileOffX - currentRenderParams.displayOffsetX;
                                                float cSY = (float)cMvtY * cScaleY + cTileOffY - currentRenderParams.displayOffsetY;
                                                rotatePoint(cSX, cSY, lastScreenX, lastScreenY);
                                                startFromCutPoint = true;
                                            } else {
                                                // No fraction yet — start from the vertex itself
                                                lastScreenX = -9999;
                                                lastScreenY = -9999;
                                            }
                                        }
                                        
                                        // 1. Calculate Screen Position using pure rotation
                                        int tileX, tileY_std, tileY_TMS;
                                        latlonToTile(iterLat, iterLon, currentTileZ, tileX, tileY_std, tileY_TMS);
                                        int mvtX, mvtY;
                                        latLonToMVTCoords(iterLat, iterLon, currentTileZ, tileX, tileY_TMS, mvtX, mvtY, currentRenderParams.layerExtent);

                                        float tileRenderWidth = (float)screenW * currentRenderParams.zoomScaleFactor;
                                        float tileRenderHeight = (float)screenH * currentRenderParams.zoomScaleFactor;
                                        float tileOffsetX = (float)(tileX - currentRenderParams.centralTileX) * tileRenderWidth;
                                        float tileOffsetY = (float)(currentRenderParams.centralTileY_TMS - tileY_TMS) * tileRenderHeight;
                                        float scaleX = (float)screenW * currentRenderParams.zoomScaleFactor / currentRenderParams.layerExtent;
                                        float scaleY = (float)screenH * currentRenderParams.zoomScaleFactor / currentRenderParams.layerExtent;
                                        float screenX = (float)mvtX * scaleX + tileOffsetX - currentRenderParams.displayOffsetX;
                                        float screenY = (float)mvtY * scaleY + tileOffsetY - currentRenderParams.displayOffsetY;

                                        int finalX, finalY;
                                        rotatePoint(screenX, screenY, finalX, finalY);

                                        if (startFromCutPoint) {
                                            int nextIdx = (int)idx + 1;
                                            if (nextIdx < (int)activeRoute.size()) {
                                                double nextLat = iterLat + (activeRoute[nextIdx].dLat / ROUTE_SCALE);
                                                double nextLon = iterLon + (activeRoute[nextIdx].dLon / ROUTE_SCALE);
                                                int nTileX, nTileY_std, nTileY_TMS;
                                                latlonToTile(nextLat, nextLon, currentTileZ, nTileX, nTileY_std, nTileY_TMS);
                                                int nMvtX, nMvtY;
                                                latLonToMVTCoords(nextLat, nextLon, currentTileZ, nTileX, nTileY_TMS, nMvtX, nMvtY, currentRenderParams.layerExtent);
                                                float nTileOffsetX = (float)(nTileX - currentRenderParams.centralTileX) * tileRenderWidth;
                                                float nTileOffsetY = (float)(currentRenderParams.centralTileY_TMS - nTileY_TMS) * tileRenderHeight;
                                                float nScreenX = (float)nMvtX * scaleX + nTileOffsetX - currentRenderParams.displayOffsetX;
                                                float nScreenY = (float)nMvtY * scaleY + nTileOffsetY - currentRenderParams.displayOffsetY;
                                                rotatePoint(nScreenX, nScreenY, finalX, finalY);
                                            }
                                        }
                                        
                                        // 2. Previous point (backtrack at segment start)
                                        if (k == 0 && !startFromCutPoint) {
                                            if (idx > 0) {
                                                double prevLat = iterLat - (activeRoute[idx].dLat / ROUTE_SCALE);
                                                double prevLon = iterLon - (activeRoute[idx].dLon / ROUTE_SCALE);
                                                int pTileX, pTileY_std, pTileY_TMS;
                                                latlonToTile(prevLat, prevLon, currentTileZ, pTileX, pTileY_std, pTileY_TMS);
                                                int pMvtX, pMvtY;
                                                latLonToMVTCoords(prevLat, prevLon, currentTileZ, pTileX, pTileY_TMS, pMvtX, pMvtY, currentRenderParams.layerExtent);
                                                float pTileOffsetX = (float)(pTileX - currentRenderParams.centralTileX) * tileRenderWidth;
                                                float pTileOffsetY = (float)(currentRenderParams.centralTileY_TMS - pTileY_TMS) * tileRenderHeight;
                                                float pScreenX = (float)pMvtX * scaleX + pTileOffsetX - currentRenderParams.displayOffsetX;
                                                float pScreenY = (float)pMvtY * scaleY + pTileOffsetY - currentRenderParams.displayOffsetY;
                                                rotatePoint(pScreenX, pScreenY, lastScreenX, lastScreenY);
                                            } else {
                                                lastScreenX = -9999;
                                            }
                                        }
                                        
                                        // 3. Draw Line from Last to Current
                                        if (lastScreenX != -9999 && 
                                            lastScreenX >= -100 && lastScreenX <= screenW + 100 &&
                                            lastScreenY >= -100 && lastScreenY <= screenH + 100) {
                                            
                                            if (drawFuture) {
                                                uint16_t color = FUTURE_ROUTE_COLOR;
                                                for (int offset = -1; offset <= 1; offset++) {
                                                    drawAntiAliasedLine(lastScreenX + offset, lastScreenY, finalX + offset, finalY, color);
                                                }
                                            }
                                        }
                                        
                                        // 4. Update state
                                        lastScreenX = finalX;
                                        lastScreenY = finalY;
                                        
                                        if (k + 1 < seg.count) {
                                            iterLat += (activeRoute[idx + 1].dLat / ROUTE_SCALE);
                                            iterLon += (activeRoute[idx + 1].dLon / ROUTE_SCALE);
                                        }
                                    }
                                };
                                
                                // Forward-only route render: only draw the portion ahead of the marker.
                                drawRoutePass(true);
                            }
                            }
                        }
                        
                        // =========================================================
                        // INTERMEDIATE WAYPOINT MARKERS (Multi-Stop)
                        // =========================================================
                        if (waypointBuffer.size() >= 2) {
                            // waypointBuffer contains: [Stop1, Stop2, ..., Destination]
                            // (Start position is NOT included - only intermediate stops + destination)
                            // Draw intermediate waypoints (skip last=destination)
                            
                            // Calculate rotation (reusable for all waypoints)
                            float wpCosTheta = cos(radians(currentRenderParams.mapRotationDegrees));
                            float wpSinTheta = sin(radians(currentRenderParams.mapRotationDegrees));
                            
                            // Draw intermediate stops (indices 0 to N-2, excluding last destination)
                            for (size_t wpIdx = 0; wpIdx < waypointBuffer.size() - 1; wpIdx++) {
                                // Skip past waypoints (already visited)
                                if (wpIdx < currentWaypointIndex) continue;
                                
                                double wpLat = waypointBuffer[wpIdx].lat;
                                double wpLon = waypointBuffer[wpIdx].lon;
                                
                                // Project to screen coords
                                int wpTileX, wpTileY_std, wpTileY_TMS;
                                latlonToTile(wpLat, wpLon, currentTileZ, wpTileX, wpTileY_std, wpTileY_TMS);
                                int wpMvtX, wpMvtY;
                                latLonToMVTCoords(wpLat, wpLon, currentTileZ, wpTileX, wpTileY_TMS, wpMvtX, wpMvtY, currentRenderParams.layerExtent);
                                
                                float wpTileRenderWidth = (float)screenW * currentRenderParams.zoomScaleFactor;
                                float wpTileRenderHeight = (float)screenH * currentRenderParams.zoomScaleFactor;
                                float wpTileOffsetX = (float)(wpTileX - currentRenderParams.centralTileX) * wpTileRenderWidth;
                                float wpTileOffsetY = (float)(currentRenderParams.centralTileY_TMS - wpTileY_TMS) * wpTileRenderHeight;
                                float wpScaleX = wpTileRenderWidth / currentRenderParams.layerExtent;
                                float wpScaleY = wpTileRenderHeight / currentRenderParams.layerExtent;
                                float wpScreenX = (float)wpMvtX * wpScaleX + wpTileOffsetX - currentRenderParams.displayOffsetX;
                                float wpScreenY = (float)wpMvtY * wpScaleY + wpTileOffsetY - currentRenderParams.displayOffsetY;
                                
                                // Apply rotation
                                float wpTransX = wpScreenX - (screenW / 2);
                                float wpTransY = wpScreenY - currentRenderParams.pivotY;
                                float wpRotX = wpTransX * wpCosTheta - wpTransY * wpSinTheta;
                                float wpRotY = wpTransX * wpSinTheta + wpTransY * wpCosTheta;
                                int wpX = round(wpRotX + (screenW / 2));
                                int wpY = round(wpRotY + currentRenderParams.pivotY);
                                
                                // Clamp to screen edges if off-screen (floating marker effect)
                                const int MARKER_MARGIN = 10; // Distance from screen edge
                                int drawX = wpX;
                                int drawY = wpY;
                                
                                // Clamp X coordinate to screen bounds
                                if (drawX < MARKER_MARGIN) drawX = MARKER_MARGIN;
                                if (drawX > screenW - MARKER_MARGIN) drawX = screenW - MARKER_MARGIN;
                                
                                // Clamp Y coordinate to screen bounds  
                                if (drawY < STATUS_BAR_HEIGHT + MARKER_MARGIN) drawY = STATUS_BAR_HEIGHT + MARKER_MARGIN;
                                if (drawY > screenH - MARKER_MARGIN) drawY = screenH - MARKER_MARGIN;
                                
                                // Color based on waypoint status
                                // Current waypoint (where we're navigating to now): Blue (same as active route)
                                // Future waypoints: Orange
                                uint16_t wpColor = (wpIdx == currentWaypointIndex) ? 0x059A : WAYPOINT_MARKER_COLOR;
                                
                                // Draw waypoint marker (Circle for intermediate stops)
                                drawTriangleAndCircle(drawX, drawY, 0, wpColor, false, true);
                            }
                        }
                        
                        // =========================================================
                        // DESTINATION MARKER (Final Destination from waypointBuffer)
                        // =========================================================
                        if (!waypointBuffer.empty()) {
                            // Use the LAST waypoint as the final destination
                            double destLat = waypointBuffer.back().lat;
                            double destLon = waypointBuffer.back().lon;
                            
                            // Project to screen coords
                            int dTileX, dTileY_std, dTileY_TMS;
                            latlonToTile(destLat, destLon, currentTileZ, dTileX, dTileY_std, dTileY_TMS);
                            int dMvtX, dMvtY;
                            latLonToMVTCoords(destLat, destLon, currentTileZ, dTileX, dTileY_TMS, dMvtX, dMvtY, currentRenderParams.layerExtent);
                            
                            float dTileRenderWidth = (float)screenW * currentRenderParams.zoomScaleFactor;
                            float dTileRenderHeight = (float)screenH * currentRenderParams.zoomScaleFactor;
                            float dTileOffsetX = (float)(dTileX - currentRenderParams.centralTileX) * dTileRenderWidth;
                            float dTileOffsetY = (float)(currentRenderParams.centralTileY_TMS - dTileY_TMS) * dTileRenderHeight;
                            float dScaleX = dTileRenderWidth / currentRenderParams.layerExtent;
                            float dScaleY = dTileRenderHeight / currentRenderParams.layerExtent;
                            float dScreenX = (float)dMvtX * dScaleX + dTileOffsetX - currentRenderParams.displayOffsetX;
                            float dScreenY = (float)dMvtY * dScaleY + dTileOffsetY - currentRenderParams.displayOffsetY;
                            
                            // Apply rotation
                            float destCosTheta = cos(radians(currentRenderParams.mapRotationDegrees));
                            float destSinTheta = sin(radians(currentRenderParams.mapRotationDegrees));
                            float dTransX = dScreenX - (screenW / 2);
                            float dTransY = dScreenY - currentRenderParams.pivotY;
                            float dRotX = dTransX * destCosTheta - dTransY * destSinTheta;
                            float dRotY = dTransX * destSinTheta + dTransY * destCosTheta;
                            int destX = round(dRotX + (screenW / 2));
                            int destY = round(dRotY + currentRenderParams.pivotY);
                            
                            // Clamp to screen edges if off-screen (floating marker effect)
                            const int MARKER_MARGIN = 10;
                            int drawX = destX;
                            int drawY = destY;
                            
                            // Clamp X coordinate to screen bounds
                            if (drawX < MARKER_MARGIN) drawX = MARKER_MARGIN;
                            if (drawX > screenW - MARKER_MARGIN) drawX = screenW - MARKER_MARGIN;
                            
                            // Clamp Y coordinate to screen bounds
                            if (drawY < STATUS_BAR_HEIGHT + MARKER_MARGIN) drawY = STATUS_BAR_HEIGHT + MARKER_MARGIN;
                            if (drawY > screenH - MARKER_MARGIN) drawY = screenH - MARKER_MARGIN;
                            
                            // Color based on progress: Blue if it's the current target, Orange if future
                            // Destination is current target if currentWaypointIndex >= all intermediate waypoints
                            size_t numIntermediateWaypoints = waypointBuffer.size() - 1; // Exclude destination itself
                            bool isCurrentTarget = (currentWaypointIndex >= numIntermediateWaypoints);
                            uint16_t destColor = isCurrentTarget ? 0x059A : WAYPOINT_MARKER_COLOR; // Blue for current, Orange for future
                            
                            // Draw destination marker (Circle, not triangle)
                            drawTriangleAndCircle(drawX, drawY, 0, destColor, false, true);
                        }
                        
                xSemaphoreGive(routeMutex);
            }

            // Draw elevation bubble (bridge/tunnel) pinned to the map at the road side
            if (routeAvailable && !activeManeuverList.empty()) {
                float cosT = cosf(radians(currentRenderParams.mapRotationDegrees));
                float sinT = sinf(radians(currentRenderParams.mapRotationDegrees));
                drawElevationBubbleOnMap(sprite, currentRenderParams, cosT, sinT, currentRenderParams.pivotY);
            }

            // Draw the navigation arrow, using the calculated base center Y
            drawNavigationArrow(screenW / 2, arrowBaseCenterY, arrowSize, NAVIGATION_ARROW_COLOR);
            } // End of STARTUP_MAPPING else block

            int statusBarY = 0; // Set status bar to the top
            for (int y = statusBarY; y < STATUS_BAR_HEIGHT; ++y) { // Loop for the height of the status bar
                for (int x = 0; x < screenW; ++x) {
                    // Check if we are in a non-mapping state (White BG)
                    // If so, we might want a different blending or solid bar?
                    // For now, standard blending
                    uint16_t currentPixelColor = sprite.readPixel(x, y);
                    uint16_t blendedColor = blendColors(currentPixelColor, STATUS_BAR_COLOR, STATUS_BAR_ALPHA);
                    sprite.drawPixel(x, y, blendedColor);
                }
            }
            
            // STARTUP OVERLAY FIX: Ensure status icons are visible on white background
            // If currentStartupState != STARTUP_MAPPING, background is white.
            // Status bar draws a semi-transparent black bar, so white icons should be visible on top.
            // But we used black text/icons for white bg?
            // Existing icons are colored (Red/Green/Blue). They should be visible on dark status bar.

            // Draw GPS icon on the top left of the status bar
            // Center X for GPS icon: half its width + a small margin from left edge
            int gpsIconCenterX = (GPS_16_WIDTH / 2) + 2; // 2 pixels margin from left
            // Center Y for GPS icon: half status bar height
            int gpsIconCenterY = statusBarY + (STATUS_BAR_HEIGHT / 2);
            
            // Check for phone GPS timeout (10 seconds without any commands) to prevent flickering
            if (phoneGpsActive && (millis() - lastPhoneCommandTime > 10000)) {
                phoneGpsActive = false; // Mark as inactive after 10s timeout
            }
            
            // GPS Icon Color Logic:
            // - Red: No GPS module detected OR phone GPS timed out
            // - Blue: Phone GPS active (receiving location from phone)
            // - Green: GPS module has fix
            uint16_t gpsIconColor = TFT_RED; // Default: No module
            if (gpsModulePresent && gpsHasFix) {
                gpsIconColor = TFT_GREEN; // GPS module has fix
            } else if (phoneGpsActive) {
                gpsIconColor = TFT_BLUE; // Phone GPS active
            }
            drawIcon(IconType::GPS, gpsIconCenterX, gpsIconCenterY, gpsIconColor);
            
            // Draw GPS rate next to the icon (only when phone GPS active)
            // Decay GPS Rate if idle
            if (xSemaphoreTake(gpsMutex, 0) == pdTRUE) {
                 if (millis() - gpsState.timestamp > 1500) {
                      gpsRate = 0.0f;
                 }
                 xSemaphoreGive(gpsMutex);
            }

            if (phoneGpsActive && gpsRate > 0.1f) {
                // GPS Rate display removed based on user request
            }

            // Speed display (km/h) — centred in the top status bar
            {
                float speedKmh = 0.0f;
                if (xSemaphoreTake(gpsMutex, 0) == pdTRUE) {
                    speedKmh = gpsState.speed * 3.6f; // m/s → km/h
                    xSemaphoreGive(gpsMutex);
                }
                char speedBuf[16];
                snprintf(speedBuf, sizeof(speedBuf), "%dkm/h", (int)roundf(speedKmh));
                sprite.setTextSize(1);
                sprite.setTextDatum(MC_DATUM);
                sprite.setTextColor(TFT_YELLOW, 0x0000);
                sprite.drawString(speedBuf, screenW / 2, statusBarY + (STATUS_BAR_HEIGHT / 2));
                sprite.setTextDatum(TL_DATUM); // Restore default
            }

            // Draw Connected icon on the top right of the status bar
            // Center X for Connected icon: screen width - half its width - a small margin from right edge
            int connectedIconCenterX = screenW - (CONNECTED_16_WIDTH / 2) - 2; // 2 pixels margin from right
            int connectedIconCenterY = statusBarY + (STATUS_BAR_HEIGHT / 2);
            
            // Determine Color and Visibility based on bleIconMode
            // 0: Blink Red (Window Open, Not Connected)
            // 1: Solid Red (Disconnected or Connected but Unauth)
            // 2: Solid Blue (Authenticated)
            uint16_t connectedIconColor = TFT_RED; // Default Red
            bool shouldDrawConnectedIcon = true;
            
            if (currentControlParams.bleIconMode == 0) {
                connectedIconColor = TFT_RED;
                shouldDrawConnectedIcon = iconBlinkVisible;
            } else if (currentControlParams.bleIconMode == 2) {
                connectedIconColor = TFT_BLUE;
            } else {
                // Mode 1: Solid Red
                connectedIconColor = TFT_RED;
            }

            if (shouldDrawConnectedIcon) {
                drawIcon(IconType::Connected, connectedIconCenterX, connectedIconCenterY, connectedIconColor); 
            }



            // Draw Maneuver Instruction Bar
            drawManeuverIndicator(sprite);

            // PIN Overlay - Drawn on top of everything if enabled
            if (currentControlParams.showPIN) {
                // Improved PIN Overlay UI
                int boxW = screenW - 20;
                int boxH = 90;
                int boxX = 10;
                int boxY = 40;

                // Semi-transparent background (simulated by solid color on sprite)
                sprite.fillRoundRect(boxX, boxY, boxW, boxH, 8, TFT_BLACK); 
                sprite.drawRoundRect(boxX, boxY, boxW, boxH, 8, TFT_CYAN);
                
                // Header "PAIR DEVICE"
                sprite.setTextSize(1);
                sprite.setTextColor(TFT_CYAN, TFT_BLACK); 
                int headerWidth = sprite.textWidth("PAIRING REQUEST"); // Changed text to be more descriptive but smaller font
                sprite.setCursor((screenW - headerWidth) / 2, boxY + 12);
                sprite.print("PAIRING REQUEST");
                
                // PIN Code (Large & Central)
                sprite.setTextSize(4);
                sprite.setTextColor(TFT_WHITE, TFT_BLACK);
                int pinWidth = sprite.textWidth(currentControlParams.pinCode);
                sprite.setCursor((screenW - pinWidth) / 2, boxY + 35);
                sprite.print(currentControlParams.pinCode);
                
                // Device Name (Small footer)
                sprite.setTextSize(1);
                sprite.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
                int nameWidth = sprite.textWidth(currentControlParams.deviceName);
                sprite.setCursor((screenW - nameWidth) / 2, boxY + 75);
                sprite.print(currentControlParams.deviceName);

                // Reset text size for other elements
                sprite.setTextSize(1); 

                // Debug print (throttle to not spam)
                static unsigned long lastPinLog = 0;
                if (millis() - lastPinLog > 2000) {
                   // Log less frequently
                   // Serial.printf("Render: PIN Overlay Active. PIN: %s\n", currentControlParams.pinCode);
                   lastPinLog = millis();
                }
            }

            // ROUTE SYNC OVERLAY
            if (isRouteSyncing) {
                int textY = 5; // Even higher, practically at the top

                // No Box, just text
                // sprite.fillRoundRect... (Removed)
                // sprite.drawRoundRect... (Removed)

                sprite.setTextColor(TFT_WHITE, TFT_BLACK); // High contrast text
                sprite.setTextDatum(MC_DATUM); // Middle-Center
                sprite.setTextSize(1); 
                
                if (routeTotalBytes > 0) {
                     int percent = (routeSyncProgressBytes * 100) / routeTotalBytes;
                     if (percent > 100) percent = 100;
                     if (percent < 0) percent = 0;
                     
                     char msg[32];
                     snprintf(msg, sizeof(msg), "Syncing Routes %d%%", percent);
                     sprite.drawString(msg, screenW / 2, textY + 10);
                } else {
                     sprite.drawString("Syncing Routes...", screenW / 2, textY + 10);
                }
                
                // Reset Text Datum
                sprite.setTextDatum(TL_DATUM); 
            }



            sprite.pushSprite(0, 0); // Only push if update is needed

            // lastSentRotationAngle = internalCurrentRotationAngle; // REMOVED: Managed by Deadband Logic now
            lastSentZoomFactor = currentControlParams.zoomFactor;
        }

        unsigned long frameElapsedMs = millis() - now;
        if (frameElapsedMs < RENDER_FRAME_MIN_MS) {
            vTaskDelay(pdMS_TO_TICKS(RENDER_FRAME_MIN_MS - frameElapsedMs));
        } else {
            vTaskDelay(pdMS_TO_TICKS(1)); // Always yield if frame already exceeded budget
        }
    }
}
