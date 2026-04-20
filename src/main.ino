// Flash Bee — handheld AS3935 lightning detector.
// Reworked against the AS3935 datasheet rev 1.07 §8.10–§8.11.
// NOT a life-safety device. Always follow the NWS 30/30 rule.

#include <Wire.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <math.h>
#include <Preferences.h>

// ── Display ─────────────────────────────────────────────────────
TFT_eSPI    tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);
#define CX  120
#define CY  120

// ── Colors ──────────────────────────────────────────────────────
#define C_BG      0x0000
#define C_GOLD    0xFEA0
#define C_ORANGE  0xFBE0
#define C_REDORG  0xF9A0
#define C_RED     0xF800
#define C_DIM     0x2104
#define C_DIMRING 0x10A2
#define C_GREEN   0x07E0
#define C_WHITE   0xFFFF
#define C_GREY    0x4208

// ── AS3935 register map (datasheet §8.10) ──────────────────────
#define AS3935_I2C_ADDR       0x03

#define REG_AFE_GAIN          0x00
  #define AFE_GB_FIELD_SHIFT  1
  #define AFE_GB_FIELD_MASK   (0x1F << AFE_GB_FIELD_SHIFT)
  // Datasheet values for the 5-bit AFE_GB field, placed into bits [5:1]:
  #define AFE_GB_INDOOR       (0b10010 << AFE_GB_FIELD_SHIFT)  // 0x24
  #define AFE_GB_OUTDOOR      (0b01110 << AFE_GB_FIELD_SHIFT)  // 0x1C
  #define PWD_BIT             0x01

#define REG_NF_WDG            0x01
  #define NF_MASK             (0x07 << 4)
  #define WDTH_MASK           0x0F

#define REG_CLSTAT_SREJ       0x02
  #define CL_STAT_EN_BIT      (1 << 7)
  #define CL_STAT_BIT         (1 << 6)
  #define MIN_NUM_LIGH_MASK   (0x03 << 4)
  #define SREJ_MASK           0x0F

#define REG_LCO_INT           0x03
  #define MASK_DIST_BIT       (1 << 5)
  #define INT_MASK            0x0F
  #define INT_NH              0x01
  #define INT_D               0x04
  #define INT_L               0x08

#define REG_ENERGY_LSB        0x04
#define REG_ENERGY_MID        0x05
#define REG_ENERGY_MSB        0x06

#define REG_DISTANCE          0x07
  #define DIST_FIELD_MASK     0x3F
  #define DIST_OUT_OF_RANGE   0x3F
  #define DIST_OVERHEAD       0x01
  #define DIST_UNKNOWN        0x00

#define REG_TUN_CAP           0x08
  #define TUN_CAP_MASK        0x0F
  #define DISP_TRCO_BIT       (1 << 5)

#define REG_TRCO_CAL          0x3A
#define REG_SRCO_CAL          0x3B
  #define CALIB_DONE_BIT      (1 << 7)
  #define CALIB_NOK_BIT       (1 << 6)

#define REG_PRESET_DEFAULT    0x3C
#define REG_CALIB_RCO         0x3D
#define DIRECT_CMD_VALUE      0x96

// ── Tunables ────────────────────────────────────────────────────
// Handheld → outdoor by default. Override with build_flags:
//   -DAS3935_AFE_GB=AFE_GB_INDOOR
#ifndef AS3935_AFE_GB
  #define AS3935_AFE_GB     AFE_GB_OUTDOOR
#endif
// Antenna LC tank tune (0..15). Factory-tuned per board; verify by
// watching the LCO on IRQ (reg 0x08 bit 7). Leave 0 until tuned.
#ifndef AS3935_TUN_CAP
  #define AS3935_TUN_CAP    0
#endif

// I²C uses the XIAO variant's default SDA/SCL pins (D4/D5 on the
// silkscreen); the underlying GPIO numbers differ between C3 and C6
// but `Wire.begin(SDA, SCL)` resolves correctly on either.
#define I2C_HZ              100000
#define I2C_TIMEOUT_MS      50
#define I2C_FAIL_LIMIT      8

// AS3935 IRQ pin → XIAO GPIO. Required for antenna tune; a separate
// fly-lead from the AS3935 module's INT pad to a free XIAO pin.
// Instructables build already asks you to remove the Grove connector
// and solder direct wires — add one more for INT.
// Default: D2 on the XIAO silkscreen (GPIO4 on C3, GPIO2 on C6).
#ifndef AS3935_INT_PIN
  #define AS3935_INT_PIN    D2
#endif

// Antenna-tune params (datasheet §8.11, app-note AN-LDS1.1)
// LCO routed to IRQ pin via DISP_LCO, divided by LCO_FDIV=128 so the
// output is slow enough for attachInterrupt() on a 160 MHz MCU to
// count reliably. Target: 500 kHz / 128 ≈ 3906 Hz, tolerance ±3.5 %.
#define LCO_TARGET_HZ       500000UL
#define LCO_DIV_SHIFT       7                           // 128 = 1<<7
#define LCO_EXPECTED_HZ     (LCO_TARGET_HZ >> LCO_DIV_SHIFT)
#define LCO_TOL_HZ          ((LCO_EXPECTED_HZ * 35) / 1000)
#define TUNE_WINDOW_MS      200
#define TUNE_SETTLE_MS      60
#define TUNE_PROMPT_MS      5000                        // 5 s on first boot
#define TUNE_PROMPT_MS_SAVED 1500                       // 1.5 s on subsequent boots

#define SENSE_EASE_MS       15000UL   // loosen filters after quiet
#define NF_DECAY_MS         60000UL   // drop NF after sustained quiet
#define DIST_STALE_MS       300000UL  // dim the hero value after 5 min
#define SENSOR_WDT_MS       600000UL  // re-init if no IRQ in 10 min

#define NVS_NAMESPACE       "flashbee"
#define NVS_KEY_TUNCAP      "tuncap"
#define NVS_KEY_TUNHZ       "tunhz"

// ── Sensor + UI state ──────────────────────────────────────────
uint8_t  noiseFloor   = 2;
uint8_t  watchdogLvl  = 2;
uint8_t  spikeRej     = 2;
uint32_t senseLastAdj = 0;
uint32_t nfLastDecay  = 0;
uint32_t lastIrqMs    = 0;

int      strikeCount  = 0;
uint32_t maxEnergy    = 0;
uint32_t lastEnergy   = 0;
uint8_t  lastDistRaw  = DIST_UNKNOWN;
uint32_t lastStrikeMs = 0;
float    radarAngle   = 0;

bool     sensorOk     = false;
bool     noiseFault   = false;
const bool outdoorMode = (AS3935_AFE_GB == AFE_GB_OUTDOOR);
uint8_t  i2cFailStreak = 0;

uint8_t  activeTunCap = AS3935_TUN_CAP;   // overridden from NVS at boot
uint32_t activeLcoHz  = 0;                 // 0 = untuned / unknown

Preferences prefs;

// ── Ticker ──────────────────────────────────────────────────────
#define MAX_TICKS 20
uint32_t tickEnergy[MAX_TICKS];
int      tickHead  = 0;
int      tickCount = 0;

// ── I²C with error tracking ────────────────────────────────────
bool i2cRead(uint8_t reg, uint8_t &out) {
  Wire.beginTransmission(AS3935_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) { i2cFailStreak++; return false; }
  if (Wire.requestFrom((uint8_t)AS3935_I2C_ADDR, (uint8_t)1) != 1) {
    i2cFailStreak++; return false;
  }
  if (!Wire.available()) { i2cFailStreak++; return false; }
  out = Wire.read();
  i2cFailStreak = 0;
  return true;
}

uint8_t readReg(uint8_t reg) { uint8_t v = 0xFF; i2cRead(reg, v); return v; }

bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(AS3935_I2C_ADDR);
  Wire.write(reg);
  Wire.write(val);
  if (Wire.endTransmission(true) != 0) { i2cFailStreak++; return false; }
  i2cFailStreak = 0;
  return true;
}

bool maskWrite(uint8_t reg, uint8_t mask, uint8_t val) {
  uint8_t cur;
  if (!i2cRead(reg, cur)) return false;
  return writeReg(reg, (cur & ~mask) | (val & mask));
}

// ── Energy + ticker ─────────────────────────────────────────────
uint32_t readEnergy() {
  return ((uint32_t)(readReg(REG_ENERGY_MSB) & 0x1F) << 16)
       | ((uint32_t) readReg(REG_ENERGY_MID)         <<  8)
       |             readReg(REG_ENERGY_LSB);
}
void pushTick(uint32_t e) {
  tickEnergy[tickHead] = e;
  tickHead = (tickHead + 1) % MAX_TICKS;
  if (tickCount < MAX_TICKS) tickCount++;
}

// ── Filter adjustments ─────────────────────────────────────────
// Datasheet §8.4: higher WDTH/SREJ = more disturber rejection =
// LOWER detection efficiency. The names below reflect that trade.
void applyWatchdogSpike() {
  maskWrite(REG_NF_WDG, WDTH_MASK, watchdogLvl & WDTH_MASK);
  maskWrite(REG_CLSTAT_SREJ, SREJ_MASK, spikeRej & SREJ_MASK);
}
void tightenFilters() {
  if (spikeRej < watchdogLvl) { if (spikeRej < 11) spikeRej++; }
  else                        { if (watchdogLvl < 10) watchdogLvl++; }
  applyWatchdogSpike();
}
void loosenFilters() {
  if (spikeRej > watchdogLvl) { if (spikeRej > 0) spikeRej--; }
  else                        { if (watchdogLvl > 0) watchdogLvl--; }
  applyWatchdogSpike();
}
void raiseNoiseFloor() {
  if (noiseFloor < 7) {
    noiseFloor++;
    maskWrite(REG_NF_WDG, NF_MASK, (noiseFloor << 4) & NF_MASK);
    noiseFault = false;
  } else {
    // At the hardware limit: environment is too noisy for the AS3935
    // to operate per datasheet. Surface this instead of hiding it.
    noiseFault = true;
  }
}
void lowerNoiseFloor() {
  if (noiseFloor > 0) {
    noiseFloor--;
    maskWrite(REG_NF_WDG, NF_MASK, (noiseFloor << 4) & NF_MASK);
  }
  noiseFault = false;
}

// ── Init sequence (datasheet §8.11) ────────────────────────────
bool initAS3935() {
  // 1. PRESET_DEFAULT: restore register defaults.
  if (!writeReg(REG_PRESET_DEFAULT, DIRECT_CMD_VALUE)) return false;
  delay(3);

  // 2. CALIB_RCO: calibrate the internal RCO timebase. Result lives
  // in volatile memory, so it must be re-run after every POR.
  if (!writeReg(REG_CALIB_RCO, DIRECT_CMD_VALUE)) return false;
  maskWrite(REG_TUN_CAP, DISP_TRCO_BIT, DISP_TRCO_BIT);
  delay(3);
  maskWrite(REG_TUN_CAP, DISP_TRCO_BIT, 0);

  uint8_t trco = readReg(REG_TRCO_CAL);
  uint8_t srco = readReg(REG_SRCO_CAL);
  if (!(trco & CALIB_DONE_BIT) || (trco & CALIB_NOK_BIT)) {
    Serial.print("[CAL] TRCO fail: 0x"); Serial.println(trco, HEX);
    return false;
  }
  if (!(srco & CALIB_DONE_BIT) || (srco & CALIB_NOK_BIT)) {
    Serial.print("[CAL] SRCO fail: 0x"); Serial.println(srco, HEX);
    return false;
  }

  // 3. AFE gain + PWD=0.
  if (!maskWrite(REG_AFE_GAIN, AFE_GB_FIELD_MASK | PWD_BIT, AS3935_AFE_GB))
    return false;

  // 4. Antenna tune cap (from NVS if calibrated, else build-time default).
  if (!maskWrite(REG_TUN_CAP, TUN_CAP_MASK, activeTunCap & TUN_CAP_MASK))
    return false;

  // 5. NF + WDTH.
  noiseFloor = 2; watchdogLvl = 2; spikeRej = 2;
  if (!writeReg(REG_NF_WDG, (noiseFloor << 4) | (watchdogLvl & WDTH_MASK)))
    return false;

  // 6. MIN_NUM_LIGH = 0 (single strike), SREJ, preserving CL_STAT_EN /
  // CL_STAT in bits [7:6].
  if (!maskWrite(REG_CLSTAT_SREJ,
                 MIN_NUM_LIGH_MASK | SREJ_MASK,
                 (0 << 4) | (spikeRej & SREJ_MASK))) return false;

  // 7. Unmask disturbers so the tuner can see them.
  if (!maskWrite(REG_LCO_INT, MASK_DIST_BIT, 0)) return false;

  return true;
}

// ── UI helpers ─────────────────────────────────────────────────
uint16_t energyColor(uint32_t e) {
  if      (e > 500000) return C_WHITE;
  else if (e > 150000) return C_GOLD;
  else if (e > 75000)  return C_ORANGE;
  else if (e > 20000)  return C_REDORG;
  else                 return C_DIM;
}
void drawGaugeArc(TFT_eSprite &s, uint16_t fillColor, float pct) {
  s.drawArc(CX, CY, 116, 106, 225, 135, 0x1082, C_BG, false);
  if (pct > 0.01f) {
    uint16_t endAngle = (uint16_t)(225 + pct * 270) % 360;
    s.drawArc(CX, CY, 116, 106, 225, endAngle, fillColor, C_BG, true);
  }
}
void drawGaugeTicks(TFT_eSprite &s, int count) {
  for (int i = 0; i <= count; i++) {
    float deg = 225 + (270.0f / count) * i;
    float rad = deg * DEG_TO_RAD;
    int x1 = CX + (int)(104 * cos(rad));
    int y1 = CY + (int)(104 * sin(rad));
    int x2 = CX + (int)(118 * cos(rad));
    int y2 = CY + (int)(118 * sin(rad));
    s.drawLine(x1, y1, x2, y2, 0x2945);
  }
}
void flashScreen() {
  for (int i = 0; i < 2; i++) {
    spr.fillSprite(0x2945); spr.pushSprite(0, 0); delay(35);
    spr.fillSprite(C_BG);   spr.pushSprite(0, 0); delay(20);
  }
}
void drawBoot(const char* line1, const char* line2, uint16_t color) {
  tft.fillScreen(C_BG);
  tft.drawCircle(CX, CY, 118, C_DIMRING);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(C_GOLD, C_BG);
  tft.setTextSize(2);
  tft.drawString(line1, CX, CY - 14);
  tft.setTextSize(1);
  tft.setTextColor(color, C_BG);
  tft.drawString(line2, CX, CY + 12);
}
void drawSensorLost() {
  spr.fillSprite(C_BG);
  spr.drawCircle(CX, CY, 118, C_DIMRING);
  spr.setTextDatum(MC_DATUM);
  spr.setTextColor(C_RED, C_BG);
  spr.setTextSize(2);
  spr.drawString("SENSOR", CX, CY - 18);
  spr.drawString("LOST",   CX, CY + 4);
  spr.setTextSize(1);
  spr.setTextColor(C_GREY, C_BG);
  spr.drawString("I2C fault - retrying", CX, CY + 32);
  spr.pushSprite(0, 0);
}

// ── Main UI ────────────────────────────────────────────────────
void drawUI() {
  if (!sensorOk) { drawSensorLost(); return; }

  spr.fillSprite(C_BG);
  float pct = (lastEnergy > 0) ? constrain(lastEnergy / 2097151.0f, 0.f, 1.f) : 0;
  drawGaugeArc(spr, energyColor(lastEnergy), pct);
  drawGaugeTicks(spr, 10);
  spr.drawCircle(CX, CY, 119, 0x2124);
  spr.drawCircle(CX, CY, 90,  C_DIM);
  spr.drawCircle(CX, CY, 58,  C_DIM);

  // Decorative radar sweep — AS3935 is non-directional. Position
  // of the sweep does NOT indicate strike bearing.
  float rad = radarAngle * DEG_TO_RAD;
  for (int len = 20; len <= 86; len += 4) {
    float    tr = (radarAngle - (86 - len) * 0.4f) * DEG_TO_RAD;
    uint8_t   g = (uint8_t)((len / 86.0f) * 40);
    spr.drawPixel(CX + (int)(len * cos(tr)),
                  CY + (int)(len * sin(tr)),
                  spr.color565(0, g, 0));
  }
  spr.drawLine(CX, CY,
               CX + (int)(86 * cos(rad)),
               CY + (int)(86 * sin(rad)), 0x05C0);

  spr.setTextDatum(TC_DATUM);
  spr.setTextColor(C_GREY, C_BG);
  spr.setTextSize(1);
  spr.drawString("LIGHTNING DETECTOR", CX, 16);

  if (noiseFault) {
    spr.setTextColor(C_RED, C_BG);
    spr.drawString("ENV TOO NOISY", CX, 27);
  } else {
    String tuneStr = (outdoorMode ? "OUT " : "IN  ");
    tuneStr += "WD:" + String(watchdogLvl) + " SR:" + String(spikeRej);
    bool tight = (watchdogLvl > 5 || spikeRej > 5);
    spr.setTextColor(tight ? C_ORANGE : C_DIM, C_BG);
    spr.drawString(tuneStr, CX, 27);
  }

  // Hero distance.
  spr.setTextDatum(MC_DATUM);
  bool stale = strikeCount > 0 && (millis() - lastStrikeMs) > DIST_STALE_MS;
  uint16_t heroColor = stale ? C_DIM : C_GOLD;

  if (strikeCount == 0) {
    spr.setTextColor(C_DIM, C_BG);
    spr.setTextSize(3); spr.drawString("--", CX, 84);
    spr.setTextSize(1); spr.setTextColor(C_GREY, C_BG);
    spr.drawString("listening...", CX, 108);
  } else if (lastDistRaw == DIST_UNKNOWN) {
    spr.setTextColor(C_DIM, C_BG);
    spr.setTextSize(3); spr.drawString("--", CX, 84);
    spr.setTextSize(1); spr.setTextColor(C_GREY, C_BG);
    spr.drawString("distance unknown", CX, 108);
  } else if (lastDistRaw == DIST_OUT_OF_RANGE) {
    spr.setTextColor(heroColor, C_BG);
    spr.setTextSize(3); spr.drawString(">40", CX, 80);
    spr.setTextSize(2); spr.drawString("km", CX, 105);
  } else if (lastDistRaw == DIST_OVERHEAD) {
    spr.setTextColor(C_REDORG, C_BG);
    spr.setTextSize(2); spr.drawString("OVERHEAD", CX, 78);
    spr.setTextColor(heroColor, C_BG);
    spr.drawString("< 6 km", CX, 100);
  } else {
    spr.setTextColor(heroColor, C_BG);
    spr.setTextSize(6); spr.drawString(String(lastDistRaw), CX, 80);
    spr.setTextSize(3); spr.drawString("km", CX, 112);
  }

  spr.drawFastHLine(44, 128, 152, C_DIM);

  spr.setTextDatum(ML_DATUM);
  spr.setTextColor(C_GREY, C_BG);
  spr.setTextSize(1);
  spr.drawString("STRIKES", 40, 140);
  spr.setTextColor(heroColor, C_BG);
  spr.setTextSize(2);
  spr.drawString(String(strikeCount), 40, 156);

  spr.setTextDatum(MR_DATUM);
  spr.setTextColor(C_GREY, C_BG);
  spr.setTextSize(1);
  spr.drawString("ENERGY", 200, 140);
  spr.setTextColor(energyColor(lastEnergy), C_BG);
  spr.setTextSize(2);
  String eStr = lastEnergy > 999 ? String(lastEnergy / 1000) + "K"
                                 : String(lastEnergy);
  spr.drawString(eStr, 200, 156);

  int bax = 32, bay = 176, baw = 176, bah = 30;
  int bw  = baw / MAX_TICKS - 1;
  spr.drawRoundRect(bax - 2, bay - 2, baw + 4, bah + 6, 4, C_DIM);
  for (int i = 0; i < tickCount; i++) {
    int idx = (tickHead - tickCount + i + MAX_TICKS) % MAX_TICKS;
    float bpct = constrain(tickEnergy[idx] / 2097151.0f, 0.f, 1.f);
    int bh = max(2, (int)(bpct * bah));
    int bx = bax + i * (bw + 1);
    int by = bay + bah - bh;
    uint16_t bc = (i == tickCount - 1) ? C_GOLD : energyColor(tickEnergy[idx]);
    spr.fillRect(bx, by, bw, bh, bc);
  }
  spr.setTextDatum(TC_DATUM);
  spr.setTextColor(C_GREY, C_BG);
  spr.setTextSize(1);
  spr.drawString(stale ? "HISTORY (stale)" : "ENERGY HISTORY", CX, 212);

  spr.pushSprite(0, 0);
}

// ── Antenna tune ───────────────────────────────────────────────
volatile uint32_t lcoEdgeCount = 0;
void IRAM_ATTR lcoISR() { lcoEdgeCount++; }

// Route LCO to INT pin, set FDIV=128. Returns true on success.
bool enableLcoOut() {
  // REG_LCO_INT bits [7:6] = LCO_FDIV; 11b = /128.
  if (!maskWrite(REG_LCO_INT, 0xC0, 0xC0)) return false;
  // REG_TUN_CAP bit 7 = DISP_LCO.
  if (!maskWrite(REG_TUN_CAP, 0x80, 0x80)) return false;
  return true;
}
bool disableLcoOut() { return maskWrite(REG_TUN_CAP, 0x80, 0); }

// Write TUN_CAP + count rising edges for windowMs. Returns Hz.
uint32_t measureLcoHz(uint8_t cap, uint16_t windowMs) {
  maskWrite(REG_TUN_CAP, TUN_CAP_MASK, cap & TUN_CAP_MASK);
  delay(TUNE_SETTLE_MS);
  noInterrupts(); lcoEdgeCount = 0; interrupts();
  uint32_t t0 = millis();
  while (millis() - t0 < windowMs) { delay(1); }
  noInterrupts(); uint32_t n = lcoEdgeCount; interrupts();
  return (uint32_t)((uint64_t)n * 1000UL / windowMs);
}

void drawTuneProgress(uint8_t cap, uint32_t hz, int32_t dev) {
  spr.fillSprite(C_BG);
  spr.drawCircle(CX, CY, 118, C_DIMRING);
  spr.setTextDatum(TC_DATUM);
  spr.setTextColor(C_GOLD, C_BG);
  spr.setTextSize(2);
  spr.drawString("ANTENNA TUNE", CX, 30);

  spr.setTextDatum(MC_DATUM);
  spr.setTextColor(C_GREY, C_BG);
  spr.setTextSize(1);
  char buf[24];
  snprintf(buf, sizeof(buf), "TUN_CAP %2u / 15", cap);
  spr.drawString(buf, CX, 80);

  spr.setTextColor(abs(dev) <= (int32_t)LCO_TOL_HZ ? C_GREEN : C_ORANGE, C_BG);
  spr.setTextSize(3);
  snprintf(buf, sizeof(buf), "%lu", (unsigned long)hz);
  spr.drawString(buf, CX, 110);
  spr.setTextSize(1);
  spr.setTextColor(C_GREY, C_BG);
  spr.drawString("Hz", CX, 134);

  snprintf(buf, sizeof(buf), "target %lu +-%lu",
           (unsigned long)LCO_EXPECTED_HZ, (unsigned long)LCO_TOL_HZ);
  spr.drawString(buf, CX, 150);

  // Progress bar
  int bx = 40, by = 180, bw = 160, bh = 14;
  spr.drawRoundRect(bx - 2, by - 2, bw + 4, bh + 4, 3, C_DIM);
  int filled = (int)((cap + 1) * bw / 16);
  spr.fillRect(bx, by, filled, bh, C_GOLD);

  spr.pushSprite(0, 0);
}

void drawTuneResult(bool ok, uint8_t cap, uint32_t hz) {
  spr.fillSprite(C_BG);
  spr.drawCircle(CX, CY, 118, C_DIMRING);
  spr.setTextDatum(TC_DATUM);
  spr.setTextColor(ok ? C_GREEN : C_RED, C_BG);
  spr.setTextSize(2);
  spr.drawString(ok ? "TUNED" : "OUT OF", CX, 26);
  if (!ok) spr.drawString("RANGE", CX, 48);

  spr.setTextDatum(MC_DATUM);
  char buf[32];
  spr.setTextColor(C_GOLD, C_BG);
  spr.setTextSize(3);
  snprintf(buf, sizeof(buf), "CAP=%u", cap);
  spr.drawString(buf, CX, 100);

  spr.setTextColor(C_GREY, C_BG);
  spr.setTextSize(2);
  snprintf(buf, sizeof(buf), "%lu Hz", (unsigned long)hz);
  spr.drawString(buf, CX, 140);

  spr.setTextSize(1);
  spr.setTextColor(ok ? C_GREY : C_ORANGE, C_BG);
  spr.drawString(ok ? "saved to NVS" : "not saved", CX, 170);
  spr.drawString("rebooting...", CX, 190);
  spr.pushSprite(0, 0);
}

void drawTuneError(const char* line1, const char* line2) {
  spr.fillSprite(C_BG);
  spr.drawCircle(CX, CY, 118, C_DIMRING);
  spr.setTextDatum(MC_DATUM);
  spr.setTextColor(C_RED, C_BG);
  spr.setTextSize(2);
  spr.drawString(line1, CX, CY - 20);
  spr.setTextSize(1);
  spr.setTextColor(C_GREY, C_BG);
  spr.drawString(line2, CX, CY + 8);
  spr.pushSprite(0, 0);
}

// Runs the full TUN_CAP sweep. Persists the best value to NVS if
// at least one cap reads a plausible LCO frequency. Returns true
// on success (tune completed, value saved).
bool runAntennaTune() {
  Serial.println(F("\n=== Antenna tune ==="));
  Serial.print  (F("Target: ")); Serial.print(LCO_EXPECTED_HZ);
  Serial.print  (F(" Hz +/- "));  Serial.print(LCO_TOL_HZ);
  Serial.println(F(" Hz (500 kHz/128, +/-3.5%)"));

  // Bring AS3935 to a known state for cal.
  if (!writeReg(REG_PRESET_DEFAULT, DIRECT_CMD_VALUE)) {
    drawTuneError("I2C FAIL", "no ACK from AS3935");
    return false;
  }
  delay(3);
  writeReg(REG_CALIB_RCO, DIRECT_CMD_VALUE);
  delay(3);

  if (!enableLcoOut()) {
    drawTuneError("I2C FAIL", "enable DISP_LCO failed");
    return false;
  }

  pinMode(AS3935_INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(AS3935_INT_PIN), lcoISR, RISING);

  uint32_t freqs[16];
  uint32_t maxHz = 0;
  int32_t  bestAbsDev = INT32_MAX;
  uint8_t  bestCap = 0;

  for (uint8_t cap = 0; cap < 16; cap++) {
    freqs[cap] = measureLcoHz(cap, TUNE_WINDOW_MS);
    int32_t dev = (int32_t)freqs[cap] - (int32_t)LCO_EXPECTED_HZ;
    Serial.printf("TUN_CAP=%2u -> %5lu Hz (%+5ld)\n",
                  cap, (unsigned long)freqs[cap], (long)dev);
    if (freqs[cap] > maxHz) maxHz = freqs[cap];
    int32_t adev = dev < 0 ? -dev : dev;
    if (adev < bestAbsDev) { bestAbsDev = adev; bestCap = cap; }
    drawTuneProgress(cap, freqs[cap], dev);
  }

  detachInterrupt(digitalPinToInterrupt(AS3935_INT_PIN));
  disableLcoOut();

  // Sanity: if nothing looks like a real LCO, the INT wire is missing.
  if (maxHz < 500) {
    Serial.println(F("ERROR: no LCO edges detected on AS3935_INT_PIN."));
    Serial.println(F("       Check the INT jumper from the AS3935 module"));
    Serial.print  (F("       to XIAO GPIO (arduino pin #"));
    Serial.print  (AS3935_INT_PIN);
    Serial.println(F(")"));
    drawTuneError("NO SIGNAL", "check INT wire");
    delay(4000);
    return false;
  }

  bool inTol = (bestAbsDev <= (int32_t)LCO_TOL_HZ);
  Serial.printf("best: TUN_CAP=%u @ %lu Hz (dev %+ld, %s)\n",
                bestCap, (unsigned long)freqs[bestCap],
                (long)((int32_t)freqs[bestCap] - (int32_t)LCO_EXPECTED_HZ),
                inTol ? "WITHIN TOL" : "OUT OF TOL");

  prefs.begin(NVS_NAMESPACE, false);
  prefs.putUChar(NVS_KEY_TUNCAP, bestCap);
  prefs.putUInt (NVS_KEY_TUNHZ, freqs[bestCap]);
  prefs.end();

  activeTunCap = bestCap;
  activeLcoHz  = freqs[bestCap];

  drawTuneResult(inTol, bestCap, freqs[bestCap]);
  delay(3500);
  return inTol;
}

// Poll serial for a "tune" line within windowMs. Non-blocking-ish.
bool pollForTuneCmd(uint32_t windowMs) {
  String buf;
  uint32_t start = millis();
  while (millis() - start < windowMs) {
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\r' || c == '\n') {
        buf.trim();
        if (buf.equalsIgnoreCase("tune")) return true;
        buf = "";
      } else {
        buf += c;
        if (buf.length() > 16) buf = "";
      }
    }
    delay(5);
  }
  return false;
}

// ── Setup ──────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(C_BG);
  spr.createSprite(240, 240);
  spr.setTextFont(1);
  memset(tickEnergy, 0, sizeof(tickEnergy));

  Wire.begin(SDA, SCL);
  Wire.setClock(I2C_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);

  // Load saved antenna calibration, if any.
  prefs.begin(NVS_NAMESPACE, true);
  bool haveSavedTune = prefs.isKey(NVS_KEY_TUNCAP);
  if (haveSavedTune) {
    activeTunCap = prefs.getUChar(NVS_KEY_TUNCAP, AS3935_TUN_CAP);
    activeLcoHz  = prefs.getUInt (NVS_KEY_TUNHZ, 0);
  }
  prefs.end();

  // Prompt for antenna tune over serial. Longer window on first boot
  // (no saved calibration) because the user should really do this.
  if (haveSavedTune) {
    Serial.printf("[tune] using saved TUN_CAP=%u (LCO %lu Hz)\n",
                  activeTunCap, (unsigned long)activeLcoHz);
    drawBoot("AS3935", "send 'tune' to recal", C_GREY);
  } else {
    Serial.println(F("[tune] no saved antenna calibration."));
    Serial.println(F("[tune] send 'tune' on serial to calibrate now."));
    drawBoot("AS3935", "send 'tune' to calibr", C_ORANGE);
  }

  if (pollForTuneCmd(haveSavedTune ? TUNE_PROMPT_MS_SAVED : TUNE_PROMPT_MS)) {
    runAntennaTune();
  }

  drawBoot("AS3935", "initializing...", C_GREY);
  sensorOk = initAS3935();
  if (sensorOk) {
    drawBoot("AS3935", outdoorMode ? "outdoor mode" : "indoor mode", C_GREEN);
    Serial.print("[AS3935] ready — "); Serial.println(outdoorMode ? "OUTDOOR" : "INDOOR");
    delay(800);
  } else {
    drawBoot("AS3935", "INIT FAILED", C_RED);
    Serial.println("[AS3935] init failed — will retry in loop");
    delay(1500);
  }

  uint32_t now = millis();
  senseLastAdj = now;
  nfLastDecay  = now;
  lastIrqMs    = now;

  drawUI();
}

// ── Loop ───────────────────────────────────────────────────────
void loop() {
  uint32_t now = millis();

  if (!sensorOk) {
    static uint32_t nextRetry = 0;
    if ((int32_t)(now - nextRetry) >= 0) {
      sensorOk = initAS3935();
      nextRetry = now + 3000;
      if (sensorOk) {
        Serial.println("[AS3935] recovered");
        lastIrqMs = now;
      }
    }
    radarAngle += 2.5f;
    if (radarAngle >= 360) radarAngle -= 360;
    drawUI();
    delay(80);
    return;
  }

  uint8_t intReg;
  if (!i2cRead(REG_LCO_INT, intReg)) {
    if (i2cFailStreak >= I2C_FAIL_LIMIT) {
      Serial.println("[I2C] sensor lost");
      sensorOk = false;
    }
    delay(50);
    return;
  }

  uint8_t reason = intReg & INT_MASK;
  if (reason != 0) lastIrqMs = now;

  if (reason == INT_NH) {
    Serial.println("[NH] noise floor too high");
    raiseNoiseFloor();
    senseLastAdj = now;
    nfLastDecay  = now;
  } else if (reason == INT_D) {
    Serial.println("[D] disturber");
    tightenFilters();
    senseLastAdj = now;
  } else if (reason == INT_L) {
    uint32_t e = readEnergy();
    uint8_t  d = readReg(REG_DISTANCE) & DIST_FIELD_MASK;
    strikeCount++;
    lastEnergy   = e;
    lastDistRaw  = d;
    lastStrikeMs = now;
    if (e > maxEnergy) maxEnergy = e;
    pushTick(e);
    Serial.printf("strike #%d d=0x%02X e=%lu\n",
                  strikeCount, d, (unsigned long)e);
    flashScreen();
  }

  if (now - senseLastAdj > SENSE_EASE_MS) {
    senseLastAdj = now;
    loosenFilters();
  }
  if (now - nfLastDecay > NF_DECAY_MS) {
    nfLastDecay = now;
    lowerNoiseFloor();
  }
  if (now - lastIrqMs > SENSOR_WDT_MS) {
    Serial.println("[WDT] no IRQ in watchdog window — re-init");
    sensorOk = initAS3935();
    lastIrqMs = now;
  }

  radarAngle += 2.5f;
  if (radarAngle >= 360) radarAngle -= 360;

  drawUI();
  delay(50);
}
