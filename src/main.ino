// Flash Bee — handheld AS3935 lightning detector.
// Reworked against the AS3935 datasheet rev 1.07 §8.10–§8.11.
// NOT a life-safety device. Always follow the NWS 30/30 rule.

#include <Wire.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <math.h>

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

#define I2C_SDA_PIN         6
#define I2C_SCL_PIN         7
#define I2C_HZ              100000
#define I2C_TIMEOUT_MS      50
#define I2C_FAIL_LIMIT      8

#define SENSE_EASE_MS       15000UL   // loosen filters after quiet
#define NF_DECAY_MS         60000UL   // drop NF after sustained quiet
#define DIST_STALE_MS       300000UL  // dim the hero value after 5 min
#define SENSOR_WDT_MS       600000UL  // re-init if no IRQ in 10 min

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

  // 4. Antenna tune cap.
  if (!maskWrite(REG_TUN_CAP, TUN_CAP_MASK, AS3935_TUN_CAP & TUN_CAP_MASK))
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

// ── Setup ──────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(C_BG);
  spr.createSprite(240, 240);
  spr.setTextFont(1);
  memset(tickEnergy, 0, sizeof(tickEnergy));

  drawBoot("AS3935", "initializing...", C_GREY);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);

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
