// Flash Bee — handheld AS3935 lightning detector.
// Reworked against the AS3935 datasheet rev 1.07 §8.10–§8.11.
// NOT a life-safety device. Always follow the NWS 30/30 rule.

#include <Wire.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <math.h>
#include <Preferences.h>
#include <esp_sleep.h>
#include <driver/gpio.h>

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

// Safety thresholds. NWS 30/30 rule: anything inside ~10 km means
// the next strike could be on top of you; stay in shelter until
// 30 min after the last close strike.
#define CLOSE_STRIKE_KM     10
#define DANGER_WINDOW_MS    (30UL * 60UL * 1000UL)

#define NVS_NAMESPACE       "flashbee"
#define NVS_KEY_TUNCAP      "tuncap"
#define NVS_KEY_TUNHZ       "tunhz"
#define NVS_KEY_AFE         "afe"
#define NVS_KEY_TIMEOUT     "tmout"
#define NVS_KEY_SLEEP       "sleep"

// Round Display v1.1 KE switch (per Seeed wiki):
//   KE switch ON  → A0 connects to the battery voltage divider and
//                   D6 connects to the backlight MOSFET.
//   KE switch OFF → A0 and D6 are released as plain XIAO GPIOs.
// We drive D6 for backlight. With KE on the LED follows; with KE
// off the write is a no-op. A0 is only ever read as analog in.
#ifndef TFT_BL_PIN
  #define TFT_BL_PIN        D6
#endif
#ifndef BAT_ADC_PIN
  #define BAT_ADC_PIN       A0
#endif
// Battery divider on Round Display v1.1 is R28/R29 (470K each),
// so the ADC tap sees VBAT/2. Multiply back up.
#define BAT_DIVIDER_NUM   2
#define BAT_READ_MS       2000UL   // refresh cadence
#define BAT_PLAUSIBLE_LO  2500     // mV; below this = no battery / KE off
#define BAT_PLAUSIBLE_HI  4400     // mV; above this = wrong pin / KE off

// Inactivity-timeout options for the settings UI. 0 == NEVER.
static const uint32_t TIMEOUT_OPTIONS_MS[] = {
  30000UL, 60000UL, 120000UL, 300000UL, 600000UL, 0UL
};
static const char* const TIMEOUT_LABELS[] = {
  "30s", "1 min", "2 min", "5 min", "10 min", "NEVER"
};
#define TIMEOUT_OPTIONS_N   6
#define TIMEOUT_DEFAULT_IDX 2   // 2 min

// Deep-idle -> light sleep options. 0 == NEVER. Light sleep wakes
// on AS3935 INT rising (strike/disturber/NH) or on a touch. Serial
// (USB-CDC) disconnects while sleeping; it re-enumerates on wake.
static const uint32_t SLEEP_OPTIONS_MS[] = {
  300000UL, 900000UL, 1800000UL, 3600000UL, 7200000UL, 0UL
};
static const char* const SLEEP_LABELS[] = {
  "5 min", "15 min", "30 min", "1 h", "2 h", "NEVER"
};
#define SLEEP_OPTIONS_N     6
#define SLEEP_DEFAULT_IDX   5   // NEVER — Tier 3 is opt-in until debugged

// Global Tier-3 kill-switch. Flip to 0 to completely disable light
// sleep regardless of the NVS-saved value, useful while debugging.
#ifndef ENABLE_LIGHT_SLEEP
  #define ENABLE_LIGHT_SLEEP 1
#endif

// Touch controller (CHSC6X) shares Wire with the AS3935. Different
// address (0x2E vs 0x03), so no bus conflict.
#define TOUCH_I2C_ADDR      0x2E
#ifndef TOUCH_INT
  #define TOUCH_INT         D7
#endif
// Swipe: |dx| must exceed this to count (pixels).
#define SWIPE_MIN_PX        25
// Tap: |dx|, |dy| must be under this, and held less than this time.
#define TAP_MAX_PX          15
#define TAP_MAX_MS          700

// ── Sensor + UI state ──────────────────────────────────────────
uint8_t  noiseFloor   = 2;
uint8_t  watchdogLvl  = 2;
uint8_t  spikeRej     = 2;
uint32_t senseLastAdj = 0;
uint32_t nfLastDecay  = 0;
uint32_t lastIrqMs    = 0;

int      strikeCount       = 0;
uint32_t maxEnergy         = 0;
uint32_t lastEnergy        = 0;
uint8_t  lastDistRaw       = DIST_UNKNOWN;
uint32_t lastStrikeMs      = 0;
uint32_t lastCloseStrikeMs = 0;   // 0 = no close strike seen yet
float    radarAngle        = 0;

bool     sensorOk     = false;
bool     noiseFault   = false;
uint8_t  activeAfeGb  = AS3935_AFE_GB;     // full reg byte value, bits [5:1]
bool     outdoorMode  = (AS3935_AFE_GB == AFE_GB_OUTDOOR);
uint8_t  i2cFailStreak = 0;

uint8_t  activeTunCap = AS3935_TUN_CAP;   // overridden from NVS at boot
uint32_t activeLcoHz  = 0;                 // 0 = untuned / unknown

enum Screen : uint8_t { SCREEN_MAIN = 0, SCREEN_SETTINGS = 1 };
Screen   currentScreen = SCREEN_MAIN;

bool     touchPressed  = false;
int16_t  touchDownX    = -1, touchDownY = -1;
int16_t  touchX        = -1, touchY = -1;
uint32_t touchDownMs   = 0;

// Button press-flash feedback. Keeps a button filled for a short
// window after it's tapped so the user sees the action register
// even when the underlying value didn't visibly change.
enum ButtonId : int8_t {
  BTN_NONE    = -1,
  BTN_INDOOR  = 0,
  BTN_OUTDOOR = 1,
  BTN_TIMEOUT = 2,
  BTN_SLEEP   = 3,
  BTN_RESET   = 4,
};
ButtonId flashButton    = BTN_NONE;
uint32_t flashUntilMs   = 0;
#define  BTN_FLASH_MS   350

// Tier-1 interrupt-driven AS3935 handling. The ISR just sets a
// flag; the event-register read + handling happens in loop context
// so I²C is not invoked from interrupt context.
volatile bool as3935IrqPending = false;
void IRAM_ATTR as3935IrqHandler() { as3935IrqPending = true; }

// Tier-2 backlight / inactivity state.
bool     displayOn     = true;
uint32_t lastActivityMs = 0;
uint8_t  timeoutIdx    = TIMEOUT_DEFAULT_IDX;
uint8_t  sleepIdx      = SLEEP_DEFAULT_IDX;

// Battery sense state.
uint16_t batteryMv       = 0;        // 0 == not yet read / implausible
uint32_t batteryLastMs   = 0;

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

  // 3. AFE gain + PWD=0 (runtime value, may have been overridden from NVS).
  if (!maskWrite(REG_AFE_GAIN, AFE_GB_FIELD_MASK | PWD_BIT, activeAfeGb))
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

  // 8. Hook the hardware IRQ line. Order matters:
  //    (a) clear any latched event so INT isn't currently high;
  //    (b) attach RISING-edge ISR;
  //    (c) if a new event slipped between (a) and (b), the line is
  //        already high — set the pending flag so we don't miss it.
  pinMode(AS3935_INT_PIN, INPUT);
  uint8_t discard;
  i2cRead(REG_LCO_INT, discard);
  detachInterrupt(digitalPinToInterrupt(AS3935_INT_PIN));
  attachInterrupt(digitalPinToInterrupt(AS3935_INT_PIN),
                  as3935IrqHandler, RISING);
  if (digitalRead(AS3935_INT_PIN) == HIGH) as3935IrqPending = true;

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
  if (currentScreen == SCREEN_SETTINGS) { drawSettings(); return; }

  spr.fillSprite(C_BG);

  // Danger-window calculation: did we see a close strike in the
  // last 30 minutes? If yes, the whole UI flips into alarm mode.
  uint32_t nowMs = millis();
  bool dangerActive = (lastCloseStrikeMs > 0) &&
                      ((nowMs - lastCloseStrikeMs) < DANGER_WINDOW_MS);
  uint32_t dangerElapsed = dangerActive ? (nowMs - lastCloseStrikeMs) : 0;

  float pct = (lastEnergy > 0) ? constrain(lastEnergy / 2097151.0f, 0.f, 1.f) : 0;
  drawGaugeArc(spr, energyColor(lastEnergy), pct);
  drawGaugeTicks(spr, 10);
  spr.drawCircle(CX, CY, 119, 0x2124);
  spr.drawCircle(CX, CY, 90,  C_DIM);
  spr.drawCircle(CX, CY, 58,  C_DIM);

  // "Listening" indicator — two concentric rings expand outward
  // from the centre and fade as they grow, offset by half a period
  // so one is always visible. Purely radial — the AS3935 has a
  // single loop antenna, so anything rotating would be a lie.
  uint32_t ripplePhase = nowMs % 2000;      // 2 s period
  for (int i = 0; i < 2; i++) {
    uint32_t p = (ripplePhase + (uint32_t)(i * 1000)) % 2000;
    float    t = p / 2000.0f;               // 0..1
    int      r = 8 + (int)(t * 82);         // 8..90 px radius
    uint8_t  v = (uint8_t)((1.0f - t) * 40);
    uint16_t rc = dangerActive ? spr.color565(v, v / 4, 0)   // red-amber
                               : spr.color565(0, v, v / 3);  // green-teal
    spr.drawCircle(CX, CY, r, rc);
  }

  spr.setTextDatum(TC_DATUM);
  spr.setTextSize(1);
  if (dangerActive) {
    // Blink red / orange at 2 Hz so it catches peripheral vision.
    bool blinkOn = (nowMs / 500) & 1;
    spr.setTextColor(blinkOn ? C_RED : C_ORANGE, C_BG);
    spr.drawString("!! SHELTER !!", CX, 16);
  } else {
    spr.setTextColor(C_GREY, C_BG);
    spr.drawString("LIGHTNING DETECTOR", CX, 16);
  }

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
  uint16_t heroColor = dangerActive ? C_RED : (stale ? C_DIM : C_GOLD);

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
  if (dangerActive) {
    // Shelter countdown: how long until the 30/30 safe window clears.
    // Display counts the minutes elapsed since last close strike so
    // the user can reason about when they can move again (NWS: wait
    // 30 min after last close strike).
    spr.setTextColor(C_RED, C_BG);
    spr.setTextSize(1);
    spr.drawString("SHELTER", 200, 140);
    uint32_t secs = dangerElapsed / 1000;
    char buf[12];
    snprintf(buf, sizeof(buf), "%lu:%02lu",
             (unsigned long)(secs / 60), (unsigned long)(secs % 60));
    spr.setTextSize(2);
    spr.drawString(buf, 200, 156);
  } else {
    spr.setTextColor(C_GREY, C_BG);
    spr.setTextSize(1);
    spr.drawString("ENERGY", 200, 140);
    spr.setTextColor(energyColor(lastEnergy), C_BG);
    spr.setTextSize(2);
    String eStr = lastEnergy > 999 ? String(lastEnergy / 1000) + "K"
                                   : String(lastEnergy);
    spr.drawString(eStr, 200, 156);
  }

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
    Serial.printf("TUN_CAP=%2u -> %5lu Hz (%+5ld)\r\n",
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
  Serial.printf("best: TUN_CAP=%u @ %lu Hz (dev %+ld, %s)\r\n",
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

// ── Battery sense ──────────────────────────────────────────────
// Piecewise-linear LiPo discharge curve. Approximate, unloaded.
// Good enough to tell "full" from "half" from "get to a charger".
uint8_t batteryPercent(uint16_t mv) {
  static const struct { uint16_t mv; uint8_t pct; } curve[] = {
    {4200, 100}, {4100, 90}, {4000, 75}, {3900, 55},
    {3800, 40},  {3700, 25}, {3600, 10}, {3500,  5},
    {3300,  0},
  };
  const int N = sizeof(curve) / sizeof(curve[0]);
  if (mv >= curve[0].mv)     return 100;
  if (mv <= curve[N-1].mv)   return 0;
  for (int i = 0; i < N - 1; i++) {
    if (mv <= curve[i].mv && mv >= curve[i+1].mv) {
      uint16_t dmv  = curve[i].mv  - curve[i+1].mv;
      uint8_t  dpct = curve[i].pct - curve[i+1].pct;
      return (uint8_t)(curve[i+1].pct +
             (uint32_t)(mv - curve[i+1].mv) * dpct / dmv);
    }
  }
  return 0;
}

void updateBattery() {
  uint32_t now = millis();
  if (now - batteryLastMs < BAT_READ_MS && batteryLastMs != 0) return;
  batteryLastMs = now;
  uint32_t sum = 0;
  const int N = 8;
  for (int i = 0; i < N; i++) sum += analogReadMilliVolts(BAT_ADC_PIN);
  uint16_t tap = (uint16_t)(sum / N);
  uint16_t v   = tap * BAT_DIVIDER_NUM;
  // Range-check so a floating A0 (KE switch off, no divider) doesn't
  // show a ghost reading. Store 0 when implausible.
  batteryMv = (v >= BAT_PLAUSIBLE_LO && v <= BAT_PLAUSIBLE_HI) ? v : 0;
}

// ── Tier-2 backlight + panel sleep ─────────────────────────────
// Driving D6 cuts the backlight when the Round Display v1.1 KE
// switch is in the ON position. The GC9A01 controller is also
// put into DISPOFF + SLPIN so pixel output stops even if the KE
// switch is OFF (user will still see a dark/black screen, just
// with the LED still lit).
void setBacklight(bool on) {
  pinMode(TFT_BL_PIN, OUTPUT);
  if (on) {
    tft.writecommand(0x11);   // SLPOUT
    delay(5);
    tft.writecommand(0x29);   // DISPON
    digitalWrite(TFT_BL_PIN, HIGH);
  } else {
    digitalWrite(TFT_BL_PIN, LOW);
    tft.writecommand(0x28);   // DISPOFF
    tft.writecommand(0x10);   // SLPIN
  }
  displayOn = on;
  Serial.printf("[bl] %s\r\n", on ? "on" : "off");
}

// Reset the inactivity timer. If the screen was off, wake it.
void markActivity() {
  lastActivityMs = millis();
  if (!displayOn) {
    setBacklight(true);
  }
}

// ── Runtime settings (touch-UI hooks) ──────────────────────────
void setTimeoutIdx(uint8_t idx) {
  if (idx >= TIMEOUT_OPTIONS_N) idx = 0;
  timeoutIdx = idx;
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putUChar(NVS_KEY_TIMEOUT, timeoutIdx);
  prefs.end();
  Serial.printf("[timeout] -> %s\r\n", TIMEOUT_LABELS[timeoutIdx]);
}

void cycleTimeout() {
  setTimeoutIdx((timeoutIdx + 1) % TIMEOUT_OPTIONS_N);
  markActivity();  // start the new timer fresh
}

void setSleepIdx(uint8_t idx) {
  if (idx >= SLEEP_OPTIONS_N) idx = 0;
  sleepIdx = idx;
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putUChar(NVS_KEY_SLEEP, sleepIdx);
  prefs.end();
  Serial.printf("[sleep-cfg] -> %s\r\n", SLEEP_LABELS[sleepIdx]);
}

void cycleSleep() {
  setSleepIdx((sleepIdx + 1) % SLEEP_OPTIONS_N);
  markActivity();
}

// ── Tier-3 light sleep ─────────────────────────────────────────
// Entered when backlight is already off AND deep-idle timeout
// elapsed. Wake sources: AS3935 INT high level, touch low level.
// A safety timer wakes us at most once/hour even if both GPIO
// sources fail, so the firmware can't get permanently stuck.
void enterLightSleep() {
  // 1. If there's a latched AS3935 INT, level-wake would fire
  //    immediately. Clear it first; if the read fails, bail out
  //    and try again on the next loop tick.
  if (sensorOk) {
    uint8_t tmp;
    if (!i2cRead(REG_LCO_INT, tmp)) {
      Serial.println("[sleep] abort: could not clear AS3935 INT");
      return;
    }
  }
  if (digitalRead(AS3935_INT_PIN) == HIGH) {
    Serial.println("[sleep] abort: AS3935 INT still high after clear");
    as3935IrqPending = true;
    return;
  }
  // Touch should be idle-high; if it's currently low the user is
  // pressing right now. Abort so we don't sleep/wake instantly.
  if (digitalRead(TOUCH_INT) == LOW) {
    Serial.println("[sleep] abort: touch currently pressed");
    return;
  }

  Serial.printf("[sleep] entering light sleep (AS3935=%d TOUCH=%d)\r\n",
                digitalRead(AS3935_INT_PIN), digitalRead(TOUCH_INT));
  Serial.flush();

  // Explicit ESP-IDF gpio_config before arming wake — Arduino's
  // pinMode(INPUT_PULLUP) programs the pull-up via the GPIO matrix
  // but light-sleep wake on ESP32-C6 reads the pad level via the
  // LP/HP GPIO peripheral; the pull-up state has to be set through
  // gpio_config() for that path to see it.
  gpio_config_t touch_cfg = {
    .pin_bit_mask = BIT64(TOUCH_INT),
    .mode         = GPIO_MODE_INPUT,
    .pull_up_en   = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type    = GPIO_INTR_DISABLE,
  };
  gpio_config(&touch_cfg);
  gpio_config_t as3935_cfg = {
    .pin_bit_mask = BIT64(AS3935_INT_PIN),
    .mode         = GPIO_MODE_INPUT,
    .pull_up_en   = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type    = GPIO_INTR_DISABLE,
  };
  gpio_config(&as3935_cfg);

  // Wake sources — ESP-IDF 5.x API.
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_err_t e1 = gpio_wakeup_enable((gpio_num_t)AS3935_INT_PIN, GPIO_INTR_HIGH_LEVEL);
  esp_err_t e2 = gpio_wakeup_enable((gpio_num_t)TOUCH_INT,      GPIO_INTR_LOW_LEVEL);
  esp_err_t e3 = esp_sleep_enable_gpio_wakeup();
  esp_err_t e4 = esp_sleep_enable_timer_wakeup((uint64_t)3600ULL * 1000ULL * 1000ULL);
  if (e1 || e2 || e3 || e4) {
    Serial.printf("[sleep] wakeup config failed: %d %d %d %d\r\n",
                  (int)e1, (int)e2, (int)e3, (int)e4);
    return;
  }

  uint32_t t0 = millis();
  esp_light_sleep_start();   // blocks until wake event
  uint32_t slept = millis() - t0;

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  bool fromAs3935 = (digitalRead(AS3935_INT_PIN) == HIGH);
  bool fromTouch  = (digitalRead(TOUCH_INT)      == LOW);
  Serial.printf("[sleep] woke cause=%d as3935=%d touch=%d after %lums\r\n",
                (int)cause, fromAs3935, fromTouch, (unsigned long)slept);

  // Our pre-sleep gpio_config disabled the regular GPIO interrupts
  // on AS3935_INT_PIN. Re-attach so post-wake strike events still
  // fire the ISR path.
  attachInterrupt(digitalPinToInterrupt(AS3935_INT_PIN),
                  as3935IrqHandler, RISING);

  // Only touch unconditionally wakes the display. An AS3935 event
  // gets deferred to the main loop — if it's a strike the handler
  // will call markActivity() and turn the screen on; if it's a
  // disturber we process silently and fall back to sleep.
  if (fromTouch) markActivity();
  if (fromAs3935) as3935IrqPending = true;
}

void setAfeMode(uint8_t gbValue) {
  // Clamp to the two datasheet values; anything else is ignored.
  if (gbValue != AFE_GB_INDOOR && gbValue != AFE_GB_OUTDOOR) return;
  activeAfeGb  = gbValue;
  outdoorMode  = (gbValue == AFE_GB_OUTDOOR);
  if (sensorOk) {
    maskWrite(REG_AFE_GAIN, AFE_GB_FIELD_MASK | PWD_BIT, activeAfeGb);
  }
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putUChar(NVS_KEY_AFE, activeAfeGb);
  prefs.end();
  Serial.printf("[afe] mode -> %s\r\n", outdoorMode ? "OUTDOOR" : "INDOOR");
}

// Clear NF/WD/SR back to defaults, clear AS3935 lightning stats (datasheet
// §8.7: toggle CL_STAT 1 -> 0 -> 1 to flush the running energy window),
// reset the local UI counters so the ticker is empty.
void resetFilters() {
  noiseFloor = 2; watchdogLvl = 2; spikeRej = 2;
  if (sensorOk) {
    writeReg(REG_NF_WDG, (noiseFloor << 4) | (watchdogLvl & WDTH_MASK));
    maskWrite(REG_CLSTAT_SREJ, SREJ_MASK, spikeRej & SREJ_MASK);
    maskWrite(REG_CLSTAT_SREJ, CL_STAT_BIT, CL_STAT_BIT);
    delay(2);
    maskWrite(REG_CLSTAT_SREJ, CL_STAT_BIT, 0);
    delay(2);
    maskWrite(REG_CLSTAT_SREJ, CL_STAT_BIT, CL_STAT_BIT);
  }
  noiseFault   = false;
  strikeCount  = 0;
  lastEnergy   = 0;
  maxEnergy    = 0;
  lastDistRaw  = DIST_UNKNOWN;
  lastStrikeMs = 0;
  tickHead     = 0;
  tickCount    = 0;
  memset(tickEnergy, 0, sizeof(tickEnergy));
  Serial.println("[filters] reset: NF=2 WD=2 SR=2, stats cleared");
}

// ── Touch (CHSC6X @ 0x2E) ──────────────────────────────────────
bool touchReadPoint(int16_t &x, int16_t &y) {
  uint8_t n = Wire.requestFrom((uint8_t)TOUCH_I2C_ADDR, (uint8_t)5);
  if (n != 5) return false;
  uint8_t b[5];
  for (uint8_t i = 0; i < 5; i++) {
    if (!Wire.available()) return false;
    b[i] = Wire.read();
  }
  if (b[0] != 0x01) return false;              // no touch point reported
  if (b[2] >= 240 || b[4] >= 240) return false; // out-of-range = I2C glitch
  x = b[2];
  y = b[4];
  return true;
}

void onSwipe() {
  currentScreen = (currentScreen == SCREEN_MAIN) ? SCREEN_SETTINGS : SCREEN_MAIN;
  Serial.printf("[touch] swipe -> screen=%u\r\n", (unsigned)currentScreen);
}

void flashFeedback(ButtonId which) {
  flashButton  = which;
  flashUntilMs = millis() + BTN_FLASH_MS;
}

void onTap(int16_t x, int16_t y) {
  Serial.printf("[touch] tap %d,%d screen=%u\r\n", x, y, (unsigned)currentScreen);
  if (currentScreen != SCREEN_SETTINGS) return;
  // INDOOR:  x[40..110]  y[38..66]
  if (x >= 40 && x <= 110 && y >= 38 && y <= 66) {
    setAfeMode(AFE_GB_INDOOR);
    flashFeedback(BTN_INDOOR);
  }
  // OUTDOOR: x[130..200] y[38..66]
  else if (x >= 130 && x <= 200 && y >= 38 && y <= 66) {
    setAfeMode(AFE_GB_OUTDOOR);
    flashFeedback(BTN_OUTDOOR);
  }
  // TIMEOUT tile: x[40..200] y[74..98]
  else if (x >= 40 && x <= 200 && y >= 74 && y <= 98) {
    cycleTimeout();
    flashFeedback(BTN_TIMEOUT);
  }
  // SLEEP tile: x[40..200] y[104..128]
  else if (x >= 40 && x <= 200 && y >= 104 && y <= 128) {
    cycleSleep();
    flashFeedback(BTN_SLEEP);
  }
  // RESET FILTERS: x[50..190] y[138..168]
  else if (x >= 50 && x <= 190 && y >= 138 && y <= 168) {
    resetFilters();
    flashFeedback(BTN_RESET);
  }
}

void pollTouch() {
  bool isLow = (digitalRead(TOUCH_INT) == LOW);
  if (isLow) {
    // Any touch counts as user activity — restart inactivity timer
    // and, if the screen was off, wake it and swallow this press.
    bool wokeFromOff = !displayOn;
    markActivity();
    if (wokeFromOff) {
      Serial.println("[wake] touch");
      touchPressed = false;
      return;
    }
    int16_t rx, ry;
    if (touchReadPoint(rx, ry)) {
      if (!touchPressed) {
        touchDownX  = rx;
        touchDownY  = ry;
        touchDownMs = millis();
        Serial.printf("[touch] down %d,%d\r\n", rx, ry);
      }
      touchX = rx;
      touchY = ry;
      touchPressed = true;
    }
    return;
  }
  if (touchPressed) {
    int16_t dx = touchX - touchDownX;
    int16_t dy = touchY - touchDownY;
    int16_t adx = dx < 0 ? -dx : dx;
    int16_t ady = dy < 0 ? -dy : dy;
    uint32_t held = millis() - touchDownMs;
    Serial.printf("[touch] up %d,%d dx=%d dy=%d held=%ums\r\n",
                  touchX, touchY, dx, dy, (unsigned)held);
    // Tap: both deltas under threshold and released quickly.
    // Swipe: anything else where horizontal motion dominates. No
    // dead zone between the two — short intentional flicks used to
    // fall in a gap (|dx| 16..24) and get silently dropped.
    if (adx <= TAP_MAX_PX && ady <= TAP_MAX_PX && held < TAP_MAX_MS) {
      onTap(touchX, touchY);
    } else if (adx > ady) {
      onSwipe();
    }
    touchPressed = false;
  }
}

// ── Settings screen ────────────────────────────────────────────
void drawSettings() {
  spr.fillSprite(C_BG);
  spr.drawCircle(CX, CY, 119, C_DIMRING);

  spr.setTextDatum(TC_DATUM);
  spr.setTextColor(C_GOLD, C_BG);
  spr.setTextSize(2);
  spr.drawString("SETTINGS", CX, 10);

  spr.setTextDatum(MC_DATUM);
  spr.setTextColor(C_GREY, C_BG);
  spr.setTextSize(1);
  spr.drawString("AFE MODE", CX, 30);

  bool flashActive = (flashButton != BTN_NONE && millis() < flashUntilMs);
  char buf[40];

  // INDOOR / OUTDOOR (row 1, y=38..66)
  uint16_t inBorder  = !outdoorMode ? C_GOLD : C_DIM;
  uint16_t outBorder = outdoorMode  ? C_GOLD : C_DIM;
  bool flashIn  = flashActive && flashButton == BTN_INDOOR;
  bool flashOut = flashActive && flashButton == BTN_OUTDOOR;

  if (flashIn) {
    spr.fillRoundRect(40, 38, 70, 28, 5, C_GOLD);
    spr.setTextColor(C_BG, C_GOLD);
  } else {
    spr.drawRoundRect(40, 38, 70, 28, 5, inBorder);
    if (!outdoorMode) spr.drawRoundRect(41, 39, 68, 26, 4, inBorder);
    spr.setTextColor(!outdoorMode ? C_GOLD : C_GREY, C_BG);
  }
  spr.drawString("INDOOR", 75, 52);

  if (flashOut) {
    spr.fillRoundRect(130, 38, 70, 28, 5, C_GOLD);
    spr.setTextColor(C_BG, C_GOLD);
  } else {
    spr.drawRoundRect(130, 38, 70, 28, 5, outBorder);
    if (outdoorMode) spr.drawRoundRect(131, 39, 68, 26, 4, outBorder);
    spr.setTextColor(outdoorMode ? C_GOLD : C_GREY, C_BG);
  }
  spr.drawString("OUTDOOR", 165, 52);

  // TIMEOUT tile (row 2, y=74..98)
  bool flashTimeout = flashActive && flashButton == BTN_TIMEOUT;
  snprintf(buf, sizeof(buf), "SCREEN  %s", TIMEOUT_LABELS[timeoutIdx]);
  if (flashTimeout) {
    spr.fillRoundRect(40, 74, 160, 24, 4, C_GOLD);
    spr.setTextColor(C_BG, C_GOLD);
  } else {
    spr.drawRoundRect(40, 74, 160, 24, 4, C_DIM);
    spr.setTextColor(C_GREY, C_BG);
  }
  spr.drawString(buf, CX, 86);

  // SLEEP tile (row 3, y=104..128)
  bool flashSleep = flashActive && flashButton == BTN_SLEEP;
  snprintf(buf, sizeof(buf), "SLEEP   %s", SLEEP_LABELS[sleepIdx]);
  if (flashSleep) {
    spr.fillRoundRect(40, 104, 160, 24, 4, C_GOLD);
    spr.setTextColor(C_BG, C_GOLD);
  } else {
    spr.drawRoundRect(40, 104, 160, 24, 4, C_DIM);
    spr.setTextColor(C_GREY, C_BG);
  }
  spr.drawString(buf, CX, 116);

  // RESET FILTERS (row 4, y=138..168)
  bool flashReset = flashActive && flashButton == BTN_RESET;
  if (flashReset) {
    spr.fillRoundRect(50, 138, 140, 30, 5, C_ORANGE);
    spr.setTextColor(C_BG, C_ORANGE);
    spr.drawString("RESET!", CX, 153);
  } else {
    spr.drawRoundRect(50, 138, 140, 30, 5, C_ORANGE);
    spr.setTextColor(C_ORANGE, C_BG);
    spr.drawString("RESET FILTERS", CX, 153);
  }

  // Status footer
  spr.setTextColor(C_GREY, C_BG);
  if (batteryMv > 0) {
    snprintf(buf, sizeof(buf), "BAT %u.%02uV  %u%%",
             batteryMv / 1000, (batteryMv % 1000) / 10,
             batteryPercent(batteryMv));
  } else {
    snprintf(buf, sizeof(buf), "BAT --");
  }
  spr.drawString(buf, CX, 186);
  snprintf(buf, sizeof(buf), "NF %u  WD %u  SR %u",
           noiseFloor, watchdogLvl, spikeRej);
  spr.drawString(buf, CX, 200);

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

  Wire.begin(SDA, SCL);
  Wire.setClock(I2C_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);

  // Load saved antenna calibration + AFE mode, if any.
  prefs.begin(NVS_NAMESPACE, true);
  bool haveSavedTune = prefs.isKey(NVS_KEY_TUNCAP);
  if (haveSavedTune) {
    activeTunCap = prefs.getUChar(NVS_KEY_TUNCAP, AS3935_TUN_CAP);
    activeLcoHz  = prefs.getUInt (NVS_KEY_TUNHZ, 0);
  }
  if (prefs.isKey(NVS_KEY_AFE)) {
    uint8_t savedAfe = prefs.getUChar(NVS_KEY_AFE, AS3935_AFE_GB);
    if (savedAfe == AFE_GB_INDOOR || savedAfe == AFE_GB_OUTDOOR) {
      activeAfeGb = savedAfe;
      outdoorMode = (savedAfe == AFE_GB_OUTDOOR);
    }
  }
  if (prefs.isKey(NVS_KEY_TIMEOUT)) {
    uint8_t t = prefs.getUChar(NVS_KEY_TIMEOUT, TIMEOUT_DEFAULT_IDX);
    if (t < TIMEOUT_OPTIONS_N) timeoutIdx = t;
  }
  if (prefs.isKey(NVS_KEY_SLEEP)) {
    uint8_t s = prefs.getUChar(NVS_KEY_SLEEP, SLEEP_DEFAULT_IDX);
    if (s < SLEEP_OPTIONS_N) sleepIdx = s;
  }
  prefs.end();

#if !ENABLE_LIGHT_SLEEP
  // Force NEVER while Tier 3 is disabled so the UI doesn't claim a
  // sleep timeout is active when the firmware can't actually sleep.
  sleepIdx = SLEEP_OPTIONS_N - 1;
#endif

  // Touch INT pin (CHSC6X controller). Idle-high, pulled low on touch.
  pinMode(TOUCH_INT, INPUT_PULLUP);

  // Backlight pin as output + panel awake.
  pinMode(TFT_BL_PIN, OUTPUT);
  setBacklight(true);
  lastActivityMs = millis();

  // Prompt for antenna tune over serial. Longer window on first boot
  // (no saved calibration) because the user should really do this.
  if (haveSavedTune) {
    Serial.printf("[tune] using saved TUN_CAP=%u (LCO %lu Hz)\r\n",
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
    pollTouch();
    drawUI();
    delay(80);
    return;
  }

  pollTouch();
  updateBattery();

  // Safety net: AS3935 INT is level-latched, so a failed i2cRead
  // leaves the pin high with no new rising edge for the ISR. Poll
  // the pin level twice a second and rescue any stuck-high state.
  static uint32_t intSafetyMs = 0;
  if (sensorOk && !as3935IrqPending && (now - intSafetyMs) > 500) {
    intSafetyMs = now;
    if (digitalRead(AS3935_INT_PIN) == HIGH) {
      as3935IrqPending = true;
      Serial.println("[AS3935] rescued latched INT");
    }
  }

  // Tier-1: only talk to the AS3935 when its INT pin has asserted.
  if (as3935IrqPending) {
    as3935IrqPending = false;
    uint8_t intReg;
    if (!i2cRead(REG_LCO_INT, intReg)) {
      if (i2cFailStreak >= I2C_FAIL_LIMIT) {
        Serial.println("[I2C] sensor lost");
        sensorOk = false;
      }
    } else {
      uint8_t reason = intReg & INT_MASK;
      if (reason != 0) lastIrqMs = now;   // sensor-watchdog only

      // markActivity() is called only for user-facing events:
      //   strikes, touches, and NH-with-fault. Raw disturbers and
      //   NF-ratchet events happen silently so they don't prevent
      //   the power-saving timers from ever reaching their goal.
      if (reason == INT_NH) {
        Serial.println("[NH] noise floor too high");
        raiseNoiseFloor();
        senseLastAdj = now;
        nfLastDecay  = now;
        if (noiseFault) markActivity();   // wake to show ENV TOO NOISY
      } else if (reason == INT_D) {
        Serial.println("[D] disturber");
        tightenFilters();
        senseLastAdj = now;
        // silent — no markActivity
      } else if (reason == INT_L) {
        uint32_t e = readEnergy();
        uint8_t  d = readReg(REG_DISTANCE) & DIST_FIELD_MASK;
        strikeCount++;
        lastEnergy   = e;
        lastDistRaw  = d;
        lastStrikeMs = now;
        if (e > maxEnergy) maxEnergy = e;
        pushTick(e);
        // Close-strike marker: overhead (0x01 = <6 km) or any
        // bin reporting <= CLOSE_STRIKE_KM km, but not 0x3F (OOR)
        // and not 0x00 (unknown).
        bool close = (d == DIST_OVERHEAD) ||
                     (d >= 5 && d <= CLOSE_STRIKE_KM);
        if (close) {
          lastCloseStrikeMs = now;
          Serial.println("⚠ CLOSE STRIKE - SHELTER NOW");
        }
        Serial.printf("strike #%d d=0x%02X e=%lu%s\r\n",
                      strikeCount, d, (unsigned long)e,
                      close ? " [CLOSE]" : "");
        markActivity();
        flashScreen();
      }
    }
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

  // Tier-2: turn off the backlight after inactivity. Skip the UI
  // redraw entirely while dark — biggest CPU + power win.
  uint32_t timeoutMs = TIMEOUT_OPTIONS_MS[timeoutIdx];
  bool keepOn = noiseFault;   // keep the "env too noisy" warning visible
  // Signed comparison below: pollTouch()/onEvent may have updated
  // lastActivityMs to a value slightly *later* than the `now` we
  // cached at the top of loop(), making (now - lastActivityMs)
  // underflow to ~4.29e9 ms and fire an immediate spurious blank.
  int32_t idleMs = (int32_t)(now - lastActivityMs);
  if (displayOn && timeoutMs > 0 && !keepOn &&
      idleMs > (int32_t)timeoutMs) {
    Serial.printf("[blank] idle %ds — panel sleep\r\n", idleMs / 1000);
    setBacklight(false);
  }

  // Tier-3: after the longer deep-idle window, light-sleep the CPU.
  // Gated by ENABLE_LIGHT_SLEEP until we've got sleep/wake reliability
  // tested end-to-end on the board.
#if ENABLE_LIGHT_SLEEP
  uint32_t sleepMs = SLEEP_OPTIONS_MS[sleepIdx];
  if (!displayOn && !keepOn && sensorOk &&
      sleepMs > 0 && idleMs > (int32_t)sleepMs) {
    enterLightSleep();
  }
#endif

  if (displayOn) drawUI();
  delay(20);
}
