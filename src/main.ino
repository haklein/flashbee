#include <Wire.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <math.h>

// ── Display ───────────────────────────────────────────────────
TFT_eSPI     tft = TFT_eSPI();
TFT_eSprite  spr = TFT_eSprite(&tft);

#define CX  120
#define CY  120

// ── Colors ────────────────────────────────────────────────────
#define C_BG      0x0000
#define C_GOLD    0xFEA0
#define C_ORANGE  0xFBE0
#define C_REDORG  0xF9A0
#define C_DIM     0x2104
#define C_DIMRING 0x10A2
#define C_GREEN   0x07E0
#define C_WHITE   0xFFFF
#define C_GREY    0x4208

// ── AS3935 ────────────────────────────────────────────────────
#define AS3935_ADDR 3
#define SENSE_INCREASE_INTERVAL 15000UL

uint8_t  noiseFloor   = 2;
uint8_t  watchdog     = 2;
uint8_t  spikeRej     = 2;
uint32_t senseLastAdj = 0;

int      strikeCount  = 0;
uint32_t maxEnergy    = 0;
uint32_t lastEnergy   = 0;
uint8_t  lastDist     = 0;
float    radarAngle   = 0;

// ── Ticker ────────────────────────────────────────────────────
#define MAX_TICKS 20
uint32_t tickEnergy[MAX_TICKS];
int      tickHead  = 0;
int      tickCount = 0;

// ── AS3935 helpers ────────────────────────────────────────────
uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(AS3935_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(AS3935_ADDR, 1);
  return Wire.available() ? Wire.read() : 0xFF;
}

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(AS3935_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

void maskWrite(uint8_t reg, uint8_t mask, uint8_t val) {
  writeReg(reg, (readReg(reg) & ~mask) | (val & mask));
}

uint32_t readEnergy() {
  return ((uint32_t)(readReg(0x06) & 0x1F) << 16)
       | ((uint32_t) readReg(0x05)          <<  8)
       |             readReg(0x04);
}

void pushTick(uint32_t e) {
  tickEnergy[tickHead] = e;
  tickHead = (tickHead + 1) % MAX_TICKS;
  if (tickCount < MAX_TICKS) tickCount++;
}

void applyWatchdogSpike() {
  writeReg(0x01, (noiseFloor << 4) | (watchdog & 0x0F));
  maskWrite(0x02, 0x0F, spikeRej & 0x0F);
}

void increaseSensitivity() {
  if (spikeRej < watchdog) {
    if (spikeRej < 11) { spikeRej++; Serial.print("[TUNE] Spike up: "); Serial.println(spikeRej); }
  } else {
    if (watchdog  < 10) { watchdog++;  Serial.print("[TUNE] WD up: ");   Serial.println(watchdog);  }
  }
  applyWatchdogSpike();
}

void decreaseSensitivity() {
  if (spikeRej > watchdog) {
    if (spikeRej > 0) { spikeRej--; Serial.print("[TUNE] Spike dn: "); Serial.println(spikeRej); }
  } else {
    if (watchdog  > 0) { watchdog--;  Serial.print("[TUNE] WD dn: ");   Serial.println(watchdog);  }
  }
  applyWatchdogSpike();
}

void raiseNoiseFloor() {
  if (noiseFloor < 7) {
    noiseFloor++;
    maskWrite(0x01, 0x70, noiseFloor << 4);
    Serial.print("[NF] Raised to "); Serial.println(noiseFloor);
  }
}

// ── Intensity color ───────────────────────────────────────────
uint16_t energyColor(uint32_t e) {
  if      (e > 500000) return C_WHITE;
  else if (e > 300000) return C_GOLD;
  else if (e > 150000) return C_GOLD;
  else if (e > 75000)  return C_ORANGE;
  else if (e > 20000)  return C_REDORG;
  else                 return C_DIM;
}

// ── Arc angle mapping ─────────────────────────────────────────
// TFT_eSPI drawArc: 0° = 6 o'clock, clockwise
// Our gauge: starts bottom-left (225°) sweeps clockwise to bottom-right (135°)
// In TFT_eSPI coords: start=225, end=135 going through 0
// We map pct 0.0-1.0 → start_angle=225 to end_angle=135 (270° sweep)
// Arc background: full 225→135 sweep
// Arc fill:       225 → 225 + pct*270

void drawGaugeArc(TFT_eSprite &s, uint16_t fillColor, float pct) {
  uint8_t r_outer = 116;
  uint8_t r_inner = 106;   // thickness = 10px

  // Background arc — full sweep
  // drawArc is on tft, not sprite — we need to draw on sprite
  // Use sprite's drawArc if available, else pixel method
  // TFT_eSprite inherits drawArc from TFT_eSPI so it works on sprite too

  // Background: 225 → 135 (full 270°)
  s.drawArc(CX, CY, r_outer, r_inner, 225, 135, 0x1082, C_BG, false);

  // Filled portion
  if (pct > 0.01f) {
    uint16_t endAngle = (uint16_t)(225 + pct * 270) % 360;
    s.drawArc(CX, CY, r_outer, r_inner, 225, endAngle, fillColor, C_BG, true);
  }
}

// ── Tick marks on gauge ───────────────────────────────────────
void drawGaugeTicks(TFT_eSprite &s, int count) {
  for (int i = 0; i <= count; i++) {
    // map tick i onto 225°→135° sweep in screen coords
    // TFT_eSPI 0°=6 o'clock, our 225° = bottom-left
    float deg = 225 + (270.0f / count) * i;
    float rad = deg * DEG_TO_RAD;
    int x1 = CX + (int)(104 * cos(rad));
    int y1 = CY + (int)(104 * sin(rad));
    int x2 = CX + (int)(118 * cos(rad));
    int y2 = CY + (int)(118 * sin(rad));
    s.drawLine(x1, y1, x2, y2, 0x2945);
  }
}

// ── Flash on real lightning ───────────────────────────────────
void flashScreen() {
  for (int i = 0; i < 2; i++) {
    spr.fillSprite(0x2945);
    spr.pushSprite(0, 0);
    delay(35);
    spr.fillSprite(C_BG);
    spr.pushSprite(0, 0);
    delay(20);
  }
}

// ── Boot screen ───────────────────────────────────────────────
void drawBoot(const char* msg, uint16_t color) {
  tft.fillScreen(C_BG);
  tft.drawCircle(CX, CY, 118, C_DIMRING);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(C_GOLD, C_BG);
  tft.setTextSize(2);
  tft.drawString("AS3935", CX, CY - 14);
  tft.setTextSize(1);
  tft.setTextColor(color, C_BG);
  tft.drawString(msg, CX, CY + 12);
}

// ── Main UI ───────────────────────────────────────────────────
void drawUI() {
  spr.fillSprite(C_BG);

  float pct = (lastEnergy > 0)
              ? constrain(lastEnergy / 2097151.0f, 0, 1)
              : 0;

  // ── Energy gauge arc ──────────────────────────────────────
  drawGaugeArc(spr, energyColor(lastEnergy), pct);
  drawGaugeTicks(spr, 10);

  // ── Outer border circle ───────────────────────────────────
  spr.drawCircle(CX, CY, 119, 0x2124);

  // ── Inner guide rings ─────────────────────────────────────
  spr.drawCircle(CX, CY, 90, C_DIM);
  spr.drawCircle(CX, CY, 58, C_DIM);

  // ── Radar sweep ───────────────────────────────────────────
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

  // ── TOP LABEL ─────────────────────────────────────────────
  spr.setTextDatum(TC_DATUM);
  spr.setTextColor(C_GREY, C_BG);
  spr.setTextSize(1);
  spr.drawString("LIGHTNING DETECTOR", CX, 16);

  // ── WD / SR tune indicator ────────────────────────────────
  String tuneStr = "WD:" + String(watchdog) + " SR:" + String(spikeRej);
  spr.setTextColor(C_DIM, C_BG);
  spr.setTextSize(1);
  spr.setTextDatum(TC_DATUM);
  spr.drawString(tuneStr, CX, 27);

  // ── DISTANCE — hero value (YELLOW, BIG) ───────────────────
  spr.setTextDatum(MC_DATUM);

  if (strikeCount == 0) {
    spr.setTextColor(C_DIM, C_BG);
    spr.setTextSize(3);
    spr.drawString("--", CX, 84);
    spr.setTextSize(1);
    spr.setTextColor(C_GREY, C_BG);
    spr.drawString("waiting...", CX, 108);

  } else if (lastDist == 0x3F) {
    // out of range
    spr.setTextColor(C_GOLD, C_BG);
    spr.setTextSize(3);
    spr.drawString(">40", CX, 80);
    spr.setTextSize(2);
    spr.setTextColor(C_GOLD, C_BG);
    spr.drawString("km", CX, 105);

  } else if (lastDist == 0x01) {
    // overhead
    spr.setTextColor(C_REDORG, C_BG);
    spr.setTextSize(2);
    spr.drawString("OVERHEAD", CX, 78);
    spr.setTextSize(2);
    spr.setTextColor(C_GOLD, C_BG);
    spr.drawString("< 1 km", CX, 100);

  } else {
    // normal distance — SIZE 6 yellow number + SIZE 3 km
    spr.setTextColor(C_GOLD, C_BG);
    spr.setTextSize(6);
    spr.drawString(String(lastDist), CX, 80);
    spr.setTextColor(C_GOLD, C_BG);
    spr.setTextSize(3);
    spr.drawString("km", CX, 112);
  }

  // ── DIVIDER ───────────────────────────────────────────────
  spr.drawFastHLine(44, 128, 152, C_DIM);

  // ── STRIKES (left) ────────────────────────────────────────
  spr.setTextDatum(ML_DATUM);
  spr.setTextColor(C_GREY, C_BG);
  spr.setTextSize(1);
  spr.drawString("STRIKES", 40, 140);
  spr.setTextColor(C_GOLD, C_BG);
  spr.setTextSize(2);
  spr.drawString(String(strikeCount), 40, 156);

  // ── ENERGY (right) ────────────────────────────────────────
  spr.setTextDatum(MR_DATUM);
  spr.setTextColor(C_GREY, C_BG);
  spr.setTextSize(1);
  spr.drawString("ENERGY", 200, 140);
  spr.setTextColor(energyColor(lastEnergy), C_BG);
  spr.setTextSize(2);
  String eStr = lastEnergy > 999
                ? String(lastEnergy / 1000) + "K"
                : String(lastEnergy);
  spr.drawString(eStr, 200, 156);

  // ── TICKER BAR ────────────────────────────────────────────
  int bax = 32, bay = 176, baw = 176, bah = 30;
  int bw  = baw / MAX_TICKS - 1;

  spr.drawRoundRect(bax - 2, bay - 2, baw + 4, bah + 6, 4, C_DIM);

  for (int i = 0; i < tickCount; i++) {
    int      idx  = (tickHead - tickCount + i + MAX_TICKS) % MAX_TICKS;
    float    bpct = constrain(tickEnergy[idx] / 2097151.0f, 0, 1);
    int      bh   = max(2, (int)(bpct * bah));
    int      bx   = bax + i * (bw + 1);
    int      by   = bay + bah - bh;
    uint16_t bc   = (i == tickCount - 1)
                    ? C_GOLD
                    : energyColor(tickEnergy[idx]);
    spr.fillRect(bx, by, bw, bh, bc);
  }

  spr.setTextDatum(TC_DATUM);
  spr.setTextColor(C_GREY, C_BG);
  spr.setTextSize(1);
  spr.drawString("ENERGY HISTORY", CX, 212);

  spr.pushSprite(0, 0);
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(C_BG);

  spr.createSprite(240, 240);
  spr.setTextFont(1);
  memset(tickEnergy, 0, sizeof(tickEnergy));

  drawBoot("INITIALIZING...", C_GREY);

  Wire.begin(6, 7);
  Wire.setClock(100000);

  writeReg(0x00, 0b00010010);   // indoor AFE
  delay(200);

  Wire.beginTransmission(AS3935_ADDR);
  Wire.write(0x3A);
  Wire.endTransmission(false);
  delay(200);

  Wire.beginTransmission(AS3935_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  delay(200);

  Wire.requestFrom(AS3935_ADDR, 1);
  if (Wire.available()) {
    drawBoot("FOUND!", C_GREEN);
    Serial.println("[AS3935] Found!");
    delay(1200);
  } else {
    drawBoot("NOT FOUND", 0xF800);
    Serial.println("[AS3935] Not found");
    while (true) delay(1000);
  }

  noiseFloor = 2;
  watchdog   = 2;
  spikeRej   = 2;

  writeReg(0x01, (noiseFloor << 4) | watchdog);
  writeReg(0x02, spikeRej);
  writeReg(0x03, 0x00);   // disturbers unmasked for auto-tune

  senseLastAdj = millis();

  Serial.println("[AS3935] Ready — polling, auto-tune active");
  drawUI();
}

// ── Loop ──────────────────────────────────────────────────────
void loop() {
  uint8_t intReg = readReg(0x03);
  uint8_t reason = intReg & 0x0F;

  if (reason == 0x01) {
    Serial.println("[NH] Noise floor too high");
    raiseNoiseFloor();
    senseLastAdj = millis();

  } else if (reason == 0x04) {
    Serial.println("[D] Disturber — tuning up");
    increaseSensitivity();
    senseLastAdj = millis();
    // silent — no flash, no screen update

  } else if (reason == 0x08) {
    delay(5);
    uint32_t e = readEnergy();
    uint8_t  d = readReg(0x07) & 0x3F;
    if (d == 0x00) d = 0x01;

    strikeCount++;
    lastEnergy = e;
    lastDist   = d;
    if (e > maxEnergy) maxEnergy = e;
    pushTick(e);

    Serial.print("⚡ #"); Serial.print(strikeCount);
    Serial.print(" D:");   Serial.print(lastDist);
    Serial.print("km E:"); Serial.println(lastEnergy);

    flashScreen();
  }

  // ── Auto-recover sensitivity every 15s ────────────────────
  if (millis() - senseLastAdj > SENSE_INCREASE_INTERVAL) {
    senseLastAdj = millis();
    Serial.println("[TUNE] Quiet — easing sensitivity down");
    decreaseSensitivity();
  }

  radarAngle += 2.5f;
  if (radarAngle >= 360) radarAngle = 0;

  drawUI();
  delay(50);
}
