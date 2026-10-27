#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MAX30105.h>
#include "heartRate.h"

/* ============================================================
   RAKSHAK WATCH 3
   Classic ESP32 Dev Module
   I2C: MPU6050 + MAX30102 + QMC5883L
   SPI: Waveshare 1.69-inch ST7789V2 240x280 display
   ESP-NOW: broadcast; receives Watch 1 and Watch 2
   ============================================================ */

#define RAKSHAK_WATCH_ID 3

// I2C bus shared by all three sensors
#define I2C_SDA 21
#define I2C_SCL 23

// Waveshare ST7789V2 display pins (classic ESP32 Dev Module)
#define TFT_SCLK 18
#define TFT_MOSI 13
#define TFT_CS   27
#define TFT_DC   26
#define TFT_RST  25
#define TFT_BL   33

#define ESPNOW_CHANNEL 1
#define PEER_TIMEOUT_MS 10000  // Hold a teammate marker through brief ESP-NOW packet gaps
#define TX_INTERVAL_MS  100
#define UI_INTERVAL_MS  150

// QMC5883L default I2C address and registers
#define QMC5883L_ADDR       0x0D
#define QMC5883L_REG_DATA   0x00
#define QMC5883L_REG_STATUS 0x06
#define QMC5883L_REG_CTRL1  0x09
#define QMC5883L_REG_CTRL2  0x0A
#define QMC5883L_REG_SETRESET 0x0B

// Keep this structure byte-for-byte compatible with network_packet.h
struct __attribute__((packed)) Packet {
  uint8_t watch_id;
  float heading;
  float gx;
  float gy;
  float gz;
  float heartRate;
  uint8_t health;
  uint8_t gpsFix;
  uint8_t sats;
  float hdop;
  float lat;
  float lon;
};

static_assert(sizeof(Packet) == 36, "Packet layout mismatch: check network_packet.h");

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

Adafruit_ST7789 tft(&SPI, TFT_CS, TFT_DC, TFT_RST);
Adafruit_MPU6050 mpu;
MAX30105 particleSensor;

bool mpuOK = false;
bool qmcOK = false;
bool maxOK = false;

Packet txPacket = {};

struct PeerState {
  Packet packet;
  int rssi;
  uint32_t lastSeen;
  bool valid;
};

// Declared with the other data types so Arduino's auto-generated prototypes
// can see this type before functions that use it.
struct MarkerPosition {
  int x;
  int y;
  bool visible;
};

// Entries 1 and 2 are used; index 0 is unused.
PeerState peers[3] = {};
portMUX_TYPE peerMux = portMUX_INITIALIZER_UNLOCKED;

float localHeadingDeg = 0.0f;
float localGX = 0.0f, localGY = 0.0f, localGZ = 0.0f;
float localHeartRate = 0.0f;
uint32_t lastBeatMs = 0;
float beatRates[4] = {0, 0, 0, 0};
uint8_t beatIndex = 0;
uint8_t beatCount = 0;
uint32_t lastTX = 0;
uint32_t lastUI = 0;

// ---------- Low-level QMC5883L helpers ----------
bool qmcWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool qmcReadRegs(uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  size_t got = Wire.requestFrom((uint8_t)QMC5883L_ADDR, (uint8_t)len);
  if (got != len) return false;
  for (size_t i = 0; i < len; ++i) buf[i] = Wire.read();
  return true;
}

bool QMC_Init() {
  Wire.beginTransmission(QMC5883L_ADDR);
  if (Wire.endTransmission() != 0) return false;
  // Soft reset, set/reset period, continuous mode, 200 Hz, +/- 8 gauss.
  qmcWriteReg(QMC5883L_REG_CTRL2, 0x80);
  delay(10);
  if (!qmcWriteReg(QMC5883L_REG_SETRESET, 0x01)) return false;
  if (!qmcWriteReg(QMC5883L_REG_CTRL1, 0x1D)) return false;
  delay(10);
  return true;
}

bool QMC_ReadHeading(float &headingDeg) {
  uint8_t status = 0;
  if (!qmcReadRegs(QMC5883L_REG_STATUS, &status, 1)) return false;
  if ((status & 0x01) == 0) return false; // no new data

  uint8_t raw[6];
  if (!qmcReadRegs(QMC5883L_REG_DATA, raw, sizeof(raw))) return false;
  int16_t mx = (int16_t)((uint16_t)raw[1] << 8 | raw[0]);
  int16_t my = (int16_t)((uint16_t)raw[3] << 8 | raw[2]);
  // Basic heading only. Calibrate hard/soft-iron offsets for useful compass accuracy.
  float h = atan2f((float)my, (float)mx) * 180.0f / PI;
  if (h < 0) h += 360.0f;
  headingDeg = h;
  return true;
}

// ---------- Sensor initialization and updates ----------
void Sensors_Init() {
  mpuOK = mpu.begin(0x68, &Wire);
  if (!mpuOK) mpuOK = mpu.begin(0x69, &Wire); // AD0 may be high on some breakout boards
  if (mpuOK) {
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("[SENSOR] MPU6050 OK");
  } else {
    Serial.println("[SENSOR] MPU6050 not found at 0x68 (check AD0/address)");
  }

  qmcOK = QMC_Init();
  Serial.println(qmcOK ? "[SENSOR] QMC5883L OK" : "[SENSOR] QMC5883L not found at 0x0D");

  maxOK = particleSensor.begin(Wire, I2C_SPEED_STANDARD);
  if (maxOK) {
    // Typical SparkFun MAX3010x settings; HR is indicative, not medical-grade.
    particleSensor.setup(60, 4, 2, 100, 411, 4096);
    particleSensor.setPulseAmplitudeRed(0x0A);
    particleSensor.setPulseAmplitudeGreen(0);
    Serial.println("[SENSOR] MAX30102/MAX30105-compatible device OK");
  } else {
    Serial.println("[SENSOR] MAX30102 not found (check power/I2C)");
  }
}

void Sensors_Update() {
  if (mpuOK) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    // Adafruit reports gyro in rad/s; retain those units in the packet.
    localGX = g.gyro.x;
    localGY = g.gyro.y;
    localGZ = g.gyro.z;
  }

  if (qmcOK) {
    float heading;
    if (QMC_ReadHeading(heading)) localHeadingDeg = heading;
  }

  if (maxOK) {
    long irValue = particleSensor.getIR();
    // A low IR reading usually means no finger/contact on the sensor.
    if (irValue < 50000) {
      localHeartRate = 0.0f;
      beatCount = 0;
      beatIndex = 0;
    } else if (checkForBeat(irValue)) {
      uint32_t now = millis();
      if (lastBeatMs != 0) {
        float bpm = 60000.0f / (float)(now - lastBeatMs);
        if (bpm >= 35.0f && bpm <= 220.0f) {
          beatRates[beatIndex] = bpm;
          beatIndex = (beatIndex + 1) % 4;
          if (beatCount < 4) beatCount++;
          float sum = 0;
          for (uint8_t i = 0; i < beatCount; i++) sum += beatRates[i];
          localHeartRate = sum / beatCount;
        }
      }
      lastBeatMs = now;
    }
    // Don't leave a stale HR value displayed indefinitely.
    if (lastBeatMs && millis() - lastBeatMs > 5000) localHeartRate = 0.0f;
  }
}

// ---------- ESP-NOW ----------
void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!data || len != sizeof(Packet)) return;
  Packet incoming;
  memcpy(&incoming, data, sizeof(incoming));
  if (incoming.watch_id == RAKSHAK_WATCH_ID) return;
  if (incoming.watch_id != 1 && incoming.watch_id != 2) return;

  int rssi = -127;
  if (info && info->rx_ctrl) rssi = info->rx_ctrl->rssi;

  portENTER_CRITICAL(&peerMux);
  peers[incoming.watch_id].packet = incoming;
  peers[incoming.watch_id].rssi = rssi;
  peers[incoming.watch_id].lastSeen = millis();
  peers[incoming.watch_id].valid = true;
  portEXIT_CRITICAL(&peerMux);
}

bool ESPNow_Init() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setChannel(ESPNOW_CHANNEL);

  Serial.print("[ESP-NOW] Watch 3 MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("[ESP-NOW] Channel: ");
  Serial.println(WiFi.channel());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW init failed");
    return false;
  }

  esp_now_register_recv_cb(onReceive);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0; // current radio channel
  peerInfo.encrypt = false;
  if (!esp_now_is_peer_exist(broadcastAddress)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("[ERROR] Could not add broadcast peer");
      return false;
    }
  }
  Serial.println("[ESP-NOW] Ready; broadcasting as Watch 3");
  return true;
}

void BuildPacket() {
  memset(&txPacket, 0, sizeof(txPacket));
  txPacket.watch_id = RAKSHAK_WATCH_ID;
  txPacket.heading = localHeadingDeg;
  txPacket.gx = localGX;
  txPacket.gy = localGY;
  txPacket.gz = localGZ;
  txPacket.heartRate = localHeartRate;
  txPacket.health = 0; // Not yet implemented: do not infer a medical/field status.
  txPacket.gpsFix = 0;
  txPacket.sats = 0;
  txPacket.hdop = 0.0f;
  txPacket.lat = 0.0f;
  txPacket.lon = 0.0f;
}

void ESPNow_SendPacket() {
  esp_err_t result = esp_now_send(broadcastAddress, (const uint8_t *)&txPacket, sizeof(txPacket));
  if (result != ESP_OK) {
    Serial.print("[ESP-NOW] send error: ");
    Serial.println((int)result);
  }
}

void CopyPeers(PeerState out[3]) {
  portENTER_CRITICAL(&peerMux);
  out[1] = peers[1];
  out[2] = peers[2];
  portEXIT_CRITICAL(&peerMux);
}

// ---------- Radar UI (flicker-reduced incremental redraw) ----------
static const int RADAR_CX = 120;
static const int RADAR_CY = 128;
static const int RADAR_R = 99;
static const uint16_t RADAR_BG = ST77XX_BLACK;
static const uint16_t RADAR_RING = 0x0265;
static const uint16_t RADAR_EDGE = 0x03A7;
static const uint16_t RADAR_CROSS = 0x0143;
static const uint16_t HR_PANEL_BG = 0x0108;
static const uint16_t HR_LABEL = 0x7E5F;

MarkerPosition previousMarkers[3] = {};
int previousDisplayedHR = -999;
bool previousHRValid = false;

uint16_t peerColor(uint8_t id) {
  (void)id;
  return ST77XX_GREEN;
}

void DrawRadarBackground() {
  // Draw only the static radar lines. This function never clears the screen.
  tft.drawCircle(RADAR_CX, RADAR_CY, RADAR_R, RADAR_EDGE);
  tft.drawCircle(RADAR_CX, RADAR_CY, 74, RADAR_RING);
  tft.drawCircle(RADAR_CX, RADAR_CY, 49, RADAR_RING);
  tft.drawCircle(RADAR_CX, RADAR_CY, 24, RADAR_RING);
  tft.drawFastHLine(RADAR_CX - RADAR_R, RADAR_CY, RADAR_R * 2, RADAR_RING);
  tft.drawFastVLine(RADAR_CX, RADAR_CY - RADAR_R, RADAR_R * 2, RADAR_RING);
  tft.drawLine(RADAR_CX - 70, RADAR_CY - 70, RADAR_CX + 70, RADAR_CY + 70, RADAR_CROSS);
  tft.drawLine(RADAR_CX - 70, RADAR_CY + 70, RADAR_CX + 70, RADAR_CY - 70, RADAR_CROSS);
  tft.drawCircle(RADAR_CX, RADAR_CY, 8, ST77XX_WHITE);
  tft.fillCircle(RADAR_CX, RADAR_CY, 4, ST77XX_WHITE);
}

MarkerPosition CalculateMarker(const PeerState &p, uint8_t id, int maxR) {
  MarkerPosition pos = {0, 0, false};
  // Keep the last known marker visible during short packet-loss bursts.
  // It becomes stale only after PEER_TIMEOUT_MS without a valid packet.
  if (!p.valid || (uint32_t)(millis() - p.lastSeen) > PEER_TIMEOUT_MS) return pos;

  // RSSI estimates rough proximity only; angle is a fixed display placeholder,
  // not a measured bearing or real relative position.
  float normalized = ((float)p.rssi + 95.0f) / 55.0f;
  if (normalized < 0.0f) normalized = 0.0f;
  if (normalized > 1.0f) normalized = 1.0f;
  int radius = (int)(maxR * (0.88f - 0.58f * normalized));
  float angle = (id == 1) ? -140.0f : -40.0f;
  float rad = angle * PI / 180.0f;
  pos.x = RADAR_CX + (int)(cosf(rad) * radius);
  pos.y = RADAR_CY + (int)(sinf(rad) * radius);
  pos.visible = true;
  return pos;
}

void DrawPeerMarker(const MarkerPosition &pos, uint8_t id) {
  if (!pos.visible) return;
  uint16_t color = peerColor(id);
  tft.drawCircle(pos.x, pos.y, 8, color);
  tft.fillCircle(pos.x, pos.y, 4, color);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_GREEN, RADAR_BG);
  tft.setCursor(pos.x + 7, pos.y - 12);
  tft.printf("W%u", id);
}

void DrawHeartRatePanel(bool force = false) {
  int shownHR = (localHeartRate >= 35.0f && localHeartRate <= 220.0f)
                  ? (int)lroundf(localHeartRate) : -1;
  if (!force && previousHRValid && shownHR == previousDisplayedHR) return;

  const int bx = 151, by = 235, bw = 84, bh = 42;
  tft.fillRoundRect(bx, by, bw, bh, 5, HR_PANEL_BG);
  tft.drawRoundRect(bx, by, bw, bh, 5, RADAR_EDGE);
  tft.setTextSize(1);
  tft.setTextColor(HR_LABEL, HR_PANEL_BG);
  tft.setCursor(bx + 7, by + 6);
  tft.print("HEART RATE");
  tft.setTextColor(ST77XX_GREEN, HR_PANEL_BG);
  tft.setTextSize(2);
  tft.setCursor(bx + 7, by + 20);
  if (shownHR >= 0) {
    tft.print(shownHR);
    tft.setTextSize(1);
    tft.print(" BPM");
  } else {
    tft.print("-- BPM");
  }
  previousDisplayedHR = shownHR;
  previousHRValid = true;
}

void Display_Init() {
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);
  tft.init(240, 280);
  tft.setRotation(0); // Change to 1/2/3 if the physical mounting needs rotation.
  tft.fillScreen(RADAR_BG);  // Clear exactly once during initialization.
  DrawRadarBackground();
  DrawHeartRatePanel(true);
}

void Display_Update() {
  PeerState snapshot[3] = {};
  CopyPeers(snapshot);

  MarkerPosition current[3] = {};
  current[1] = CalculateMarker(snapshot[1], 1, RADAR_R - 12);
  current[2] = CalculateMarker(snapshot[2], 2, RADAR_R - 12);

  // Erase only the previous marker/label areas. No full-screen clear, which
  // caused the visible blink. The static radar is then redrawn over those areas.
  bool markerNeedsRedraw = false;
  for (uint8_t id = 1; id <= 2; ++id) {
    const MarkerPosition &oldPos = previousMarkers[id];
    const MarkerPosition &newPos = current[id];
    bool changed = oldPos.visible != newPos.visible ||
                   (oldPos.visible && newPos.visible &&
                    (oldPos.x != newPos.x || oldPos.y != newPos.y));
    if (changed && oldPos.visible) {
      // Covers the circle and W1/W2 label, including its black text background.
      tft.fillRect(oldPos.x - 10, oldPos.y - 15, 43, 28, RADAR_BG);
      markerNeedsRedraw = true;
    }
    if (changed && newPos.visible) markerNeedsRedraw = true;
  }

  if (markerNeedsRedraw) {
    DrawRadarBackground();
    DrawPeerMarker(current[1], 1);
    DrawPeerMarker(current[2], 2);
  } else {
    // Draw a newly received marker even on the first valid packet.
    for (uint8_t id = 1; id <= 2; ++id) {
      if (current[id].visible && !previousMarkers[id].visible) {
        DrawPeerMarker(current[id], id);
      }
    }
  }

  previousMarkers[1] = current[1];
  previousMarkers[2] = current[2];
  DrawHeartRatePanel(false);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nRAKSHAK WATCH 3 booting");

  Wire.begin(I2C_SDA, I2C_SCL, 100000);
  Sensors_Init();
  Display_Init();

  if (!ESPNow_Init()) {
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_RED);
    tft.setTextSize(2);
    tft.setCursor(12, 100);
    tft.println("ESP-NOW");
    tft.println("FAILED");
    Serial.println("ESP-NOW initialization failed; check serial log.");
  }

  Display_Update();
}

void loop() {
  Sensors_Update();

  uint32_t now = millis();
  if (now - lastTX >= TX_INTERVAL_MS) {
    lastTX = now;
    BuildPacket();
    ESPNow_SendPacket();
  }

  if (now - lastUI >= UI_INTERVAL_MS) {
    lastUI = now;
    Display_Update();
  }

  delay(2);
}
