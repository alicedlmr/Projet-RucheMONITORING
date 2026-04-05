#include <Arduino.h>
#include <Wire.h>
#include "HX711.h"
#include <BH1750.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DHT.h"
#include <WiFi.h>
#include "esp_bt.h"
#include "esp_sleep.h"

struct TxResult {
  bool done;
  bool joinNeeded;
  bool hasDownlink;
  int  dlPort;
  String dlHex;
};

// =============================================================
// 1) CONFIGURATION
// =============================================================

#define LOADSW_PIN 2

const int SENSOR_POWER_ON_DELAY_MS = 500;
const int DHT_POWER_ON_DELAY_MS    = 800;

#define HX711_DOUT 25
#define HX711_SCK  26
const long  SCALE_OFFSET = 145192;
const float SCALE_FACTOR = 31616.384766;
const int   SCALE_SIGN   = 1;

// Correction "balance ruche -> vraie balance"
const float WEIGHT_A = 1.0062217f;
const float WEIGHT_B = 5.3509883f;

#define I2C_SDA 21
#define I2C_SCL 22

#define ONE_WIRE_BUS 33

#define DHT1_PIN 27
#define DHT2_PIN 19
#define DHTTYPE DHT22

#define VBAT_PIN 35
const float VBAT_RATIO   = 1.407f;   // calibré
const int   VBAT_SAMPLES = 30;

#define RX2_PIN 16
#define TX2_PIN 17

#define BUZZER_PIN 23

const int16_t SENTINEL_I16 = 32767;

// --- Mode nuit ---
const float    LUX_NIGHT_THRESHOLD = 20.0f;
const uint8_t  NIGHT_MAX_SENDS     = 15;
const uint16_t NIGHT_PERIOD_MIN    = 45;
const uint16_t LOW_BATT_PERIOD_MIN = 60;

// Gardé si jamais on force le mode TIME
const uint16_t NIGHT_START_MIN = 21 * 60;   // 21:00
const uint16_t NIGHT_END_MIN   = 4 * 60;    // 04:00

// --- Enums ---
enum NightMode : uint8_t {
  NIGHT_AUTO = 0,
  NIGHT_LUX  = 1,
  NIGHT_TIME = 2
};

enum RunMode : uint8_t {
  RUN_NORMAL   = 0,
  RUN_OVERRIDE = 1,
  RUN_LOW_BATT = 2,
  RUN_NIGHT    = 3
};

// --- Valeurs par défaut ---
const uint16_t DEFAULT_HIGH_PERIOD_MIN = 10;
const uint16_t DEFAULT_OVERRIDE_MIN    = 10;
const uint16_t DEFAULT_MINUTE_OF_DAY   = 12 * 60;
const uint8_t  DEFAULT_NIGHT_MODE      = NIGHT_LUX;

// --- RTC DATA ---
RTC_DATA_ATTR uint16_t highPeriodMin = DEFAULT_HIGH_PERIOD_MIN;
RTC_DATA_ATTR bool loraJoined = false;

RTC_DATA_ATTR uint16_t rtc_minute_of_day = DEFAULT_MINUTE_OF_DAY;
RTC_DATA_ATTR bool     time_synced       = false;

RTC_DATA_ATTR bool     nightActive     = false;
RTC_DATA_ATTR bool     nightUsedToday  = false;
RTC_DATA_ATTR uint8_t  nightSendCount  = 0;

RTC_DATA_ATTR bool     periodOverride  = false;
RTC_DATA_ATTR uint16_t overrideMin     = DEFAULT_OVERRIDE_MIN;

RTC_DATA_ATTR uint8_t nightMode = DEFAULT_NIGHT_MODE;

// --- Alerte essaimage ---
RTC_DATA_ATTR float    lastWeightKg     = -1.0f;
RTC_DATA_ATTR uint32_t lastWeightAgeMin = 9999;

const float    SWARM_DROP_KG    = 1.5f;
const uint16_t SWARM_MAX_DT_MIN = 60;

// --- Accusé du dernier downlink reçu ---
RTC_DATA_ATTR uint8_t lastDlCmd  = 0;
RTC_DATA_ATTR uint8_t lastDlArg1 = 0;
RTC_DATA_ATTR uint8_t lastDlArg2 = 0;

// =============================================================
// 2) OBJETS
// =============================================================

HX711 scale;
BH1750 lightMeter;
bool bh_ok = false;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

DHT dht1(DHT1_PIN, DHTTYPE);
DHT dht2(DHT2_PIN, DHTTYPE);

// =============================================================
// 3) BUZZER
// =============================================================

void playTone(int freq, int durationMs, int pauseMs) {
  if (freq <= 0) {
    ledcWrite(BUZZER_PIN, 0);
    delay(durationMs + pauseMs);
    return;
  }
  ledcWriteTone(BUZZER_PIN, freq);
  delay(durationMs);
  ledcWrite(BUZZER_PIN, 0);
  delay(pauseMs);
}

void playMelodyStartup() {
  playTone(659, 120, 20);
  playTone(784, 120, 20);
  playTone(988, 140, 30);
  playTone(1319, 180, 50);
  playTone(1175, 120, 20);
  playTone(1047, 120, 20);
  playTone(988, 140, 30);
  playTone(784, 220, 80);
}

void buzzerInit() {
  ledcAttach(BUZZER_PIN, 2000, 10);
  ledcWrite(BUZZER_PIN, 0);
}

void buzzerOff() {
  ledcWrite(BUZZER_PIN, 0);
}

// =============================================================
// 4) HELPERS ENCODAGE
// =============================================================

int16_t to_i16_scaled(float v, float scale, int16_t sentinel) {
  if (isnan(v)) return sentinel;
  long x = lroundf(v * scale);
  if (x > 32766) x = 32766;
  if (x < -32768) x = -32768;
  return (int16_t)x;
}

uint16_t to_u16_clamped(float v) {
  if (isnan(v) || v < 0) return 0;
  if (v > 65535.0f) return 65535;
  return (uint16_t)lroundf(v);
}

// =============================================================
// 5) POWER / CAPTEURS
// =============================================================

void setupLoadSwitch() {
  pinMode(LOADSW_PIN, OUTPUT);
  digitalWrite(LOADSW_PIN, LOW);
}

void powerSensorsOn() {
  digitalWrite(LOADSW_PIN, HIGH);
  delay(SENSOR_POWER_ON_DELAY_MS);
}

void powerSensorsOff() {
  digitalWrite(LOADSW_PIN, LOW);
}

void setupScale() {
  scale.begin(HX711_DOUT, HX711_SCK);
}

void setupLight() {
  Wire.begin(I2C_SDA, I2C_SCL);
  bh_ok = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
}

void setupTempInt() {
  sensors.begin();
  sensors.setWaitForConversion(true);
}

void setupDHT() {
  dht1.begin();
  dht2.begin();
}

void setupBattery() {
  pinMode(VBAT_PIN, INPUT);
  analogSetPinAttenuation(VBAT_PIN, ADC_11db);
}

void initAllSensorsAfterPowerOn() {
  setupScale();
  setupLight();
  setupTempInt();
  setupDHT();
  delay(DHT_POWER_ON_DELAY_MS);
}

// =============================================================
// 6) LORA (AT) + DOWNLINK
// =============================================================

String readLine(uint32_t timeoutMs) {
  uint32_t t0 = millis();
  String line;
  while (millis() - t0 < timeoutMs) {
    while (Serial2.available()) {
      char c = (char)Serial2.read();
      if (c == '\r') continue;
      if (c == '\n') {
        line.trim();
        if (line.length() > 0) return line;
        line = "";
      } else {
        line += c;
      }
    }
    delay(5);
  }
  return "";
}

bool joinNetwork(uint32_t timeoutMs) {
  Serial2.println("AT+JOIN");
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    String line = readLine(800);
    if (line.length()) {
      Serial.println(line);
      String up = line;
      up.toUpperCase();

      if (up.indexOf("JOINED ALREADY") >= 0) return true;
      if (up.indexOf("+JOIN: JOINED") >= 0) return true;
      if (up.indexOf("JOIN") >= 0 &&
         (up.indexOf("DONE") >= 0 || up.indexOf("SUCCESS") >= 0 || up.indexOf("JOINED") >= 0)) {
        return true;
      }

      if (up.indexOf("FAIL") >= 0 || up.indexOf("DENIED") >= 0 || up.indexOf("NO FREE") >= 0) {
        return false;
      }
    }
  }
  return false;
}

void loraLowPower() {
  Serial2.println("AT+LOWPOWER");
  delay(200);
  while (Serial2.available()) Serial2.read();
}

void setupLoRa() {
  Serial2.begin(9600, SERIAL_8N1, RX2_PIN, TX2_PIN);
  delay(150);

  Serial2.println("AT");
  delay(150);
  while (Serial2.available()) Serial2.read();

  Serial2.println("AT+ADR=ON");
  delay(150);
  while (Serial2.available()) Serial2.read();
}

// Downlink commands (Port 1):
// 01 mm      : override période (minutes)
// 02 hi lo   : set time (minutes since midnight)
// 03         : force night start
// 04         : force night stop
// 05 mm      : set highPeriodMin (10 or 15)
// 06 vv      : set nightMode (00=AUTO,01=LUX,02=TIME)
// 07         : reset config to defaults

bool hexByte(const char *p, uint8_t &out) {
  auto h = [](char c)->int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    return -1;
  };
  int hi = h(p[0]);
  int lo = h(p[1]);
  if (hi < 0 || lo < 0) return false;
  out = (uint8_t)((hi << 4) | lo);
  return true;
}

bool extractDownlink(const String &acc, int &port, String &hex) {
  port = -1;
  hex = "";

  String up = acc;
  up.toUpperCase();

  int pPort = up.indexOf("PORT");
  if (pPort >= 0) {
    int pSep = up.indexOf(":", pPort);
    if (pSep < 0) pSep = up.indexOf("=", pPort);
    if (pSep >= 0) {
      int i = pSep + 1;
      while (i < (int)up.length() && up[i] == ' ') i++;
      String num;
      while (i < (int)up.length() && isDigit(up[i])) {
        num += up[i];
        i++;
      }
      if (num.length()) port = num.toInt();
    }
  }

  int pRx = up.indexOf("RX:");
  if (pRx < 0) pRx = up.indexOf("RX=");
  if (pRx >= 0) {
    int i = pRx + 3;
    while (i < (int)up.length() && (up[i] == ' ' || up[i] == '"')) i++;
    String h;
    while (i < (int)up.length()) {
      char c = up[i];
      if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F')) h += c;
      else break;
      i++;
    }
    if (h.length() >= 2) {
      hex = h;
      if (port < 0) port = 1;
      return true;
    }
  }
  return false;
}

void resetConfigToDefaults() {
  highPeriodMin     = DEFAULT_HIGH_PERIOD_MIN;
  periodOverride    = false;
  overrideMin       = DEFAULT_OVERRIDE_MIN;
  nightMode         = DEFAULT_NIGHT_MODE;
  rtc_minute_of_day = DEFAULT_MINUTE_OF_DAY;
  time_synced       = false;
  nightActive       = false;
  nightUsedToday    = false;
  nightSendCount    = 0;
}

void applyDownlink(int port, const String &hex) {
  if (port != 1) return;
  if (hex.length() < 2) return;

  uint8_t cmd;
  if (!hexByte(hex.c_str(), cmd)) return;

  uint8_t arg1 = 0;
  uint8_t arg2 = 0;

  if (hex.length() >= 4) hexByte(hex.c_str() + 2, arg1);
  if (hex.length() >= 6) hexByte(hex.c_str() + 4, arg2);

  // On mémorise toujours le dernier downlink reçu
  lastDlCmd  = cmd;
  lastDlArg1 = arg1;
  lastDlArg2 = arg2;

  if (cmd == 0x01 && hex.length() >= 4) {
    uint8_t mm = arg1;
    if (mm >= 1) {
      overrideMin = mm;
      periodOverride = true;
      Serial.print("DL: override period=");
      Serial.print(overrideMin);
      Serial.println(" min");
    }
  }
  else if (cmd == 0x02 && hex.length() >= 6) {
    uint16_t m = (uint16_t)((arg1 << 8) | arg2);
    if (m < 1440) {
      rtc_minute_of_day = m;
      time_synced = true;
      Serial.print("DL: time sync minute_of_day=");
      Serial.println(rtc_minute_of_day);
    }
  }
  else if (cmd == 0x03) {
    nightActive = true;
    nightUsedToday = true;
    nightSendCount = 0;
    Serial.println("DL: force NIGHT START");
  }
  else if (cmd == 0x04) {
    nightActive = false;
    nightSendCount = 0;
    Serial.println("DL: force NIGHT STOP");
  }
  else if (cmd == 0x05 && hex.length() >= 4) {
    uint8_t mm = arg1;
    if (mm == 10 || mm == 15) {
      highPeriodMin = mm;
      Serial.print("DL: highPeriodMin=");
      Serial.println(highPeriodMin);
    }
  }
  else if (cmd == 0x06 && hex.length() >= 4) {
    uint8_t vv = arg1;
    if (vv <= 2) {
      nightMode = vv;
      Serial.print("DL: nightMode=");
      Serial.println((int)nightMode);
    }
  }
  else if (cmd == 0x07) {
    resetConfigToDefaults();
    Serial.println("DL: RESET TO DEFAULTS");
  }
}

// =============================================================
// 7) LECTURES
// =============================================================

float getWeight(bool &ok) {
  ok = true;

  const int n = 5;
  long sum = 0;

  for (int i = 0; i < n; i++) {
    uint32_t t0 = millis();
    while (!scale.is_ready()) {
      if (millis() - t0 > 500) {
        ok = false;
        return 0.0f;
      }
      delay(5);
    }
    sum += scale.read();
  }

  long rawAvg = sum / n;
  float w = (float)(SCALE_SIGN * (rawAvg - SCALE_OFFSET)) / SCALE_FACTOR;

  w = WEIGHT_A * w + WEIGHT_B;

  if (w < 0.0f) w = 0.0f;

  return w;
}

float getLux() {
  return lightMeter.readLightLevel();
}

void getDHT(float &t1, float &h1, float &t2, float &h2) {
  t1 = dht1.readTemperature();
  h1 = dht1.readHumidity();
  t2 = dht2.readTemperature();
  h2 = dht2.readHumidity();
}

// ---------- Batterie calibrée ----------
uint16_t getBatteryAdcRawAvg() {
  uint32_t sum = 0;
  for (int i = 0; i < VBAT_SAMPLES; i++) {
    sum += analogRead(VBAT_PIN);
    delay(3);
  }
  uint32_t avg = sum / VBAT_SAMPLES;
  if (avg > 4095) avg = 4095;
  return (uint16_t)avg;
}

float getBatteryVoltageFromRaw(uint16_t adcRaw) {
  return VBAT_RATIO * ((float)adcRaw / 4095.0f) * 3.3f;
}

uint16_t getBatteryMilliVoltsFromRaw(uint16_t adcRaw) {
  float vbat = getBatteryVoltageFromRaw(adcRaw);
  int mv = (int)lroundf(vbat * 1000.0f);
  if (mv < 0) mv = 0;
  if (mv > 65535) mv = 65535;
  return (uint16_t)mv;
}

int getBatteryPercentFromMilliVolts(uint16_t mv) {
  if (mv >= 4200) return 100;
  if (mv <= 3300) return 0;

  struct Pt { int mv; int p; };
  static const Pt table[] = {
    {4200,100},{4100,90},{4000,80},{3900,60},
    {3800,40},{3700,20},{3600,10},{3500,5},{3300,0}
  };

  for (int i = 0; i < 8; i++) {
    int v1 = table[i].mv, v2 = table[i + 1].mv;
    int p1 = table[i].p,  p2 = table[i + 1].p;
    if (mv <= v1 && mv >= v2) {
      float u = (float)(mv - v2) / (float)(v1 - v2);
      int percent = (int)(p2 + u * (p1 - p2) + 0.5f);
      if (percent < 0) percent = 0;
      if (percent > 100) percent = 100;
      return percent;
    }
  }
  return 0;
}

// =============================================================
// 8) MODE NUIT + SCHEDULER
// =============================================================

bool isNightByTime(uint16_t minuteOfDay) {
  return (minuteOfDay >= NIGHT_START_MIN) || (minuteOfDay < NIGHT_END_MIN);
}

void resetNightUsedIfNeeded() {
  if (time_synced) {
    if (rtc_minute_of_day >= 8 * 60 && rtc_minute_of_day < 10 * 60) {
      nightUsedToday = false;
    }
  }
}

void updateNightState(int batP, float lux) {
  if (batP < 20) {
    nightActive = false;
    nightSendCount = 0;
    return;
  }

  resetNightUsedIfNeeded();

  bool nightDetected = false;

  if (nightMode == NIGHT_LUX) {
    if (bh_ok && lux >= 0) nightDetected = (lux < LUX_NIGHT_THRESHOLD);
    else nightDetected = false;
  }
  else if (nightMode == NIGHT_TIME) {
    if (time_synced) nightDetected = isNightByTime(rtc_minute_of_day);
    else nightDetected = false;
  }
  else { // AUTO
    if (bh_ok && lux >= 0) nightDetected = (lux < LUX_NIGHT_THRESHOLD);
    else if (time_synced) nightDetected = isNightByTime(rtc_minute_of_day);
    else nightDetected = false;
  }

  if (!nightActive && !nightUsedToday && nightDetected) {
    nightActive = true;
    nightUsedToday = true;
    nightSendCount = 0;
    Serial.println("NIGHT: start");
  }

  if (nightActive) {
    if (!nightDetected) {
      nightActive = false;
      nightSendCount = 0;
      Serial.println("NIGHT: stop (day detected)");
    } else if (nightSendCount >= NIGHT_MAX_SENDS) {
      nightActive = false;
      nightSendCount = 0;
      Serial.println("NIGHT: stop (max sends)");
    }
  }
}

uint8_t computeRunMode(int batP) {
  if (periodOverride) return RUN_OVERRIDE;
  if (batP < 20) return RUN_LOW_BATT;
  if (nightActive) return RUN_NIGHT;
  return RUN_NORMAL;
}

uint16_t computePeriodMin(int batP) {
  if (periodOverride) return overrideMin;
  if (batP < 20) return LOW_BATT_PERIOD_MIN;
  if (nightActive) return NIGHT_PERIOD_MIN;

  if (batP >= 80) return highPeriodMin;
  if (batP >= 60) return 20;
  if (batP >= 40) return 30;
  if (batP >= 20) return 45;
  return 60;
}

// =============================================================
// 9) ENVOI LORA
// Payload = 24 octets
// tInt1,tInt2,tDht1,hDht1,tDht2,hDht2,lux,poids,bat%,swarm,runMode,vbat_mV,lastDlCmd,lastDlArg1,lastDlArg2
// =============================================================

TxResult envoyerPayloadLoRa(const char *payloadHex, uint32_t timeoutMs) {
  while (Serial2.available()) Serial2.read();

  Serial2.print("AT+MSGHEX=\"");
  Serial2.print(payloadHex);
  Serial2.println("\"");

  TxResult r;
  r.done = false;
  r.joinNeeded = false;
  r.hasDownlink = false;
  r.dlPort = -1;
  r.dlHex = "";

  String acc;
  uint32_t t0 = millis();

  bool doneSeen = false;
  uint32_t doneTime = 0;
  const uint32_t POST_DONE_LISTEN_MS = 700;

  while (millis() - t0 < timeoutMs) {
    while (Serial2.available()) {
      char c = (char)Serial2.read();
      acc += c;
      if (acc.length() > 1000) acc.remove(0, 500);

      String up = acc;
      up.toUpperCase();

      if (up.indexOf("PLEASE JOIN NETWORK FIRST") >= 0) {
        r.joinNeeded = true;
      }

      if (!doneSeen && (up.indexOf("+MSGHEX: DONE") >= 0 || up.indexOf("DONE") >= 0)) {
        r.done = true;
        doneSeen = true;
        doneTime = millis();
      }

      if (!r.hasDownlink) {
        int port;
        String hex;
        if (extractDownlink(acc, port, hex)) {
          r.hasDownlink = true;
          r.dlPort = port;
          r.dlHex = hex;
        }
      }
    }

    if (doneSeen) {
      if (r.hasDownlink) break;
      if (millis() - doneTime >= POST_DONE_LISTEN_MS) break;
    }

    delay(5);
  }

  if (acc.length()) {
    acc.replace("\r", "");
    Serial.print(acc);
  }

  return r;
}

bool envoyerDonneesLoRa(int16_t tInt1Val, int16_t tInt2Val,
                        int16_t tDht1Val, int16_t hDht1Val,
                        int16_t tDht2Val, int16_t hDht2Val,
                        uint16_t luxVal, int16_t poidsVal,
                        uint8_t batPercent,
                        uint8_t swarmAlert,
                        uint8_t runMode,
                        uint16_t vbat_mV,
                        uint8_t dlCmd,
                        uint8_t dlArg1,
                        uint8_t dlArg2) {
  char payload[128];

  sprintf(payload, "%04X%04X%04X%04X%04X%04X%04X%04X%02X%02X%02X%04X%02X%02X%02X",
          (uint16_t)tInt1Val, (uint16_t)tInt2Val,
          (uint16_t)tDht1Val, (uint16_t)hDht1Val,
          (uint16_t)tDht2Val, (uint16_t)hDht2Val,
          luxVal, (uint16_t)poidsVal,
          batPercent,
          swarmAlert,
          runMode,
          vbat_mV,
          dlCmd,
          dlArg1,
          dlArg2);

  Serial.print("Payload LoRa : ");
  Serial.println(payload);

  TxResult r = envoyerPayloadLoRa(payload, 30000);

  if (r.joinNeeded) {
    if (!joinNetwork(30000)) {
      loraJoined = false;
      return false;
    }
    loraJoined = true;
    r = envoyerPayloadLoRa(payload, 30000);
  }

  if (r.hasDownlink) {
    Serial.print("Downlink: port=");
    Serial.print(r.dlPort);
    Serial.print(" hex=");
    Serial.println(r.dlHex);
    applyDownlink(r.dlPort, r.dlHex);
  }

  return r.done;
}

// =============================================================
// 10) SETUP / LOOP
// =============================================================

void setup() {
  Serial.begin(115200);
  delay(200);

  buzzerInit();

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  if (cause == ESP_SLEEP_WAKEUP_UNDEFINED) {
    playMelodyStartup();
  }

  WiFi.mode(WIFI_OFF);
  btStop();

  setupLoadSwitch();
  setupBattery();
  setupLoRa();
}

void loop() {
  powerSensorsOn();
  initAllSensorsAfterPowerOn();

  // --- Capteurs ---
  bool okHX = false;
  float poids = getWeight(okHX);

  float lux = bh_ok ? getLux() : -1.0f;

  float tDHT1, hDHT1, tDHT2, hDHT2;
  getDHT(tDHT1, hDHT1, tDHT2, hDHT2);

  uint16_t batRaw  = getBatteryAdcRawAvg();
  uint16_t vbat_mV = getBatteryMilliVoltsFromRaw(batRaw);
  int batP         = getBatteryPercentFromMilliVolts(vbat_mV);

  int dsCount = sensors.getDeviceCount();

  float tInt1 = DEVICE_DISCONNECTED_C;
  float tInt2 = DEVICE_DISCONNECTED_C;

  sensors.requestTemperatures();

  if (dsCount >= 1) {
    tInt1 = sensors.getTempCByIndex(0);
    if (tInt1 == 85.0f) tInt1 = DEVICE_DISCONNECTED_C;
  }
  if (dsCount >= 2) {
    tInt2 = sensors.getTempCByIndex(1);
    if (tInt2 == 85.0f) tInt2 = DEVICE_DISCONNECTED_C;
  }

  bool okDHT1 = (!isnan(tDHT1) && !isnan(hDHT1));
  bool okDHT2 = (!isnan(tDHT2) && !isnan(hDHT2));

  // --- Etat courant avant envoi ---
  updateNightState(batP, lux);

  // --- Essaimage ---
  uint8_t swarmAlert = 0;
  if (lastWeightKg >= 0.0f && lastWeightAgeMin <= SWARM_MAX_DT_MIN) {
    float delta = poids - lastWeightKg;
    if (delta <= -SWARM_DROP_KG) swarmAlert = 1;
  }

  // --- Encodage ---
  int16_t tInt1Val = (tInt1 != DEVICE_DISCONNECTED_C) ? to_i16_scaled(tInt1, 10.0f, SENTINEL_I16) : SENTINEL_I16;
  int16_t tInt2Val = (tInt2 != DEVICE_DISCONNECTED_C) ? to_i16_scaled(tInt2, 10.0f, SENTINEL_I16) : SENTINEL_I16;

  int16_t tDht1Val = okDHT1 ? to_i16_scaled(tDHT1, 10.0f, SENTINEL_I16) : SENTINEL_I16;
  int16_t hDht1Val = okDHT1 ? to_i16_scaled(hDHT1, 10.0f, SENTINEL_I16) : SENTINEL_I16;

  int16_t tDht2Val = okDHT2 ? to_i16_scaled(tDHT2, 10.0f, SENTINEL_I16) : SENTINEL_I16;
  int16_t hDht2Val = okDHT2 ? to_i16_scaled(hDHT2, 10.0f, SENTINEL_I16) : SENTINEL_I16;

  uint16_t luxVal = bh_ok ? to_u16_clamped(lux) : 0;

  int16_t poidsVal = to_i16_scaled(poids, 100.0f, SENTINEL_I16);
  if (poidsVal < 0) poidsVal = 0;

  uint8_t batPercent = (batP < 0) ? 0 : (batP > 100) ? 100 : (uint8_t)batP;

  // Mode envoyé dans l'uplink courant
  uint8_t runModePayload = computeRunMode(batP);

  powerSensorsOff();

  bool uplinkSent = false;
  bool nightWasActiveBeforeTx = nightActive;

  if (!loraJoined) {
    loraJoined = joinNetwork(30000);
  }

  if (loraJoined) {
    uplinkSent = envoyerDonneesLoRa(
      tInt1Val, tInt2Val,
      tDht1Val, hDht1Val,
      tDht2Val, hDht2Val,
      luxVal, poidsVal,
      batPercent,
      swarmAlert,
      runModePayload,
      vbat_mV,
      lastDlCmd,
      lastDlArg1,
      lastDlArg2
    );
  }

  // IMPORTANT : on recalcule après le downlink éventuellement reçu
  updateNightState(batP, lux);

  uint16_t periodMin = computePeriodMin(batP);
  uint32_t sleepSec  = (uint32_t)periodMin * 60UL;

  if (uplinkSent && nightWasActiveBeforeTx) {
    nightSendCount++;
  }

  lastWeightKg = poids;
  lastWeightAgeMin = periodMin;

  rtc_minute_of_day = (uint16_t)((rtc_minute_of_day + periodMin) % 1440);

  loraLowPower();
  buzzerOff();

  Serial.flush();
  esp_sleep_enable_timer_wakeup((uint64_t)sleepSec * 1000000ULL);
  esp_deep_sleep_start();
}