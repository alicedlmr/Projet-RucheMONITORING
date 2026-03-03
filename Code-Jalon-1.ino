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
};

#define LOADSW_PIN 2
const int SENSOR_POWER_ON_DELAY_MS = 800;
const int DHT_POWER_ON_DELAY_MS    = 1500;

#define HX711_DOUT 25
#define HX711_SCK  26
const long  SCALE_OFFSET = 145192;
const float SCALE_FACTOR = 31616.384766;
const int   SCALE_SIGN   = 1;

#define I2C_SDA 21
#define I2C_SCL 22

#define ONE_WIRE_BUS 33

#define DHT1_PIN 27
#define DHT2_PIN 19
#define DHTTYPE DHT22

#define VBAT_PIN 35
const float VBAT_RATIO = 1.435f;
const int   VBAT_SAMPLES = 30;

#define RX2_PIN 16
#define TX2_PIN 17
#define LED_PIN 23

const uint32_t SLEEP_PERIOD_SEC = 30;
const int16_t SENTINEL_I16 = 32767;

RTC_DATA_ATTR bool loraJoined = false;

HX711 scale;
BH1750 lightMeter;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

DeviceAddress tempDeviceAddress1;
DeviceAddress tempDeviceAddress2;
bool ds18_1_ok = false;
bool ds18_2_ok = false;

DHT dht1(DHT1_PIN, DHTTYPE);
DHT dht2(DHT2_PIN, DHTTYPE);

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
  lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
}

void setupTempInt() {
  sensors.begin();
  int count = sensors.getDeviceCount();
  
  if (count >= 1) {
    if (sensors.getAddress(tempDeviceAddress1, 0)) {
      sensors.setResolution(tempDeviceAddress1, 12);
      ds18_1_ok = true;
    }
  } else {
    ds18_1_ok = false;
  }

  if (count >= 2) {
    if (sensors.getAddress(tempDeviceAddress2, 1)) {
      sensors.setResolution(tempDeviceAddress2, 12);
      ds18_2_ok = true;
    }
  } else {
    ds18_2_ok = false;
  }
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
    String line = readLine(600);
    if (line.length()) {
      Serial.println(line);
      String up = line;
      up.toUpperCase();
      if (up.indexOf("JOINED ALREADY") >= 0) return true;
      if (up.indexOf("+JOIN: JOINED") >= 0) return true;
      if (up.indexOf("JOIN") >= 0 && (up.indexOf("DONE") >= 0 || up.indexOf("SUCCESS") >= 0 || up.indexOf("JOINED") >= 0)) {
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
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  delay(100);

  Serial2.println("AT");
  uint32_t t = millis();
  while (millis() - t < 500) {
    while (Serial2.available()) {
      Serial2.read();
    }
  }
}

float getWeight() {
  long sum = 0;
  int n = 10;
  for (int i = 0; i < n; i++) {
    while (!scale.is_ready()) delay(5);
    sum += scale.read();
  }
  long rawAvg = sum / n;
  return (float)(SCALE_SIGN * (rawAvg - SCALE_OFFSET)) / SCALE_FACTOR;
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

float getBatteryVoltage() {
  uint32_t sum = 0;
  for (int i = 0; i < VBAT_SAMPLES; i++) {
    sum += analogRead(VBAT_PIN);
    delay(3);
  }
  float adc = (float)sum / (float)VBAT_SAMPLES;
  return VBAT_RATIO * (adc / 4095.0f) * 3.3f;
}

int getBatteryPercent() {
  float vbat = getBatteryVoltage();
  if (vbat >= 4.20f) return 100;
  if (vbat <= 3.30f) return 0;
  
  struct Pt { float v; int p; };
  static const Pt table[] = {
    {4.20f,100},{4.10f,90},{4.00f,80},{3.90f,60},
    {3.80f,40},{3.70f,20},{3.60f,10},{3.50f,5},{3.30f,0}
  };

  for (int i = 0; i < 8; i++) {
    float v1 = table[i].v, v2 = table[i + 1].v;
    int   p1 = table[i].p, p2 = table[i + 1].p;
    if (vbat <= v1 && vbat >= v2) {
      float u = (vbat - v2) / (v1 - v2);
      float p = p2 + u * (p1 - p2);
      int percent = (int)(p + 0.5f);
      if (percent < 0) percent = 0;
      if (percent > 100) percent = 100;
      return percent;
    }
  }
  return 0;
}

TxResult envoyerPayloadLoRa(const char *payloadHex, uint32_t timeoutMs) {
  while (Serial2.available()) Serial2.read();
  digitalWrite(LED_PIN, HIGH);
  Serial2.print("AT+MSGHEX=\"");
  Serial2.print(payloadHex);
  Serial2.println("\"");

  TxResult r;
  r.done = false;
  r.joinNeeded = false;

  String acc;
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    while (Serial2.available()) {
      char c = (char)Serial2.read();
      acc += c;
      if (acc.length() > 400) acc.remove(0, 200);
      String up = acc;
      up.toUpperCase();
      if (up.indexOf("PLEASE JOIN NETWORK FIRST") >= 0) r.joinNeeded = true;
      if (up.indexOf("+MSGHEX: DONE") >= 0 || up.indexOf("DONE") >= 0) {
        r.done = true;
        break;
      }
    }
    if (r.done) break;
    delay(5);
  }
  digitalWrite(LED_PIN, LOW);
  if (acc.length()) {
    acc.replace("\r", "");
    Serial.print(acc);
  }
  return r;
}

bool envoyerDonneesLoRa(int16_t tInt1Val, int16_t tInt2Val, int16_t tDht1Val, int16_t hDht1Val, int16_t tDht2Val, int16_t hDht2Val, uint16_t luxVal, int16_t poidsVal, uint8_t batPercent, uint16_t vbat_mV) {
  char payload[80];
  sprintf(payload, "%04X%04X%04X%04X%04X%04X%04X%04X%02X%04X",
          (uint16_t)tInt1Val, (uint16_t)tInt2Val, 
          (uint16_t)tDht1Val, (uint16_t)hDht1Val, 
          (uint16_t)tDht2Val, (uint16_t)hDht2Val, 
          luxVal, (uint16_t)poidsVal,
          batPercent, vbat_mV);

  Serial.print("Payload LoRa : ");
  Serial.println(payload);

  TxResult r = envoyerPayloadLoRa(payload, 30000);
  if (r.done) return true;

  if (r.joinNeeded) {
    if (!joinNetwork(30000)) {
      loraJoined = false;
      return false;
    }
    loraJoined = true;
    r = envoyerPayloadLoRa(payload, 30000);
    return r.done;
  }

  return r.done;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(LED_PIN, OUTPUT);

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  if (cause == ESP_SLEEP_WAKEUP_UNDEFINED) {
    Serial.println("\n--- DEMARRAGE COMPLET ---");
    digitalWrite(LED_PIN, HIGH);
    delay(5000);
    digitalWrite(LED_PIN, LOW);
  } else {
    Serial.println("\n--- REVEIL DEEP SLEEP ---");
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

  float poids = getWeight();
  float lux = getLux();
  
  float tempDHT1, humDHT1, tempDHT2, humDHT2;
  getDHT(tempDHT1, humDHT1, tempDHT2, humDHT2);
  
  float vbat = getBatteryVoltage();
  int batP = getBatteryPercent();

  float tempInt1 = DEVICE_DISCONNECTED_C;
  float tempInt2 = DEVICE_DISCONNECTED_C;
  
  sensors.requestTemperatures();
  
  if (ds18_1_ok) {
    tempInt1 = sensors.getTempC(tempDeviceAddress1);
    if (tempInt1 == 85.0f) tempInt1 = DEVICE_DISCONNECTED_C;
  }
  if (ds18_2_ok) {
    tempInt2 = sensors.getTempC(tempDeviceAddress2);
    if (tempInt2 == 85.0f) tempInt2 = DEVICE_DISCONNECTED_C;
  }

  bool okDHT1 = (!isnan(tempDHT1) && !isnan(humDHT1));
  bool okDHT2 = (!isnan(tempDHT2) && !isnan(humDHT2));

  Serial.print("DS18B20 (1) : "); Serial.println((tempInt1 != DEVICE_DISCONNECTED_C) ? String(tempInt1) + " C" : "Err");
  Serial.print("DS18B20 (2) : "); Serial.println((tempInt2 != DEVICE_DISCONNECTED_C) ? String(tempInt2) + " C" : "Err");
  Serial.print("DHT22 (P27) : "); Serial.println(okDHT1 ? String(tempDHT1) + " C | " + String(humDHT1) + " %" : "Err");
  Serial.print("DHT22 (P19) : "); Serial.println(okDHT2 ? String(tempDHT2) + " C | " + String(humDHT2) + " %" : "Err");
  Serial.print("BH1750      : "); Serial.println(String(lux) + " lux");
  Serial.print("HX711       : "); Serial.println(String(poids, 3) + " kg");
  Serial.print("BATT        : "); Serial.println(String(vbat, 3) + " V | " + String(batP) + " %");

  int16_t tInt1Val = (tempInt1 != DEVICE_DISCONNECTED_C) ? (int16_t)(tempInt1 * 10.0f) : SENTINEL_I16;
  int16_t tInt2Val = (tempInt2 != DEVICE_DISCONNECTED_C) ? (int16_t)(tempInt2 * 10.0f) : SENTINEL_I16;
  
  int16_t tDht1Val = okDHT1 ? (int16_t)(tempDHT1 * 10.0f) : SENTINEL_I16;
  int16_t hDht1Val = okDHT1 ? (int16_t)(humDHT1  * 10.0f) : SENTINEL_I16;
  
  int16_t tDht2Val = okDHT2 ? (int16_t)(tempDHT2 * 10.0f) : SENTINEL_I16;
  int16_t hDht2Val = okDHT2 ? (int16_t)(humDHT2  * 10.0f) : SENTINEL_I16;
  
  uint16_t luxVal = (lux < 0) ? 0 : (lux > 65535) ? 65535 : (uint16_t)lux;
  int16_t poidsVal = (int16_t)(poids * 100.0f);
  
  int vbat_mV_i = (int)(vbat * 1000.0f + 0.5f);
  if (vbat_mV_i < 0) vbat_mV_i = 0;
  if (vbat_mV_i > 65535) vbat_mV_i = 65535;
  uint16_t vbat_mV = (uint16_t)vbat_mV_i;
  
  uint8_t batPercent = (batP < 0) ? 0 : (batP > 100) ? 100 : (uint8_t)batP;

  if (!loraJoined) {
    loraJoined = joinNetwork(30000);
  }

  if (loraJoined) {
    envoyerDonneesLoRa(tInt1Val, tInt2Val, tDht1Val, hDht1Val, tDht2Val, hDht2Val, luxVal, poidsVal, batPercent, vbat_mV);
  }

  powerSensorsOff();
  loraLowPower();

  esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_PERIOD_SEC * 1000000ULL);
  esp_deep_sleep_start();
}