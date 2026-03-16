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

// --- CONFIGURATION PINS ---
#define LOADSW_PIN 2
#define BUZZER_PIN 23  // Buzzer sur GPIO 23
#define HX711_DOUT 25
#define HX711_SCK  26
#define I2C_SDA 21
#define I2C_SCL 22
#define ONE_WIRE_BUS 33
#define DHT1_PIN 27
#define DHT2_PIN 19
#define DHTTYPE DHT22
#define VBAT_PIN 35
#define RX2_PIN 16
#define TX2_PIN 17

// --- PARAMÈTRES ---
const long  SCALE_OFFSET = 145192;
const float SCALE_FACTOR = 31616.384766;
const int   SCALE_SIGN   = 1;
const float VBAT_RATIO = 1.435f;
const int   VBAT_SAMPLES = 30;
const uint32_t SLEEP_PERIOD_SEC = 30;
const int16_t SENTINEL_I16 = 32767;

RTC_DATA_ATTR bool loraJoined = false;

// --- INSTANCES ---
HX711 scale;
BH1750 lightMeter;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress1, tempDeviceAddress2;
bool ds18_1_ok = false, ds18_2_ok = false;
DHT dht1(DHT1_PIN, DHTTYPE);
DHT dht2(DHT2_PIN, DHTTYPE);

// --- FONCTIONS AUDIO (CORE 3.x.x) ---
void playTone(int freq, int durationMs, int pauseMs) {
  if (freq <= 0) {
    ledcWrite(BUZZER_PIN, 0); 
    delay(durationMs + pauseMs);
    return;
  }
  ledcWriteTone(BUZZER_PIN, freq); 
  delay(durationMs);
  ledcWrite(BUZZER_PIN, 0); // Arrêt du son
  delay(pauseMs);
}

void playMelodyStartup() {
  playTone(523, 150, 30);  // C5
  playTone(659, 150, 30);  // E5
  playTone(784, 150, 30);  // G5
  playTone(1046, 200, 80); // C6
}

void playMelodyTx() {
  playTone(880, 100, 50);  // Petit bip court pour l'envoi
}

// --- GESTION CAPTEURS ---
void powerSensorsOn() {
  digitalWrite(LOADSW_PIN, HIGH);
  delay(800);
}

void powerSensorsOff() {
  digitalWrite(LOADSW_PIN, LOW);
}

void setupLoRa() {
  Serial2.begin(9600, SERIAL_8N1, RX2_PIN, TX2_PIN);
  delay(100);
  Serial2.println("AT");
}

float getBatteryVoltage() {
  uint32_t sum = 0;
  for (int i = 0; i < VBAT_SAMPLES; i++) { sum += analogRead(VBAT_PIN); delay(3); }
  return VBAT_RATIO * ((float)sum / VBAT_SAMPLES / 4095.0f) * 3.3f;
}

// --- COMMUNICATION ---
String readLine(uint32_t timeoutMs) {
  uint32_t t0 = millis();
  String line = "";
  while (millis() - t0 < timeoutMs) {
    while (Serial2.available()) {
      char c = (char)Serial2.read();
      if (c == '\n') { line.trim(); if (line.length() > 0) return line; line = ""; }
      else if (c != '\r') line += c;
    }
    delay(5);
  }
  return "";
}

bool joinNetwork(uint32_t timeoutMs) {
  Serial2.println("AT+JOIN");
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    String line = readLine(1000);
    if (line.indexOf("JOINED") >= 0) return true;
  }
  return false;
}

TxResult envoyerPayloadLoRa(const char *payloadHex, uint32_t timeoutMs) {
  while (Serial2.available()) Serial2.read();
  
  playMelodyTx(); // Bip au début de l'envoi

  Serial2.print("AT+MSGHEX=\"");
  Serial2.print(payloadHex);
  Serial2.println("\"");

  TxResult r = {false, false};
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    String line = readLine(500);
    if (line.indexOf("JOIN") >= 0 && line.indexOf("FIRST") >= 0) r.joinNeeded = true;
    if (line.indexOf("DONE") >= 0) { r.done = true; break; }
  }
  return r;
}

// --- MAIN ---
void setup() {
  Serial.begin(115200);

  // Initialisation Buzzer pour ESP32 Core 3.x.x
  ledcAttach(BUZZER_PIN, 2000, 10); 
  ledcWrite(BUZZER_PIN, 0);

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  if (cause == ESP_SLEEP_WAKEUP_UNDEFINED) {
    Serial.println("\n--- DEMARRAGE COMPLET ---");
    playMelodyStartup();
  } else {
    Serial.println("\n--- REVEIL DEEP SLEEP ---");
  }

  WiFi.mode(WIFI_OFF);
  btStop();
  
  pinMode(LOADSW_PIN, OUTPUT);
  digitalWrite(LOADSW_PIN, LOW);
  pinMode(VBAT_PIN, INPUT);
  analogSetPinAttenuation(VBAT_PIN, ADC_11db);
  
  setupLoRa();
}

void loop() {
  powerSensorsOn();
  
  // Init capteurs après alimentation
  scale.begin(HX711_DOUT, HX711_SCK);
  Wire.begin(I2C_SDA, I2C_SCL);
  lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
  dht1.begin();
  dht2.begin();
  sensors.begin();
  
  // Recherche des adresses DS18B20
  ds18_1_ok = sensors.getAddress(tempDeviceAddress1, 0);
  ds18_2_ok = sensors.getAddress(tempDeviceAddress2, 1);
  if(ds18_1_ok) sensors.setResolution(tempDeviceAddress1, 12);
  if(ds18_2_ok) sensors.setResolution(tempDeviceAddress2, 12);

  delay(1500); // Attente stabilisation DHT22

  // Lecture
  long sumWeight = 0;
  for (int i = 0; i < 10; i++) {
    while (!scale.is_ready()) delay(5);
    sumWeight += scale.read();
  }
  float poids = (float)(SCALE_SIGN * (sumWeight / 10 - SCALE_OFFSET)) / SCALE_FACTOR;
  
  float lux = lightMeter.readLightLevel();
  float tD1 = dht1.readTemperature(), hD1 = dht1.readHumidity();
  float tD2 = dht2.readTemperature(), hD2 = dht2.readHumidity();
  
  sensors.requestTemperatures();
  float tI1 = ds18_1_ok ? sensors.getTempC(tempDeviceAddress1) : DEVICE_DISCONNECTED_C;
  float tI2 = ds18_2_ok ? sensors.getTempC(tempDeviceAddress2) : DEVICE_DISCONNECTED_C;
  
  float vbat = getBatteryVoltage();

  // Préparation Payload
  int16_t t1 = (tI1 != DEVICE_DISCONNECTED_C && tI1 != 85.0) ? tI1*10 : SENTINEL_I16;
  int16_t t2 = (tI2 != DEVICE_DISCONNECTED_C && tI2 != 85.0) ? tI2*10 : SENTINEL_I16;
  int16_t td1 = !isnan(tD1) ? tD1*10 : SENTINEL_I16;
  int16_t hd1 = !isnan(hD1) ? hD1*10 : SENTINEL_I16;
  int16_t td2 = !isnan(tD2) ? tD2*10 : SENTINEL_I16;
  int16_t hd2 = !isnan(hD2) ? hD2*10 : SENTINEL_I16;
  uint16_t luxVal = (lux < 0) ? 0 : (lux > 65535) ? 65535 : (uint16_t)lux;
  int16_t pVal = (int16_t)(poids * 100);
  uint16_t vbat_mV = (uint16_t)(vbat * 1000);

  char payload[80];
  sprintf(payload, "%04X%04X%04X%04X%04X%04X%04X%04X%02X%04X", 
          (uint16_t)t1, (uint16_t)t2, (uint16_t)td1, (uint16_t)hd1, (uint16_t)td2, (uint16_t)hd2, luxVal, (uint16_t)pVal, 50, vbat_mV);

  // Envoi LoRa
  if (!loraJoined) loraJoined = joinNetwork(30000);
  if (loraJoined) {
    TxResult res = envoyerPayloadLoRa(payload, 30000);
    if (res.joinNeeded) {
      if (joinNetwork(30000)) envoyerPayloadLoRa(payload, 30000);
    }
  }

  powerSensorsOff();
  Serial2.println("AT+LOWPOWER");
  
  esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_PERIOD_SEC * 1000000ULL);
  esp_deep_sleep_start();
}