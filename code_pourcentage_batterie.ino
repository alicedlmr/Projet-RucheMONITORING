#include <Arduino.h>

#define RX2_PIN 16
#define TX2_PIN 17
#define LED_PIN 23

// Batterie uPesy : sense sur GPIO35
#define VBAT_PIN 35
const float VBAT_RATIO = 1.435f;
const int   VBAT_SAMPLES = 30;

// ----------------------- Helpers UART AT -----------------------
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
  Serial.println("AT+JOIN");
  Serial2.println("AT+JOIN");

  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    String line = readLine(700);
    if (!line.length()) continue;

    Serial.println(line);
    String up = line; up.toUpperCase();

    if (up.indexOf("JOINED ALREADY") >= 0) return true;
    if (up.indexOf("+JOIN: JOINED") >= 0) return true;
    if (up.indexOf("JOIN") >= 0 && (up.indexOf("DONE") >= 0 || up.indexOf("SUCCESS") >= 0 || up.indexOf("JOINED") >= 0)) return true;

    if (up.indexOf("FAIL") >= 0 || up.indexOf("DENIED") >= 0 || up.indexOf("NO FREE") >= 0) return false;
  }
  return false;
}

bool sendHexPayload(const char* payloadHex, uint32_t timeoutMs) {
  while (Serial2.available()) Serial2.read();

  digitalWrite(LED_PIN, HIGH);

  Serial.print("AT+MSGHEX=\""); Serial.print(payloadHex); Serial.println("\"");
  Serial2.print("AT+MSGHEX=\"");
  Serial2.print(payloadHex);
  Serial2.println("\"");

  bool done = false;
  bool joinNeeded = false;

  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    String line = readLine(700);
    if (!line.length()) continue;

    Serial.println(line);
    String up = line; up.toUpperCase();

    if (up.indexOf("PLEASE JOIN NETWORK FIRST") >= 0) joinNeeded = true;
    if (up.indexOf("DONE") >= 0) { done = true; break; }
  }

  digitalWrite(LED_PIN, LOW);

  if (joinNeeded) return false;
  return done;
}

// ----------------------- Batterie -----------------------
float getBatteryVoltage() {
  pinMode(VBAT_PIN, INPUT);
  analogSetPinAttenuation(VBAT_PIN, ADC_11db);

  uint32_t sum = 0;
  for (int i = 0; i < VBAT_SAMPLES; i++) {
    sum += analogRead(VBAT_PIN);   // 0..4095
    delay(3);
  }
  float adc = (float)sum / (float)VBAT_SAMPLES;
  return VBAT_RATIO * (adc / 4095.0f) * 3.3f;
}

int getBatteryPercent() {
  float v = getBatteryVoltage();

  if (v >= 4.20f) return 100;
  if (v <= 3.30f) return 0;

  struct Pt { float v; int p; };
  static const Pt t[] = {
    {4.20f,100},{4.10f,90},{4.00f,80},{3.90f,60},
    {3.80f,40},{3.70f,20},{3.60f,10},{3.50f,5},{3.30f,0}
  };

  for (int i = 0; i < 8; i++) {
    float v1 = t[i].v, v2 = t[i+1].v;
    int p1 = t[i].p, p2 = t[i+1].p;
    if (v <= v1 && v >= v2) {
      float u = (v - v2) / (v1 - v2);
      int percent = (int)(p2 + u * (p1 - p2) + 0.5f);
      if (percent < 0) percent = 0;
      if (percent > 100) percent = 100;
      return percent;
    }
  }
  return 0;
}

// ----------------------- Main -----------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial2.begin(9600, SERIAL_8N1, RX2_PIN, TX2_PIN);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  bool ok = joinNetwork(30000);
  if (ok) Serial.println("JOIN OK.");
  else    Serial.println("JOIN ECHEC (envoi peut echouer).");
}

void loop() {
  float vbat = getBatteryVoltage();
  int pct = getBatteryPercent();

  Serial.print("VBAT=");
  Serial.print(vbat, 3);
  Serial.print(" V | BAT=");
  Serial.print(pct);
  Serial.println(" %");

  // Payload = 1 octet (00..64) => 2 caractères hex
  char payload[3];
  sprintf(payload, "%02X", (uint8_t)pct);

  bool ok = sendHexPayload(payload, 20000);
  if (!ok) {
    // Si pas join, on tente join puis renvoi une fois
    Serial.println("Envoi non confirme, tentative JOIN puis renvoi...");
    if (joinNetwork(30000)) {
      sendHexPayload(payload, 20000);
    }
  }

  delay(30000);
}