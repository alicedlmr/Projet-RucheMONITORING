#include <Arduino.h>

#define RX2_PIN 16
#define TX2_PIN 17
#define LED_PIN 23

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
  while (Serial2.available()) Serial2.read(); // purge

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

  if (joinNeeded) {
    Serial.println("Erreur: pas join au reseau.");
    return false;
  }
  return done;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial2.begin(9600, SERIAL_8N1, RX2_PIN, TX2_PIN);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Join (une fois)
  bool ok = joinNetwork(30000);
  if (!ok) {
    Serial.println("JOIN ECHEC.");
  } else {
    Serial.println("JOIN OK.");
  }
}

void loop() {
  // Exemple payload HEX (10 octets ici par ex)
  const char* payload = "01020304AABBCCDD1122";

  bool ok = sendHexPayload(payload, 20000);
  if (ok) Serial.println("Envoi OK.");
  else    Serial.println("Envoi non confirme.");

  delay(30000);
}