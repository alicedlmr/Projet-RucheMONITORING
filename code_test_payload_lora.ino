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
  Serial2.println("AT+JOIN");
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    String line = readLine(600);
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

bool sendMsgHexWaitDone(const char* payloadHex, uint32_t timeoutMs) {
  while (Serial2.available()) Serial2.read();

  digitalWrite(LED_PIN, HIGH);

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

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial2.begin(9600, SERIAL_8N1, RX2_PIN, TX2_PIN);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println("Test unitaire trame LoRa (payload fixe projet)");

  if (!joinNetwork(30000)) {
    Serial.println("JOIN ECHEC.");
  } else {
    Serial.println("JOIN OK.");
  }
}

void loop() {
  const char* payload = "00D700DF00B9020C00C001DE04D204D24D0F23";

  Serial.print("Payload test : ");
  Serial.println(payload);

  bool ok = sendMsgHexWaitDone(payload, 20000);
  Serial.println(ok ? "Envoi OK" : "Envoi non confirme");

  delay(30000);
}