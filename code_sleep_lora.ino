#include <Arduino.h>

#define RX2_PIN 16
#define TX2_PIN 17

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

// Mettre le LoRa-E5 en veille
bool loraEnterLowPower(uint32_t timeoutMs = 2000) {
  while (Serial2.available()) Serial2.read(); // purge
  Serial2.println("AT+LOWPOWER");

  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    String line = readLine(300);
    if (!line.length()) continue;
    Serial.println(line);

    String up = line; up.toUpperCase();
    // selon firmwares: "+LOWPOWER: SLEEP", "OK", etc.
    if (up.indexOf("LOWPOWER") >= 0 || up == "OK") return true;
    if (up.indexOf("ERROR") >= 0) return false;
  }
  return false; // pas de confirmation, mais commande peut quand même avoir été prise
}

// Réveiller le LoRa-E5 (souvent il suffit d'envoyer "AT")
bool loraWake(uint32_t timeoutMs = 2000) {
  while (Serial2.available()) Serial2.read(); // purge
  Serial2.println("AT");

  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    String line = readLine(300);
    if (!line.length()) continue;
    Serial.println(line);

    String up = line; up.toUpperCase();
    if (up == "OK") return true;
    if (up.indexOf("LOWPOWER: WAKEUP") >= 0) {
      // souvent suivi de "OK" ensuite, on continue un peu
    }
    if (up.indexOf("ERROR") >= 0) return false;
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial2.begin(9600, SERIAL_8N1, RX2_PIN, TX2_PIN);

  Serial.println("Test LowPower LoRa-E5");

  Serial.println("Wake test...");
  loraWake();

  Serial.println("Enter low power...");
  loraEnterLowPower();

  Serial.println("Attente 5s (module en veille)...");
  delay(5000);

  Serial.println("Wake again...");
  loraWake();
}

void loop() {
}