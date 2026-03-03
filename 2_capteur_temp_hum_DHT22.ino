#include <Arduino.h>
#include "DHT.h"

#define DHTTYPE DHT22

#define DHT1_PIN 27
#define DHT2_PIN 14

DHT dht1(DHT1_PIN, DHTTYPE);
DHT dht2(DHT2_PIN, DHTTYPE);

void printDHT(const char* name, float t, float h) {
  Serial.print(name);
  Serial.print(" - ");

  if (isnan(t) || isnan(h)) {
    Serial.println("Erreur lecture");
    return;
  }

  Serial.print("Temp: ");
  Serial.print(t, 1);
  Serial.print(" C | Hum: ");
  Serial.print(h, 1);
  Serial.println(" %");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  dht1.begin();
  dht2.begin();

  Serial.println("Test 2x DHT22");
}

void loop() {
  float t1 = dht1.readTemperature();
  float h1 = dht1.readHumidity();

  float t2 = dht2.readTemperature();
  float h2 = dht2.readHumidity();

  printDHT("DHT22 #1", t1, h1);
  printDHT("DHT22 #2", t2, h2);

  Serial.println();
  delay(2000); // DHT22: pas plus rapide que ~2s
}
