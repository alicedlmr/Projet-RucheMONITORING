#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 33   // GPIO utilisé pour le DS18B20

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup() {
  Serial.begin(115200);
  delay(500);

  sensors.begin();

  int count = sensors.getDeviceCount();
  Serial.print("DS18B20 detectes: ");
  Serial.println(count);

  if (count == 0) {
    Serial.println("Erreur: aucun capteur detecte.");
  } else {
    // Option: résolution (9 à 12 bits). 12 bits = plus précis mais plus lent (~750 ms)
    sensors.setResolution(12);
    Serial.println("Capteur pret.");
  }
}

void loop() {
  sensors.requestTemperatures();                 // lance la conversion
  float t = sensors.getTempCByIndex(0);          // lit le 1er capteur

  if (t == DEVICE_DISCONNECTED_C || t == 85.0f) {
    Serial.println("Erreur lecture DS18B20.");
  } else {
    Serial.print("Temperature: ");
    Serial.print(t, 2);
    Serial.println(" C");
  }

  delay(1000);
}
