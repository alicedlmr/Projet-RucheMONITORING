#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 33

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void printAddress(const DeviceAddress addr) {
  for (uint8_t i = 0; i < 8; i++) {
    if (addr[i] < 16) Serial.print("0");
    Serial.print(addr[i], HEX);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  sensors.begin();

  int count = sensors.getDeviceCount();
  Serial.print("DS18B20 detectes: ");
  Serial.println(count);

  if (count == 0) {
    Serial.println("Erreur: aucun capteur detecte.");
    return;
  }

  // 12 bits = plus précis mais plus lent (~750 ms)
  sensors.setResolution(12);

  // Affiche les adresses des capteurs trouvés
  for (int i = 0; i < count; i++) {
    DeviceAddress addr;
    if (sensors.getAddress(addr, i)) {
      Serial.print("Capteur ");
      Serial.print(i);
      Serial.print(" adresse: ");
      printAddress(addr);
      Serial.println();
    } else {
      Serial.print("Erreur lecture adresse capteur ");
      Serial.println(i);
    }
  }
}

void loop() {
  // Lance la conversion pour tous les capteurs du bus
  sensors.requestTemperatures();

  int count = sensors.getDeviceCount();

  for (int i = 0; i < count; i++) {
    float t = sensors.getTempCByIndex(i);

    Serial.print("Capteur ");
    Serial.print(i);
    Serial.print(" : ");

    if (t == DEVICE_DISCONNECTED_C || t == 85.0f) {
      Serial.println("Erreur lecture");
    } else {
      Serial.print(t, 2);
      Serial.println(" C");
    }
  }

  Serial.println();
  delay(1000);
}
