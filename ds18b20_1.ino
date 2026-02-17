#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 33

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

DeviceAddress adresseCapteur;

void afficherAdresse(DeviceAddress addr) {
  for (uint8_t i = 0; i < 8; i++) {
    if (addr[i] < 16) Serial.print("0");
    Serial.print(addr[i], HEX);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n===== DEMARRAGE =====");

  sensors.begin();

  if (sensors.getDeviceCount() == 0) {
    Serial.println("Aucun capteur detecte !");
    return;
  }

  if (!sensors.getAddress(adresseCapteur, 0)) {
    Serial.println("Impossible de lire l'adresse !");
    return;
  }

  Serial.print("Numero du capteur : ");
  afficherAdresse(adresseCapteur);
  Serial.println();
}

void loop() {
  sensors.requestTemperatures();

  float temperature = sensors.getTempC(adresseCapteur);

  Serial.print("Numero : ");
  afficherAdresse(adresseCapteur);

  Serial.print("  |  Temperature : ");
  Serial.print(temperature);
  Serial.println(" °C");

  delay(2000);
}
