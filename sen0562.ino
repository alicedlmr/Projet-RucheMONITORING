#include <Wire.h>
#include <BH1750.h>

BH1750 capteur;

void setup() {
  Serial.begin(115200);

  // Initialisation I2C (SDA = 21, SCL = 22)
  Wire.begin(21, 22);

  if (capteur.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println("Capteur BH1750 initialise !");
  } else {
    Serial.println("Erreur initialisation capteur !");
    while (1);
  }
}

void loop() {
  float luminosite = capteur.readLightLevel();

  Serial.print("Luminosite : ");
  Serial.print(luminosite);
  Serial.println(" lux");

  delay(1000);
}
