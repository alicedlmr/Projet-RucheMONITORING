#include <Arduino.h>
#include <Wire.h>
#include <BH1750.h>

#define I2C_SDA 21
#define I2C_SCL 22

BH1750 lightMeter;

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(I2C_SDA, I2C_SCL);

  // Mode continu haute résolution
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println("BH1750 OK");
  } else {
    Serial.println("Erreur: BH1750 non detecte (verifier I2C / adresse).");
  }
}

void loop() {
  float lux = lightMeter.readLightLevel();

  Serial.print("Luminosite: ");
  Serial.print(lux, 2);
  Serial.println(" lux");

  delay(1000);
}
