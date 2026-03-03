#include <Arduino.h>
#include "HX711.h"

// Pins HX711
#define HX711_DOUT 25
#define HX711_SCK  26

// Tes valeurs de calibration
const long  SCALE_OFFSET = 145192;
const float SCALE_FACTOR = 31616.384766;
const int   SCALE_SIGN   = 1;   // mettre -1 si le poids diminue quand tu ajoutes du poids

HX711 scale;

long readScaleRawAvg(int n) {
  long sum = 0;
  for (int i = 0; i < n; i++) {
    while (!scale.is_ready()) delay(5);
    sum += scale.read();
  }
  return sum / n;
}

float getWeightKg() {
  long raw = readScaleRawAvg(10); // moyenne sur 10 mesures
  float weight = (float)(SCALE_SIGN * (raw - SCALE_OFFSET)) / SCALE_FACTOR;
  return weight;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  scale.begin(HX711_DOUT, HX711_SCK);

  Serial.println("HX711 OK");
  Serial.print("Offset = "); Serial.println(SCALE_OFFSET);
  Serial.print("Facteur = "); Serial.println(SCALE_FACTOR);
}

void loop() {
  float w = getWeightKg();

  Serial.print("Poids: ");
  Serial.print(w, 3);
  Serial.println(" kg");

  delay(1000);
}
