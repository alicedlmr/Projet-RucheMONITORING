#include <Arduino.h>
#include "HX711.h"

#define HX711_DOUT 25
#define HX711_SCK  26

HX711 scale;

long readRawAvg(int n) {
  long sum = 0;
  for (int i = 0; i < n; i++) {
    while (!scale.is_ready()) delay(5);
    sum += scale.read();
  }
  return sum / n;
}

void waitEnter(const char* msg) {
  Serial.println(msg);
  Serial.println("Appuie sur ENTER dans le moniteur serie...");
  while (true) {
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      break;
    }
    delay(50);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  scale.begin(HX711_DOUT, HX711_SCK);

  Serial.println("HX711 Calibration atelier");
  Serial.println("Etape 1: TARE (plateforme vide)");
  Serial.println("Etape 2: Pose une masse connue");
  Serial.println("Etape 3: Calcul du facteur");
}

void loop() {
  // 1) Tare
  waitEnter("1) Mettre la plateforme a vide, stable, puis ENTER.");
  long offset = readRawAvg(20);
  Serial.print("OFFSET (tare) = ");
  Serial.println(offset);

  // 2) Masse connue
  waitEnter("2) Poser une masse connue, stable, puis ENTER.");
  long rawWithMass = readRawAvg(20);
  Serial.print("RAW avec masse = ");
  Serial.println(rawWithMass);

  // 3) Saisie masse
  Serial.println("3) Entrer la masse en kg (ex: 2.50) puis ENTER:");
  while (!Serial.available()) delay(50);
  float massKg = Serial.parseFloat();
  while (Serial.available()) Serial.read();

  if (massKg <= 0.0f) {
    Serial.println("Masse invalide. Recommence.");
    delay(1000);
    return;
  }

  long delta = rawWithMass - offset;
  float factor = (float)delta / massKg; // unités: counts/kg

  Serial.println("----- RESULTATS -----");
  Serial.print("OFFSET = "); Serial.println(offset);
  Serial.print("FACTOR (counts/kg) = "); Serial.println(factor, 6);

  Serial.println("Dans ton code, le poids (kg) = (raw - OFFSET) / FACTOR");
  Serial.println("Si le poids diminue quand tu ajoutes une masse, inverse le signe (SCALE_SIGN = -1).");
  Serial.println("---------------------");

  Serial.println("Calibration terminee. Redemarre pour refaire.");
  while (true) delay(1000);
}