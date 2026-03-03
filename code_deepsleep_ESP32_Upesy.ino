#include <Arduino.h>
#include "esp_sleep.h"

const uint32_t SLEEP_SEC = 30;

void setup() {
  Serial.begin(115200);
  delay(200);

  // optionnel : afficher la cause de réveil
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  if (cause == ESP_SLEEP_WAKEUP_TIMER) {
    Serial.println("Wake-up: timer");
  } else {
    Serial.println("Boot normal");
  }
}

void loop() {
  // 1) travail à chaque cycle
  Serial.println("Cycle: mesures / envoi ...");
  delay(1000); // remplace par tes mesures/envoi LoRa

  // 2) Programmer le réveil
  esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_SEC * 1000000ULL);

  // 3) Aller en deep sleep (le code s'arrête ici)
  Serial.println("Deep sleep...");
  Serial.flush();
  esp_deep_sleep_start();

  // Jamais exécuté
  // delay(1000);
}