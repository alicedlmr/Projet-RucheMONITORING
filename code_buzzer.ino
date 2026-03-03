#include <Arduino.h>

#define BUZZER_PIN 23   // remplace LED_PIN par BUZZER_PIN

// Joue une note (fréquence en Hz) pendant durationMs, puis pause pauseMs
void playTone(int freq, int durationMs, int pauseMs) {
  if (freq <= 0) { // silence
    ledcWriteTone(0, 0);
    delay(durationMs);
    delay(pauseMs);
    return;
  }
  ledcWriteTone(0, freq);
  delay(durationMs);
  ledcWriteTone(0, 0);
  delay(pauseMs);
}

// Petite mélodie (type "startup")
void playMelody() {
  playTone(523, 150, 30);  // C5
  playTone(659, 150, 30);  // E5
  playTone(784, 150, 30);  // G5
  playTone(1046, 200, 80); // C6
  playTone(0, 100, 0);     // silence
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // Setup PWM audio (canal 0)
  ledcAttachPin(BUZZER_PIN, 0);
  ledcWriteTone(0, 0);

  playMelody();
}

void loop() {
  // Exemple: rejouer la mélodie toutes les 10 secondes
  delay(10000);
  playMelody();
}