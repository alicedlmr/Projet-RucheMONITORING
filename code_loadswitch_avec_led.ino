#include <Arduino.h>

#define LOADSW_PIN 2
#define LED_PIN 23

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(LOADSW_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(LOADSW_PIN, LOW); // capteurs OFF au départ
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  Serial.println("Capteurs ON");
  digitalWrite(LOADSW_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);
  delay(3000);

  Serial.println("Capteurs OFF");
  digitalWrite(LOADSW_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  delay(3000);
}