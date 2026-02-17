#include "DHT.h"

#define DHTPIN 27
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

#define RX2_PIN 16
#define TX2_PIN 17

#define LED_PIN 23

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RX2_PIN, TX2_PIN);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  dht.begin();
  
  delay(2000); 

  Serial2.println("AT+JOIN");
  delay(5000); 
}

void loop() {
  float humidite = dht.readHumidity();
  float temperature = dht.readTemperature();

  if (isnan(humidite) || isnan(temperature)) {
    Serial.println("Erreur : Echec de lecture du capteur DHT22.");
    delay(5000);
    return;
  }

  Serial.print("Temperature : "); 
  Serial.print(temperature);
  Serial.print(" C | Humidite : "); 
  Serial.print(humidite); 
  Serial.println(" %");

  int tempInt = temperature * 10; 
  int humInt = humidite * 10;

  char payload[20];
  sprintf(payload, "%04X%04X", tempInt, humInt); 
  
  Serial.print("Envoi LoRa payload : ");
  Serial.println(payload);

  digitalWrite(LED_PIN, HIGH);

  Serial2.print("AT+MSGHEX=\"");
  Serial2.print(payload);
  Serial2.println("\"");

  long startTime = millis();
  while(millis() - startTime < 10000) { 
    if (Serial2.available()) {
      Serial.write(Serial2.read());
    }
  }

  digitalWrite(LED_PIN, LOW);

  delay(30000); 
}