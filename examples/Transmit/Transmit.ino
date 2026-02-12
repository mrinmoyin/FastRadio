#include <SPI.h>
#include <FastRadio.h>

#define SCK 14
#define MISO 12
#define MOSI 13
#define SS 15

#define LED_PIN 2

Radio radio;

byte txBuff[4] = { 200, 201, 202, 203 };

void setup() {
  Serial.begin(115200);

  while (!Serial);

  Serial.println("Namaste CC1101!");

  pinMode(SCK, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(SS, OUTPUT);

  pinMode(LED_PIN, OUTPUT);

  digitalWrite(SS, HIGH);

  SPI.begin();

  while (!radio.init()) {
    Serial.println("Radio not found!");
    delay(2000);
  }

  Serial.println("Radio initialized!");
}

void loop() {
  digitalWrite(LED_PIN, LOW);
  if (radio.write(txBuff)) {
    Serial.print("Sent: [");
    for (int i = 0; i < sizeof(txBuff); i++) {
      if (i != 0) Serial.print(", ");
      Serial.print(txBuff[i]);
    }
    Serial.print("] Length: ");
    Serial.println(sizeof(txBuff));
  } else {
    Serial.println("Error sending packet");
  }
  digitalWrite(LED_PIN, HIGH);

  delay(2000);
}
