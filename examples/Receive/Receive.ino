#include <SPI.h>
#include <FastRadio.h>

#define LED_PIN 5

#define MISO 21
#define SCK 13
#define MOSI 11
#define SS 9

SPIClass spi(FSPI);
Radio radio(SS, SCK, MISO, MOSI, spi);
byte rxBuff[4];

void setup() {
  Serial.begin(115200);

  while (!Serial);

  Serial.println("Namaste CC1101!");

  pinMode(SCK, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(SS, OUTPUT);

  digitalWrite(SS, HIGH);

  spi.begin(SCK, MISO, MOSI, SS);

  while (!radio.init()) {
    Serial.println(F("Radio not found!"));
    delay(2000);
  }
  Serial.println(F("Radio initialized!"));
  Serial.println(radio.partnum);
  Serial.println(radio.version);
}

void loop() {
  if (radio.read(rxBuff)) {
    Serial.print("Recieved: [");
    for (int i = 0; i < sizeof(rxBuff); i++) {
      if (i != 0) Serial.print(", ");
      Serial.print(rxBuff[i]);
    }
    Serial.print("] Length: ");
    Serial.print(sizeof(rxBuff));
    Serial.print(" RSSI: ");
    Serial.print(radio.rssi);
    Serial.print(" LQI: ");
    Serial.println(radio.lqi);
  } else {
    Serial.println("Error recieving rxBuff");
  }
  Serial.println();
}
