#include <SPI.h>
#include <FastRadio.h>

#define FSPI_MOSI 10
#define FSPI_MISO 20
#define FSPI_SCK 12
#define TX_SS 14

#define HSPI_MOSI 11
#define HSPI_MISO 21
#define HSPI_SCK 13
#define RX_SS 9

TaskHandle_t txTaskHandle;
TaskHandle_t rxTaskHandle;

SPIClass fspi(FSPI);
SPIClass hspi(HSPI);

CC1101 txRadio(FSPI_SCK, FSPI_MISO, FSPI_MOSI, TX_SS, fspi);
CC1101 rxRadio(HSPI_SCK, HSPI_MISO, HSPI_MOSI, RX_SS, hspi);

void setup() {
  Serial.begin(115200);

  while (!Serial);

  Serial.println("Namaste CC1101!");

  xTaskCreatePinnedToCore(
    txTask,
    "TX Task",
    10000,
    NULL,
    1,
    &txTaskHandle,
    0);

  xTaskCreatePinnedToCore(
    rxTask,
    "RX Task",
    10000,
    NULL,
    1,
    &rxTaskHandle,
    1);
}

void txTask(void *pvParameters) {
  Serial.print("TX Task running on core ");
  Serial.println(xPortGetCoreID());

  pinMode(FSPI_SCK, OUTPUT);
  pinMode(FSPI_MISO, INPUT);
  pinMode(FSPI_MOSI, OUTPUT);
  pinMode(TX_SS, OUTPUT);

  fspi.begin(FSPI_SCK, FSPI_MISO, FSPI_MOSI, TX_SS);

  while (!txRadio.init()) {
    Serial.println(F("TX Radio not found!"));
    delay(2000);
  }
  Serial.println(F("TX Radio initialized!"));
  Serial.print("Chip Part Number: ");
  Serial.println(txRadio.partnum);
  Serial.print("Chip Version: ");
  Serial.println(txRadio.version);

  byte txBuff[4] = { 200, 201, 202, 203 };
  byte rxBuff[4];

  while (true) {
    if (txRadio.link(txBuff, rxBuff, 1000)) {
      Serial.println("Send and Received Packet. [TX]");
    } else {
      Serial.println("Receiving Timeout. [TX]");
    }
  }
}

void rxTask(void *pvParameters) {
  Serial.print("RX Task running on core ");
  Serial.println(xPortGetCoreID());

  pinMode(HSPI_SCK, OUTPUT);
  pinMode(HSPI_MISO, INPUT);
  pinMode(HSPI_MOSI, OUTPUT);
  pinMode(RX_SS, OUTPUT);

  hspi.begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI, RX_SS);

  while (!rxRadio.init()) {
    Serial.println(F("RX Radio not found!"));
    delay(2000);
  }
  Serial.println(F("RX Radio initialized!"));
  Serial.print("Chip Part Number: ");
  Serial.println(rxRadio.partnum);
  Serial.print("Chip Version: ");
  Serial.println(rxRadio.version);

  byte txBuff[4] = { 204, 205, 206, 207 };
  byte rxBuff[4];

  while (true) {
    if (rxRadio.link(txBuff, rxBuff, 2000)) {
      Serial.println("Send and Received Packet. [RX]");
    } else {
      Serial.println("Receiving Timeout. [RX]");
    }
  }
}

void loop() {
}
