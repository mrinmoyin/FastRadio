#include "CC1101_RAW.h"

bool Radio::begin(Modulation mod, double freq, double drate, int8_t power) {
  hardReset();

  partnum = getChipPartNumber();
  version = getChipVersion();

  if(partnum != CC1101_PARTNUM || version != CC1101_VERSION || version != CC1101_VERSION_LEGACY) {
    return false;
  }

  setRegs();
  setMod(mod);
  setFreq(freq);
  setDrate(drate);
  setPower(power);

    return true;
}

bool Radio::transmit(uint8_t *buffer, uint8_t lenght){};
bool Radio::receive(uint8_t *buffer, uint8_t lenght){};

void Radio::start() {
  spi.beginTransaction(spiSettings);
  digitalWrite(ss, LOW);

  #if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
  return;
  #endif

  while (digitalRead(miso))
    ;
}
void Radio::stop() {
  digitalWrite(ss, HIGH);
  spi.endTransaction(spiSettings);
}

void Radio::hardReset() {
  start();
  spi.transfer(CC1101_CMD_RES);
  stop();
}
void Radio::flushRxBuffer(){};
void Radio::flushTxBuffer(){};

void Radio::setRegs(){};
void Radio::setMod(Modulation mod){};
void Radio::setFreq(double freq){};
void Radio::setDrate(double drate){};
void Radio::setPower(int8_t drate){};

uint8_t Radio::readReg(uint8_t addr){};
uint8_t Radio::readRegField(uint8_t addr, uint8_t hi, uint8_t lo){};
uint8_t Radio::readRegBurst(uint8_t addr, uint8_t *buffer, uint8_t lenght){};

void Radio::writeRegField(uint8_t addr, uint8_t data, uint8_t hi, uint8_t lo){};
void Radio::writeReg(uint8_t addr, uint8_t buff){};
void Radio::writeRegBurst(uint8_t addr, uint8_t *buff, uint8_t length){};


