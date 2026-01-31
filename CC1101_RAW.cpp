#include "CC1101_RAW.h"

bool Radio::begin(Modulation mod, double freq, double drate, int8_t power) {
  hardReset();

  partnum = readStatusReg(CC1101_REG_PARTNUM);
  version = readStatusReg(CC1101_REG_VERSION);

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

bool Radio::write(uint8_t *buffer, uint8_t size){};
bool Radio::read(uint8_t *buffer, uint8_t size){};

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

uint8_t Radio::readReg(uint8_t addr){
  uint8_t header = CC1101_READ | (addr & 0b111111);

  start();
  spi.transfer(header);
  uint8_t data = spi.transfer(0x00);
  stop();

  return data;
};
uint8_t Radio::readStatusReg(uint8_t addr){
  // uint8_t header = CC1101_READ | CC1101_BURST | (addr & 0b111111);
  uint8_t header = CC1101_READ | (addr & 0b111111);
  header |= CC1101_BURST;

  start();
  spi.transfer(header);
  uint8_t data = spi.transfer(0x00);
  stop();

  return data;
};
uint8_t Radio::readRegField(uint8_t addr, uint8_t hi, uint8_t lo){
  return readReg((addr) >> lo) & ((1 << (hi - lo + 1)) -1);
};
uint8_t Radio::readRegBurst(uint8_t addr, uint8_t *buff, uint8_t size){
  uint8_t header = CC1101_READ | CC1101_BURST | (addr & 0b111111);

  start();
  spi.transfer(header);
  for (uint8_t i = 0; i < size; i++) {
    buff[i] = spi.transfer(0x00);
  }
  stop();
};

void Radio::writeReg(uint8_t addr, uint8_t buff){
  uint8_t header = CC1101_WRITE | (addr & 0b111111);

  start();
    spi.transfer(header);
    spi.transfer(buff);
  stop();
};
void Radio::writeStatusReg(uint8_t addr, uint8_t buff){
  uint8_t header = CC1101_WRITE | (addr & 0b111111);

  start();
    spi.transfer(header);
  stop();
};
void Radio::writeRegField(uint8_t addr, uint8_t data, uint8_t hi, uint8_t lo){
  // buff <<= lo;
  uint8_t mask = ((1 << (hi - lo +1)) -1) << lo;
  writeReg(addr, (readReg & ~mask) | ((buff <<= lo) & mask));
};
void Radio::writeRegBurst(uint8_t addr, uint8_t *buff, uint8_t size){
  uint8_t header = CC1101_WRITE | CC1101_BURST | (addr & 0b111111);

  start();
    spi.transfer(header);
    for (uint8_t i = 0; i < size; i++) {
       spi.transfer(buff[i]);
    }
  end();
};
