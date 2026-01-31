#include "CC1101_RAW.h"

bool Radio::begin() {
  hardReset();

  partnum = readStatusReg(CC1101_REG_PARTNUM);
  version = readStatusReg(CC1101_REG_VERSION);

  if(partnum != CC1101_PARTNUM || version != CC1101_VERSION || version != CC1101_VERSION_LEGACY) {
    return false;
  }

  if (
      !((freq >= 300.0 && freq <= 348.0) ||
        (freq >= 387.0 && freq <= 464.0) ||
        (freq >= 779.0 && freq <= 928.0)) ||
      (drate < drateRange[mod][0] || drate > drateRange[mod][1])
      ) return false;

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
void Radio::flushRxBuffer(){
  writeStatusReg(CC1101_CMD_FRX);
};
void Radio::flushTxBuffer(){
  writeStatusReg(CC1101_CMD_FTX);
};

void Radio::setRegs(){
  /* Automatically calibrate when going from IDLE to RX or TX. */
  writeRegField(CC1101_REG_MCSM0, 1, 5, 4);

  /* Enable append status */
  writeRegField(CC1101_REG_PKTCTRL1, 1, 2, 2);

  /* Disable data whitening. */
  // setDataWhitening(false);
};
bool Radio::setMod(Modulation mod){
  writeRegField(CC1101_REG_MDMCFG2, (uint8_t)mod, 6, 4);
  // setPower(power);
};
bool Radio::setFreq(double freq){
  uint32_t f = ((freq * 65536.0) / CC1101_CRYSTAL_FREQ) 

    writeReg(CC1101_REG_FREQ, f & 0xff);
    writeReg(CC1101_REG_FREQ1, (f >> 8) & 0xff);
    writeReg(CC1101_REG_FREQ2, (f >> 16) & 0xff);

    // setPower(power);
};
void Radio::setDrate(double drate){
  uint32_t xosc = CC1101_CRYSTAL_FREQ * 1000;
  uint8_t e = log2((drate * (double)((uint32_t)1 << 20)) / xosc);
  uint32_t m = round(drate * ((double)((uint32_t)1 << (28 - e)) / xosc) - 256.);

  if (m == 256) {
    m = 0;
    e++;
  }

  writeRegField(CC1101_REG_MDMCFG4, e, 3, 0);
  writeReg(CC1101_REG_MDMCFG3, (uint8_t)m);
};
void Radio::setPower(int8_t drate){
  uint8_t powerIdx, freqIdx;

  switch(freq) {
    case <= 348.0:
      freqIdx = 0;
    case <= 464.0:
      freqIdx = 1;
    case <= 891.5:
      freqIdx = 2;
    default:
      freqIdx = 3;
  }

  switch(power) {
    case <= -30:
      powerIdx = 0;
    case <= -20:
      powerIdx = 1;
    case <= -15:
      powerIdx = 2;
    case <= -10:
      powerIdx = 3;
    case <= 0:
      powerIdx = 4;
    case <= 5:
      powerIdx = 5;
    case <= 7:
      powerIdx = 6;
    default:
      powerIdx = 7;
  }

  if(mod == MOD_ASK_OOK) {
    writeRegBurst(CC1101_REG_PATABLE, {CC1101_WRITE, powerRange[freqIdx][powerIdx]}, 2);
    writeRegField(CC1101_REG_FREND0, 1, 2, 0);
  } else {
    writeReg(CC1101_REG_PATABLE, powerRange[freqIdx][powerIdx]);
    writeRegField(CC1101_REG_FREND0, 0, 2, 0);
  }
};

uint8_t Radio::readReg(uint8_t addr){
  uint8_t header = CC1101_READ | (addr & 0b111111);

  start();
  spi.transfer(header);
  uint8_t data = spi.transfer(CC1101_WRITE);
  stop();

  return data;
};
uint8_t Radio::readStatusReg(uint8_t addr){
  uint8_t header = CC1101_READ | CC1101_BURST | (addr & 0b111111);
  // uint8_t header = CC1101_READ | (addr & 0b111111);
  // header |= CC1101_BURST;

  start();
  spi.transfer(header);
  uint8_t data = spi.transfer(CC1101_WRITE);
  stop();

  return data;
};
uint8_t Radio::readRegField(uint8_t addr, uint8_t hi, uint8_t lo){
  return readStatusReg((addr) >> lo) & ((1 << (hi - lo + 1)) -1);
};
uint8_t Radio::readRegBurst(uint8_t addr, uint8_t *buff, uint8_t size){
  uint8_t header = CC1101_READ | CC1101_BURST | (addr & 0b111111);

  start();
  spi.transfer(header);
  for (uint8_t i = 0; i < size; i++) {
    buff[i] = spi.transfer(CC1101_WRITE);
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
void Radio::writeStatusReg(uint8_t addr){
  uint8_t header = CC1101_WRITE | (addr & 0b111111);

  start();
    spi.transfer(header);
  stop();
};
void Radio::writeRegField(uint8_t addr, uint8_t data, uint8_t hi, uint8_t lo){
  uint8_t mask = ((1 << (hi - lo +1)) -1) << lo;
  writeStatusReg(addr, (readReg & ~mask) | ((buff <<= lo) & mask));
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
