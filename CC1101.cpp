#include "CC1101.h"

bool Radio::begin() {
  reset();
  delayMicroseconds(50);
  yield();

  if(!getChipInfo() || 
      !getFreqBand(freq, freqTable) ||
      !(drate > drateTable[mod][0] && drate < drateTable[mod][1])) 
    return false;

  setMod(mod);
  setFreq(freq);
  setDrate(drate);
  setPwr(freqBand, pwr, pwrTable);

  setAddr(addr);
  setCRC(isCRC);
  setFEC(isFEC);
  setAutoCalib(isAutoCalib);
  setManchester(isManchester);
  setAppendStatus(isAppendStatus);
  setDataWhitening(isDataWhitening);
  setVariablePktLen(isVariablePktLen, pktLen);
  setSync(syncMode, syncWord, preambleLen);

  return true;
}

#if isTwoWay
bool Radio::readWrite(uint8_t *rxBuff, uint8_t *txBuff) {
  setIdleState();
  flushTxBuff();
  flushRxBuff();
  setRxState();

  while (true) {
    getRxBytes(pktLen);
    readRxFifo(rxBuff);
    writeTxFifo(txBuff);
  }

  return true;
};
bool Radio::writeRead(uint8_t *txBuff, uint8_t *rxBuff) {
  setIdleState();
  flushTxBuff();
  flushRxBuff();
  setRxState();

  while (true) {
    writeTxFifo(txBuff);
    getRxBytes(pktLen);
    readRxFifo(rxBuff);
  }

  return true;
};
#else
bool Radio::read(uint8_t *buff){
  setIdleState();
  flushRxBuff();
  setRxState();

  uint8_t rxBytes = getRxBytes(pktLen);

  Serial.print("bytesInRXFifo before: ");
  Serial.println(rxBytes);

  readRxFifo(buff);

  Serial.print("bytesInTXFifo after: ");
  Serial.println(readRegField(REG_RXBYTES, 6, 0));

  waitForIdleState();

  // flushRxBuff();
  // setRxState();

  return true;
};
bool Radio::write(uint8_t *buff){
  setIdleState();
  flushTxBuff();

  writeTxFifo(buff);

  setTxState();

  waitForIdleState();

  Serial.print("bytesInTXFifo after: ");
  Serial.println(readRegField(REG_TXBYTES, 6, 0));

  flushTxBuff();

  return true;
};
#endif

bool Radio::getChipInfo() {
  partnum = readStatusReg(REG_PARTNUM);
  version = readStatusReg(REG_VERSION);

  if(partnum == PARTNUM && version == VERSION) return true;
  return false;
};
void Radio::start() {
  spi.beginTransaction(spiSettings);
  digitalWrite(ss, LOW);

  #if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
    return;
  #endif

  while (digitalRead(miso));
}
void Radio::stop() {
  digitalWrite(ss, HIGH);
  spi.endTransaction();
}

void Radio::reset() {
  digitalWrite(ss, HIGH);
  delayMicroseconds(5);
  digitalWrite(ss, LOW);
  delayMicroseconds(5);
  digitalWrite(ss, HIGH);
  delayMicroseconds(40);

  start();
  spi.transfer(REG_RES);
  stop();
}
void Radio::flushRxBuff(){
  if(getState() != (STATE_IDLE || STATE_RXFIFO_OVERFLOW)) return;
  writeStatusReg(REG_FRX);
  delayMicroseconds(50);
  yield();
};
void Radio::flushTxBuff(){
  if(getState() != (STATE_IDLE || STATE_TXFIFO_UNDERFLOW)) return;
  writeStatusReg(REG_FTX);
  delayMicroseconds(50);
  yield();
};

void Radio::setCRC(bool en) {
  writeRegField(REG_PKTCTRL0, (byte)en, 2, 2);
};
void Radio::setFEC(bool en) {
  if(isVariablePktLen) return;

  writeRegField(REG_MDMCFG1, (byte)en, 7, 7);
};
void Radio::setAddr(byte addr) {
  writeRegField(REG_PKTCTRL1, addr > 0 ? 1 : 0, 1, 0);
  writeReg(REG_ADDR, addr);
};
void Radio::setSync(SyncMode syncMode, uint16_t syncWord, uint8_t preambleLen) {
  writeRegField(REG_MDMCFG2, syncMode, 2, 0);

  writeReg(REG_SYNC1, syncWord >> 8);
  writeReg(REG_SYNC0, syncWord & 0xff);

  writeRegField(REG_MDMCFG1, getPreambleIdx(preambleLen), 6, 4);
};
void Radio::setAutoCalib(bool en) {
  writeRegField(REG_MCSM0, (byte)en, 5, 4);
};
void Radio::setManchester(bool en) {
  if(mod == MOD_MSK || mod == MOD_4FSK) return;
  writeRegField(REG_MDMCFG2, (byte)en, 3, 3);
};
void Radio::setAppendStatus(bool en) {
  writeRegField(REG_PKTCTRL1, (byte)en, 2, 2);
};
void Radio::setDataWhitening(bool en) {
  writeRegField(REG_PKTCTRL0, (byte)en, 6, 6);
};
void Radio::setVariablePktLen(bool en, uint8_t pktlLen) {
  writeRegField(REG_PKTCTRL0, (byte)en, 1, 0);
  writeReg(REG_PKTLEN, pktLen);
};
void Radio::setMod(Modulation mod){
  writeRegField(REG_MDMCFG2, (uint8_t)mod, 6, 4);
};
void Radio::setFreq(double freq){
  uint32_t f = ((freq * 65536.0) / CRYSTAL_FREQ); 

  writeReg(REG_FREQ0, f & 0xff);
  writeReg(REG_FREQ1, (f >> 8) & 0xff);
  writeReg(REG_FREQ2, (f >> 16) & 0xff);

};
void Radio::setDrate(double drate){
  uint32_t xosc = CRYSTAL_FREQ * 1000;
  uint8_t e = log2((drate * (double)((uint32_t)1 << 20)) / xosc);
  uint32_t m = round(drate * ((double)((uint32_t)1 << (28 - e)) / xosc) - 256);

  if (m == 256) {
    m = 0;
    e++;
  }

  writeRegField(REG_MDMCFG4, e, 3, 0);
  writeRegField(REG_MDMCFG3, (uint8_t)m, 7, 0);
  // writeReg(REG_MDMCFG3, (uint8_t)m);
};
void Radio::setPwr(FreqBand freqBand, PowerMW pwr, const uint8_t pwrTable[][8]){
  // if(mod == MOD_ASK_OOK) {
  //   uint8_t paTable[2] = {WRITE, pwrTable[freqBand][pwr]};
  //   writeRegBurst(REG_PATABLE, paTable, 2);
  //   writeRegField(REG_FREND0, 1, 2, 0);
  // } else {
  //   writeReg(REG_PATABLE, pwrTable[freqBand][pwr]);
  //   writeRegField(REG_FREND0, 0, 2, 0);
  // }
  if(mod == MOD_ASK_OOK) {
    writeRegField(REG_FREND0, 1, 2, 0);
  } else {
    writeRegField(REG_FREND0, 0, 2, 0);
  }
  writeReg(REG_PATABLE, pwrTable[freqBand][pwr]);
};
void Radio::setRxState() {
  while(getState() != STATE_RX) {
    if (state == STATE_RXFIFO_OVERFLOW) flushRxBuff();
    else if (state != (STATE_CALIB || STATE_SETTLING)) writeStatusReg(REG_RX);
    delayMicroseconds(50);
    yield();
  }
};
void Radio::setTxState() {
  while(getState() != STATE_TX) {
    if(state == STATE_TXFIFO_UNDERFLOW) flushTxBuff();
    else if (state != (STATE_CALIB || STATE_SETTLING)) writeStatusReg(REG_TX);
    delayMicroseconds(50);
    yield();
  }
};
void Radio::setIdleState() {
  while(getState() != STATE_IDLE) {
    writeStatusReg(REG_IDLE);
    delayMicroseconds(50);
    yield();
  }
};
void Radio::setTwoWay(bool isTwoWay) {
  if(isTwoWay) {
    writeRegField(REG_MDMCFG1, 2, 3, 2);
    writeRegField(REG_MDMCFG1, 3, 1, 0);
  } else {
    writeRegField(REG_MDMCFG1, 0, 3, 2);
    writeRegField(REG_MDMCFG1, 0, 1, 0);
  }
};

byte Radio::getState() {
  writeStatusReg(REG_NOP);
  return state;
};
uint8_t Radio::getRxBytes(uint8_t len) {
  uint8_t bytes;
  do {
    bytes = readRegField(REG_RXBYTES, 6, 0);
    delayMicroseconds(50);
    yield();
  } while (bytes < len);
  return bytes;
};
uint8_t Radio::getTxBytes(uint8_t len) {
  uint8_t bytes;
  do {
    bytes = readRegField(REG_TXBYTES, 6, 0);
    delayMicroseconds(50);
    yield();
  } while (bytes < len);
  return bytes;
};
bool Radio::getFreqBand(double freq, const double freqTable[][2]) {
  for(int i = 0; i < 4; i++) {
    if(freq >= freqTable[i][0] && freq <= freqTable[i][1]) {
      freqBand = (FreqBand)i;
      return true;
    }
  }
  return false;
};
uint8_t Radio::getPreambleIdx(uint8_t len) {
  switch (len) {
    case 16:
      return 0;
    break;
    case 24:
      return 1;
    break;
    case 32:
      return 2;
    break;
    case 48:
      return 3;
    break;
    case 64:
      return 4;
    break;
    case 96:
      return 5;
    break;
    case 128:
      return 6;
    break;
    case 192:
      return 7;
    break;
    default:
      return 0;;
  }
};

void Radio::waitForIdleState() {
  while (getState() != STATE_IDLE){
    delayMicroseconds(50);
    yield();
  };
};

void Radio::readRxFifo(uint8_t *buff) {
  if(isVariablePktLen) {
    pktLen = readReg(REG_FIFO);
  }
  readRegBurst(REG_FIFO, buff, pktLen);
  if(isAppendStatus) {
    uint8_t r = readReg(REG_FIFO);
    if(r >= 128) rssi = ((rssi - 256) / 2) - RSSI_OFFSET;
    else rssi = (rssi / 2) - RSSI_OFFSET;
    lqi = readReg(REG_FIFO) & 0x7f;
    // if(!(r >> 7) & 1) return false; // CRC Mismatch
  }
};
void Radio::writeTxFifo(uint8_t *buff) {
  if(isVariablePktLen) {
    pktLen = sizeof(buff);
    writeReg(REG_FIFO, pktLen);
  }
  // if(addr > 0) {
  //   writeReg(REG_FIFO, addr);
  // }
  writeRegBurst(REG_FIFO, buff, pktLen);
};

byte Radio::readReg(byte addr) {
  start();
  spi.transfer(0x80 | (addr & 0b111111));
  // spi.transfer(addr | READ);
  uint8_t data = spi.transfer(WRITE);
  stop();

  return data;
};
byte Radio::readStatusReg(byte addr){
  start();
  // byte header = 0x80 | (addr & 0b111111);
  // header |= 0x40;
  // spi.transfer(header);
  spi.transfer(addr | READ_BURST);
  uint8_t data = spi.transfer(WRITE);
  stop();

  return data;
};
byte Radio::readRegField(byte addr, byte hi, byte lo){
  return readStatusReg((addr) >> lo) & ((1 << (hi - lo + 1)) -1);
};
void Radio::readRegBurst(byte addr, uint8_t *buff, size_t size){
  start();
  spi.transfer(0x80 | 0x40 | (addr & 0b111111));
  // spi.transfer(addr | READ_BURST);
  for (uint8_t i = 0; i < size; i++) {
    buff[i] = spi.transfer(WRITE);
  }
  stop();
};

void Radio::writeReg(byte addr, byte val){
  start();
    spi.transfer(0x00 | (addr & 0b111111));
    // spi.transfer(addr);
    spi.transfer(val);
  stop();
};
void Radio::writeStatusReg(byte addr){
  start();
  // state =(spi.transfer(0x00 | (addr & 0b111111)) >> 4) & 0b00111;
  state =(spi.transfer(addr) >> 4) & 0b00111;
  // spi.transfer(0x00 | (addr & 0b111111)
  // spi.transfer(addr);
  stop();
};
void Radio::writeRegField(byte addr, byte val, byte hi, byte lo){
  uint8_t mask = ((1 << (hi - lo + 1)) -1) << lo;
  writeReg(addr, (readReg(addr) & ~mask) | ((val <<= lo) & mask));
};
void Radio::writeRegBurst(byte addr, uint8_t *buff, size_t size){
  start();
    spi.transfer(0x00 | 0x40 | (addr & 0b111111));
    // spi.transfer(addr | WRITE_BURST);
    for (uint8_t i = 0; i < size; i++) {
       spi.transfer(buff[i]);
    }
  stop();
};
