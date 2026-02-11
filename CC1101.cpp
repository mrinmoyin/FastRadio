#include "CC1101.h"
#include "RadioUtils.h"

bool Radio::begin() {
  reset();
  delayMicroseconds(500);

  partnum = readStatusReg(REG_PARTNUM);
  version = readStatusReg(REG_VERSION);

  freqIdx = getFreqIdx(freq, freqTable);
  pwrIdx = getPwrIdx(pwr);

  Serial.println(freq);
  Serial.println(freqIdx);
  Serial.println(pwrIdx);
  if(!(partnum == PARTNUM && version == VERSION) ||
      !(drate > drateTable[mod][0] && drate < drateTable[mod][1]) || 
      freqIdx == -1) return false;

  setAddr(addr);
  setCRC(isCRC);
  setFEC(isFEC);
  setAutoCalib(isAutoCalib);
  setManchester(isManchester);
  setAppendStatus(isAppendStatus);
  setDataWhitening(isDataWhitening);
  setVariablePktLen(isVariablePktLen);
  setSync(syncMode, syncWord, preambleLen);

  setMod(mod);
  setFreq(freq);
  setDrate(drate);
  setPwr(freqIdx, pwrIdx, pwrTable);

  return true;
}

 bool Radio::read(uint8_t *buff){
  // uint8_t bytesInFifo = readStatusReg(REG_RXBYTES);
  uint8_t bytesInFifo = readRegField(REG_RXBYTES, 6, 0);
  setIdleState();
  flushRxBuff();
  setRxState();

  while (bytesInFifo < pktLen) {
    bytesInFifo = readRegField(REG_RXBYTES, 6, 0);
    delayMicroseconds(50);
    yield();
  };
    Serial.print("bytesInFifo: ");
    Serial.println(bytesInFifo);
    Serial.print("state: ");
    Serial.println(getState());

  // uint8_t size = readReg(REG_FIFO);
  readRegBurst(REG_FIFO, buff, pktLen);
  if(isAppendStatus) {
    uint8_t r = readReg(REG_FIFO);
    if(r >= 128) rssi = ((rssi - 256) / 2) - RSSI_OFFSET;
    else rssi = (rssi / 2) - RSSI_OFFSET;
    lqi = readReg(REG_FIFO) & 0x7f;
    // if(rssi) rssi = (uint8_t*)readReg(REG_FIFO);
    // byte rawLqi = readReg(REG_FIFO);
    // if(lqi) {
    //   lqi = (uint8_t*)(rawLqi & 0x7f);
    // };
    // if(!(rawLqi >> 7) & 1) return false; // CRC Mismatch
  }
  
  while (getState() != STATE_IDLE){
    flushRxBuff();
    delayMicroseconds(50);
    yield();
  };

  bytesInFifo = readRegField(REG_RXBYTES, 6, 0);
    Serial.print("bytesInFifo: ");
    Serial.println(bytesInFifo);
  setRxState();

  return true;
};
bool Radio::write(uint8_t *buff){
  setIdleState();
  flushTxBuff();

  // writeReg(REG_FIFO, pktLen);
  writeRegBurst(REG_FIFO, buff, pktLen);

  uint8_t bytesInFifo = readStatusReg(REG_TXBYTES);

  setTxState();

  while (getState() != STATE_IDLE){
    delayMicroseconds(50);
    yield();
  };

  flushTxBuff();

  return true;
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
};
void Radio::flushTxBuff(){
  if(getState() != (STATE_IDLE || STATE_TXFIFO_UNDERFLOW)) return;
  writeStatusReg(REG_FTX);
};

void Radio::setCRC(bool en) {
  writeRegField(REG_PKTCTRL0, (byte)en, 2, 2);
};
void Radio::setFEC(bool en) {
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
  writeRegField(REG_MDMCFG2, (byte)en, 3, 3);
};
void Radio::setAppendStatus(bool en) {
  writeRegField(REG_PKTCTRL1, (byte)en, 2, 2);
};
void Radio::setDataWhitening(bool en) {
  writeRegField(REG_PKTCTRL0, (byte)en, 6, 6);
};
void Radio::setVariablePktLen(bool en) {
  writeRegField(REG_PKTCTRL0, (byte)en, 1, 0);
  writeReg(REG_PKTLEN, pktLen);
};
void Radio::setMod(Modulation mod){
  writeRegField(REG_MDMCFG2, (uint8_t)mod, 6, 4);
  // setPwr(power);
};
void Radio::setFreq(double freq){
  uint32_t f = ((freq * 65536.0) / CRYSTAL_FREQ); 

  writeReg(REG_FREQ0, f & 0xff);
  writeReg(REG_FREQ1, (f >> 8) & 0xff);
  writeReg(REG_FREQ2, (f >> 16) & 0xff);

    // setPwr(power);
};
void Radio::setDrate(double drate){
  uint32_t xosc = CRYSTAL_FREQ * 1000;
  uint8_t e = log2((drate * (double)((uint32_t)1 << 20)) / xosc);
  uint32_t m = round(drate * ((double)((uint32_t)1 << (28 - e)) / xosc) - 256.);

  if (m == 256) {
    m = 0;
    e++;
  }

  writeRegField(REG_MDMCFG4, e, 3, 0);
  writeReg(REG_MDMCFG3, (uint8_t)m);
};
void Radio::setPwr(uint8_t freqIdx, uint8_t pwrIdx, const uint8_t pwrTable[][8]){
  if(mod == MOD_ASK_OOK) {
    uint8_t buff[2] = {WRITE, pwrTable[freqIdx][pwrIdx]};
    writeRegBurst(REG_PATABLE, buff, sizeof(buff));
    writeRegField(REG_FREND0, 1, 2, 0);
  } else {
    writeReg(REG_PATABLE, pwrTable[freqIdx][pwrIdx]);
    writeRegField(REG_FREND0, 0, 2, 0);
  }
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
byte Radio::getState() {
  writeStatusReg(REG_NOP);
  return state;
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
