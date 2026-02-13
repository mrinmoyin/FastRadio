#include "CC1101.h"

bool CC1101::init() {
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

bool CC1101::read(uint8_t *buff){
  setIdleState();
  flushRxBuff();
  setRxState();

  waitForRxBytes(pktLen);

  readRxFifo(buff);

  waitForState();

  // flushRxBuff();
  // setRxState();

  return true;
};
bool CC1101::write(uint8_t *buff){
  setIdleState();
  flushTxBuff();

  setTxState();
   
  writeTxFifo(buff);

  waitForState();

  flushTxBuff();

  return true;
};
void CC1101::link(uint8_t *txBuff, uint8_t *rxBuff) {
  // setTwoWay();

  while(true) {
    Serial.println("link");
    setIdleState();
    flushTxBuff();
    setTxState();

    writeTxFifo(txBuff);
    waitForState();
    Serial.println("Sent packet.");

    flushRxBuff();
    setRxState();

    uint32_t lastMillis = millis();

    // if(millis() - lastMillis > 5000) {
    //   continue;
    // }
    while (millis() - lastMillis < 5000) {
      // waitForRxBytes(pktLen);
      if(readRegField(REG_RXBYTES, 6, 0) == 0) {
        // delayMicroseconds(50);
        // yield();
        delay(500);
        Serial.println("rxBytes is 0 ");
        continue;
      }
      lastMillis = millis();

      readRxFifo(rxBuff);
      waitForState();
      Serial.println("Received packet.");
    }
    Serial.println("timeout");
  }
};

bool CC1101::getChipInfo() {
  partnum = readStatusReg(REG_PARTNUM);
  version = readStatusReg(REG_VERSION);

  if(partnum == PARTNUM && version == VERSION) return true;
  return false;
};
void CC1101::start() {
  spi.beginTransaction(spiSettings);
  digitalWrite(ss, LOW);

  #if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
    return;
  #endif

  while (digitalRead(miso));
}
void CC1101::stop() {
  digitalWrite(ss, HIGH);
  spi.endTransaction();
}

void CC1101::reset() {
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
void CC1101::flushRxBuff(){
  if(getState() != (STATE_IDLE || STATE_RXFIFO_OVERFLOW)) return;
  writeStatusReg(REG_FRX);
  delayMicroseconds(50);
  yield();
};
void CC1101::flushTxBuff(){
  if(getState() != (STATE_IDLE || STATE_TXFIFO_UNDERFLOW)) return;
  writeStatusReg(REG_FTX);
  delayMicroseconds(50);
  yield();
};

void CC1101::setCRC(bool en) {
  writeRegField(REG_PKTCTRL0, (byte)en, 2, 2);
};
void CC1101::setFEC(bool en) {
  if(isVariablePktLen) return;

  writeRegField(REG_MDMCFG1, (byte)en, 7, 7);
};
void CC1101::setAddr(byte addr) {
  writeRegField(REG_PKTCTRL1, addr > 0 ? 1 : 0, 1, 0);
  writeReg(REG_ADDR, addr);
};
void CC1101::setSync(SyncMode syncMode, uint16_t syncWord, uint8_t preambleLen) {
  writeRegField(REG_MDMCFG2, syncMode, 2, 0);

  writeReg(REG_SYNC1, syncWord >> 8);
  writeReg(REG_SYNC0, syncWord & 0xff);

  writeRegField(REG_MDMCFG1, getPreambleIdx(preambleLen), 6, 4);
};
void CC1101::setAutoCalib(bool en) {
  writeRegField(REG_MCSM0, (byte)en, 5, 4);
};
void CC1101::setManchester(bool en) {
  if(mod == MOD_MSK || mod == MOD_4FSK) return;
  writeRegField(REG_MDMCFG2, (byte)en, 3, 3);
};
void CC1101::setAppendStatus(bool en) {
  writeRegField(REG_PKTCTRL1, (byte)en, 2, 2);
};
void CC1101::setDataWhitening(bool en) {
  writeRegField(REG_PKTCTRL0, (byte)en, 6, 6);
};
void CC1101::setVariablePktLen(bool en, uint8_t pktlLen) {
  writeRegField(REG_PKTCTRL0, (byte)en, 1, 0);
  writeReg(REG_PKTLEN, pktLen);
};
void CC1101::setMod(Modulation mod){
  writeRegField(REG_MDMCFG2, (uint8_t)mod, 6, 4);
};
void CC1101::setFreq(double freq){
  uint32_t f = ((freq * 65536.0) / CRYSTAL_FREQ); 

  writeReg(REG_FREQ0, f & 0xff);
  writeReg(REG_FREQ1, (f >> 8) & 0xff);
  writeReg(REG_FREQ2, (f >> 16) & 0xff);

};
void CC1101::setDrate(double drate){
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
void CC1101::setPwr(FreqBand freqBand, PowerMW pwr, const uint8_t pwrTable[][8]){
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
void CC1101::setRxState() {
  while(getState() != STATE_RX) {
    if (state == STATE_RXFIFO_OVERFLOW) flushRxBuff();
    else if (state != (STATE_CALIB || STATE_SETTLING)) writeStatusReg(REG_RX);
    delayMicroseconds(50);
    yield();
  }
};
void CC1101::setTxState() {
  while(getState() != STATE_TX) {
    if(state == STATE_TXFIFO_UNDERFLOW) flushTxBuff();
    else if (state != (STATE_CALIB || STATE_SETTLING)) writeStatusReg(REG_TX);
    delayMicroseconds(50);
    yield();
  }
};
void CC1101::setIdleState() {
  while(getState() != STATE_IDLE) {
    writeStatusReg(REG_IDLE);
    delayMicroseconds(50);
    yield();
  }
};
void CC1101::setTwoWay() {
    writeRegField(REG_MCSM1, 0, 5, 4); /* Disabl CCA */
    writeRegField(REG_MCSM1, 2, 3, 2); /* Set RXOFF to TX */
    writeRegField(REG_MCSM1, 3, 1, 0); /* Set TXOFF to RX */
};

byte CC1101::getState() {
  writeStatusReg(REG_NOP);
  return state;
};
bool CC1101::getFreqBand(double freq, const double freqTable[][2]) {
  for(int i = 0; i < 4; i++) {
    if(freq >= freqTable[i][0] && freq <= freqTable[i][1]) {
      freqBand = (FreqBand)i;
      return true;
    }
  }
  return false;
};
uint8_t CC1101::getPreambleIdx(uint8_t len) {
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

void CC1101::waitForState(State state) {
  while (getState() != state){
    delayMicroseconds(50);
    yield();
  };
};
void CC1101::waitForRxBytes(uint8_t len) {
  uint8_t bytes;
  do {
    bytes = readRegField(REG_RXBYTES, 6, 0);
    delayMicroseconds(50);
    yield();
  } while (bytes < len);
};
void CC1101::waitForTxBytes(uint8_t len) {
  uint8_t bytes;
  do {
    bytes = readRegField(REG_TXBYTES, 6, 0);
    delayMicroseconds(50);
    yield();
  } while (bytes < len);
};

void CC1101::readRxFifo(uint8_t *buff) {
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
void CC1101::writeTxFifo(uint8_t *buff) {
  if(isVariablePktLen) {
    pktLen = sizeof(buff);
    writeReg(REG_FIFO, pktLen);
  }
  // if(addr > 0) {
  //   writeReg(REG_FIFO, addr);
  // }
  writeRegBurst(REG_FIFO, buff, pktLen);
};

byte CC1101::readReg(byte addr) {
  start();
  spi.transfer(0x80 | (addr & 0b111111));
  // spi.transfer(addr | READ);
  uint8_t data = spi.transfer(WRITE);
  stop();

  return data;
};
byte CC1101::readStatusReg(byte addr){
  start();
  // byte header = 0x80 | (addr & 0b111111);
  // header |= 0x40;
  // spi.transfer(header);
  spi.transfer(addr | READ_BURST);
  uint8_t data = spi.transfer(WRITE);
  stop();

  return data;
};
byte CC1101::readRegField(byte addr, byte hi, byte lo){
  return readStatusReg((addr) >> lo) & ((1 << (hi - lo + 1)) -1);
};
void CC1101::readRegBurst(byte addr, uint8_t *buff, size_t size){
  start();
  spi.transfer(0x80 | 0x40 | (addr & 0b111111));
  // spi.transfer(addr | READ_BURST);
  for (uint8_t i = 0; i < size; i++) {
    buff[i] = spi.transfer(WRITE);
  }
  stop();
};

void CC1101::writeReg(byte addr, byte val){
  start();
    spi.transfer(0x00 | (addr & 0b111111));
    // spi.transfer(addr);
    spi.transfer(val);
  stop();
};
void CC1101::writeStatusReg(byte addr){
  start();
  // state =(spi.transfer(0x00 | (addr & 0b111111)) >> 4) & 0b00111;
  state =(spi.transfer(addr) >> 4) & 0b00111;
  // spi.transfer(0x00 | (addr & 0b111111)
  // spi.transfer(addr);
  stop();
};
void CC1101::writeRegField(byte addr, byte val, byte hi, byte lo){
  uint8_t mask = ((1 << (hi - lo + 1)) -1) << lo;
  writeReg(addr, (readReg(addr) & ~mask) | ((val <<= lo) & mask));
};
void CC1101::writeRegBurst(byte addr, uint8_t *buff, size_t size){
  start();
    spi.transfer(0x00 | 0x40 | (addr & 0b111111));
    // spi.transfer(addr | WRITE_BURST);
    for (uint8_t i = 0; i < size; i++) {
       spi.transfer(buff[i]);
    }
  stop();
};
