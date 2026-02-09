#include "CC1101.h"

bool Radio::begin() {
  reset();
  delayMicroseconds(500);

  partnum = readStatusReg(REG_PARTNUM);
  version = readStatusReg(REG_VERSION);

  if(!(partnum == PARTNUM || version == (VERSION || VERSION_LEGACY)) ||
      !((freq >= 300.0 && freq <= 348.0) ||
        (freq >= 387.0 && freq <= 464.0) ||
        (freq >= 779.0 && freq <= 928.0)) || 
      !(drate > drateRange[mod][0] || drate < drateRange[mod][1])
    ) return false;

  setRegs();
  setMod(mod);
  setFreq(freq);
  setDrate(drate);
  setPower(power);
  setAddr(addr);

  return true;
}

bool Radio::read(uint8_t *buff){
  uint8_t bytesInFifo;

  // writeReg(REG_ADDR, addr);

  setIdleState();
  delayMicroseconds(500);
  flushRxBuff();
  delayMicroseconds(500);
  flushTxBuff();
  delayMicroseconds(500);
  Serial.print("flush State: ");
  Serial.println(getState());
  setRxState();
  delayMicroseconds(500);
  Serial.print("rx State: ");
  Serial.println(getState());

  // do {
  //   bytesInFifo = readStatusReg(REG_RXBYTES);
  //   // bytesInFifo = readRegField(REG_RXBYTES, 6, 0);
  //   delayMicroseconds(500);
  //   yield();
  //   Serial.print("bytesInFifo: ");
  //   Serial.println(bytesInFifo);
  // } while (bytesInFifo < buffLen);
  while (readStatusReg(REG_RXBYTES) < buffLen) {
    delayMicroseconds(500);
    yield();
  }

    Serial.print("write State: ");
    Serial.println(getState());
  // uint8_t size = readReg(REG_FIFO);
  readRegBurst(REG_FIFO, buff, buffLen);
  delayMicroseconds(500);
  
  // uint8_t rssi_raw = readReg(REG_FIFO);
  // rssi = rssi_raw >= 128 ? ((rssi_raw - 256) / 2) - RSSI_OFFSET : (rssi_raw / 2) - RSSI_OFFSET;

  while (getState() != STATE_IDLE){
    delayMicroseconds(50);
    yield();
  };
    Serial.print("idle State: ");
    Serial.println(getState());

  setRxState();
    Serial.print("end State: ");
    Serial.println(getState());

  return true;
};
bool Radio::write(uint8_t *buff){
  setIdleState();
  flushTxBuff();
  delayMicroseconds(500);

  // writeReg(REG_FIFO, buffLen);
  writeRegBurst(REG_FIFO, buff, buffLen);
  delayMicroseconds(500);

  uint8_t bytesInFifo = readStatusReg(REG_TXBYTES);

  setTxState();

  while (getState() != STATE_IDLE){
    delayMicroseconds(50);
    yield();
  };

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

void Radio::setRegs(){
  /* Enable automatic calibration when going from IDLE state */
  writeRegField(REG_MCSM0, 1, 5, 4);

  /* Disable append status */
  writeRegField(REG_PKTCTRL1, 0, 2, 2);

  /* Disable variable packet length */
  writeRegField(REG_PKTCTRL0, 0, 1, 0);
  writeReg(REG_PKTLEN, buffLen);
  /* Disable addr filtering */
  writeRegField(REG_PKTCTRL1, 0, 1, 0);
  // /* Disable CRC */
  // writeRegField(REG_PKTCTRL0, 0, 2, 2);
  /* Disable DataWhitening */
  // writeRegField(REG_PKTCTRL0, 0, 6, 6);
  /* Disable manchester */
  // writeRegField(REG_MDMCFG2, 0, 3, 3);
  /* Disable FEC */
  // writeRegField(REG_MDMCFG1, 0, 7, 7);
  /* Disable preamble/sync */
  // writeRegField(REG_MDMCFG2, 0, 2, 0);
  /* Set sync word */
  // writeRegField(REG_MDMCFG1, 0, 6, 4);
  /* Set preamble length */
  // writeRegField(REG_MDMCFG1, 0, 6, 4);
};
void Radio::setMod(Modulation mod){
  writeRegField(REG_MDMCFG2, (uint8_t)mod, 6, 4);
  // setPower(power);
};
void Radio::setFreq(double freq){
  uint32_t f = ((freq * 65536.0) / CRYSTAL_FREQ); 

  writeReg(REG_FREQ0, f & 0xff);
  writeReg(REG_FREQ1, (f >> 8) & 0xff);
  writeReg(REG_FREQ2, (f >> 16) & 0xff);

    // setPower(power);
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
void Radio::setPower(int8_t power){
  uint8_t powerIdx, freqIdx;

  if(freq <= 348.0) {
    freqIdx = 0;
  } else if (freq <= 464.0) {
    freqIdx = 1;
  } else if (freq <= 891.5) {
    freqIdx = 2;
  } else {
    freqIdx = 3;
  }

  if(power <= -30) {
      powerIdx = 0;
  } else if (power <= -20) {
      powerIdx = 1;
  } else if (power <= -15) {
      powerIdx = 2;
  } else if (power <= -10) {
      powerIdx = 3;
  } else if (power <= 0) {
      powerIdx = 4;
  } else if (power <= 5) {
      powerIdx = 5;
  } else if (power <= 7) {
      powerIdx = 6;
  } else {
      powerIdx = 7;
  }

  if(mod == MOD_ASK_OOK) {
    uint8_t buff[2] = { WRITE, powerRange[freqIdx][powerIdx] };
    writeRegBurst(REG_PATABLE, buff, sizeof(buff));
    writeRegField(REG_FREND0, 1, 2, 0);
  } else {
    writeReg(REG_PATABLE, powerRange[freqIdx][powerIdx]);
    writeRegField(REG_FREND0, 0, 2, 0);
  }
};
void Radio::setAddr(byte addr) {
  setIdleState();
  if(addr) {
    writeReg(REG_ADDR, addr);
    writeRegField(REG_PKTCTRL1, 1, 1, 0);
  } else {
    writeRegField(REG_PKTCTRL1, 0, 1, 0);
  }
};
void Radio::setRxState() {
  while(getState() != STATE_RX) {
    if (state == STATE_RXFIFO_OVERFLOW) flushRxBuff();
    else if (state == STATE_TXFIFO_UNDERFLOW) flushTxBuff();
    else if (state != (STATE_CALIB || STATE_SETTLING)) writeStatusReg(REG_RX);
    delayMicroseconds(50);
    yield();
    Serial.print("setRxState: ");
    Serial.println(state);
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
