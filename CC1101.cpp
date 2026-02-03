#include "CC1101.h"

bool Radio::begin() {
  hardReset();
  delay(10);

  partnum = readStatusReg(REG_PARTNUM);
  version = readStatusReg(REG_VERSION);

  if(
      (
       partnum != PARTNUM || 
       !(version != VERSION || version != VERSION_LEGACY)
       ) || !(
        (freq >= 300.0 && freq <= 348.0) ||
        (freq >= 387.0 && freq <= 464.0) ||
        (freq >= 779.0 && freq <= 928.0)
        ) || (
          drate < drateRange[mod][0] ||
          drate > drateRange[mod][1]
          )
    ) return false;

  setRegs();
  setMod(mod);
  setFreq(freq);
  setDrate(drate);
  setPower(power);

  return true;
}

bool Radio::read(uint8_t *buff){
  uint8_t rxBytes = readStatusReg(REG_RXBYTES);
  uint8_t bytesInFifo = readReg(REG_FIFO);

  // writeStatusReg(REG_FRX);
  // writeStatusReg(REG_IDLE);
  // writeStatusReg(REG_RX);
  // while (state != 1);

  // do {
  //   bytesInFifo = readRegField(REG_RXBYTES, 6, 0);
  // } while (bytesInFifo < 4);

  setIDLEState();
  flushRxBuffer();
  setRXState();
  
  readRegBurst(REG_FIFO, buff, buffLen);

  while (getState() != 0) {
    delayMicroseconds(50);
    yield();
  }

  rssi = readReg(REG_FIFO);
  lqi = readReg(REG_FIFO) & 0x7f;
  flushRxBuffer();
  // writeStatusReg(REG_FRX);
  // writeStatusReg(REG_RX);
  return true;
};
bool Radio::write(uint8_t *buff){
  uint8_t txBytes = readStatusReg(REG_TXBYTES);

  if(getState() != 1) {
    setIDLEState();
    flushTxBuffer();
    flushRxBuffer();
    setRXState();
  }
  setTXState();

  if(getState() == 1) {
    return false;
  }
    
  // writeReg(REG_FIFO, FIFO_SIZE);
  writeReg(REG_FIFO, buffLen);
  writeRegBurst(REG_FIFO, buff, buffLen);
  // while(readStatusReg(REG_NOP) > 0);
  while (getState() != 0){
    delayMicroseconds(50);
    yield();
  };
  setIDLEState();
  flushTxBuffer();
  setRXState();
  return true;
};

void Radio::start() {
  spi.beginTransaction(spiSettings);
  digitalWrite(ss, LOW);

  #if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
    return;
  #endif

  while (digitalRead(miso) > 0);
}
void Radio::stop() {
  digitalWrite(ss, HIGH);
  spi.endTransaction();
}

void Radio::hardReset() {
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
void Radio::flushRxBuffer(){
  writeStatusReg(REG_FRX);
};
void Radio::flushTxBuffer(){
  writeStatusReg(REG_FTX);
};

void Radio::setRegs(){
  /* Automatically calibrate when going from IDLE to RX or TX. */
  writeRegField(REG_MCSM0, 1, 5, 4);

  /* Enable append status */
  writeRegField(REG_PKTCTRL1, 1, 2, 2);

  /* Disable data whitening. */
  // setDataWhitening(false);
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
void Radio::setPower(int8_t drate){
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

  if(mod == ASK_OOK) {
    uint8_t buff[2] = { WRITE, powerRange[freqIdx][powerIdx] };
    writeRegBurst(REG_PATABLE, buff, sizeof(buff));
    writeRegField(REG_FREND0, 1, 2, 0);
  } else {
    writeReg(REG_PATABLE, powerRange[freqIdx][powerIdx]);
    writeRegField(REG_FREND0, 0, 2, 0);
  }
};
void Radio::setRXState(){
  while(true) {
    byte state = getState();
    if (state == 0b001) break;
    else if (state == 0b110) writeStatusReg(REG_FRX);
    else if (state == 0b111) writeStatusReg(REG_FTX);
    writeStatusReg(REG_RX);
    delayMicroseconds(50);
    yield();
  //   switch (getState()) {
  //     case 0b001:
  //       break;
  //     case 0b110:
  //       writeStatusReg(REG_FRX);
  //       break;
  //     case 0b111:
  //       writeStatusReg(REG_FTX);
  //       break;
  //   }
  //   writeStatusReg(REG_RX);
  //   break;
  }
};
void Radio::setTXState(){
  writeStatusReg(REG_TX);
  while(getState() != 2);
};
void Radio::setIDLEState(){
  writeStatusReg(REG_IDLE);
  while(getState() != 0);
};
byte Radio::getState(){
  // byte oldState = writeStatusReg(REG_NOP);
  // while(true) {
  //   byte state = writeStatusReg(REG_NOP);
  //   if (state == oldState) {
  //     return (state>>4)&0b00111;
  //     Serial.print("State: ");
  //     Serial.println(state);
  //   }
  //   oldState = state;
  // }
  writeStatusReg(REG_NOP);
  return state;
};

uint8_t Radio::readReg(byte addr){
  start();
  // spi.transfer(READ | (addr & 0b111111));
  spi.transfer(addr | READ);
  uint8_t data = spi.transfer(WRITE);
  stop();

  Serial.print("readReg ");
  Serial.print(String(addr));
  Serial.print(" : ");
  Serial.println(data);
  return data;
};
uint8_t Radio::readStatusReg(byte addr){
  start();
  spi.transfer(addr | READ_BURST);
  uint8_t data = spi.transfer(WRITE);
  stop();

  Serial.print("readStatusReg ");
  Serial.print(addr);
  Serial.print(" : ");
  Serial.println(data);
  return data;
};
uint8_t Radio::readRegField(byte addr, byte hi, byte lo){
  return readStatusReg((addr) >> lo) & ((1 << (hi - lo + 1)) -1);
};
void Radio::readRegBurst(byte addr, uint8_t *buff, size_t size){
  start();
  // spi.transfer(READ | WRITE_BURST | (addr & 0b111111));
  spi.transfer(addr | READ_BURST);
  for (uint8_t i = 0; i < size; i++) {
    buff[i] = spi.transfer(WRITE);
  }
  stop();
};

void Radio::writeReg(byte addr, byte val){
  start();
    // spi.transfer(WRITE | (addr & 0b111111));
    spi.transfer(addr);
    spi.transfer(val);
  stop();
};
byte Radio::writeStatusReg(byte addr){
  start();
  // spi.transfer(WRITE | (addr & 0b111111));
  uint8_t status = spi.transfer(addr);
  state = (status >> 4) & 0b00111;
  // Serial.print("State: ");
  // Serial.println(state);
  stop();
  return status;
};
void Radio::writeRegField(byte addr, byte val, byte hi, byte lo){
  uint8_t mask = ((1 << (hi - lo +1)) -1) << lo;
  writeReg(addr, (readReg(addr) & ~mask) | ((val <<= lo) & mask));
};
void Radio::writeRegBurst(byte addr, uint8_t *buff, size_t size){
  start();
    // spi.transfer(WRITE | BURST | (addr & 0b111111));
    spi.transfer(addr | WRITE_BURST);
    for (uint8_t i = 0; i < size; i++) {
       spi.transfer(buff[i]);
    }
  stop();
};
