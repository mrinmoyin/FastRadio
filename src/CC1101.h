#pragma once

#ifndef CC1101_H
#define CC1101_H

#include <Arduino.h>
#include <SPI.h>

#define SPI_MAX_FREQ       6500000  
#define SPI_DATA_ORDER     MSBFIRST
#define SPI_DATA_MODE      SPI_MODE0  

#define FIFO_SIZE          64    
#define CRYSTAL_FREQ       26   

#define RSSI_OFFSET        74

#define READ               0x80
#define WRITE              0x00
#define READ_BURST         0xC0
#define WRITE_BURST        0x40

#define PARTNUM            0x00
#define VERSION            0x14
#define VERSION_LEGACY     0x04

/* Command strobes */
#define REG_RES            0x30  
#define REG_RX             0x34  
#define REG_TX             0x35  
#define REG_IDLE           0x36  
#define REG_FRX            0x3a  
#define REG_FTX            0x3b  
#define REG_NOP            0x3d  

/* Registers */
#define REG_IOCFG0         0x02
#define REG_SYNC1          0x04  
#define REG_SYNC0          0x05  
#define REG_PKTLEN         0x06
#define REG_PKTCTRL1       0x07
#define REG_PKTCTRL0       0x08  
#define REG_ADDR           0x09

#define REG_CHANNR         0x0a
#define REG_FREQ2          0x0d
#define REG_FREQ1          0x0e
#define REG_MDMCFG4        0x10
#define REG_MDMCFG3        0x11
#define REG_MDMCFG2        0x12  
#define REG_MDMCFG1        0x13
#define REG_MDMCFG0        0x14
#define REG_DEVIATN        0x15
#define REG_FREQ0          0x0f

#define REG_MCSM2          0x16
#define REG_MCSM1          0x17
#define REG_MCSM0          0x18
#define REG_FREND0         0x22  

#define REG_PATABLE        0x3e
#define REG_FIFO           0x3f

/* Status registers */
#define REG_PARTNUM        0x30
#define REG_VERSION        0x31
#define REG_TXBYTES        0x3a
#define REG_RXBYTES        0x3b
#define REG_RCCTRL0_STATUS 0x3d

enum State {
  STATE_IDLE              = 0,
  STATE_RX                = 1,
  STATE_TX                = 2,
  STATE_FSTXON            = 3,
  STATE_CALIB             = 4,
  STATE_SETTLING          = 5,
  STATE_RXFIFO_OVERFLOW   = 6,
  STATE_TXFIFO_UNDERFLOW  = 7,
};

enum Modulation {
  MOD_2FSK    = 0,
  MOD_GFSK    = 1,
  MOD_ASK_OOK = 3,
  MOD_4FSK    = 4,
  MOD_MSK     = 7
};

enum FreqBand {
  FREQ_BAND_315 = 0,
  FREQ_BAND_433 = 1,
  FREQ_BAND_868 = 2,
  FREQ_BAND_915 = 3,
};

enum PowerMW {
  POWER_1MW   = 4,  /* 1mw / 0dbm */
  POWER_3MW   = 5,  /* 3.16mw / 5dbm */
  POWER_5MW   = 6,  /* 5.01mw / 7dbm */
  POWER_10MW  = 7, /* 10mw / 10dbm */
};

enum SyncMode {
  SYNC_MODE_NO_PREAMBLE    = 0,  /* No preamble/sync */
  SYNC_MODE_15_16          = 1,  /* 15/16 sync word bits detected */
  SYNC_MODE_16_16          = 2,  /* 16/16 sync word bits detected */
  SYNC_MODE_30_32          = 3,  /* 30/32 sync word bits detected */
  SYNC_MODE_NO_PREAMBLE_CS = 4,  /* No preamble/sync, carrier-sense above threshold */
  SYNC_MODE_15_16_CS       = 5,  /* 15/16 + carrier-sense above threshold */
  SYNC_MODE_16_16_CS       = 6,  /* 16/16 + carrier-sense above threshold */
  SYNC_MODE_30_32_CS       = 7,  /* 30/32 + carrier-sense above threshold */
};

static const double freqTable[][2] = {
  [FREQ_BAND_315] = { 300.0, 348.0 },
  [FREQ_BAND_433] = { 387.0, 464.0 },
  [FREQ_BAND_868] = { 779.0, 891.5 },
  [FREQ_BAND_915] = { 896.6, 928.0 },
};

static const double drateTable[][2] = {
  [MOD_2FSK]    = {  0.6, 500.0 },
  [MOD_GFSK]    = {  0.6, 250.0 },
  [2]           = {  0.0, 0.0   }, 
  [MOD_ASK_OOK] = {  0.6, 250.0 },
  [MOD_4FSK]    = {  0.6, 300.0 },
  [5]           = {  0.0, 0.0   },
  [6]           = {  0.0, 0.0   },
  [MOD_MSK]     = { 26.0, 500.0 }
};

static const uint8_t pwrTable[][8] = {
  [FREQ_BAND_315] = { 0x12, 0x0d, 0x1c, 0x34, 0x51, 0x85, 0xcb, 0xc2 },
  [FREQ_BAND_433] = { 0x12, 0x0e, 0x1d, 0x34, 0x60, 0x84, 0xc8, 0xc0 },
  [FREQ_BAND_868] = { 0x03, 0x0f, 0x1e, 0x27, 0x50, 0x81, 0xcb, 0xc2 },
  [FREQ_BAND_915] = { 0x03, 0x0e, 0x1e, 0x27, 0x8e, 0xcd, 0xc7, 0xc0 },
};

class CC1101 {
  public:
    CC1101(
        int8_t ss = SS,
        int8_t sck = SCK,
        int8_t miso = MISO,
        int8_t mosi = MOSI,
        SPIClass &spi = SPI
        ):
      ss(ss),
      sck(sck),
      miso(miso),
      mosi(mosi),
      spi(spi),
      spiSettings(SPI_MAX_FREQ, SPI_DATA_ORDER, SPI_DATA_MODE),
      mod(MOD_2FSK),
      freq(433.8),
      drate(4.0),
      pwr(POWER_1MW),
      addr(0),
      pktLen(4),
      syncMode(SYNC_MODE_16_16),
      syncWord(0x1234),
      preambleLen(64),
      isCRC(true), 
      isFEC(false),
      isAutoCalib(true),
      isManchester(false),
      isAppendStatus(true),
      isDataWhitening(false),
      isVariablePktLen(false) {};

  int8_t partnum = -1, version = -1;
  uint8_t rssi, lqi;

  bool init();

  bool read(uint8_t *buff);
  bool write(uint8_t *buff);
  void link(uint8_t *txBuff, uint8_t *rxBuff);

  private: 
    uint8_t sck, miso, mosi, ss;
    SPIClass &spi;
    SPISettings spiSettings;

    Modulation mod;
    SyncMode syncMode;
    PowerMW pwr;
    FreqBand freqBand;
    double freq, drate;
    uint8_t pktLen, preambleLen;
    uint16_t syncWord;
    byte addr;
    uint8_t state;
    bool isCRC, 
         isFEC,
         isAutoCalib,
         isManchester,
         isAppendStatus,
         isDataWhitening,
         isVariablePktLen;
    int8_t pwrIdx = -1, preambleIdx = -1;

    bool getChipInfo();
    void start();
    void stop();

    void reset();
    void flushRxBuff();
    void flushTxBuff();

    void setCRC(bool en);
    void setFEC(bool en);
    void setAddr(byte addr);
    void setSync(SyncMode syncMode, uint16_t syncWord, uint8_t preambleLen);
    void setAutoCalib(bool en);
    void setManchester(bool en);
    void setAppendStatus(bool en);
    void setDataWhitening(bool en);
    void setVariablePktLen(bool en, uint8_t pktLen);
    void setMod(Modulation mod);
    void setFreq(double freq);
    void setDrate(double drate);
    void setPwr(FreqBand freqBand, PowerMW pwr, const uint8_t pwrTable[][8]);
    void setRxState();
    void setTxState();
    void setIdleState();
    void setTwoWay();

    byte getState();
    bool getFreqBand(double freq, const double freqTable[][2]);
    uint8_t getPreambleIdx(uint8_t len);

    void waitForState(State state = STATE_IDLE);
    void waitForRxBytes(uint8_t len = 1);
    void waitForTxBytes(uint8_t len = 1);

    void readRxFifo(uint8_t *buff);
    void writeTxFifo(uint8_t *buff);

    byte readReg(byte addr);
    byte readStatusReg(byte addr);
    byte readRegField(byte addr, byte hi, byte lo);
    void readRegBurst(byte addr, uint8_t *buff, size_t size);

    void writeReg(byte addr, byte val);
    void writeStatusReg(byte addr);
    void writeRegField(byte addr, byte val, byte hi, byte lo);
    void writeRegBurst(byte addr, uint8_t *buff, size_t size);
};

#endif
