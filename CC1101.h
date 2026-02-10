// #pragma once
#ifndef CC1101
#define CC1101

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
#define REG_RES            0x30  /* Reset chip */
#define REG_RX             0x34  /* Enable RX */
#define REG_TX             0x35  /* Enable TX */
#define REG_IDLE           0x36  /* Enable IDLE */
#define REG_FRX            0x3a  /* Flush the RX FIFO buffer */
#define REG_FTX            0x3b  /* Flush the TX FIFO buffer */
#define REG_NOP            0x3d  /* No operation */

/* Registers */
#define REG_IOCFG0         0x02
#define REG_SYNC1          0x04  /* Sync Word, High Byte */
#define REG_SYNC0          0x05  /* Sync Word, Low Byte */
#define REG_PKTLEN         0x06
#define REG_PKTCTRL1       0x07
#define REG_PKTCTRL0       0x08  /* Packet Automation Control */
#define REG_ADDR           0x09

#define REG_CHANNR         0x0a
#define REG_FREQ2          0x0d
#define REG_FREQ1          0x0e
#define REG_MDMCFG4        0x10
#define REG_MDMCFG3        0x11
#define REG_MDMCFG2        0x12  /* Modem Configuration */
#define REG_MDMCFG1        0x13
#define REG_MDMCFG0        0x14
#define REG_DEVIATN        0x15
#define REG_FREQ0          0x0f

#define REG_MCSM2          0x16
#define REG_MCSM1          0x17
#define REG_MCSM0          0x18
#define REG_FREND0         0x22  /* Front End TX Configuration */

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

enum Frequency {
  FREQ_315 = 0,
  FREQ_433 = 1,
  FREQ_868 = 2,
};

static const double freqRange[][2] = {
  [FREQ_315] = { 300.0, 348.0 },
  [FREQ_433] = { 387.0, 464.0 },
  [FREQ_868] = { 779.0, 928.0 },
};

static const uint8_t powerRange[][8] = {
  [FREQ_315] = { 0x12, 0x0d, 0x1c, 0x34, 0x51, 0x85, 0xcb, 0xc2 },
  [FREQ_433] = { 0x12, 0x0e, 0x1d, 0x34, 0x60, 0x84, 0xc8, 0xc0 },
  [FREQ_868] = { 0x03, 0x0f, 0x1e, 0x27, 0x50, 0x81, 0xcb, 0xc2 },
};

static const double drateRange[][2] = {
  [MOD_2FSK]    = {  0.6, 500.0 },
  [MOD_GFSK]    = {  0.6, 250.0 },
  [2]           = {  0.0, 0.0   }, 
  [MOD_ASK_OOK] = {  0.6, 250.0 },
  [MOD_4FSK]    = {  0.6, 300.0 },
  [5]           = {  0.0, 0.0   },
  [6]           = {  0.0, 0.0   },
  [MOD_MSK]     = { 26.0, 500.0 }
};

class Radio {
  public:
    Radio(
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
      freq(433.0),
      drate(4.0),
      addr(0),
      pktLen(4),
      isCRC(false), 
      isFEC(false),
      isAutoCalib(true),
      isManchester(false),
      isAppendStatus(false),
      isDataWhitening(false),
      isVariablePktLen(false) {};

  uint8_t partnum = -1, version, rssi, lqi;

  bool begin();
  bool read(uint8_t *buff);
  bool write(uint8_t *buff);

  private: 
    uint8_t sck, miso, mosi, ss;
    SPIClass &spi;
    SPISettings spiSettings;

    Modulation mod;
    double freq, drate;
    int8_t power;
    uint8_t pktLen;
    byte addr;
    uint8_t state;
    bool isCRC, 
         isFEC,
         isAutoCalib,
         isManchester,
         isAppendStatus,
         isDataWhitening,
         isVariablePktLen;

    void start();
    void stop();

    void reset();
    void flushRxBuff();
    void flushTxBuff();

    void setCRC(bool en);
    void setFEC(bool en);
    void setAddr(byte addr);
    void setPreamble(byte len);
    void setAutoCalib(bool en);
    void setManchester(bool en);
    void setAppendStatus(bool en);
    void setDataWhitening(bool en);
    void setVariablePktLen(bool en);
    void setMod(Modulation mod);
    void setFreq(double freq);
    void setDrate(double drate);
    void setPower(int8_t power);
    void setRxState();
    void setTxState();
    void setIdleState();
    byte getState();

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
