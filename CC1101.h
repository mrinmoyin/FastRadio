// #pragma once
#ifndef CC1101
#define CC1101

#include <Arduino.h>
#include <SPI.h>

#define SPI_MAX_FREQ       6500000    /* 6.5 MHz */
#define SPI_DATA_ORDER     MSBFIRST
#define SPI_DATA_MODE      SPI_MODE0  /* clk low, leading edge */

#define FIFO_SIZE          64    /* 64 B */
#define CRYSTAL_FREQ       26    /* 26 MHz */

#define STATE_IDLE              0b000
#define STATE_RX                0b001
#define STATE_TX                0b010
#define STATE_FSTXON            0b011
#define STATE_CALIB             0b100
#define STATE_SETTLING          0b101
#define STATE_RXFIFO_OVERFLOW   0b110
#define STATE_TXFIFO_UNDERFLOW  0b111

#define RSSI_OFFSET        74
#define PKT_LEN_FIXED      0
#define PKT_LEN_VARIABLE   1

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

enum Modulation {
  MOD_2FSK    = 0,
  MOD_GFSK    = 1,
  MOD_ASK_OOK = 3,
  MOD_4FSK    = 4,
  MOD_MSK     = 7
};

static const double drateRange[][2] = {
  [MOD_2FSK]    = {  0.6, 500.0 },  /* 0.6 - 500 kBaud */
  [MOD_GFSK]    = {  0.6, 250.0 },
  [2]           = {  0.0, 0.0   },  /* gap */
  [MOD_ASK_OOK] = {  0.6, 250.0 },
  [MOD_4FSK]    = {  0.6, 300.0 },
  [5]           = {  0.0, 0.0   },  /* gap */
  [6]           = {  0.0, 0.0   },  /* gap */
  [MOD_MSK]     = { 26.0, 500.0 }
};

static const uint8_t powerRange[][8] = {
  [0 /* 315 Mhz */ ] = { 0x12, 0x0d, 0x1c, 0x34, 0x51, 0x85, 0xcb, 0xc2 },
  [1 /* 433 Mhz */ ] = { 0x12, 0x0e, 0x1d, 0x34, 0x60, 0x84, 0xc8, 0xc0 },
  [2 /* 868 Mhz */ ] = { 0x03, 0x0f, 0x1e, 0x27, 0x50, 0x81, 0xcb, 0xc2 },
  [3 /* 915 MHz */ ] = { 0x03, 0x0e, 0x1e, 0x27, 0x8e, 0xcd, 0xc7, 0xc0 }
};

class Radio {
  public:
    Radio(
        Modulation mod = MOD_2FSK,
        double freq = 433.0,
        double drate = 4.0,
        byte addr = 0,
        int8_t sck = SCK,
        int8_t miso = MISO,
        int8_t mosi = MOSI,
        int8_t ss = SS,
        SPIClass &spi = SPI
        ):
      mod(mod),
      freq(freq),
      drate(drate),
      addr(addr),
      sck(sck),
      miso(miso),
      mosi(mosi),
      ss(ss),
      spi(spi),
      spiSettings(SPI_MAX_FREQ, SPI_DATA_ORDER, SPI_DATA_MODE),
      buffLen(4) {};

    Radio(
        int8_t sck,
        int8_t miso,
        int8_t mosi,
        int8_t ss,
        SPIClass &spi = SPI
        ): Radio(MOD_2FSK, 433.0, 4.0, 0, sck, miso, mosi, ss, spi) {};

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
    uint8_t buffLen;
    uint8_t state;
    byte addr;

    void start();
    void stop();

    void reset();
    void flushRxBuff();
    void flushTxBuff();

    void setRegs();
    void setMod(Modulation mod);
    void setFreq(double freq);
    void setDrate(double drate);
    void setPower(int8_t power);
    void setAddr(byte addr);
    void setRxState();
    void setTxState();
    void setIdleState();
    void updateState();

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
