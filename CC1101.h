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
  FSK2    = 0,
  GFSK    = 1,
  ASK_OOK = 3,
  FSK4    = 4,
  MSK     = 7
};

  static const double drateRange[][2] = {
    [FSK2]    = {  0.6, 500.0 },  /* 0.6 - 500 kBaud */
    [GFSK]    = {  0.6, 250.0 },
    [2]           = {  0.0, 0.0   },  /* gap */
    [ASK_OOK] = {  0.6, 250.0 },
    [FSK4]    = {  0.6, 300.0 },
    [5]           = {  0.0, 0.0   },  /* gap */
    [6]           = {  0.0, 0.0   },  /* gap */
    [MSK]     = { 26.0, 500.0 }
  };

  static const uint8_t powerRange[][8] = {
    [0 /* 315 Mhz */ ] = { 0x12, 0x0d, 0x1c, 0x34, 0x51, 0x85, 0xcb, 0xc2 },
    [1 /* 433 Mhz */ ] = { 0x12, 0x0e, 0x1d, 0x34, 0x60, 0x84, 0xc8, 0xc0 },
    [2 /* 868 Mhz */ ] = { 0x03, 0x0f, 0x1e, 0x27, 0x50, 0x81, 0xcb, 0xc2 },
    [3 /* 915 MHz */ ] = { 0x03, 0x0e, 0x1e, 0x27, 0x8e, 0xcd, 0xc7, 0xc0 }
  };

class Radio {
  public
    : Radio(
        Modulation mod = FSK2,
        double freq = 433.0,
        double drate = 4.0,
        int8_t sck = SCK,
        int8_t miso = MISO,
        int8_t mosi = MOSI,
        int8_t ss = SS,
        SPIClass &spi = SPI
        ):
      mod(mod),
      freq(freq),
      drate(drate),
      sck(sck),
      miso(miso),
      mosi(mosi),
      ss(ss),
      spi(spi),
      spiSettings(SPI_MAX_FREQ, SPI_DATA_ORDER, SPI_DATA_MODE) {};

  uint8_t partnum, version, rssi, lqi;

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

    void start();
    void stop();

    void hardReset();
    void flushRxBuffer();
    void flushTxBuffer();

    void setRegs();
    void setMod(Modulation mod);
    void setFreq(double freq);
    void setDrate(double drate);
    void setPower(int8_t power);

    uint8_t readReg(uint8_t addr);
    uint8_t readStatusReg(uint8_t addr);
    uint8_t readRegField(uint8_t addr, uint8_t hi, uint8_t lo);
    uint8_t readRegBurst(uint8_t addr, uint8_t *buff, uint8_t size);

    void writeReg(uint8_t addr, uint8_t buff);
    void writeStatusReg(uint8_t addr);
    void writeRegField(uint8_t addr, uint8_t buff, uint8_t hi, uint8_t lo);
    void writeRegBurst(uint8_t addr, uint8_t *buff, uint8_t size);
};

#endif
