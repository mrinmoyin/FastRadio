// #pragma once
#ifndef CC1101_RAW
#define CC1101_RAW

#include <Arduino.h>
#include <SPI.h>

#define CC1101_SPI_MAX_FREQ       6500000    /* 6.5 MHz */
#define CC1101_SPI_DATA_ORDER     MSBFIRST
#define CC1101_SPI_DATA_MODE      SPI_MODE0  /* clk low, leading edge */

#define CC1101_FIFO_SIZE          64    /* 64 B */
#define CC1101_CRYSTAL_FREQ       26    /* 26 MHz */

#define CC1101_WRITE              0x00
#define CC1101_READ               0x80
#define CC1101_BURST              0x40

#define CC1101_PARTNUM            0x00
#define CC1101_VERSION            0x14
#define CC1101_VERSION_LEGACY     0x04

/* Command strobes */
#define CC1101_CMD_RES            0x30  /* Reset chip */
#define CC1101_CMD_RX             0x34  /* Enable RX */
#define CC1101_CMD_TX             0x35  /* Enable TX */
#define CC1101_CMD_IDLE           0x36  /* Enable IDLE */
#define CC1101_CMD_FRX            0x3a  /* Flush the RX FIFO buffer */
#define CC1101_CMD_FTX            0x3b  /* Flush the TX FIFO buffer */
#define CC1101_CMD_NOP            0x3d  /* No operation */

/* Registers */
#define CC1101_REG_IOCFG0         0x02
#define CC1101_REG_SYNC1          0x04  /* Sync Word, High Byte */
#define CC1101_REG_SYNC0          0x05  /* Sync Word, Low Byte */
#define CC1101_REG_PKTLEN         0x06
#define CC1101_REG_PKTCTRL1       0x07
#define CC1101_REG_PKTCTRL0       0x08  /* Packet Automation Control */
#define CC1101_REG_ADDR           0x09

#define CC1101_REG_CHANNR         0x0a
#define CC1101_REG_FREQ2          0x0d
#define CC1101_REG_FREQ1          0x0e
#define CC1101_REG_MDMCFG4        0x10
#define CC1101_REG_MDMCFG3        0x11
#define CC1101_REG_MDMCFG2        0x12  /* Modem Configuration */
#define CC1101_REG_MDMCFG1        0x13
#define CC1101_REG_MDMCFG0        0x14
#define CC1101_REG_DEVIATN        0x15
#define CC1101_REG_FREQ0          0x0f

#define CC1101_REG_MCSM2          0x16
#define CC1101_REG_MCSM1          0x17
#define CC1101_REG_MCSM0          0x18
#define CC1101_REG_FREND0         0x22  /* Front End TX Configuration */

#define CC1101_REG_PATABLE        0x3e
#define CC1101_REG_FIFO           0x3f

/* Status registers */
#define CC1101_REG_PARTNUM        0x30
#define CC1101_REG_VERSION        0x31
#define CC1101_REG_TXBYTES        0x3a
#define CC1101_REG_RXBYTES        0x3b
#define CC1101_REG_RCCTRL0_STATUS 0x3d

class Radio:{
  public
    : Radio(int8_t sck = SCK, int8_t miso = MISO, int8_t mosi = MOSI, int8_t ss = SS, SPIClass spi = SPI):sck(sck), miso(miso), mosi(mosi), ss(ss), spi(spi) {};

  private: 
    uint8_t sck, miso, mosi, ss;
    SPIClass spi;
};

#endif
