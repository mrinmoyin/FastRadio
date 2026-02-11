#include "RadioUtils.h"

int8_t getFreqIdx(double freq, const double freqTable[][2]) {
  if (freq >= freqTable[0][0] && freq <= freqTable[0][1]) {
    return 0;
  } else if (freq >= freqTable[1][0] && freq <= freqTable[1][1]) {
    return 1;
  } else if (freq >= freqTable[2][0] && freq <= freqTable[2][1]) {
    return 2;
  } else {
    return -1;
  }
};
uint8_t getPwrIdx(int8_t pwr) {
  if(pwr <= -30) {
      return 0;
  } else if (pwr <= -20) {
      return 1;
  } else if (pwr <= -15) {
      return 2;
  } else if (pwr <= -10) {
      return 3;
  } else if (pwr <= 0) {
      return 4;
  } else if (pwr <= 5) {
      return 5;
  } else if (pwr <= 7) {
      return 6;
  } else {
      return 7;
  }
};
uint8_t getPreambleIdx(uint8_t len) {
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
