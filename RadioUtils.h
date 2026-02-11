#pragma once

#ifndef RadioUtils 
#define RadioUtils

#include <Arduino.h>

int8_t getFreqIdx(double freq, const double freqTable[][2]);
uint8_t getPwrIdx(int8_t pwr);
uint8_t getPreambleIdx(uint8_t len);

#endif
