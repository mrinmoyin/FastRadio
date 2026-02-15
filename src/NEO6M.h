#pragma once

#ifndef NEO6M_H
#define NEO6M_H

#include <Arduino.h>
#include <sstream>
#include <string>

class NEO6M {
  public:
    NEO6M(UART ss = Serial): ss(Serial) {};

    String time, lat, lon;

    bool update();

  private: 
    UART ss;

    void readLine(char *delemeter, std::string *buff, size_t size);
};

#endif
