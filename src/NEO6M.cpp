#include "NEO6M.h"

bool NEO6M::update() {
  std::string gngga[14];
  readLine("$GNGGA,", gngga, 14);
  time = gngga[0].c_str();
  lat = gngga[1].c_str();
  lon = gngga[3].c_str();
  return true;
}

void NEO6M::readLine(char *delemeter, std::string *buff, size_t size) {
  while (ss.available()) {
    if (ss.find(delemeter)) {
      char bytes[100];
      int length = ss.readBytesUntil('\n', bytes, 100);

      std::stringstream ss;
      for (int i = 0; i < length; i++) {
        ss << bytes[i];
      }
      std::string str = ss.str();

      for (int i = 0; i < size; i++) {
        uint8_t delIdx = str.find(",");
        buff[i] = str.substr(0, delIdx);
        str.erase(0, delIdx + 1);
      }
    }
    ss.flush();
  }
}
