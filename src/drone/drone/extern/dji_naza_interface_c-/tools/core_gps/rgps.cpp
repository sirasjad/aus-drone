#include "libnaza/naza_serial_gps.h"
#include "libnaza/pca9685.h"
#include <errno.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringSerial.h>

int main() {
  double lat, lon, alt, speed, heading;
  uint8_t sats;

  NazaDecoder.getAll(lat, lon, alt, speed, heading, sats);
  std::cout << round(sats) << "," << lat << "," << lon << "," << heading << ","
            << alt << "," << speed << " \n";

  return 0;
}
