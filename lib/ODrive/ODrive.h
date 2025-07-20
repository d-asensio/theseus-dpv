#ifndef ODRIVE_H
#define ODRIVE_H

#include <Arduino.h>
#include <HardwareSerial.h>

#include "logger.h"

class ODrive
{
private:
  logging::Logger *logger;
  HardwareSerial *serial;

public:
  ODrive(logging::Logger *logger, HardwareSerial *serial);

  void setVelocity(uint8_t axis_id, int16_t velocity, float torqueFF = 0.0f);
};

#endif // ODRIVE_H