#ifndef ROTATION_ENCODER_H
#define ROTATION_ENCODER_H

#include <Arduino.h>
#include <Adafruit_ADS1X15.h>

#include "logger.h"

class RotationEncoder
{
private:
  logging::Logger *logger;
  Adafruit_ADS1115 *ads;

  int16_t calibration_min_value;
  int16_t calibration_max_value;
  int8_t direction;

  int16_t current_angle;
  int16_t previous_angle;
  int16_t angle_delta;

public:
  RotationEncoder(logging::Logger *logger, Adafruit_ADS1115 *ads);

  void setup(int16_t calibration_min_value, int16_t calibration_max_value, int8_t direction);

  int16_t getAngle();
  int16_t getAngleDelta();

  void loop();
};

#endif // ROTATION_ENCODER_H