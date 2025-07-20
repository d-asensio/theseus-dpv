#include "RotationEncoder.h"

RotationEncoder::RotationEncoder(logging::Logger *logger, Adafruit_ADS1115 *ads)
    : logger(logger), ads(ads), current_angle(0), previous_angle(0), angle_delta(0), angle_changed(false)
{
  if (logger)
  {
    logger->log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "RotationEncoder", "Initialized");
  }
}

void RotationEncoder::setup(int16_t calibration_min_value, int16_t calibration_max_value, int8_t direction)
{
  this->calibration_min_value = calibration_min_value;
  this->calibration_max_value = calibration_max_value;
  this->direction = direction;

  // Initialize with current angle
  current_angle = getAngle();
  previous_angle = current_angle;

  if (logger)
  {
    logger->log(logging::LoggerLevel::LOGGER_LEVEL_DEBUG, "RotationEncoder", "Setup complete - min: %d, max: %d, direction: %d",
                calibration_min_value, calibration_max_value, direction);
  }
}

int16_t RotationEncoder::getAngle()
{
  int16_t analog_value = ads->readADC_SingleEnded(0);

  return map(
      analog_value,
      calibration_min_value,
      calibration_max_value,
      0,
      360);
}

int16_t RotationEncoder::getAngleDelta()
{
  return angle_delta;
}

bool RotationEncoder::hasChanged()
{
  return angle_changed;
}

void RotationEncoder::loop()
{
  previous_angle = current_angle;
  current_angle = getAngle();

  // Calculate the raw difference
  int16_t raw_delta = current_angle - previous_angle;

  // Handle wrapping by finding the shortest path
  // The shortest angular distance between two angles on a circle
  // is always <= 180 degrees in either direction

  if (raw_delta > 180)
  {
    // Wrapped clockwise: subtract 360 to get the shorter path
    raw_delta -= 360;
  }
  else if (raw_delta < -180)
  {
    // Wrapped counter-clockwise: add 360 to get the shorter path
    raw_delta += 360;
  }

  angle_delta = raw_delta * direction;
  angle_changed = (angle_delta != 0);
}