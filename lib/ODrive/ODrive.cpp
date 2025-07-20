#include "ODrive.h"

ODrive::ODrive(logging::Logger *logger, HardwareSerial *serial)
    : logger(logger), serial(serial)
{
  if (logger)
  {
    logger->log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "ODrive", "Initialized");
  }
}

void ODrive::setVelocity(uint8_t axis_id, int16_t velocity, float torqueFF)
{
  if (!serial)
  {
    if (logger)
    {
      logger->log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "ODrive", "Cannot set velocity - serial interface is null");
    }
    return;
  }

  /* Build the command:
     "v <axis> <velocity> <torqueFF>\n"
     dtostrf() converts the float → char[] with fixed width/precision
  */
  char tBuf[12];
  dtostrf(torqueFF, 0, 4, tBuf);

  serial->print("v ");
  serial->print(axis_id);
  serial->print(' ');
  serial->print(velocity);
  serial->print(' ');
  serial->print(tBuf);
  serial->print('\n');

  if (logger)
  {
    logger->log(logging::LoggerLevel::LOGGER_LEVEL_DEBUG, "ODrive", "Set velocity to %d on axis %d with torque FF %.4f", velocity, axis_id, torqueFF);
  }
}