#ifndef BONEX_MOTOR_CONTROLLER_H
#define BONEX_MOTOR_CONTROLLER_H

#include <Arduino.h>
#include "logger.h"
#include "ODrive.h"
#include "EMIFilterSwitch.h"
#include "RotationEncoder.h"

class BonexMotorController
{
private:
  logging::Logger *logger;
  ODrive *odrive;
  EMIFilterSwitch *trigger_switch;
  EMIFilterSwitch *reverse_switch;
  RotationEncoder *encoder;

  // Configuration (needed across multiple loop calls)
  uint8_t axis_id;
  float max_torque;
  int16_t min_velocity;
  int16_t max_velocity;

  // State
  int16_t velocity_setpoint;

private:
  // Helper methods
  int16_t fromEncoderAngleDeltaToVelocityDelta(int16_t angle_delta);

  // Core processing methods
  void processEncoderInput();
  void handleMotorControl();

public:
  BonexMotorController(
      logging::Logger *logger,
      ODrive *odrive,
      EMIFilterSwitch *trigger_switch,
      EMIFilterSwitch *reverse_switch,
      RotationEncoder *encoder);

  void setup(uint8_t axis_id, float max_torque, int16_t min_velocity, int16_t max_velocity, int16_t startup_velocity);
  void loop();
};

#endif // BONEX_MOTOR_CONTROLLER_H