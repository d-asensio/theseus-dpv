#ifndef BONEX_MOTOR_CONTROLLER_H
#define BONEX_MOTOR_CONTROLLER_H

#include <Arduino.h>
#include "logger.h"
#include "ODrive.h"
#include "EMIFilterSwitch.h"
#include "RotationEncoder.h"

struct MotorControllerConfig
{
  uint8_t axis_id;
  float max_torque;
  int16_t min_velocity;
  int16_t max_velocity;
  int16_t min_reverse_velocity;
  int16_t max_reverse_velocity;
  int16_t startup_velocity;
};

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
  int16_t min_reverse_velocity;
  int16_t max_reverse_velocity;

  // State
  int16_t velocity_setpoint;

private:
  // Helper methods
  int16_t fromEncoderAngleDeltaToVelocityDelta(int16_t angle_delta);
  int16_t applyVelocityConstraints(int16_t velocity, bool is_reverse_mode);

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

  void setup(const MotorControllerConfig &config);
  void loop();
};

#endif // BONEX_MOTOR_CONTROLLER_H