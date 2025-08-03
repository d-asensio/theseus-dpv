#ifndef DIVERTUG_MOTOR_CONTROLLER_H
#define DIVERTUG_MOTOR_CONTROLLER_H

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
  int8_t forward_direction_multiplier;
};

class DivertugMotorController
{
private:
  logging::Logger *logger;
  ODrive *odrive;
  EMIFilterSwitch *trigger_switch;

  // Configuration (needed across multiple loop calls)
  uint8_t axis_id;
  float max_torque;
  int16_t min_velocity;
  int16_t max_velocity;
  int16_t min_reverse_velocity;
  int16_t max_reverse_velocity;
  int8_t forward_direction_multiplier;

  // State
  int16_t velocity_setpoint;
  bool direction_locked;    // Prevents direction changes while trigger is pressed
  bool locked_reverse_mode; // Stores the reverse mode state when trigger was pressed

private:
  // Helper methods
  int16_t fromEncoderAngleDeltaToVelocityDelta(int16_t angle_delta);
  int16_t applyVelocityConstraints(int16_t velocity, bool is_reverse_mode);

  // Core processing methods
  void processEncoderInput();
  void handleMotorControl();

public:
  DivertugMotorController(
      logging::Logger *logger,
      ODrive *odrive,
      EMIFilterSwitch *trigger_switch);

  void setup(const MotorControllerConfig &config);
  void loop();
};

#endif // DIVERTUG_MOTOR_CONTROLLER_H