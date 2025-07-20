#include "BonexMotorController.h"

BonexMotorController::BonexMotorController(
    logging::Logger *logger,
    ODrive *odrive,
    EMIFilterSwitch *trigger_switch,
    EMIFilterSwitch *reverse_switch,
    RotationEncoder *encoder) : logger(logger), odrive(odrive), trigger_switch(trigger_switch),
                                reverse_switch(reverse_switch), encoder(encoder), axis_id(0), max_torque(0),
                                min_velocity(0), max_velocity(0), velocity_setpoint(0), velocity_changed(false)
{
  if (logger)
  {
    logger->log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "BonexMotorController", "Initialized");
  }
}

void BonexMotorController::setup(uint8_t axis_id, float max_torque, int16_t min_velocity, int16_t max_velocity, int16_t startup_velocity)
{
  this->axis_id = axis_id;
  this->max_torque = max_torque;
  this->min_velocity = min_velocity;
  this->max_velocity = max_velocity;
  this->velocity_setpoint = startup_velocity;

  if (logger)
  {
    logger->log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "BonexMotorController", "Setup complete - axis: %d, max_torque: %.2f, velocity_range: %d-%d",
                axis_id, max_torque, min_velocity, max_velocity);
  }
}

void BonexMotorController::loop()
{
  // Update switch states
  trigger_switch->loop();
  reverse_switch->loop();

  // Update encoder state
  encoder->loop();

  processEncoderInput();
  handleMotorControl();
}

// Private helper method for converting encoder angle delta to velocity delta
int16_t BonexMotorController::fromEncoderAngleDeltaToVelocityDelta(int16_t angle_delta)
{
  // Convert angle delta to velocity using the same logic as before
  int16_t abs_angle = abs(angle_delta);
  int16_t mapped_velocity = map(abs_angle, 0, 360, 0, max_velocity);

  // Apply direction
  return (angle_delta < 0) ? -mapped_velocity : mapped_velocity;
}

void BonexMotorController::processEncoderInput()
{
  // Get encoder delta and convert to velocity delta
  int16_t encoder_rotation_angle_delta = encoder->getAngleDelta();
  int16_t velocity_delta = fromEncoderAngleDeltaToVelocityDelta(encoder_rotation_angle_delta);

  // Update velocity setpoint
  velocity_setpoint += velocity_delta;
  velocity_setpoint = constrain(velocity_setpoint, min_velocity, max_velocity);

  // Track if velocity changed
  velocity_changed = (velocity_delta != 0);
}

void BonexMotorController::handleMotorControl()
{
  bool trigger_pressed = trigger_switch->isPressed();
  bool trigger_changed = trigger_switch->hasChanged();

  // Handle motor control
  if ((trigger_changed || velocity_changed) && trigger_pressed)
  {
    // Apply reverse direction if reverse switch is pressed
    int16_t final_velocity = reverse_switch->isPressed() ? -velocity_setpoint : velocity_setpoint;
    odrive->setVelocity(axis_id, final_velocity, max_torque);
  }

  if (trigger_changed && !trigger_pressed)
  {
    odrive->setVelocity(axis_id, 0, max_torque);
  }
}
