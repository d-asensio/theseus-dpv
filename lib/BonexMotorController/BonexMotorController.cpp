#include "BonexMotorController.h"

BonexMotorController::BonexMotorController(
    logging::Logger *logger,
    ODrive *odrive,
    EMIFilterSwitch *trigger_switch,
    EMIFilterSwitch *reverse_switch,
    RotationEncoder *encoder) : logger(logger), odrive(odrive), trigger_switch(trigger_switch),
                                reverse_switch(reverse_switch), encoder(encoder), axis_id(0), max_torque(0),
                                min_velocity(0), max_velocity(0), velocity_setpoint(0), direction_locked(false),
                                locked_reverse_mode(false), forward_direction_multiplier(1)
{
  if (logger)
  {
    logger->log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "BonexMotorController", "Initialized");
  }
}

void BonexMotorController::setup(const MotorControllerConfig &config)
{
  this->axis_id = config.axis_id;
  this->max_torque = config.max_torque;
  this->min_velocity = config.min_velocity;
  this->max_velocity = config.max_velocity;
  this->min_reverse_velocity = config.min_reverse_velocity;
  this->max_reverse_velocity = config.max_reverse_velocity;
  this->velocity_setpoint = config.startup_velocity;
  this->forward_direction_multiplier = config.forward_direction_multiplier;

  if (logger)
  {
    logger->log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "BonexMotorController", "Setup complete - axis: %d, max_torque: %.2f, forward_velocity_range: %d-%d, reverse_velocity_range: %d-%d, forward_direction_multiplier: %d",
                config.axis_id, config.max_torque, config.min_velocity, config.max_velocity, config.min_reverse_velocity, config.max_reverse_velocity,
                config.forward_direction_multiplier);
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

int16_t BonexMotorController::applyVelocityConstraints(int16_t velocity, bool is_reverse_mode)
{
  if (is_reverse_mode)
  {
    return constrain(velocity, -max_reverse_velocity, -min_reverse_velocity);
  }

  return constrain(velocity, min_velocity, max_velocity);
}

void BonexMotorController::processEncoderInput()
{
  // Get encoder delta and convert to velocity delta
  int16_t encoder_rotation_angle_delta = encoder->getAngleDelta();
  int16_t velocity_delta = fromEncoderAngleDeltaToVelocityDelta(encoder_rotation_angle_delta);

  // Check if reverse switch state changed
  bool reverse_changed = reverse_switch->hasChanged();
  bool is_reverse = reverse_switch->isPressed();

  // Only allow direction changes if trigger is not pressed (direction not locked)
  if (reverse_changed && velocity_setpoint != 0 && !direction_locked)
  {
    velocity_setpoint = -velocity_setpoint;
  }

  // Use locked direction mode if trigger is pressed, otherwise use current reverse switch state
  bool effective_reverse_mode = direction_locked ? locked_reverse_mode : is_reverse;

  // Update velocity setpoint based on effective reverse mode
  if (effective_reverse_mode)
  {
    velocity_setpoint -= velocity_delta;
  }
  else
  {
    velocity_setpoint += velocity_delta;
  }

  // Apply constraints based on effective reverse mode, not velocity sign
  velocity_setpoint = applyVelocityConstraints(velocity_setpoint, effective_reverse_mode);
}

void BonexMotorController::handleMotorControl()
{
  bool trigger_pressed = trigger_switch->isPressed();
  bool trigger_changed = trigger_switch->hasChanged();
  bool encoder_changed = encoder->hasChanged();

  // Handle motor control
  if ((trigger_changed || encoder_changed) && trigger_pressed)
  {
    // Lock the direction when trigger is first pressed
    if (trigger_changed)
    {
      direction_locked = true;
      locked_reverse_mode = reverse_switch->isPressed();
    }

    odrive->setVelocity(
        axis_id,
        velocity_setpoint * forward_direction_multiplier,
        max_torque);
  }

  if (trigger_changed && !trigger_pressed)
  {
    // Unlock direction when trigger is released
    direction_locked = false;
    odrive->setVelocity(axis_id, 0, max_torque);
  }
}
