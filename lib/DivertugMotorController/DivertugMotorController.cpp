#include "DivertugMotorController.h"

DivertugMotorController::DivertugMotorController(
    logging::Logger *logger,
    ODrive *odrive,
    EMIFilterSwitch *trigger_switch) : logger(logger), odrive(odrive), trigger_switch(trigger_switch), axis_id(0), max_torque(0),
                                       min_velocity(0), max_velocity(0), velocity_setpoint(0), direction_locked(false),
                                       locked_reverse_mode(false), forward_direction_multiplier(1)
{
  if (logger)
  {
    logger->log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "DivertugMotorController", "Initialized");
  }
}

void DivertugMotorController::setup(const MotorControllerConfig &config)
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
    logger->log(logging::LoggerLevel::LOGGER_LEVEL_INFO, "DivertugMotorController", "Setup complete - axis: %d, max_torque: %.2f, forward_velocity_range: %d-%d, reverse_velocity_range: %d-%d, forward_direction_multiplier: %d",
                config.axis_id, config.max_torque, config.min_velocity, config.max_velocity, config.min_reverse_velocity, config.max_reverse_velocity,
                config.forward_direction_multiplier);
  }
}

void DivertugMotorController::loop()
{
  // Update switch states
  trigger_switch->loop();

  handleMotorControl();
}

// Private helper method for converting encoder angle delta to velocity delta
int16_t DivertugMotorController::fromEncoderAngleDeltaToVelocityDelta(int16_t angle_delta)
{
  // Convert angle delta to velocity using the same logic as before
  int16_t abs_angle = abs(angle_delta);
  int16_t mapped_velocity = map(abs_angle, 0, 360, 0, max_velocity);

  // Apply direction
  return (angle_delta < 0) ? -mapped_velocity : mapped_velocity;
}

int16_t DivertugMotorController::applyVelocityConstraints(int16_t velocity, bool is_reverse_mode)
{
  if (is_reverse_mode)
  {
    return constrain(velocity, -max_reverse_velocity, -min_reverse_velocity);
  }

  return constrain(velocity, min_velocity, max_velocity);
}

void DivertugMotorController::handleMotorControl()
{
  bool trigger_pressed = trigger_switch->isPressed();
  bool trigger_changed = trigger_switch->hasChanged();

  // Handle motor control
  if (trigger_changed && trigger_pressed)
  {
    odrive->setVelocity(
        axis_id,
        velocity_setpoint * forward_direction_multiplier,
        max_torque);
  }

  if (trigger_changed && !trigger_pressed)
  {
    odrive->setVelocity(axis_id, 0, max_torque);
  }
}
