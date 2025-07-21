#include <Arduino.h>
#include <HardwareSerial.h>
#include <Adafruit_ADS1X15.h>
#include <logger.h>

#include "EMIFilterSwitch.h"
#include "RotationEncoder.h"
#include "ODrive.h"
#include "BonexMotorController.h"

#define LED_BUILTIN 8
#define ODRIVE_RX_PIN 20
#define ODRIVE_TX_PIN 21
#define TRIGGER_SWITCH_PIN 10
#define REVERSE_SWITCH_PIN 7

#define ENCODER_CALIBRATION_MIN_VALUE 198
#define ENCODER_CALIBRATION_MAX_VALUE 27895
#define ENCODER_DIRECTION 1
#define ENCODER_FULL_RANGE_ANGLE 360

// Motor controller configuration constants
#define MOTOR_AXIS_ID 0
#define MOTOR_MAX_TORQUE 1.59f
#define MOTOR_MIN_VELOCITY 10
#define MOTOR_MAX_VELOCITY 50
#define MOTOR_MIN_REVERSE_VELOCITY 8
#define MOTOR_MAX_REVERSE_VELOCITY 20
#define MOTOR_STARTUP_VELOCITY 30
#define MOTOR_FORWARD_DIRECTION_MULTIPLIER -1

// EMI filtering and debouncing constants
#define TRIGGER_SWITCH_DEBOUNCE_TIME 50
#define TRIGGER_SWITCH_FILTER_SAMPLES 10

#define REVERSE_SWITCH_DEBOUNCE_TIME 50
#define REVERSE_SWITCH_FILTER_SAMPLES 10

HardwareSerial ODriveSerial(1);
Adafruit_ADS1115 ads;

// Create logger instance
logging::Logger logger;

// Create components
ODrive odrive(&logger, &ODriveSerial);

EMIFilterSwitch trigger_switch({TRIGGER_SWITCH_DEBOUNCE_TIME,
                                TRIGGER_SWITCH_FILTER_SAMPLES});

EMIFilterSwitch reverse_switch({REVERSE_SWITCH_DEBOUNCE_TIME,
                                REVERSE_SWITCH_FILTER_SAMPLES});

RotationEncoder rotation_encoder(&logger, &ads);

// Create motor controller
BonexMotorController motor_controller(&logger, &odrive, &trigger_switch, &reverse_switch, &rotation_encoder);

void setup()
{
  Serial.begin(115200);
  ODriveSerial.begin(115200, SERIAL_8N1, ODRIVE_RX_PIN, ODRIVE_TX_PIN);

  // Wait until USB serial is initialized
  while (!Serial)
  {
  }

  // Wait until ODrive serial is initialized
  while (!ODriveSerial)
  {
  }

  // Initialize logger
  logger.setSerial(&Serial);

  // Check ADS initialization
  if (!ads.begin())
  {
    while (true)
    {
      logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "ADS", "Failed to initialize ADS.");
      delay(1000);
    };
  }

  trigger_switch.setup(TRIGGER_SWITCH_PIN, INPUT_PULLDOWN);
  reverse_switch.setup(REVERSE_SWITCH_PIN, INPUT_PULLDOWN);

  rotation_encoder.setup({ENCODER_CALIBRATION_MIN_VALUE,
                          ENCODER_CALIBRATION_MAX_VALUE,
                          ENCODER_DIRECTION});

  motor_controller.setup({MOTOR_AXIS_ID,
                          MOTOR_MAX_TORQUE,
                          MOTOR_MIN_VELOCITY,
                          MOTOR_MAX_VELOCITY,
                          MOTOR_MIN_REVERSE_VELOCITY,
                          MOTOR_MAX_REVERSE_VELOCITY,
                          MOTOR_STARTUP_VELOCITY,
                          MOTOR_FORWARD_DIRECTION_MULTIPLIER});
}

void loop()
{
  motor_controller.loop();
  delay(10);
}
