#include <HardwareSerial.h>
#include <logger.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <HardwareSerial.h>
#include <logger.h>

#include "pitches.h"

#include "EMIFilterSwitch.h"
#include "ODrive.h"
#include "DivertugMotorController.h"

#define ODRIVE_RX_PIN 20
#define ODRIVE_TX_PIN 21
#define TRIGGER_SWITCH_PIN 0
#define BUZZER_PIN 1
#define LED_PIN 2

// Motor controller configuration constants
#define MOTOR_AXIS_ID 0
#define MOTOR_MAX_TORQUE 1.59f
#define MOTOR_MIN_VELOCITY 10
#define MOTOR_MAX_VELOCITY 30
#define MOTOR_MIN_REVERSE_VELOCITY 8
#define MOTOR_MAX_REVERSE_VELOCITY 20
#define MOTOR_STARTUP_VELOCITY 25
#define MOTOR_FORWARD_DIRECTION_MULTIPLIER -1

// EMI filtering and debouncing constants
#define TRIGGER_SWITCH_DEBOUNCE_TIME 50
#define TRIGGER_SWITCH_FILTER_SAMPLES 10

#define REVERSE_SWITCH_DEBOUNCE_TIME 50
#define REVERSE_SWITCH_FILTER_SAMPLES 10

HardwareSerial ODriveSerial(1);

// Create logger instance
logging::Logger logger;

// Create components
ODrive odrive(&logger, &ODriveSerial);

EMIFilterSwitch trigger_switch({TRIGGER_SWITCH_DEBOUNCE_TIME,
                                TRIGGER_SWITCH_FILTER_SAMPLES});

// Create motor controller
DivertugMotorController motor_controller(&logger, &odrive, &trigger_switch);

Adafruit_NeoPixel strip(1, LED_PIN, NEO_GRB + NEO_KHZ800);

int melody[] = {
  NOTE_E5, NOTE_E5, REST, NOTE_E5, REST, NOTE_C5, NOTE_E5,
  NOTE_G5, REST, NOTE_G4
};

int durations[] = {
  8, 8, 8, 8, 8, 8, 8,
  4, 4, 8
};

void playSuperMario()
{
  int size = sizeof(durations) / sizeof(int);

  for (int note = 0; note < size; note++) {
    int duration = 1000 / durations[note];
    tone(BUZZER_PIN, melody[note], duration);

    int pauseBetweenNotes = duration * 1.30;
    delay(pauseBetweenNotes);
  
    noTone(BUZZER_PIN);
  }
}

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

  // ---- LED ----
  strip.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();  // Turn OFF all pixels ASAP
  strip.clear();
  strip.setBrightness(255); // Set BRIGHTNESS to about 1/5 (max = 255)

  strip.setPixelColor(0, strip.Color(255, 255, 0));

  strip.show();

  // ---- BUZZER ----
  pinMode(BUZZER_PIN, OUTPUT);
  playSuperMario();

  // Initialize logger
  logger.setSerial(&Serial);

  trigger_switch.setup(TRIGGER_SWITCH_PIN, INPUT_PULLDOWN);

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
