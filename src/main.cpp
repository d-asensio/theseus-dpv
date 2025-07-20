#include <Arduino.h>
#include <HardwareSerial.h>
#include <Adafruit_ADS1X15.h>
#include <EMIFilterSwitch.h>

#define LED_BUILTIN 8
#define ODRIVE_RX_PIN 20
#define ODRIVE_TX_PIN 21
#define TRIGGER_SWITCH_PIN 10
#define REVERSE_SWITCH_PIN 7

#define ENCODER_CALIBRATION_MIN_VALUE 198
#define ENCODER_CALIBRATION_MAX_VALUE 27895
#define ENCODER_DIRECTION -1
#define ENCODER_FULL_RANGE_ANGLE 360

#define BLCD_ODRIVE_AXIS 0
#define BLCD_MAX_TORQUE 1.59
#define BLCD_MIN_VELOCITY 8
#define BLCD_STARTUP_VELOCITY 30
#define BLCD_MAX_VELOCITY 50

// EMI filtering and debouncing constants
#define TRIGGER_SWITCH_DEBOUNCE_TIME 50
#define TRIGGER_SWITCH_FILTER_SAMPLES 5
#define TRIGGER_SWITCH_THRESHOLD 3

#define REVERSE_SWITCH_DEBOUNCE_TIME 50
#define REVERSE_SWITCH_FILTER_SAMPLES 10
#define REVERSE_SWITCH_THRESHOLD 5

HardwareSerial ODriveSerial(1);
Adafruit_ADS1115 ads;

int16_t blcd_velocity_setpoint = BLCD_STARTUP_VELOCITY;
int16_t current_encoder_angle;

// Create switch instances
EMIFilterSwitch trigger_switch(TRIGGER_SWITCH_DEBOUNCE_TIME, TRIGGER_SWITCH_FILTER_SAMPLES, TRIGGER_SWITCH_THRESHOLD);
EMIFilterSwitch reverse_switch(REVERSE_SWITCH_DEBOUNCE_TIME, REVERSE_SWITCH_FILTER_SAMPLES, REVERSE_SWITCH_THRESHOLD);

void send_velocity_to_odrive(int16_t velocity, float torqueFF = 0.0f)
{
  /* Build the line:
     "v <axis> <velocity> <torqueFF>\n"
     dtostrf() converts the float → char[] with fixed width/precision
  */
  char tBuf[12];
  dtostrf(torqueFF, 0, 4, tBuf);

  ODriveSerial.print("v ");
  ODriveSerial.print(BLCD_ODRIVE_AXIS);
  ODriveSerial.print(' ');
  ODriveSerial.print(velocity);
  ODriveSerial.print(' ');
  ODriveSerial.print(tBuf);
  ODriveSerial.print('\n');
}

int16_t read_encoder_rotation_angle()
{
  int16_t analog_value = ads.readADC_SingleEnded(0);

  return map(
      analog_value,
      ENCODER_CALIBRATION_MIN_VALUE,
      ENCODER_CALIBRATION_MAX_VALUE,
      0,
      360);
}

int16_t get_angle_delta(int16_t current_angle, int16_t new_angle)
{
  // Calculate the raw difference
  int16_t raw_delta = new_angle - current_angle;
  
  // Handle wrapping by finding the shortest path
  // The shortest angular distance between two angles on a circle
  // is always <= 180 degrees in either direction
  
  if (raw_delta > 180) {
    // Wrapped clockwise: subtract 360 to get the shorter path
    raw_delta -= 360;
  } else if (raw_delta < -180) {
    // Wrapped counter-clockwise: add 360 to get the shorter path
    raw_delta += 360;
  }
  
  // Apply encoder direction
  int16_t encoder_rotation_angle_delta = raw_delta * ENCODER_DIRECTION;

  return encoder_rotation_angle_delta;
}

int16_t from_encoder_angle_to_blcd_velocity(int16_t angle)
{
  return map(
      angle,
      0,
      ENCODER_FULL_RANGE_ANGLE,
      0,
      BLCD_MAX_VELOCITY);
}

int16_t from_encoder_angle_delta_to_blcd_velocity_delta(int16_t angle_delta)
{
  int16_t mult = 1;
  if (angle_delta < 0)
  {
    mult = -1;
  }

  return from_encoder_angle_to_blcd_velocity(abs(angle_delta)) * mult;
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

  // Check ADS initialization
  if (!ads.begin())
  {
    while (true)
    {
      // Serial.println("Failed to initialize ADS.");
      delay(1000);
    };
  }

  trigger_switch.setup(TRIGGER_SWITCH_PIN, INPUT_PULLDOWN);
  reverse_switch.setup(REVERSE_SWITCH_PIN, INPUT_PULLDOWN);

  current_encoder_angle = read_encoder_rotation_angle();
}

void loop()
{
  // Update switch states
  trigger_switch.loop();
  reverse_switch.loop();

  int16_t new_encoder_angle = read_encoder_rotation_angle();
  int16_t encoder_rotation_angle_delta = get_angle_delta(current_encoder_angle, new_encoder_angle);
  int16_t blcd_velocity_delta = from_encoder_angle_delta_to_blcd_velocity_delta(encoder_rotation_angle_delta);

  Serial.println(encoder_rotation_angle_delta);

  blcd_velocity_setpoint += blcd_velocity_delta;
  blcd_velocity_setpoint = constrain(blcd_velocity_setpoint, BLCD_MIN_VELOCITY, BLCD_MAX_VELOCITY);
  bool blcd_velocity_delta_changed = blcd_velocity_delta != 0;


  // Get current filtered states
  int trigger_pressed = trigger_switch.read();
  int reverse_pressed = reverse_switch.read();

  // Check for state changes
  bool trigger_changed = trigger_switch.hasChanged();
  bool reverse_changed = reverse_switch.hasChanged();

  if (trigger_changed || blcd_velocity_delta_changed)
  {
    if (trigger_pressed == HIGH)
    {
      // Apply reverse direction if reverse switch is pressed
      int16_t final_velocity = reverse_pressed == HIGH ? -blcd_velocity_setpoint : blcd_velocity_setpoint;
      send_velocity_to_odrive(final_velocity, BLCD_MAX_TORQUE);
    }
  }

  if (trigger_changed && trigger_pressed == LOW)
  {
    send_velocity_to_odrive(0, BLCD_MAX_TORQUE);
  }

  current_encoder_angle = new_encoder_angle;

  delay(50);
}
