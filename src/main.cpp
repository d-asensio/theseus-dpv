#include <Arduino.h>
#include <HardwareSerial.h>
#include <Adafruit_ADS1X15.h>

#define LED_BUILTIN 8
#define ODRIVE_RX_PIN 20
#define ODRIVE_TX_PIN 21
#define TRIGGER_SWITCH_PIN 10
#define REVERSE_SWITCH_PIN 7

#define ENCODER_CALIBRATION_MIN_VALUE 198
#define ENCODER_CALIBRATION_MAX_VALUE 27895
#define ENCODER_DIRECTION -1
#define ENCODER_DELTA_MIN_RESOLTUION 10
#define ENCODER_FULL_RANGE_ANGLE 360

#define BLCD_ODRIVE_AXIS 0
#define BLCD_MAX_TORQUE 1.59
#define BLCD_MIN_VELOCITY 8
#define BLCD_STARTUP_VELOCITY 30
#define BLCD_MAX_VELOCITY 60

HardwareSerial ODriveSerial(1);
Adafruit_ADS1115 ads;

int16_t blcd_velocity_setpoint = BLCD_STARTUP_VELOCITY;
int16_t current_encoder_angle;
int current_trigger_pressed;

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
  if (current_angle <= 180 && new_angle >= 180)
  {
    return 0;
  }

  if (current_angle >= 180 && new_angle <= 180)
  {
    return 0;
  }

  int16_t encoder_rotation_angle_delta = (current_angle - new_angle) * ENCODER_DIRECTION;

  if (abs(encoder_rotation_angle_delta) < ENCODER_DELTA_MIN_RESOLTUION)
  {
    encoder_rotation_angle_delta = 0;
  }

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
  // Serial.begin(115200);
  ODriveSerial.begin(115200, SERIAL_8N1, ODRIVE_RX_PIN, ODRIVE_TX_PIN);

  // Wait until USB serial is initialized
  // while (!Serial)
  // {
  // }

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

  pinMode(TRIGGER_SWITCH_PIN, INPUT_PULLDOWN);
  pinMode(REVERSE_SWITCH_PIN, INPUT_PULLDOWN);

  current_encoder_angle = read_encoder_rotation_angle();
  current_trigger_pressed = digitalRead(TRIGGER_SWITCH_PIN);
}

void loop()
{
  int16_t new_encoder_angle = read_encoder_rotation_angle();
  int16_t encoder_rotation_angle_delta = get_angle_delta(current_encoder_angle, new_encoder_angle);
  int16_t blcd_velocity_delta = from_encoder_angle_delta_to_blcd_velocity_delta(encoder_rotation_angle_delta);

  blcd_velocity_setpoint += blcd_velocity_delta;
  blcd_velocity_setpoint = constrain(blcd_velocity_setpoint, BLCD_MIN_VELOCITY, BLCD_MAX_VELOCITY);

  int trigger_pressed = digitalRead(TRIGGER_SWITCH_PIN);
  int reverse_pressed = digitalRead(REVERSE_SWITCH_PIN);

  bool trigger_changed = current_trigger_pressed != trigger_pressed;
  bool blcd_velocity_delta_changed = blcd_velocity_delta != 0;

  if (trigger_changed || blcd_velocity_delta_changed)
  {
    if (trigger_pressed == HIGH)
    {
      send_velocity_to_odrive(blcd_velocity_setpoint, BLCD_MAX_TORQUE);
    }
  }

  if (trigger_changed && trigger_pressed == LOW)
  {
    send_velocity_to_odrive(0, BLCD_MAX_TORQUE);
  }

  current_trigger_pressed = trigger_pressed;
  current_encoder_angle = new_encoder_angle;

  delay(50);
}
