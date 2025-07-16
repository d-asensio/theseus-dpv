#include <Arduino.h>
#include <HardwareSerial.h>
#include <Adafruit_ADS1X15.h>

#define LED_BUILTIN         8
#define ODRIVE_RX_PIN       20
#define ODRIVE_TX_PIN       21
#define TRIGGER_SWITCH_PIN  10
#define REVERSE_SWITCH_PIN  7

#define ENCODER_CALIBRATION_MIN_VALUE 198
#define ENCODER_CALIBRATION_MAX_VALUE 27895
#define ENCODER_DIRECTION -1
#define ENCODER_DELTA_MIN_RESOLTUION 10

#define BLCD_ODRIVE_AXIS 0
#define BLCD_MAX_TORQUE 1.59
#define BLCD_MIN_VELOCITY 8
#define BLCD_STARTUP_VELOCITY 30
#define BLCD_MAX_VELOCITY 60

HardwareSerial ODriveSerial(1);
Adafruit_ADS1115 ads;

int16_t blcd_velocity_setpoint = BLCD_STARTUP_VELOCITY;
int16_t current_encoder_angle = 0;

void sendVelocity(int16_t velocity, float torqueFF = 0.0f)
{
  /* Build the line:
     "v <axis> <velocity> <torqueFF>\n"
     dtostrf() converts the float → char[] with fixed width/precision
  */
  char vBuf[12], tBuf[12];                       // buffers for the numbers
  dtostrf(velocity, 0, 4, vBuf);                 // 4 decimals
  dtostrf(torqueFF, 0, 4, tBuf);

  ODriveSerial.print("v ");
  ODriveSerial.print(BLCD_ODRIVE_AXIS);
  ODriveSerial.print(' ');
  ODriveSerial.print(vBuf);
  ODriveSerial.print(' ');
  ODriveSerial.print(tBuf);
  ODriveSerial.print('\n');
}

int16_t read_encoder_rotation_angle () {
  int16_t analog_value = ads.readADC_SingleEnded(0);

  return map(
    analog_value,
    ENCODER_CALIBRATION_MIN_VALUE,
    ENCODER_CALIBRATION_MAX_VALUE,
    0,
    360
  );
}


int16_t get_angle_delta (int16_t current_angle, int16_t new_angle) {
  if (current_angle <= 180 && new_angle >= 180) {
    return 0;
  }

  if (current_angle >= 180 && new_angle <= 180) {
    return 0;
  }

  int16_t encoder_rotation_angle_delta = (current_angle - new_angle) * ENCODER_DIRECTION;

  if (abs(encoder_rotation_angle_delta) < ENCODER_DELTA_MIN_RESOLTUION) {
    encoder_rotation_angle_delta = 0;
  }

  return encoder_rotation_angle_delta;
}

void setup()
{
  Serial.begin(115200);                          // USB-CDC
  ODriveSerial.begin(115200, SERIAL_8N1, ODRIVE_RX_PIN, ODRIVE_TX_PIN);

  // Wait until USB serial is initialized
  while (!Serial) { }

  // Wait until ODrive serial is initialized
  while (!ODriveSerial) { }

  // Check ADS initialization
  if (!ads.begin()) {
    while (true) {
      Serial.println("Failed to initialize ADS.");
      delay(1000);
    };
  }

  pinMode(TRIGGER_SWITCH_PIN, INPUT_PULLDOWN);
  pinMode(REVERSE_SWITCH_PIN, INPUT_PULLDOWN);

  current_encoder_angle = read_encoder_rotation_angle();
}

void loop()
{
  // Set motor speed
  int16_t targetRps = 20;
  sendVelocity(targetRps, BLCD_MAX_TORQUE);


  // Read encoder value
  int16_t new_encoder_angle = read_encoder_rotation_angle();

  // When transition from x<=90 to >=270
  // When transition from x>=270 to x<=90
  
  int16_t encoder_rotation_angle_delta = get_angle_delta(current_encoder_angle, new_encoder_angle);

  current_encoder_angle = new_encoder_angle;

  Serial.println("-- Encoder --");
  Serial.print("Rotation angle: ");
  Serial.println(new_encoder_angle);
  Serial.print("Delta: ");
  Serial.println(encoder_rotation_angle_delta);
  Serial.println("-------------");

  const int triggerPressed = digitalRead(TRIGGER_SWITCH_PIN);
  Serial.print("Trigger: ");
  Serial.println(triggerPressed);

  const int reversePressed = digitalRead(REVERSE_SWITCH_PIN);
  Serial.print("Reverse: ");
  Serial.println(reversePressed);

  delay(300);

  // while (ODriveSerial.available())
  //   Serial.write(ODriveSerial.read());

  // Serial.println();
}
