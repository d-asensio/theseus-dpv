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

#define BLCD_MAX_TORQUE 1.59

static const int  kAxis = 0;

HardwareSerial ODriveSerial(1);
Adafruit_ADS1115 ads;

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
  ODriveSerial.print(kAxis);
  ODriveSerial.print(' ');
  ODriveSerial.print(vBuf);
  ODriveSerial.print(' ');
  ODriveSerial.print(tBuf);
  ODriveSerial.print('\n');
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

void loop()
{
  // Set motor speed
  int16_t targetRps = 20;
  sendVelocity(targetRps, BLCD_MAX_TORQUE);


  // Read encoder value
  int16_t current_encoder_rotation_angle = read_encoder_rotation_angle();

  Serial.print("Encoder rotation angle: ");
  Serial.println(current_encoder_rotation_angle);

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
