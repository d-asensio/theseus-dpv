#include <Arduino.h>
#include <HardwareSerial.h>
#include <Adafruit_ADS1X15.h>

#define LED_BUILTIN         8
#define ODRIVE_RX_PIN       20
#define ODRIVE_TX_PIN       21
#define TRIGGER_SWITCH_PIN  10
#define REVERSE_SWITCH_PIN  7

static const int  kAxis = 0;

HardwareSerial ODriveSerial(1);
Adafruit_ADS1115 ads;

void sendVelocity(float velocity, float torqueFF = 0.0f)
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

void loop()
{
  // Set motor speed
  float targetRps = 20;
  sendVelocity(targetRps, 1.59);


  // Read encoder value

  float multiplier = 0.1875F; /* ADS1115  @ +/- 6.144V gain (16-bit results) */

  int16_t results = ads.readADC_SingleEnded(0);

  Serial.print("Differential: "); Serial.print(results); Serial.print("("); Serial.print(results * multiplier); Serial.println("mV)");


  const int triggerPressed = digitalRead(TRIGGER_SWITCH_PIN);
  Serial.print("Trigger: ");
  Serial.println(triggerPressed);

  const int reversePressed = digitalRead(REVERSE_SWITCH_PIN);
  Serial.print("Reverse: ");
  Serial.println(reversePressed);

  delay(1000);

  // while (ODriveSerial.available())
  //   Serial.write(ODriveSerial.read());

  // Serial.println();
}
