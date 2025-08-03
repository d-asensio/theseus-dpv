#include <Arduino.h>

#include <SPI.h>
#include <SD.h>
#include <Adafruit_NeoPixel.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <RTClib.h>
#include <HardwareSerial.h>
#include <logger.h>

#include "pitches.h"
#include "ODrive.h"

#define SD_MOSI 5
#define SD_MISO 6
#define SD_SCK 4
#define SD_CS 7

#define BUZZER_PIN 1
#define LED_PIN 2
#define ONE_WIRE_BUS_PIN 3
#define TRIGGER_SWITCH_PIN 0

#define ODRIVE_RX_PIN 20
#define ODRIVE_TX_PIN 21

logging::Logger logger;

Adafruit_NeoPixel strip(1, LED_PIN, NEO_GRB + NEO_KHZ800);
OneWire ourWire(ONE_WIRE_BUS_PIN);
DallasTemperature sensors(&ourWire);
RTC_DS3231 rtc;
HardwareSerial ODriveSerial(1);
ODrive odrive(&logger, &ODriveSerial);

File dataFile;
unsigned long dataCount = 0;

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
    //to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int duration = 1000 / durations[note];
    tone(BUZZER_PIN, melody[note], duration);

    //to distinguish the notes, set a minimum time between them.
    //the note's duration + 30% seems to work well:
    int pauseBetweenNotes = duration * 1.30;
    delay(pauseBetweenNotes);
    
    //stop the tone playing:
    noTone(BUZZER_PIN);
  }
}

void setup()
{
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000); // Wait for serial connection

  Serial.println("ESP32-C3 Super Mini SD Card Test");

  // Configure SPI for SD card
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI);

  // Initialize SD card
  if (!SD.begin(SD_CS))
  {
    Serial.println("SD Card initialization failed!");
    return;
  }

  Serial.println("SD Card initialized successfully.");

  // Create or open a file
  dataFile = SD.open("/data_log.txt", FILE_WRITE);

  if (!dataFile)
  {
    Serial.println("Error opening file!");
    return;
  }

  // Write header to file
  dataFile.println("ESP32-C3 Super Mini Data Log");
  dataFile.println("===========================");
  dataFile.println("Count, Timestamp, Value1, Value2");
  dataFile.flush();

  Serial.println("File opened and header written.");

  // ---- LED ----
  strip.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();  // Turn OFF all pixels ASAP
  strip.clear();
  strip.setBrightness(255); // Set BRIGHTNESS to about 1/5 (max = 255)

  strip.setPixelColor(0, strip.Color(255, 255, 0));

  strip.show();

  // ---- Temp sensor motor ----
  sensors.begin();
  sensors.requestTemperatures();

  // ---- Trigger ----
  pinMode(TRIGGER_SWITCH_PIN, INPUT);

  // ---- Clock ----
  if (!rtc.begin())
  {
    Serial.println(F("Couldn't find RTC"));
    while (1)
      ;
  }

  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Odrive
  ODriveSerial.begin(115200, SERIAL_8N1, ODRIVE_RX_PIN, ODRIVE_TX_PIN);

  while (!ODriveSerial)
  {
  }

  // Initialize logger
  logger.setSerial(&Serial);

  // ---- BUZZER ----
  pinMode(BUZZER_PIN, OUTPUT);
  playSuperMario();
}

String daysOfTheWeek[7] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };
String monthsNames[12] = { "January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December" };

void printDate(DateTime date)
{
  Serial.print(date.year(), DEC);
  Serial.print('/');
  Serial.print(date.month(), DEC);
  Serial.print('/');
  Serial.print(date.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[date.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(date.hour(), DEC);
  Serial.print(':');
  Serial.print(date.minute(), DEC);
  Serial.print(':');
  Serial.print(date.second(), DEC);
  Serial.println();
}

void loop()
{
  // // ---- SD Card write sample ----
  // // Generate some sample data
  // dataCount++;
  // unsigned long timestamp = millis();
  // float value1 = random(0, 100) / 10.0;
  // int value2 = random(0, 1000);

  // // Create data string
  // String dataString = String(dataCount) + ", " +
  //                     String(timestamp) + ", " +
  //                     String(value1) + ", " +
  //                     String(value2);

  // // Write to SD card
  // if (dataFile)
  // {
  //   dataFile.println(dataString);
  //   dataFile.flush(); // Make sure data is written to the file

  //   // Print to serial monitor
  //   Serial.print("Data written: ");
  //   Serial.println(dataString);
  // }
  // else
  // {
  //   Serial.println("Error writing to file!");
  // }

  // // Wait before next write
  // delay(100);

  // // Every 10 entries, close and reopen the file (good practice)
  // if (dataCount % 10 == 0)
  // {
  //   dataFile.close();
  //   Serial.println("File closed and reopened for safety.");
  //   dataFile = SD.open("/data_log.txt", FILE_WRITE);
  // }

  // ---- Temp sensor motor ----
  // sensors.begin();
  // sensors.requestTemperatures();
  // float temp = sensors.getTempCByIndex(0);
  // Serial.print("Temperature= ");
  // Serial.print(temp);
  // Serial.println(" C");

  // ---- Trigger ----
  boolean isTriggerPressed = digitalRead(TRIGGER_SWITCH_PIN) == HIGH;

  if (isTriggerPressed) {
    odrive.setVelocity(0, -25, 1.59f);  
  } else {
    odrive.setVelocity(0, 0, 1.59f);
  }

  // ---- Date ----
  // DateTime now = rtc.now();
  // printDate(now);
}

// #include <HardwareSerial.h>
// #include <Adafruit_ADS1X15.h>
// #include <logger.h>

// #include "EMIFilterSwitch.h"
// #include "RotationEncoder.h"
// #include "ODrive.h"
// #include "BonexMotorController.h"

// #define LED_BUILTIN 8
// #define ODRIVE_RX_PIN 20
// #define ODRIVE_TX_PIN 21
// #define TRIGGER_SWITCH_PIN 10
// #define REVERSE_SWITCH_PIN 7

// #define ENCODER_CALIBRATION_MIN_VALUE 198
// #define ENCODER_CALIBRATION_MAX_VALUE 27895
// #define ENCODER_DIRECTION 1
// #define ENCODER_FULL_RANGE_ANGLE 360

// // Motor controller configuration constants
// #define MOTOR_AXIS_ID 0
// #define MOTOR_MAX_TORQUE 1.59f
// #define MOTOR_MIN_VELOCITY 10
// #define MOTOR_MAX_VELOCITY 50
// #define MOTOR_MIN_REVERSE_VELOCITY 8
// #define MOTOR_MAX_REVERSE_VELOCITY 20
// #define MOTOR_STARTUP_VELOCITY 30
// #define MOTOR_FORWARD_DIRECTION_MULTIPLIER -1

// // EMI filtering and debouncing constants
// #define TRIGGER_SWITCH_DEBOUNCE_TIME 50
// #define TRIGGER_SWITCH_FILTER_SAMPLES 10

// #define REVERSE_SWITCH_DEBOUNCE_TIME 50
// #define REVERSE_SWITCH_FILTER_SAMPLES 10

// HardwareSerial ODriveSerial(1);
// Adafruit_ADS1115 ads;

// // Create logger instance
// logging::Logger logger;

// // Create components
// ODrive odrive(&logger, &ODriveSerial);

// EMIFilterSwitch trigger_switch({TRIGGER_SWITCH_DEBOUNCE_TIME,
//                                 TRIGGER_SWITCH_FILTER_SAMPLES});

// EMIFilterSwitch reverse_switch({REVERSE_SWITCH_DEBOUNCE_TIME,
//                                 REVERSE_SWITCH_FILTER_SAMPLES});

// RotationEncoder rotation_encoder(&logger, &ads);

// // Create motor controller
// BonexMotorController motor_controller(&logger, &odrive, &trigger_switch, &reverse_switch, &rotation_encoder);

// void setup()
// {
//   Serial.begin(115200);
//   ODriveSerial.begin(115200, SERIAL_8N1, ODRIVE_RX_PIN, ODRIVE_TX_PIN);

//   // Wait until USB serial is initialized
//   while (!Serial)
//   {
//   }

//   // Wait until ODrive serial is initialized
//   while (!ODriveSerial)
//   {
//   }

//   // Initialize logger
//   logger.setSerial(&Serial);

//   // Check ADS initialization
//   if (!ads.begin())
//   {
//     while (true)
//     {
//       logger.log(logging::LoggerLevel::LOGGER_LEVEL_ERROR, "ADS", "Failed to initialize ADS.");
//       delay(1000);
//     };
//   }

//   trigger_switch.setup(TRIGGER_SWITCH_PIN, INPUT_PULLDOWN);
//   reverse_switch.setup(REVERSE_SWITCH_PIN, INPUT_PULLDOWN);

//   rotation_encoder.setup({ENCODER_CALIBRATION_MIN_VALUE,
//                           ENCODER_CALIBRATION_MAX_VALUE,
//                           ENCODER_DIRECTION});

//   motor_controller.setup({MOTOR_AXIS_ID,
//                           MOTOR_MAX_TORQUE,
//                           MOTOR_MIN_VELOCITY,
//                           MOTOR_MAX_VELOCITY,
//                           MOTOR_MIN_REVERSE_VELOCITY,
//                           MOTOR_MAX_REVERSE_VELOCITY,
//                           MOTOR_STARTUP_VELOCITY,
//                           MOTOR_FORWARD_DIRECTION_MULTIPLIER});
// }

// void loop()
// {
//   motor_controller.loop();
//   delay(10);
// }
