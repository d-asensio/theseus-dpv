# MotorController Library

A library that coordinates motor control using ODrive, switches, and encoder input.

## Usage

```cpp
#include <MotorController.h>
#include <ODrive.h>
#include <EMIFilterSwitch.h>
#include <RotationEncoder.h>
#include <logger.h>

HardwareSerial ODriveSerial(1);
logging::Logger logger;
Adafruit_ADS1115 ads;

// Create components
ODrive odrive(&logger, &ODriveSerial);
EMIFilterSwitch trigger_switch(50, 10, 5);
EMIFilterSwitch reverse_switch(50, 10, 5);
RotationEncoder encoder(&logger, &ads);

// Create motor controller
MotorController motor_controller(&logger, &odrive, &trigger_switch, &reverse_switch, &encoder);

void setup() {
    logger.setSerial(&Serial);
    ODriveSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
    
    trigger_switch.setup(TRIGGER_PIN, INPUT_PULLDOWN);
    reverse_switch.setup(REVERSE_PIN, INPUT_PULLDOWN);
    encoder.setup(198, 27895, -1);
    
    motor_controller.setup(0, 1.59f, 8, 50, 30); // axis_id, max_torque, min_velocity, max_velocity, startup_velocity
}

void loop() {
    motor_controller.loop();
}
```

## API

### Constructor
```cpp
MotorController(
    logging::Logger* logger,
    ODrive* odrive,
    EMIFilterSwitch* trigger_switch,
    EMIFilterSwitch* reverse_switch,
    RotationEncoder* encoder
)
```

### Methods
- `void setup(uint8_t axis_id, float max_torque, int16_t min_velocity, int16_t max_velocity, int16_t startup_velocity)` - Configure and initialize the controller
- `void loop()` - Update motor control logic (call in main loop) 