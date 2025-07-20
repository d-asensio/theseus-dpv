# ODrive Library

A library for communicating with ODrive motor controllers.

## Usage

```cpp
#include <ODrive.h>
#include "logger.h"

HardwareSerial ODriveSerial(1);
logging::Logger logger;
ODrive odrive(&logger, &ODriveSerial);

void setup() {
    logger.setSerial(&Serial);
    ODriveSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
}

void loop() {
    odrive.setVelocity(0, 50);        // 50 rad/s forward on axis 0
    odrive.setVelocity(0, 30, 1.0f);  // 30 rad/s with 1.0 Nm torque FF
    odrive.setVelocity(0, 0);         // Stop motor
}
```

## API

### Constructor
```cpp
ODrive(logging::Logger* logger, HardwareSerial* serial)
```

### Methods
- `void setVelocity(uint8_t axis_id, int16_t velocity, float torqueFF = 0.0f)` 