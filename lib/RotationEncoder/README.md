# RotationEncoder Library

A library for handling rotary encoder input with proper angle wrapping detection.

## Features

- **Robust angle wrapping**: Uses shortest path algorithm to handle encoder wraps correctly
- **Configurable calibration**: Supports custom min/max values for different encoders
- **Direction support**: Configurable encoder direction
- **Clean interface**: Simple API for getting current angle and angle delta

## Usage

```cpp
#include <RotationEncoder.h>
#include <Adafruit_ADS1X15.h>
#include "logger.h"

Adafruit_ADS1115 ads;
logging::Logger logger;
RotationEncoder rotation_encoder(&logger, &ads);

void setup() {
    // Initialize logger
    logger.setSerial(&Serial);
    
    // Initialize ADS
    ads.begin();
    
    // Setup encoder with calibration values and direction
    rotation_encoder.setup(198, 27895, -1); // min, max, direction
}

void loop() {
    // Update encoder state (call this in main loop)
    rotation_encoder.loop();
    
    // Get current angle (0-360 degrees)
    int16_t angle = rotation_encoder.getAngle();
    
    // Get angle delta since last update
    int16_t delta = rotation_encoder.getAngleDelta();
}
```

## API Reference

### Constructor
```cpp
RotationEncoder(logging::Logger* logger, Adafruit_ADS1115* ads)
```

### Methods
- `void setup(int16_t calibration_min_value, int16_t calibration_max_value, int8_t direction)`
- `int16_t getAngle()` - Returns current angle (0-360 degrees)
- `int16_t getAngleDelta()` - Returns angle change since last update
- `void loop()` - Updates internal state (call in main loop)

## Angle Wrapping

The library automatically handles encoder wraps using the shortest path algorithm:
- Clockwise wrap: 350° → 10° = -20° delta
- Counter-clockwise wrap: 10° → 350° = -20° delta
- Normal movement: 90° → 120° = +30° delta 