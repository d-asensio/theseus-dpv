# EMIFilterSwitch Library

A library for handling digital switch input with EMI filtering and debouncing capabilities.

## Features

- **EMI filtering**: Uses a configurable sample buffer to filter out electromagnetic interference
- **Debouncing**: Configurable debounce time to prevent false triggers
- **Threshold-based filtering**: Requires a minimum number of consistent readings to change state
- **State change detection**: Provides methods to detect when switch state has changed
- **Clean interface**: Simple API for reading switch state and detecting changes

## Usage

```cpp
#include <EMIFilterSwitch.h>

// Create switch instance with debounce time, filter samples, and threshold
EMIFilterSwitch trigger_switch(50, 5, 3); // 50ms debounce, 5 samples, 3 threshold

void setup() {
    // Setup switch with pin number and mode
    trigger_switch.setup(10, INPUT_PULLDOWN);
}

void loop() {
    // Update switch state (call this in main loop)
    trigger_switch.loop();
    
    // Get current filtered state
    bool is_pressed = trigger_switch.isPressed();
    
    // Check if state changed since last loop
    if (trigger_switch.hasChanged()) {
        if (is_pressed) {
            // Switch was pressed
            Serial.println("Switch pressed!");
        } else {
            // Switch was released
            Serial.println("Switch released!");
        }
    }
}
```

## API Reference

### Constructor
```cpp
EMIFilterSwitch(unsigned long debounce_ms, int samples, int threshold_count)
```

**Parameters:**
- `debounce_ms`: Debounce time in milliseconds
- `samples`: Number of samples to use for filtering
- `threshold_count`: Minimum number of HIGH readings required to consider switch as pressed

### Methods
- `void setup(int pin_number, int pin_mode)` - Initialize the switch with pin and mode
- `bool isPressed()` - Returns true if switch is pressed, false if released
- `void loop()` - Updates internal state and handles filtering/debouncing
- `bool hasChanged()` - Returns true if state changed since last loop() call

## How It Works

### EMI Filtering
The library uses a circular buffer to store the last N readings from the switch. It counts how many HIGH readings are in the buffer and only considers the switch pressed if the count meets or exceeds the threshold.

### Debouncing
State changes are only registered after the debounce time has elapsed since the last change, preventing rapid switching due to mechanical bounce.

### Example Configuration
```cpp
// For a noisy environment with mechanical switch
EMIFilterSwitch switch1(100, 10, 7); // 100ms debounce, 10 samples, 7 threshold

// For a clean environment with solid-state switch
EMIFilterSwitch switch2(20, 3, 2);   // 20ms debounce, 3 samples, 2 threshold
```

## Configuration Guidelines

- **Debounce time**: 20-100ms depending on switch type and mechanical bounce
- **Filter samples**: 3-10 samples depending on EMI environment
- **Threshold**: Usually 60-80% of filter samples (e.g., 3/5, 7/10)

## Memory Usage

The library dynamically allocates memory for the readings buffer based on the number of samples specified in the constructor. Memory is automatically freed when the object is destroyed. 