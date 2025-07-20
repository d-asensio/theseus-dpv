#include "EMIFilterSwitch.h"

EMIFilterSwitch::EMIFilterSwitch(unsigned long debounce_ms, int samples, int threshold_count) {
  debounce_time = debounce_ms;
  filter_samples = samples;
  threshold = threshold_count;
  
  current_state = LOW;
  filtered_state = LOW;
  last_change = 0;
  reading_index = 0;
  initialized = false;
  state_changed = false;
  
  // Allocate memory for readings array
  readings = new int[filter_samples];
  for (int i = 0; i < filter_samples; i++) {
    readings[i] = LOW;
  }
}

EMIFilterSwitch::~EMIFilterSwitch() {
  delete[] readings;
}

void EMIFilterSwitch::setup(int pin_number, int pin_mode) {
  pin = pin_number;
  mode = pin_mode;
  pinMode(pin, mode);
  initialized = true;
  
  // Initialize with current reading
  loop(); // This will set up the initial state
}

int EMIFilterSwitch::read() {
  if (!initialized) {
    return LOW;
  }
  
  return filtered_state;
}

void EMIFilterSwitch::loop() {
  if (!initialized) {
    return;
  }
  
  // Read the current switch state
  int current_reading = digitalRead(pin);
  
  // Store the reading in the circular buffer
  readings[reading_index] = current_reading;
  reading_index = (reading_index + 1) % filter_samples;
  
  // Count how many HIGH readings we have
  int high_count = 0;
  for (int i = 0; i < filter_samples; i++) {
    if (readings[i] == HIGH) {
      high_count++;
    }
  }
  
  // Calculate new filtered state
  int new_filtered_state = (high_count >= threshold) ? HIGH : LOW;
  
  // Check for state change with debouncing
  unsigned long current_time = millis();
  if (current_time - last_change > debounce_time) {
    if (filtered_state != new_filtered_state) {
      state_changed = true;
      last_change = current_time;
      filtered_state = new_filtered_state;
    } else {
      state_changed = false;
    }
  } else {
    state_changed = false;
  }
}

bool EMIFilterSwitch::hasChanged() {
  return state_changed;
}

int EMIFilterSwitch::getState() {
  return filtered_state;
} 