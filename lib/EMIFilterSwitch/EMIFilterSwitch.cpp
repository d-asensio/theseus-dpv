#include "EMIFilterSwitch.h"

EMIFilterSwitch::EMIFilterSwitch(const SwitchConfig& config)
{
  debounce_time = config.debounce_time;
  filter_samples = config.filter_samples;

  current_state = LOW;
  filtered_state = LOW;
  last_change = 0;
  reading_index = 0;
  initialized = false;
  state_changed = false;

  // Allocate memory for readings array
  readings = new int[filter_samples];
  for (int i = 0; i < filter_samples; i++)
  {
    readings[i] = LOW;
  }
}

EMIFilterSwitch::~EMIFilterSwitch()
{
  delete[] readings;
}

void EMIFilterSwitch::setup(int pin_number, int pin_mode)
{
  pin = pin_number;
  mode = pin_mode;
  pinMode(pin, mode);
  initialized = true;

  // Initialize with current reading
  loop(); // This will set up the initial state
}

bool EMIFilterSwitch::isPressed()
{
  if (!initialized)
  {
    return false;
  }

  return filtered_state == HIGH;
}

void EMIFilterSwitch::loop()
{
  if (!initialized)
  {
    return;
  }

  updateReadings();
  updateFilteredState();
}

bool EMIFilterSwitch::hasChanged()
{
  return state_changed;
}

void EMIFilterSwitch::updateReadings()
{
  int current_reading = digitalRead(pin);
  readings[reading_index] = current_reading;
  reading_index = (reading_index + 1) % filter_samples;
}

void EMIFilterSwitch::updateFilteredState()
{
  int high_count = countHighReadings();
  int new_filtered_state = (high_count == filter_samples) ? HIGH : LOW;
  
  if (hasStateChanged(new_filtered_state) && isDebounceTimeElapsed())
  {
    state_changed = true;
    last_change = millis();
    filtered_state = new_filtered_state;
    return;
  }
  
  state_changed = false;
}

int EMIFilterSwitch::countHighReadings()
{
  int high_count = 0;
  for (int i = 0; i < filter_samples; i++)
  {
    if (readings[i] == HIGH)
    {
      high_count++;
    }
  }
  return high_count;
}

bool EMIFilterSwitch::isDebounceTimeElapsed()
{
  return (millis() - last_change) > debounce_time;
}

bool EMIFilterSwitch::hasStateChanged(int new_state)
{
  return filtered_state != new_state;
}