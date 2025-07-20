#ifndef EMIFILTERSWITCH_H
#define EMIFILTERSWITCH_H

#include <Arduino.h>

class EMIFilterSwitch
{
private:
  int pin;
  int mode;
  unsigned long debounce_time;
  int filter_samples;
  int threshold;

  int current_state;
  int filtered_state;
  unsigned long last_change;
  int *readings;
  int reading_index;
  bool initialized;
  bool state_changed;

private:
  void updateReadings();
  void updateFilteredState();
  int countHighReadings();
  bool isDebounceTimeElapsed();
  bool hasStateChanged(int new_state);

public:
  EMIFilterSwitch(unsigned long debounce_ms, int samples, int threshold_count);
  ~EMIFilterSwitch();

  void setup(int pin_number, int pin_mode);
  void loop();
  bool isPressed();
  bool hasChanged();
};

#endif // EMIFILTERSWITCH_H