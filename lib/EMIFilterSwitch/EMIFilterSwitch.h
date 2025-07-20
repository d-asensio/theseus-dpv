#ifndef EMIFILTERSWITCH_H
#define EMIFILTERSWITCH_H

#include <Arduino.h>

class EMIFilterSwitch {
private:
  int pin;
  int mode;
  unsigned long debounce_time;
  int filter_samples;
  int threshold;
  
  int current_state;
  int filtered_state;
  unsigned long last_change;
  int* readings;
  int reading_index;
  bool initialized;
  bool state_changed;

public:
  EMIFilterSwitch(unsigned long debounce_ms, int samples, int threshold_count);
  ~EMIFilterSwitch();
  
  void setup(int pin_number, int pin_mode);
  int read();           // Returns the current filtered state
  void loop();          // Updates internal state and handles debouncing
  bool hasChanged();    // Returns true if state changed since last loop() call
  int getState();       // Returns the current filtered state (same as read())
};

#endif // EMIFILTERSWITCH_H 