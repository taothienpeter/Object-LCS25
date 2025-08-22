#ifndef AUTO_CONTROL_H
#define AUTO_CONTROL_H
#include <Arduino.h>

class Timer
{
  public:
    Timer(int period) : period(period) {
      startMillis = millis();  // Initialize the start time
    }
    void start() {
      startMillis = millis();  // Reset the start time
    }
    bool update() {
      currentMillis = millis();  // Get the current "time" (actually the number of milliseconds since the program started)
      if (currentMillis - startMillis >= period) {  // Test whether the period has elapsed
        startMillis = currentMillis;  // IMPORTANT to save the start time of the current LED state.
        return true;  // Return true to indicate that the period has elapsed
      }
      return false;  // Return false if the period has not elapsed
    }

  private:
    int period;
    unsigned long startMillis;
    unsigned long currentMillis;
};

#endif