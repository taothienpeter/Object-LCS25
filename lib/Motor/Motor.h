// #include <AFMotor.h>
#include "Arduino.h"
#include <stdlib.h>

class Motor
{
private:
  uint8_t _motorPin_A;
  uint8_t _motorPin_B;

  uint8_t _lower_pwm_moving_threshold;
  float _sendedPWM;
  bool _isClockwise;
public:
  Motor(uint8_t motorPin_A, uint8_t motorPin_B, bool isClockwise = false, uint8_t lower_pwm_moving_threshold = 50);
  // ~Motor(); // use to unmount motor
  
  // setup motor rotation direction
  void setMotorRotationDirection(String rotDir);
  // send PWM to control motor
  void setMotorControl(float control); // [-255; 255] 
  // set the lower PWM moving threshold
  void setPWMMovingThreshold(uint8_t pwmThreshold); // [0; 255]
  //return max PWM threshold value
  uint8_t getPWMMovingThreshold(); // [0; 255] 
  // return PWM after processing
  int getSendedPWM(); 
};
