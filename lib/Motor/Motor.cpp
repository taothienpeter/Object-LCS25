#include "Motor.h"

Motor::Motor(uint8_t motorPin_A, uint8_t motorPin_B, bool isClockwise, uint8_t lower_pwm_moving_threshold)
:_motorPin_A(motorPin_A), _motorPin_B(motorPin_B), _lower_pwm_moving_threshold(lower_pwm_moving_threshold), _isClockwise(isClockwise)
{
  pinMode(_motorPin_A, OUTPUT);
  pinMode(_motorPin_B, OUTPUT);
}
// Motor::~Motor() { delete motor; }

void Motor::setMotorControl(float control) {
  float unclipedPWM = 0;
  if(control!=0.0){
    unclipedPWM= control*(255-_lower_pwm_moving_threshold) + copysign(_lower_pwm_moving_threshold, control);
  }
  _sendedPWM = constrain(unclipedPWM, -255, 255);
  if (abs(_sendedPWM)<_lower_pwm_moving_threshold) {
    analogWrite(_motorPin_A, 255);
    analogWrite(_motorPin_B, 255);
  } else if (_sendedPWM > 0) {
    analogWrite(_motorPin_A, abs(_sendedPWM));
    analogWrite(_motorPin_B, LOW);
  } else {
    analogWrite(_motorPin_A, LOW);
    analogWrite(_motorPin_B, abs(_sendedPWM));
  }
}
void Motor::setMotorRotationDirection(String rotDir){
  if(rotDir == "CW"){
    _isClockwise = true;
  } else if(rotDir == "CCW"){
    _isClockwise = false;
  }
}
void Motor::setPWMMovingThreshold(uint8_t pwmThreshold){
  _lower_pwm_moving_threshold = pwmThreshold;
}
uint8_t Motor::getPWMMovingThreshold(){
  return _lower_pwm_moving_threshold;
}

int Motor::getSendedPWM(){
  return _sendedPWM;
}