#include "Arduino.h"
#include "Config.h"
#include "Motor.h"
#define SPEED_MAX 255
#define SPEED_MIN 50

float speed = 0.0;
bool reverse = false;
Motor motor_test(MOTOR_BACKLEFT_A, MOTOR_BACKLEFT_B);
float accellerateTest(bool* reverse, float* speed);