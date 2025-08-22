#include "Automation_Param.h"
#include <Arduino.h>
#include <math.h>

typedef struct {
  double position;
  double velocity;
  double acceleration;
  // double jerk;
  double time;

} MotorParameter;
Semi_Autonomous::Semi_Autonomous(){ 
  // constructor for Semi_Autonomous class
  // sets things, init pins, sensors, motors
  // double P = 0.0;
  // double I = 0.0;
  // double D = 0.0;
  // Initialize other member variables or hardware components here
}
// sets things, init pins, sensors, motors
void Semi_Autonomous::Calculate_Timing() {
  // tạo một object timing
}
void Semi_Autonomous::Calculate_PID() {
  //tạo một object PID
}
short Semi_Autonomous::Drivetrain_Kinematic_Compute_Method1(float* MotorSpeed, float speed, float turn, float strafe) {
  // left_stickX = speed, left_stickY = strafe, right_stickX = turn
        *MotorSpeed       = (float) (speed + turn + strafe); //leftFrontSpeed
    *(MotorSpeed + 1) = (float) (speed - turn - strafe); //rightFrontSpeed
    *(MotorSpeed + 2) = (float) (speed + turn - strafe); //leftBackSpeed
    *(MotorSpeed + 3) = (float) (speed - turn + strafe); //rightBackSpeed
    return 0; // Return 0 to indicate success
}
short Semi_Autonomous::Drivetrain_Kinematic_Compute_Method2(float* MotorSpeed, float theta, float power, float turn) // input range (float) -1 -> 1
{
  #ifdef ADVANCE_KINEMATIC_CONTROL
    double sinValue = sin(theta - M_PI / 4);
    double cosValue = cos(theta - M_PI / 4);
    double maxValue = max(abs(sinValue), abs(cosValue));


    *MotorSpeed       = (float) (power * (cosValue/maxValue) + turn); //leftFrontSpeed
    *(MotorSpeed + 1) = (float) (power * (sinValue/maxValue) - turn); //rightFrontSpeed
    *(MotorSpeed + 2) = (float) (power * (cosValue/maxValue) + turn); //leftBackSpeed
    *(MotorSpeed + 3) = (float) (power * (sinValue/maxValue) - turn); //rightBackSpeed
    if ((power + abs(turn)) > 1) {
      *MotorSpeed       /= (float) ((power + turn)); //leftFrontSpeed
      *(MotorSpeed + 1) /= (float) ((power - turn)); //rightFrontSpeed
      *(MotorSpeed + 2) /= (float) ((power + turn)/2); //leftBackSpeed
      *(MotorSpeed + 3) /= (float) ((power - turn)/2); //rightBackSpeed
    }
  #endif
  return 0; // Return 0 to indicate success
}
short Semi_Autonomous::Kinematic_joystick_compute(short x, short y, short z, float* turn, float* theta, float*power) // x,y planar coordinates, z is the rotation value
{
  *turn = (float)z/255.0f; // turn value from -1 to 1
  *theta = (float)atan2(y,x)/255.0f; // theta value from -1 to 1
  *power = hypot(x,y)/255.0f; // power value from 0 to 1
  return 0; // Return 0 to indicate success
}
/*
  void PIDController_Init(PIDController *pid) {

    // Clear controller variables 
    pid->integrator = 0.0f;
    pid->prevError  = 0.0f;

    pid->differentiator  = 0.0f;
    pid->prevMeasurement = 0.0f;

    pid->out = 0.0f;

  }
  void PIDController_Init(PIDController *pid) {
    // Clear controller variables 
    pid->integrator = 0.0f;
    pid->prevError  = 0.0f;
    pid->differentiator  = 0.0f;
    pid->prevMeasurement = 0.0f;
    pid->out = 0.0f;
  }
  float PIDController_Update(PIDController *pid, float setpoint, float measurement) {
    //Error signal
      float error = setpoint - measurement;
    // Proportional
      float proportional = pid->Kp * error;
    // Integral
      pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);
    // Anti-wind-up via integrator clamping
      if (pid->integrator > pid->limMaxInt) {
          pid->integrator = pid->limMaxInt;
      } else if (pid->integrator < pid->limMinInt) {
          pid->integrator = pid->limMinInt;
      }
      //Derivative (band-limited differentiator)		
      pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)	 Note: derivative on measurement, therefore minus sign in front of equation!
                          + (2.0f * pid->tau - pid->T) * pid->differentiator)
                          / (2.0f * pid->tau + pid->T);
      // Compute output and apply limits
      pid->out = proportional + pid->integrator + pid->differentiator;
      if (pid->out > pid->limMax) {
          pid->out = pid->limMax;
      } else if (pid->out < pid->limMin) {
          pid->out = pid->limMin;
      }
    // Store error and measurement for later use 
      pid->prevError       = error;
      pid->prevMeasurement = measurement;
    // Return controller output 
      return pid->out;
  }*/