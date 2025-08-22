#ifndef AUTOMATION_PARRAM_H
#define AUTOMATION_PARRAM_H
#include <Arduino.h>
#define DEBUG_MODE // enable debug mode
#define ADVANCE_KINEMATIC_CONTROL // use for kinematic computing
class Semi_Autonomous {
private:
  double timing;
public:
  Semi_Autonomous();  // sets things, init pins, sensors, motors for class usage // do not use 
  void Calculate_Timing();
  void Calculate_PID();
  short Drivetrain_Kinematic_Compute_Method1(float* MotorSpeed, float speed, float turn, float strafe); // compute Drivetrain kinematic
  short Drivetrain_Kinematic_Compute_Method2(float* MotorSpeed, float theta, float power, float turn);
  short Kinematic_joystick_compute(short x, short y, short z, float* turn, float* theta, float*power);
  double P, I, D;
protected:
};
#endif
/*
  typedef struct {
    // Controller gains 
    float Kp;
    float Ki;
    float Kd;
    // Derivative low-pass filter time constant 
    float tau;
    // Output limits 
    float limMin;
    float limMax;
    // Integrator limits 
    float limMinInt;
    float limMaxInt;
    // Sample time (in seconds) 
    float T;
    // Controller "memory" 
    float integrator;
    float prevError;			// Required for integrator 
    float differentiator;
    float prevMeasurement;		// Required for differentiator 
    // Controller output 
    float out;
  } PIDController;
  void  PIDController_Init(PIDController *pid);
  float PIDController_Update(PIDController *pid, float setpoint, float measurement);
*/