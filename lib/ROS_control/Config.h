#ifndef ROS_FULLSYSTEM_H
#define ROS_FULLSYSTEM_H

#include <PS2X_lib.h>
#include <Wire.h>
#include "Manual_control.h"
#include "Auto_control.h"

#define SERIAL_DEBUG
#define SpeedThreadhold 1
#define SpeedThreadhold2 0.4
#define SubMCU_Drivetrain 1
#define SubMCU_Manipulator 2
#define rodServo 13
bool up, down, left, right;
PS2X ps2_drivetrain;
short lx, ly, rx, ry, arm_shoulder;
Timer closing_RightGripper(3*1000); // Timer for closing gripper (in ms)
Timer closing_LeftGripper(3*1000); // Timer for closing gripper (in ms)
Timer arm_suport(3*1000); // Timer for closing gripper (in ms)
Timer freeRunning_robot(4*1000); // Timer for closing gripper (in ms)
#define freeRunning_time 3000 // 3s
int error = 0; 
int error_2 = 0;
byte type = 0;
byte vibrate_1 = 0;
byte vibrate_2 = 0;
String GC_cmd;
short DirSpeed, DirAngle;

void Execute_Drivetrain_Gcode(BaseDir DirAngle) {
  Drivetrain_Kinematic_Compute(DirAngle);
    // GC_cmd=I2C_com(GC_cmd = "G1 V1 125 V2 125 V3 125 V4 125"); // G1 V1 125 V2 125 V3 125 V4 125
    GC_cmd = "G1 V1 " + String(BaseMotor.MotorSpeed[MOTOR_LF]) +" V2 " + String(BaseMotor.MotorSpeed[MOTOR_RF]) +" V3 "+ String(BaseMotor.MotorSpeed[MOTOR_LB]) +" V4 "+ String(BaseMotor.MotorSpeed[MOTOR_RB]) + "t";
    #ifdef SERIAL_DEBUG
    Serial.println(GC_cmd);
    #endif
    Serial1.println(GC_cmd);
}
void Execute_Arm_Gcode() {
    GC_cmd = "G2 V1 " + String(ArmMotor.MotorSpeed[GRIPPER_LEFT]) +" V2 " + String(ArmMotor.MotorSpeed[GRIPPER_RIGHT]) +" V3 "+ String(ArmMotor.MotorPositions[ARM_SHOULDER]) +" V4 "+ String(ArmMotor.MotorSpeed[3])+"t";
    #ifdef SERIAL_DEBUG
    Serial.println(GC_cmd);
    #endif
    Serial2.println(GC_cmd);
};
void FreeRunning(){
  delay(2000);
  Execute_Drivetrain_Gcode(FORWARD);
  delay(2150);
  Execute_Drivetrain_Gcode(STOP);
}
#endif
