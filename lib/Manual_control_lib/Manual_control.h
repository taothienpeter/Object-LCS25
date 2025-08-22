#pragma once
#include "WString.h"
#ifndef MANUAL_CONTROL_H
#define MANUAL_CONTROL_H
#include <Arduino.h>
// #include "Automation_Param.h"

#define FullSpeed 255
#define HalfSpeed 127
#define QuarterSpeed 64
#define OctorSpeed 50

#define Arm_angle_HOME 0
#define Arm_angle_CYLO 230
#define Arm_angle_CART -230 // 180

// #define KINEMATIC_CONTROL // use for kinematic computing
#define DIRECT_CONTROL // use for manual controller only
// #define GCODE_POS_CMD // enable position Gcode generation
#define GCODE_VEL_CMD // enable velocity Gcode generation
// #define GCODE_ACC_CMD // enable accelleration Gcode generation
// #define PID_CONFIG  // enable PID config Gcode generation
// #define ARM_CONTROL // enable arm control
extern enum MotorName { MOTOR_LF,
                 MOTOR_RF,
                 MOTOR_LB,
                 MOTOR_RB } MotorName;
extern enum ManipulatorName { GRIPPER_LEFT,
                 GRIPPER_RIGHT,
                 ARM_SHOULDER,
                 MAGNET,
                 SIDE_ROD } ManipulatorName;
typedef enum BaseDir { FORWARD,
               BACKWARD,
               LEFTWARD,
               RIGHTWARD,
               CLOCKWISE,
               C_CLOCKWISE,
               STOP }BaseDir; // define robot direction
typedef enum ArmState { ARM_GRIPPING,
                   ARM_CYLO,
                   ARM_CART,
                   GRIP_RIGHT,
                   GRIP_LEFT,
                   MAGNET_ON,
                   ROD_ON,
                   RESET} ArmState; // define robot arm direction
                   
// extern struct ArmMotor {
//     short MotorSpeed[4];
//     short MotorPositions[4]; 
//     short MotorAcceleration[4]; 
//     // boolean MotorDiraction[4];
    
//     boolean M_Stop;
//     short M_Angle;

//     double Kp[4];
//     double Ki[4];
//     double Kd[4];
// }ArmMotor;

struct Motor{
    short MotorSpeed[4];
    short MotorPositions[4]; 
    short MotorAcceleration[4]; 
    // boolean MotorDiraction[4];
    boolean M_Stop;
    // short M_Angle;

    double Kp[4];
    double Ki[4];
    double Kd[4];
};
extern short ROD_ServoSpeed;
extern struct Motor BaseMotor; // define robot base motor
extern struct Motor ArmMotor; // define robot arm motor
extern short DirSpeed, DirAngle;  //driving Speed and driving Angle
// extern char GC_cmd_type[6] = { 'M', 'V', 'A', 'P', 'I', 'D' };
// extern double P[4], I[4], D[4]; // storing PID varable for 4 motors
extern String GC_cmd; // storing input Gcode command for Gcode gen M1, M2, G1, G2
String SerialGcode(); // Serial Gcode command user input
void GC_decode(String* GC_return, int spaceIndex, String token, String GC_cmd, short gct, short* input_array); // Gcode small cmd for operating
void GC_decode_parram(String* GC_return, int spaceIndex, String token, String GC_cmd, short gct, double* input_parram); //// Gcode small cmd for parram
void Manipulator_Kinematic_Compute(ArmState ArmState); // compute Manipulator kinematic'
void Drivetrain_Kinematic_Compute(BaseDir DirAngle); // compute Drivetrain kinematic
String I2C_com(String GC_cmd); // for Gcode generation
#endif