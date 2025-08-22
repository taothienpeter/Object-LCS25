#include <PS2X_lib.h>
#include <Wire.h>
#include "Manual_control.h"
#include "Auto_control.h"
// #include "Automation_Param.h"
#include <Arduino.h>

#define SERIAL_DEBUG
// #define ARM_CONTROL // enable arm control
#define SpeedThreadhold 1
#define SpeedThreadhold2 0.4
#define SubMCU_Drivetrain 1
#define SubMCU_Manipulator 2
#define rodServo 13
bool up, down, left, right;
PS2X ps2_drivetrain;
PS2X ps2_arm;
// 13,12,10,11  clock, command, attention, data
// 22,23,24,25  CLK , CMD , ATT , DAT
// 52,53,50,51  SCK, SS, MISO, MOSI
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
void Execute_Drivetrain_Gcode(BaseDir DirAngle);
void FreeRunning();
// #ifdef ARM_CONTROL
void Execute_Arm_Gcode();
// #endif // ARM_CONTROL
void setup() {
  // Wire.begin(); // join I2C bus on address #1
  Serial.begin(9600);
  Serial1.begin(115200);
  Serial2.begin(115200); // for arm control
  error = ps2_drivetrain.config_gamepad(52,50,32,51, true, true);  //(clock, command, attention, data, Pressures?, Rumble?)
  #ifdef ARM_CONTROL
    error_2 = ps2_arm.config_gamepad(52,50,33,51, true, true);
  #endif // ARM_CONTROL
  Manipulator_Kinematic_Compute(RESET);
}
void loop() {
  if (error) {
    delay(20);
    error = ps2_drivetrain.config_gamepad(52,50,32,51, true, true);
    // error_2 = ps2_arm.config_gamepad(52,50,33,51, true, true);
    Serial.print(String(error));
    Serial.print(String(error_2));
    return;}
  while (Serial.available() > 0) {
      GC_cmd = Serial.readStringUntil('\n');
      Serial.println("Received Serial: " + GC_cmd);}
  while (Serial1.available() > 0) {
    String FB_cmd = Serial1.readStringUntil('t');
    Serial.println("Received UART 1: " + FB_cmd);}
  while (Serial2.available() > 0) {
    String FB_cmd = Serial2.readStringUntil('t');
    Serial.println("Received UART 2: " + FB_cmd);}

    ps2_drivetrain.read_gamepad(false, vibrate_1);
//
    lx =map(ps2_drivetrain.Analog(PSS_LX),0,256, -255, 255);
    ly =map(ps2_drivetrain.Analog(PSS_LY),-1,255, 255, -255);
    rx= map(ps2_drivetrain.Analog(PSS_RX),0,256, 255, -255);
    ry= map(ps2_drivetrain.Analog(PSS_RY),-1,255, 255, -255);
    // Serial.print(ly);
  if (ly>0) {
    DirSpeed = abs(ly*SpeedThreadhold);
    Execute_Drivetrain_Gcode(FORWARD);
  }
  if (ly<0) {
    DirSpeed = abs(ly*SpeedThreadhold);
    Execute_Drivetrain_Gcode(BACKWARD);
  }
  if (lx>0) {
    DirSpeed = abs(lx*SpeedThreadhold);
    Execute_Drivetrain_Gcode(RIGHTWARD);
  }
  if (lx<0){
    DirSpeed = abs(lx*SpeedThreadhold);
    Execute_Drivetrain_Gcode(LEFTWARD);
  }
  if (rx>0) {
    DirSpeed = abs(rx*SpeedThreadhold);
    Execute_Drivetrain_Gcode(CLOCKWISE);
  }
  if (rx<0) {
    DirSpeed = abs(rx*SpeedThreadhold);
    Execute_Drivetrain_Gcode(C_CLOCKWISE);
  }
  if (ly==0 && lx==0 && rx==0 && DirSpeed!= 0){
    DirSpeed = 0;
    Execute_Drivetrain_Gcode(STOP);
  }
  //==========================================================
  
  if(ps2_drivetrain.ButtonPressed(PSB_START)){
    Manipulator_Kinematic_Compute(RESET);
    Serial.println("Reset Arm");
    Execute_Arm_Gcode();
  }
  // if(){  
    if(ps2_drivetrain.Button(PSB_PAD_LEFT)){ //gripper #1
      DirSpeed = abs(255*SpeedThreadhold);
      Execute_Drivetrain_Gcode(LEFTWARD);
      // motorsp
    }
    if(ps2_drivetrain.Button(PSB_PAD_RIGHT)){ //gripper #1
      DirSpeed = abs(255*SpeedThreadhold);
      Execute_Drivetrain_Gcode(RIGHTWARD);
    }
    if(ps2_drivetrain.Button(PSB_PAD_UP)){ //gripper #1
      DirSpeed = abs(255*SpeedThreadhold);
      Execute_Drivetrain_Gcode(FORWARD);
      // motorsp
    }
    if(ps2_drivetrain.Button(PSB_PAD_DOWN)){ //gripper #1
      DirSpeed = abs(255*SpeedThreadhold);
      Execute_Drivetrain_Gcode(BACKWARD);
    }
    if(ps2_drivetrain.ButtonReleased(PSB_PAD_LEFT)||ps2_drivetrain.ButtonReleased(PSB_PAD_RIGHT)||ps2_drivetrain.ButtonReleased(PSB_PAD_UP)||ps2_drivetrain.ButtonReleased(PSB_PAD_DOWN)){ //gripper #1
      DirSpeed = 0;
      Execute_Drivetrain_Gcode(STOP);
    }
    //========================================
    if(ps2_drivetrain.Button(PSB_RED)){ //gripper #1
      DirSpeed = abs(255*SpeedThreadhold2);
      Execute_Drivetrain_Gcode(C_CLOCKWISE);
      // motorsp
    }
    if(ps2_drivetrain.Button(PSB_PINK)){ //gripper #1
      DirSpeed = abs(255*SpeedThreadhold2);
      Execute_Drivetrain_Gcode(CLOCKWISE);
    }
    if(ps2_drivetrain.ButtonReleased(PSB_PAD_LEFT)||ps2_drivetrain.ButtonReleased(PSB_PAD_RIGHT)){ //gripper #1
      DirSpeed = 0;
      Execute_Drivetrain_Gcode(STOP);
    }
    //============================================
    if(ps2_drivetrain.ButtonPressed(PSB_L1)){ //gripper #1
      closing_LeftGripper.start(540); // Start the timer
      ArmMotor.MotorSpeed[GRIPPER_LEFT] = QuarterSpeed;
    }
    if(ps2_drivetrain.ButtonPressed(PSB_R1)){ //gripper #1
      closing_RightGripper.start(700); // Start the timer
      ArmMotor.MotorSpeed[GRIPPER_RIGHT] = QuarterSpeed;
    }
    if(ps2_drivetrain.ButtonPressed(PSB_L2)){ //gripper #1
      closing_LeftGripper.start(180); // Start the timer
      ArmMotor.MotorSpeed[GRIPPER_LEFT] = -FullSpeed;
    }
    if(ps2_drivetrain.ButtonPressed(PSB_R2)){ //gripper #1
      closing_RightGripper.start(180); // Start the timer
      ArmMotor.MotorSpeed[GRIPPER_RIGHT] = -FullSpeed;
    }
    if(ps2_drivetrain.ButtonPressed(PSB_BLUE)){ // arm shoulder
      Manipulator_Kinematic_Compute(ARM_CYLO); // ARM_GRIPPING, ARM_CYLO, ARM_CART
      // Serial.println(ArmMotor.MotorPositions[2]);
    }
    if(ps2_drivetrain.ButtonPressed(PSB_PINK)||ps2_drivetrain.ButtonReleased(PSB_GREEN)||ps2_drivetrain.ButtonReleased(PSB_BLUE)){ // arm shoulder
      Manipulator_Kinematic_Compute(ARM_GRIPPING); // ARM_GRIPPING, ARM_CYLO, ARM_CART
    }
    if(ps2_drivetrain.ButtonPressed(PSB_GREEN)){ // arm shoulder
      Manipulator_Kinematic_Compute(ARM_CART); // ARM_GRIPPING, ARM_CYLO, ARM_CART
    }
    Execute_Arm_Gcode();
  // }
  if (closing_LeftGripper.stop()) { // Check if the timer has elapsed
    ArmMotor.MotorSpeed[GRIPPER_LEFT] = 0;
    Execute_Arm_Gcode();
  }
  if (closing_RightGripper.stop()) { // Check if the timer has elapsed
    ArmMotor.MotorSpeed[GRIPPER_RIGHT] = 0;
    Execute_Arm_Gcode();
  }
delay(50);
}
void Execute_Drivetrain_Gcode(BaseDir DirAngle) {
  Drivetrain_Kinematic_Compute(DirAngle);
    // GC_cmd=I2C_com(GC_cmd = "G1 V1 125 V2 125 V3 125 V4 125"); // G1 V1 125 V2 125 V3 125 V4 125
    GC_cmd = "G1 V1 " + String(BaseMotor.MotorSpeed[MOTOR_LF]) +" V2 " + String(BaseMotor.MotorSpeed[MOTOR_RF]) +" V3 "+ String(BaseMotor.MotorSpeed[MOTOR_LB]) +" V4 "+ String(BaseMotor.MotorSpeed[MOTOR_RB]) + "t";
    #ifdef SERIAL_DEBUG
    Serial.println(GC_cmd);
    #endif
    Serial1.println(GC_cmd);
}
// #ifdef ARM_CONTROL
void Execute_Arm_Gcode() {
    GC_cmd = "G2 V1 " + String(ArmMotor.MotorSpeed[GRIPPER_LEFT]) +" V2 " + String(ArmMotor.MotorSpeed[GRIPPER_RIGHT]) +" V3 "+ String(ArmMotor.MotorPositions[ARM_SHOULDER]) +" V4 "+ String(ArmMotor.MotorSpeed[3])+"t";
    #ifdef SERIAL_DEBUG
    Serial.println(GC_cmd);
    #endif
    Serial2.println(GC_cmd);
};
// #endif // ARM_CONTROL
bool isButtonPressed_arm() {
  return ps2_arm.ButtonPressed(PSB_GREEN) || ps2_arm.ButtonPressed(PSB_RED) || ps2_arm.ButtonPressed(PSB_BLUE) || ps2_arm.ButtonPressed(PSB_PINK) 
      || ps2_arm.ButtonPressed(PSB_L1) || ps2_arm.ButtonPressed(PSB_R1) || ps2_arm.ButtonPressed(PSB_L2) || ps2_arm.ButtonPressed(PSB_R2)
      || ps2_arm.ButtonPressed(PSB_START)||ps2_arm.ButtonPressed(PSB_SELECT) || ps2_arm.ButtonReleased(PSB_GREEN) || ps2_arm.ButtonReleased(PSB_RED)
      || ps2_arm.ButtonPressed(PSB_PAD_UP) || ps2_arm.ButtonPressed(PSB_PAD_DOWN) || ps2_arm.ButtonPressed(PSB_PAD_LEFT) || ps2_arm.ButtonPressed(PSB_PAD_RIGHT) ;
} 
bool isButtonPressed_drivetrain() {
  return ps2_drivetrain.ButtonPressed(PSB_GREEN) || ps2_drivetrain.ButtonPressed(PSB_RED) || ps2_drivetrain.ButtonPressed(PSB_BLUE) || ps2_drivetrain.ButtonPressed(PSB_PINK) 
      || ps2_drivetrain.ButtonPressed(PSB_L1) || ps2_drivetrain.ButtonPressed(PSB_R1) || ps2_drivetrain.ButtonPressed(PSB_L2) || ps2_drivetrain.ButtonPressed(PSB_R2)
      || ps2_drivetrain.ButtonPressed(PSB_START)||ps2_drivetrain.ButtonPressed(PSB_SELECT) || ps2_drivetrain.ButtonReleased(PSB_GREEN) || ps2_drivetrain.ButtonReleased(PSB_RED)
      || ps2_drivetrain.ButtonPressed(PSB_PAD_UP) || ps2_drivetrain.ButtonPressed(PSB_PAD_DOWN) || ps2_drivetrain.ButtonPressed(PSB_PAD_LEFT) || ps2_drivetrain.ButtonPressed(PSB_PAD_RIGHT) ;
} 
void FreeRunning(){
  delay(2000);
  Execute_Drivetrain_Gcode(FORWARD);
  delay(2150);
  Execute_Drivetrain_Gcode(STOP);
}