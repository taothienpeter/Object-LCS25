#include <Arduino.h>
#include "Config.h"

void Execute_Drivetrain_Gcode(BaseDir DirAngle);
void FreeRunning();
void Execute_Arm_Gcode();

void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);
  Serial2.begin(115200); // for arm control
  error = ps2_drivetrain.config_gamepad(52,50,32,51, true, true);
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
    
      
    if(ps2_drivetrain.ButtonPressed(PSB_L1)){ //gripper #1
      closing_LeftGripper.start(540); // Start the timer
      ArmMotor.MotorSpeed[GRIPPER_LEFT] = QuarterSpeed;
    }
  if (closing_RightGripper.stop()) { // Check if the timer has elapsed
    ArmMotor.MotorSpeed[GRIPPER_RIGHT] = 0;
    Execute_Arm_Gcode();
  }
delay(50);
}
/*
      DirSpeed = abs(lx*SpeedThreadhold);

      Execute_Drivetrain_Gcode(STOP);
      Execute_Drivetrain_Gcode(FORWARD);
      Execute_Drivetrain_Gcode(BACKWARD);
      Execute_Drivetrain_Gcode(RIGHTWARD);
      Execute_Drivetrain_Gcode(LEFTWARD);
      Execute_Drivetrain_Gcode(CLOCKWISE);
      Execute_Drivetrain_Gcode(C_CLOCKWISE);
      Execute_Drivetrain_Gcode(STOP);
      Manipulator_Kinematic_Compute(ARM_CYLO);
      Manipulator_Kinematic_Compute(ARM_GRIPPING);
      Manipulator_Kinematic_Compute(ARM_CART);
      Manipulator_Kinematic_Compute(RESET);
      Execute_Arm_Gcode();
*/
