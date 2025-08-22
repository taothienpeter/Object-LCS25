#include "HardwareSerial.h"
// #include "HardwareSerial.h"
// #include "WString.h"
#include "Manual_control.h"
#include <Arduino.h>
#include <Wire.h>

short ROD_ServoSpeed;
char GC_cmd_type[6] = { 'M', 'V', 'A', 'P', 'I', 'D' };
struct Motor BaseMotor; // define robot base motor
struct Motor ArmMotor; // define robot arm motor
// double P[4], I[4], D[4];
#define ARM_CONTROL
#ifdef ARM_CONTROL
void Manipulator_Kinematic_Compute(ArmState ArmState) {
    switch (ArmState) {
      case ARM_GRIPPING:
      ArmMotor.MotorPositions[ARM_SHOULDER] = 0;
        break;
      case ARM_CYLO:
      ArmMotor.MotorPositions[ARM_SHOULDER] = Arm_angle_CYLO;
        break;
      case ARM_CART:
      ArmMotor.MotorPositions[ARM_SHOULDER] = Arm_angle_CART;
        break;
      case GRIP_RIGHT:
      ArmMotor.MotorSpeed[GRIPPER_RIGHT] = -ArmMotor.MotorSpeed[GRIPPER_RIGHT];
        break;
      case GRIP_LEFT:
      ArmMotor.MotorSpeed[GRIPPER_LEFT] = -ArmMotor.MotorSpeed[GRIPPER_LEFT];
        break;
      case MAGNET_ON:
      ArmMotor.MotorSpeed[MAGNET] == 0 ? ArmMotor.MotorSpeed[MAGNET] = FullSpeed : ArmMotor.MotorSpeed[MAGNET] = 0;
        break;
      case ROD_ON:
      ROD_ServoSpeed = -ROD_ServoSpeed;
        break;
      case RESET:
      ArmMotor.MotorSpeed[GRIPPER_RIGHT] = 0; // gripper #1 aka right
      ArmMotor.MotorSpeed[GRIPPER_LEFT] = 0; // gripper #2 aka left
      ArmMotor.MotorPositions[ARM_SHOULDER] = Arm_angle_HOME; // arm shoulder
      ArmMotor.MotorSpeed[MAGNET] = 0; // magnet
      ROD_ServoSpeed = 0;
        break;
    }
  
};
#endif // ARM_CONTROL
void Drivetrain_Kinematic_Compute(BaseDir DirAngle) {
  #ifdef DIRECT_CONTROL
    switch (DirAngle) {
      case FORWARD:
        BaseMotor.MotorSpeed[MOTOR_LF] = DirSpeed;
        BaseMotor.MotorSpeed[MOTOR_RF] = DirSpeed;
        BaseMotor.MotorSpeed[MOTOR_LB] = DirSpeed;
        BaseMotor.MotorSpeed[MOTOR_RB] = DirSpeed;
        break;
      case BACKWARD:
        BaseMotor.MotorSpeed[MOTOR_LF] = -DirSpeed;
        BaseMotor.MotorSpeed[MOTOR_RF] = -DirSpeed;
        BaseMotor.MotorSpeed[MOTOR_LB] = -DirSpeed;
        BaseMotor.MotorSpeed[MOTOR_RB] = -DirSpeed;
        break;
      case LEFTWARD:
        DirSpeed = DirSpeed * 1.5; // Adjust speed for leftward movement
        BaseMotor.MotorSpeed[MOTOR_LF] = -DirSpeed + 15;
        BaseMotor.MotorSpeed[MOTOR_RF] = DirSpeed;
        BaseMotor.MotorSpeed[MOTOR_LB] = DirSpeed - 10 ;
        BaseMotor.MotorSpeed[MOTOR_RB] = -DirSpeed;
        DirSpeed = DirSpeed / 1.5; // Reset speed to original value
        break;
      case RIGHTWARD:
        DirSpeed = DirSpeed * 1.5; // Adjust speed for leftward movement
        BaseMotor.MotorSpeed[MOTOR_LF] = DirSpeed - 10;
        BaseMotor.MotorSpeed[MOTOR_RF] = -DirSpeed;
        BaseMotor.MotorSpeed[MOTOR_LB] = -DirSpeed + 10;
        BaseMotor.MotorSpeed[MOTOR_RB] = DirSpeed;
        DirSpeed = DirSpeed / 1.5; // Reset speed to original value
        break;
      case CLOCKWISE:
        BaseMotor.MotorSpeed[MOTOR_LF] = DirSpeed;
        BaseMotor.MotorSpeed[MOTOR_RF] = -DirSpeed;
        BaseMotor.MotorSpeed[MOTOR_LB] = DirSpeed;
        BaseMotor.MotorSpeed[MOTOR_RB] = -DirSpeed;
        break;
      case C_CLOCKWISE:
        BaseMotor.MotorSpeed[MOTOR_LF] = -DirSpeed;
        BaseMotor.MotorSpeed[MOTOR_RF] = DirSpeed;
        BaseMotor.MotorSpeed[MOTOR_LB] = -DirSpeed;
        BaseMotor.MotorSpeed[MOTOR_RB] = DirSpeed;
        break;
      case STOP:
        BaseMotor.MotorSpeed[MOTOR_LF] = 0;
        BaseMotor.MotorSpeed[MOTOR_RF] = 0;
        BaseMotor.MotorSpeed[MOTOR_LB] = 0;
        BaseMotor.MotorSpeed[MOTOR_RB] = 0;
        break;
    }
  #endif  // DIRECT_CONTROL
}
String SerialGcode() {
  String Serial_input = "";
  Serial_input += Serial.readStringUntil('\n'); // Clear the serial buffer
  return  Serial_input;
}
void GC_decode(String* GC_return, int spaceIndex, String token, String GC_cmd, short gct, short* input_array) {
  char motorIndexChar = token.charAt(1);
  if (motorIndexChar >= '1' && motorIndexChar <= '4') {
    int motorIndex = motorIndexChar - '1';  // Convert char to index (0-3)
    // Get the value after "M<index>"
    spaceIndex = GC_cmd.indexOf(' ');
    String valueStr;
    if (spaceIndex == -1) {
      valueStr = GC_cmd;
      GC_cmd = "";
    } else {
      valueStr = GC_cmd.substring(0, spaceIndex);
      GC_cmd = GC_cmd.substring(spaceIndex + 1);
      GC_cmd.trim();
    }
    int value = valueStr.toInt();
    if (value != 0 || valueStr == "0") {  // Check if it's a valid number
      *GC_return += " " + String(GC_cmd_type[gct]) + String(motorIndex + 1) + " " + String(value);
      *(input_array + motorIndex) = value;
    } else {
      Serial.print("Invalid value for motor ");
      Serial.println(GC_cmd_type[gct]);
      return;
    }
  } else {
    for (int i = 0; i < 4; i++) {
      *GC_return += " " + GC_cmd_type[gct] + String(i + 1) + " " + String(input_array[i]);
    }
  }
}
void GC_decode_parram(String* GC_return, int spaceIndex, String token, String GC_cmd, short gct, double* input_parram) {
  char motorIndexChar = token.charAt(1);
  if (motorIndexChar >= '1' && motorIndexChar <= '4') {
    int motorIndex = motorIndexChar - '1';  // Convert char to index (0-3)
    // Get the value after "M<index>"
    spaceIndex = GC_cmd.indexOf(' ');
    String valueStr;
    if (spaceIndex == -1) {
      valueStr = GC_cmd;
      GC_cmd = "";
    } else {
      valueStr = GC_cmd.substring(0, spaceIndex);
      GC_cmd = GC_cmd.substring(spaceIndex + 1);
      GC_cmd.trim();
    }
    int value = valueStr.toInt();
    if (value != 0 || valueStr == "0") {  // Check if it's a valid number
      *GC_return += " " + GC_cmd_type[gct] + String(motorIndex) + " " + String(value);
      // *(input_parram + motorIndex) = value;
    } else {
      Serial.print("Invalid value for motor ");
      Serial.println(GC_cmd_type[gct]);
      return;
    }
  } else {
    for (int i = 0; i < 4; i++) {
      *GC_return += " " + GC_cmd_type[gct] + String(i + 1) + " " + String(input_parram[i]);
    }
  }
}
String I2C_com(String GC_cmd) {
    String GC_return = "";
    if (GC_cmd.startsWith("G1")) {  // RUNNING mode for Pico 1 aka DRIVETRAIN
      GC_return = "G1";
      // Serial.print("G1 ");
      //======
      int spaceIndex;
      String token;
      GC_cmd = GC_cmd.substring(2);
      GC_cmd.trim();
      while (GC_cmd.length() > 0) {
        spaceIndex = GC_cmd.indexOf(' ');
        if (spaceIndex == -1) {
          token = GC_cmd;
          GC_cmd = "";
        } else {
          token = GC_cmd.substring(0, spaceIndex);
          GC_cmd = GC_cmd.substring(spaceIndex + 1);
          GC_cmd.trim();
        }  // tách cmd
        //==========
        if (token.startsWith("M")) {
  #ifdef GCODE_POS_CMD  // String* GC_return, int* spaceIndex, String* token, String* GC_cmd, short input_array[Outputindex]
          GC_decode(&GC_return, spaceIndex, token, GC_cmd, 0, BaseMotor.MotorPositions);
  #endif  // GCODE_POS_CMD
        } else if (token.startsWith("V")) {
  #ifdef GCODE_VEL_CMD
          // Serial.print("Flag 1: ");
          // Serial.print(token);
          GC_decode(&GC_return, spaceIndex, token, GC_cmd, 1, BaseMotor.MotorSpeed);
  #endif  // GCODE_VEL_CMD
        } else if (token.startsWith("A")) {
  #ifdef GCODE_ACC_CMD
          GC_decode(&GC_return, spaceIndex, token, GC_cmd, 2, BaseMotor.MotorAcceleration);
  #endif  // GCODE_ACC_CMD
        } else {
          // Serial.print("Flag 2: ");
          // Serial.print("Done generate Gcode for pico 1 MOTORS RUNNING");
        }
      }
    } else if (GC_cmd.startsWith("M1")) {  // SETTING mode for Pico 1 aka DRIVETRAIN
      GC_return = "M1";
      #ifdef PID_CONFIG
      int spaceIndex;
      String token;
      GC_cmd = GC_cmd.substring(2);
      GC_cmd.trim();
      while (GC_cmd.length() > 0) {
        spaceIndex = GC_cmd.indexOf(' ');
        if (spaceIndex == -1) {
          token = GC_cmd;
          GC_cmd = "";
        } else {
          token = GC_cmd.substring(0, spaceIndex);
          GC_cmd = GC_cmd.substring(spaceIndex + 1);
          GC_cmd.trim();
        }  // tách cmd
        //==========
        if (token.startsWith("P")) {
          GC_decode_parram(&GC_return, spaceIndex, token, GC_cmd, 3, BaseMotor.Kp); //======================
        } else if (token.startsWith("I")) {
          GC_decode_parram(&GC_return, spaceIndex, token, GC_cmd, 4, BaseMotor.Ki);
        } else if (token.startsWith("D")) {
          GC_decode_parram(&GC_return, spaceIndex, token, GC_cmd, 5, BaseMotor.Kd);
          }
          else {
            Serial.print("Done generate Gcode for pico 1 MOTORS SETTING");
          }
        }
      #endif  
      }
      else if (GC_cmd.startsWith("G2")) {
        GC_return = "G2";
        #ifdef ARM_CONTROL
        int spaceIndex;
        String token;
        GC_cmd = GC_cmd.substring(2);
        GC_cmd.trim();
        while (GC_cmd.length() > 0) {
          spaceIndex = GC_cmd.indexOf(' ');
          if (spaceIndex == -1) {
            token = GC_cmd;
            GC_cmd = "";
          } else {
            token = GC_cmd.substring(0, spaceIndex);
            GC_cmd = GC_cmd.substring(spaceIndex + 1);
            GC_cmd.trim();
          }  // tách cmd
          if (token.startsWith("M")) {
            
            GC_decode(&GC_return, spaceIndex, token, GC_cmd, 0, ArmMotor.MotorPositions);
            
          } else if (token.startsWith("V")) {
            
            GC_decode(&GC_return, spaceIndex, token, GC_cmd, 1, ArmMotor.MotorSpeed);
            
          } else if (token.startsWith("A")) {
            #ifdef GCODE_ACC_CMD
            GC_decode(&GC_return, spaceIndex, token, GC_cmd, 2, ArmMotor.MotorAcceleration);
            #endif
          } else {
            Serial.print("Done generate Gcode for pico 1 MOTORS RUNNING");
          }
        }
        #endif
      }
      else if (GC_cmd.startsWith("M2")) {
        GC_return = "M2";
        #ifdef PID_CONFIG
        int spaceIndex;
        String token;
        GC_cmd = GC_cmd.substring(2);
        GC_cmd.trim();
        while (GC_cmd.length() > 0) {
          spaceIndex = GC_cmd.indexOf(' ');
          if (spaceIndex == -1) {
            token = GC_cmd;
            GC_cmd = "";
          } else {
            token = GC_cmd.substring(0, spaceIndex);
            GC_cmd = GC_cmd.substring(spaceIndex + 1);
            GC_cmd.trim();
          }  // tách cmd
          //==========
          if (token.startsWith("P")) {
            GC_decode_parram(&GC_return, spaceIndex, token, GC_cmd, 3, ArmMotor.Kp); //======================
          } else if (token.startsWith("I")) {
            GC_decode_parram(&GC_return, spaceIndex, token, GC_cmd, 4, ArmMotor.Ki);
          } else if (token.startsWith("D")) {
            GC_decode_parram(&GC_return, spaceIndex, token, GC_cmd, 5, ArmMotor.Kd);
            }
            else {
              Serial.print("Done generate Gcode for pico 1 MOTORS SETTING");
            }
          }
        #endif
      }
      else {
        GC_return = "G0";
        int spaceIndex;
        String token;
        GC_cmd = GC_cmd.substring(2);
        GC_cmd.trim();
        while (GC_cmd.length() > 0) {
          spaceIndex = GC_cmd.indexOf(' ');
          if (spaceIndex == -1) {
            token = GC_cmd;
            GC_cmd = "";
          } else {
            token = GC_cmd.substring(0, spaceIndex);
            GC_cmd = GC_cmd.substring(spaceIndex + 1);
            GC_cmd.trim();
          }  // tách cmd
          //==========
          Drivetrain_Kinematic_Compute(STOP); 
          #ifdef ARM_CONTROL
          Manipulator_Kinematic_Compute(RESET);
          #endif
            GC_decode(&GC_return, spaceIndex, token, GC_cmd, 0, BaseMotor.MotorPositions);
            GC_decode(&GC_return, spaceIndex, token, GC_cmd, 1, BaseMotor.MotorSpeed);
            #ifdef GCODE_ACC_CMD
            GC_decode(&GC_return, spaceIndex, token, GC_cmd, 2, BaseMotor.MotorAcceleration);
            #endif
            GC_decode(&GC_return, spaceIndex, token, GC_cmd, 0, ArmMotor.MotorPositions);
            
            GC_decode(&GC_return, spaceIndex, token, GC_cmd, 1, ArmMotor.MotorSpeed);
            #ifdef GCODE_ACC_CMD
            GC_decode(&GC_return, spaceIndex, token, GC_cmd, 2, ArmMotor.MotorAcceleration);
            #endif
            Serial.print("Done generate HOME Gcode for both pico 1 and pico 2");
        }
      };
      return GC_return += 't';// add 't' to the end of the string to indicate that the command is complete
    }