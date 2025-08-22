#include <stdio.h>
#include <Arduino.h>
#include "Gcode.h"
#include <Wire.h>

Gcode::Gcode(){
  // this -> Mode_cmd = Mode_command; // Initialize the command string
    // Constructor implementation
}
/*

void Gcode::GC_decode(String* GC_return, int spaceIndex, String token, String GC_cmd, short gct, short* input_array) {
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
String Gcode::Gcode_generate(String GC_cmd) {
    // Generate G-code command
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
      */
String Gcode::Motion_process(String* command, float* input_array, String Oject_cmd, short *spaceIndex, String token, char TotalMotorChar) {
  char motorIndexChar = token.charAt(1);
  if (motorIndexChar >= '1' && motorIndexChar <= TotalMotorChar) {
      int motorIndex = motorIndexChar - '1'; // Convert char to index (0-3)
      // Get the value after "M<index>"
      String _command = *command;
      *spaceIndex = _command.indexOf(' ');
      String valueStr;
      // Serial.println("flag 2: ");
      // Serial.println(token);
      if (*spaceIndex == -1) {
          valueStr = *command;
          *command = "";
      } else {
          valueStr = _command.substring(0, *spaceIndex);
          *command = _command.substring(*spaceIndex + 1);
          _command=*command;
          _command.trim();
          *command=_command;
      }
      int value = valueStr.toInt();
      if (value != 0 || valueStr == "0") { // Check if it's a valid number
        *(input_array + (motorIndex*Sizeof_Motordata)) = value;
        // Serial.println(*(input_array+motorIndex));
          // printf("Motor %p position: ", input_array+motorIndex); // Print motor index (1-4)
          // Serial.println(value);
        } else {
          Serial.println(*command);
          return "Invalid value for motor position.";
      }
  }
  return "Motor position updated successfully.";
}
String Gcode::Config_process(String* command, float* input_array, String Oject_cmd, short* spaceIndex, String token, char TotalMotorChar) {
  char motorIndexChar = token.charAt(1);
  if (motorIndexChar >= '1' && motorIndexChar <= TotalMotorChar) {
      int motorIndex = motorIndexChar - '1'; // Convert char to index (0-3)
      // Get the value after "M<index>"
      String _command = *command;
      *spaceIndex = _command.indexOf(' ');
      String valueStr;
      if (*spaceIndex == -1) {
          valueStr = *command;
          *command = "";
      } else {
          valueStr = _command.substring(0, *spaceIndex);
          *command = _command.substring(*spaceIndex + 1);
          _command=*command;
          _command.trim();
          *command=_command;
      }
      int value = valueStr.toInt();
      if (value != 0 || valueStr == "0") { // Check if it's a valid number
          *(input_array + motorIndex) = value;
      } else {
          // Serial.println(command);
          return("Invalid value for motor position.");
      }
  }
  return "Motor position updated successfully.";
}
String Gcode::Gcode_process(String command, float* input_array, String Mode_cmd, String Object_cmd) {
    // Process G-code command
    // command.substring(2);
    command.trim(); // Remove leading/trailing whitespace
    // Serial.print("flag 0: ");
    // Serial.println(command);
    if (command.startsWith(Mode_cmd)) {
        // Split the string into tokens
        short spaceIndex;
        String token;

        // Skip "G1"
        command = command.substring(2);
        command.trim();

        while (command.length() > 0) {
          spaceIndex = command.indexOf(' ');
          if (spaceIndex == -1) {
              token = command;
              command = "";
          } else {
              token = command.substring(0, spaceIndex);
              command = command.substring(spaceIndex + 1);
              command.trim();
          }
          // Serial.println("flag 1: ");
          // Serial.println(command);
          if (token.startsWith("V")) {
              this -> Motion_process(&command, input_array, Object_cmd, &spaceIndex, token, TotalMotorChar);
          } else if(token.startsWith("M")) {
              this -> Motion_process(&command, input_array, Object_cmd, &spaceIndex, token, TotalMotorChar);
          } else if(token.startsWith("A")) {
              this -> Motion_process(&command, input_array, Object_cmd, &spaceIndex, token, TotalMotorChar);
          } else if(token.startsWith("P")) {
              // this -> Config_process(command, input_array, Object_cmd, spaceIndex, token); 
          } else {
              Serial.println(token);
              return("Invalid command format.");
          }
        }
    } else {  
      Serial.println(Mode_cmd);
      return("Command must start with G.");
    }
  return "Motor position updated successfully.";
}



