#include <Arduino.h>
#include <Wire.h>

#include "encoder.h"
#include "Gcode.h"
#define MotorNum 4
#define MAX_CMD_LEN 32
#define SERIAL_COM // for serial communication
// #define I2C_COM // for I2C communication

// Pin definitions for motor outputs
  #define Output_1A 0
  #define Output_1B 1
  #define Output_2A 3
  #define Output_2B 2
  #define Output_3A 6
  #define Output_3B 7 
  #define Output_4A 10
  #define Output_4B 11
//
  #define UART_RX 4
  #define UART_TX 5
//
  #define ENC_1A 2
  #define ENC_1B 3
  #define ENC_2A 6
  #define ENC_2B 7
  #define ENC_3A 8
  #define ENC_3B 9
  #define ENC_4A 10
  #define ENC_4B 11
//

UART Serial2(4, 5);
Encoder enc1(ENC_1A, ENC_1B, true); // pin A, pin B, clockwise
Encoder enc2(ENC_2A, ENC_2B, true);
Encoder enc3(ENC_3A, ENC_3B, true);
Encoder enc4(ENC_4A, ENC_4B, true);

typedef struct {
  uint8_t Pin_A, Pin_B;
  short Speed;
  double Kp, Ki, Kd;
} MotorData;
typedef enum { MOTOR_LF, MOTOR_RF, MOTOR_LB, MOTOR_RB } MotorName;
MotorData MotorSpeed[MotorNum];
String GC_cmd;

void initpins() {
  MotorSpeed[MOTOR_LF] = {Output_1A, Output_1B, 0};
  MotorSpeed[MOTOR_RF] = {Output_2A, Output_2B, 0};
  MotorSpeed[MOTOR_LB] = {Output_3A, Output_3B, 0};
  MotorSpeed[MOTOR_RB] = {Output_4A, Output_4B, 0};

  for (int i = 0; i < MotorNum; i++) {
    pinMode(MotorSpeed[i].Pin_A, OUTPUT);
    pinMode(MotorSpeed[i].Pin_B, OUTPUT);
    analogWrite(MotorSpeed[i].Pin_A, 0); // Ensure motors are off initially
    analogWrite(MotorSpeed[i].Pin_B, 0);
  }
}
void WritePWM(short Speed, MotorName Motor) {
  Serial.print("Motor ");
  Serial.print(Motor + 1);
  Serial.print(" Speed: ");
  Serial.println(Speed);

  if (Speed > 0) {
    analogWrite(MotorSpeed[Motor].Pin_A, Speed);
    digitalWrite(MotorSpeed[Motor].Pin_B, HIGH);
  } else if (Speed < 0) {
    digitalWrite(MotorSpeed[Motor].Pin_A, HIGH);
    analogWrite(MotorSpeed[Motor].Pin_B, -Speed);
  } else {
    analogWrite(MotorSpeed[Motor].Pin_A, 0); // Stop: both HIGH PWM
    analogWrite(MotorSpeed[Motor].Pin_B, 0);
  }
}
void Gcode_process(String command) {
  // Remove leading/trailing whitespace
  command.trim();

  // Check if command starts with "G1"
  if (!command.startsWith("G2")) {
    Serial.println("Error: Command must start with G2");
    return;
  }

  // Remove "G2" from the start
  command = command.substring(2);
  command.trim();  // Remove any spaces after "G2"

  // Parse the command while it has content
  while (command.length() > 0) {
    int spaceIndex = command.indexOf(' ');
    String token;

    // Extract the next token (e.g., "V1")
    if (spaceIndex == -1) {
      token = command;  // Last token
      command = "";
    } else {
      token = command.substring(0, spaceIndex);
      command = command.substring(spaceIndex + 1);
      command.trim();  // Remove leading spaces from remaining command
    }

    // Process the token if it’s a motor command (e.g., "V1")
    if (token.startsWith("V") && token.length() == 2) {
      char idxChar = token.charAt(1);
      if (idxChar >= '1' && idxChar <= '4') {
        int motorIndex = idxChar - '1';  // Convert '1' to 0, '2' to 1, etc.

        // Extract the speed value
        spaceIndex = command.indexOf(' ');
        String valueStr;
        if (spaceIndex == -1) {
          valueStr = command;  // Last value
          command = "";
        } else {
          valueStr = command.substring(0, spaceIndex);
          command = command.substring(spaceIndex + 1);
          command.trim();  // Trim remaining command
        }

        // Convert value to integer and validate
        int value = valueStr.toInt();
        if (value >= -255 && value <= 255) {
          MotorSpeed[motorIndex].Speed = value;
        } else {
          Serial.println("Error: Speed value out of range");
          return;
        }
      } else {
        Serial.println("Error: Invalid motor index");
        return;
      }
    } else if (token.startsWith("P") && token.length() == 2) {
      char idxChar = token.charAt(1);
      if (idxChar >= '1' && idxChar <= '4') {
        int motorIndex = idxChar - '1';  // Convert '1' to 0, '2' to 1, etc.

        // Extract the speed value
        spaceIndex = command.indexOf(' ');
        String valueStr;
        if (spaceIndex == -1) {
          valueStr = command;  // Last value
          command = "";
        } else {
          valueStr = command.substring(0, spaceIndex);
          command = command.substring(spaceIndex + 1);
          command.trim();  // Trim remaining command
        }

        // Convert value to integer and validate
        int value = valueStr.toInt();
        if (value >= -255 && value <= 255) {
          MotorSpeed[motorIndex].Speed = value;
        } else {
          Serial.println("Error: Speed value out of range");
          return;
        }
      } else {
        Serial.println("Error: Invalid motor index");
        return;
      }
    } else if (token.startsWith("A") && token.length() == 2) {
      char idxChar = token.charAt(1);
      if (idxChar >= '1' && idxChar <= '4') {
        int motorIndex = idxChar - '1';  // Convert '1' to 0, '2' to 1, etc.

        // Extract the speed value
        spaceIndex = command.indexOf(' ');
        String valueStr;
        if (spaceIndex == -1) {
          valueStr = command;  // Last value
          command = "";
        } else {
          valueStr = command.substring(0, spaceIndex);
          command = command.substring(spaceIndex + 1);
          command.trim();  // Trim remaining command
        }

        // Convert value to integer and validate
        int value = valueStr.toInt();
        if (value >= -255 && value <= 255) {
          MotorSpeed[motorIndex].Speed = value;
        } else {
          Serial.println("Error: Speed value out of range");
          return;
        }
      } else {
        Serial.println("Error: Invalid motor index");
        return;
      }
    } else{
      Serial.println("Error: Invalid token format");
      return;
    }
  }

  // Debug: Print all motor speeds
  for (int i = 0; i < MotorNum; i++) {
    Serial.print("V");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(MotorSpeed[i].Speed);
  }
}
  #ifdef I2C_COM
  void receiveEvent(int howMany) {
    while (Wire.available()) {
      char cmd = Wire.read();
      if (cmd == 't') break;
      GC_cmd += cmd;
    }
    if (GC_cmd.length() > 0) {
      Serial.println("Received: " + GC_cmd);
      Gcode_process(GC_cmd);
      GC_cmd = "";

      for (int i = 0; i < MotorNum; i++) {
        WritePWM(MotorSpeed[i].Speed, (MotorName)i);
      }
    }
  }
  #endif
  #ifdef SERIAL_COM
  void Uart_receive(){
    while (Serial2.available() > 0) {
    GC_cmd = Serial2.readStringUntil('t');
    Gcode_process(GC_cmd);
  }
  }
  #endif


void setup() {
  initpins();
  Serial.begin(9600);
  Serial2.begin(115200);
  #ifdef I2C_COM
    Wire.begin(1);
    Wire.onReceive(receiveEvent);
  #endif
}

void loop() {
  #ifdef SERIAL_COM
  Uart_receive();
  #endif
  for (int i = 0; i < MotorNum; i++) {
    WritePWM(MotorSpeed[i].Speed, (MotorName)i);
  }
  delay(10); // Minimal loop, as control is event-driven
}