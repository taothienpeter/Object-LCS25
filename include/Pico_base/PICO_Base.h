#include <Arduino.h>
#include <Wire.h>
#include <Thread.h>
#include "Gcode.h"
// #include "Encoder.h"
// #include "Pid.h"
#include "Precise_Compute.h"
#define MotorNum 4
#define SERIAL_COM // for serial communication
// #define I2C_COM // for I2C communication

Gcode GC_base;
UART Serial2(4, 5);
// Pin definitions for motor outputs
  
//
  #define GC_BOARD "G1"
typedef struct {
  uint8_t Pin_A, Pin_B;
  uint8_t encoder_A, encoder_B;
  // volatile long encoder = 0; // Encoder value
  // Encoder *encoder(Pin_A, Pin_A, true);
  float Speed;
  double Kp = 1.0, Ki = 0.0, Kd = 0.0;
} MotorData;
typedef enum { MOTOR_LF, MOTOR_RF, MOTOR_LB, MOTOR_RB } MotorName;
MotorData MotorSpeed[MotorNum];

// Wheel Motor_LF(MotorSpeed[MOTOR_LF].encoder_A, MotorSpeed[MOTOR_LF].encoder_B, true, MotorSpeed[MOTOR_LF].Kp, MotorSpeed[MOTOR_LF].Ki, MotorSpeed[MOTOR_LF].Kd);
// Wheel Motor_RF(MotorSpeed[MOTOR_RF].encoder_A, MotorSpeed[MOTOR_RF].encoder_B, true, MotorSpeed[MOTOR_RF].Kp, MotorSpeed[MOTOR_RF].Ki, MotorSpeed[MOTOR_RF].Kd);
// Wheel Motor_LB(MotorSpeed[MOTOR_LB].encoder_A, MotorSpeed[MOTOR_LB].encoder_B, true, MotorSpeed[MOTOR_LB].Kp, MotorSpeed[MOTOR_LB].Ki, MotorSpeed[MOTOR_LB].Kd);
// Wheel Motor_RB(MotorSpeed[MOTOR_RB].encoder_A, MotorSpeed[MOTOR_RB].encoder_B, true, MotorSpeed[MOTOR_RB].Kp, MotorSpeed[MOTOR_RB].Ki, MotorSpeed[MOTOR_RB].Kd);

String GC_cmd;

void initpins(){
  // init motor pins and set encoder = 0
  MotorSpeed[MOTOR_LF] = {MOTOR_LF_A, MOTOR_LF_B, ENCODER_LF_A, ENCODER_LF_B, 0};
  MotorSpeed[MOTOR_RF] = {MOTOR_RF_A, MOTOR_RF_B, ENCODER_RF_A, ENCODER_RF_B, 0};
  MotorSpeed[MOTOR_LB] = {MOTOR_LB_A, MOTOR_LB_B, ENCODER_LB_A, ENCODER_LB_B, 0};
  MotorSpeed[MOTOR_RB] = {MOTOR_RB_A, MOTOR_RB_B, ENCODER_RB_A, ENCODER_RB_B, 0};
  pinMode(25, OUTPUT); // LED pin for indicating data received

  for(int i=0; i<MotorNum; i++){
    pinMode(MotorSpeed[i].Pin_A, OUTPUT);
    pinMode(MotorSpeed[i].Pin_B, OUTPUT);  
  }
  #ifdef I2C_COM
  Wire.begin(1);
  Wire.onReceive(receiveEvent);
  #endif
};

// Motor control function
void WritePWM(short Speed, MotorName Motor){
  // Serial.print(String(Motor));
  // Serial.println(Speed);
  if(Speed>0){
    analogWrite(MotorSpeed[Motor].Pin_A, abs(Speed));
    analogWrite(MotorSpeed[Motor].Pin_B, LOW);
  }else if(Speed<0){
    analogWrite(MotorSpeed[Motor].Pin_A, LOW);
    analogWrite(MotorSpeed[Motor].Pin_B, abs(Speed));
  }else {
    analogWrite(MotorSpeed[Motor].Pin_A, 255);
    analogWrite(MotorSpeed[Motor].Pin_B, 255);
  }
};
// Process G-code command
// String Gcode_process(String command, float* input_array, String Mode_cmd, String Object_cmd){
// command.trim(); // Remove leading/trailing whitespace
    
// if (command.startsWith(Mode_cmd)) {
//     // Split the string into tokens
//     short spaceIndex;
//     String token;

//     // Skip "G1"
//     command = command.substring(2);
//     command.trim();

//     while (command.length() > 0) {
//       spaceIndex = command.indexOf(' ');
//       if (spaceIndex == -1) {
//           token = command;
//           command = "";
//       } else {
//           token = command.substring(0, spaceIndex);
//           command = command.substring(spaceIndex + 1);
//           command.trim();
//       }
//       Serial.println(token);
//       if (token.startsWith("V")) {
//          char motorIndexChar = token.charAt(1);
//           if (motorIndexChar >= '1' && motorIndexChar <= '4') {
//               int motorIndex = motorIndexChar - '1'; // Convert char to index (0-3)
//               // Get the value after "M<index>"
//               spaceIndex = command.indexOf(' ');
//               String valueStr;
//               if (spaceIndex == -1) {
//                   valueStr = command;
//                   command = "";
//               } else {
//                   valueStr = command.substring(0, spaceIndex);
//                   command = command.substring(spaceIndex + 1);
//                   command.trim();
//               }
//               int value = valueStr.toInt();
//               if (value != 0 || valueStr == "0") { // Check if it's a valid number
//                   *(input_array + motorIndex) = value;
//               } else {
//                   // Serial.println(command);
//                   return("Invalid value for motor position.");
//               }
//           }
//           return "Motor position updated successfully.";
//         }
//       }
//     }
//     return "0";
// }
// IC2 communication function
void receiveEvent(int howMany) {
  #ifdef I2C_COM
  char cmd;
  while (Wire.available()) {
    cmd = Wire.read();
    if(cmd == 't') break;//complete string
    GC_cmd += cmd;
  }
  // cmd= '\0';
  // GC_cmd += cmd;
  Serial.println(GC_cmd);
  Gcode_process(GC_cmd);
  GC_cmd = "";
  WritePWM(MotorSpeed[MOTOR_LF].Speed, MotorName(MOTOR_LF));
  WritePWM(MotorSpeed[MOTOR_RF].Speed, MotorName(MOTOR_RF));
  WritePWM(MotorSpeed[MOTOR_LB].Speed, MotorName(MOTOR_LB));
  WritePWM(MotorSpeed[MOTOR_RB].Speed, MotorName(MOTOR_RB));      
  #endif       
}
// Uart communication function
void Uart_receive(){
  #ifdef SERIAL_COM
  // Receive data from Serial2 and process it
  while (Serial2.available() > 0) {
    GC_cmd = Serial2.readStringUntil('t');
    digitalWrite(25, HIGH); // Turn on the LED to indicate data received
    // Serial.println(GC_cmd);
    GC_cmd = GC_base.Gcode_process(GC_cmd, &MotorSpeed->Speed, GC_BOARD, "V");
    // printf("Motor %p position: ", &MotorSpeed->Speed);
    // Serial.println(GC_cmd);
    // Serial.print(sizeof(MotorSpeed));
    for (int i = 0; i < MotorNum; i++) {
      // MotorSpeed[i].Speed = Motor_LF.reachAngularVelocity(MotorSpeed[i].Speed, 0.05)*255; // Update the motor speed
      
      Serial.print(MotorSpeed[i].Speed);
      WritePWM(MotorSpeed[i].Speed, (MotorName)i);
    }
    Serial.println();
    digitalWrite(25, LOW); // Turn off the LED after processing
  }
  
  // Reverse motor info through serial port 2
  // Serial.print(">");
  // Serial.print(Motor_LF.getCurrentAngularVelocity());
  // Serial.print(",");
  // Serial.print(Motor_RF.getCurrentAngularVelocity()); 
  // Serial.print(",");
  // Serial.print(Motor_LB.getCurrentAngularVelocity());
  // Serial.print(",");
  // Serial.println(Motor_RB.getCurrentAngularVelocity());
  
  // Serial2.print(Motor_LF.getCurrentAngularVelocity());
  // Serial2.print("\t");
  // Serial2.print(Motor_RF.getCurrentAngularVelocity()); 
  // Serial2.print("\t");
  // Serial2.print(Motor_LB.getCurrentAngularVelocity());
  // Serial2.print("\t");
  // Serial2.print(Motor_RB.getCurrentAngularVelocity());
  // Serial2.print('t');
  #endif
}   

// void updateE_LF_A(){Motor_LF.triggerA();}
// void updateE_LF_B(){Motor_LF.triggerB();}
// void updateE_RF_A(){Motor_RF.triggerA();}
// void updateE_RF_B(){Motor_RF.triggerB();}
// void updateE_LB_A(){Motor_LB.triggerA();}
// void updateE_LB_B(){Motor_LB.triggerB();}
// void updateE_RB_A(){Motor_RB.triggerA();}
// void updateE_RB_B(){Motor_RB.triggerB();}

void setup() {
  // put your setup code here, to run once:
  initpins();
  Serial.begin(9600);
  Serial2.begin(115200);
  // Serial2.setTimeout(100);// change this
  // attachInterrupt(digitalPinToInterrupt(MotorSpeed[MOTOR_LF].encoder_A), updateE_LF_A, RISING);
  // attachInterrupt(digitalPinToInterrupt(MotorSpeed[MOTOR_LF].encoder_B), updateE_LF_B, RISING);
  // attachInterrupt(digitalPinToInterrupt(MotorSpeed[MOTOR_RF].encoder_A), updateE_RF_A, RISING);
  // attachInterrupt(digitalPinToInterrupt(MotorSpeed[MOTOR_RF].encoder_B), updateE_RF_B, RISING);
  // attachInterrupt(digitalPinToInterrupt(MotorSpeed[MOTOR_LB].encoder_A), updateE_LB_A, RISING);
  // attachInterrupt(digitalPinToInterrupt(MotorSpeed[MOTOR_LB].encoder_B), updateE_LB_B, RISING);
  // attachInterrupt(digitalPinToInterrupt(MotorSpeed[MOTOR_RB].encoder_A), updateE_RB_A, RISING);
  // attachInterrupt(digitalPinToInterrupt(MotorSpeed[MOTOR_RB].encoder_B), updateE_RB_B, RISING);
}

void loop() {
  #ifdef SERIAL_COM
  Uart_receive();
  #endif
  delay(100);
}
