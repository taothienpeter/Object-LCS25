#include <Arduino.h>
#include <Wire.h>
#include <Thread.h>
#include "Gcode.h"
#include "Precise_Compute.h"
#define MotorNum 4
#define SERIAL_COM // for serial communication
// #define I2C_COM // for I2C communication
Gcode GC_base;
UART Serial2(4, 5);
  // int count=0;
typedef struct {
  uint8_t Pin_A, Pin_B;
  uint8_t encoder_A, encoder_B;
  // volatile long encoder = 0; // Encoder value
  // Encoder *encoder(Pin_A, Pin_A, true);
  float Speed;
  
  bool isHoming = false; // Flag to indicate if the encoder is homing
  double Kp = 1.0, Ki = 0.0, Kd = 0.0;

} MotorData;
MotorData MotorSpeed[MotorNum];

typedef enum { MOTOR_LF, MOTOR_RF, MOTOR_LB, MOTOR_RB } MotorName;

// Wheel Motor_LF(MotorSpeed[MOTOR_LF].encoder_A, MotorSpeed[MOTOR_LF].encoder_B, true, MotorSpeed[MOTOR_LF].Kp, MotorSpeed[MOTOR_LF].Ki, MotorSpeed[MOTOR_LF].Kd);
// Wheel Motor_RF(MotorSpeed[MOTOR_RF].encoder_A, MotorSpeed[MOTOR_RF].encoder_B, true, MotorSpeed[MOTOR_RF].Kp, MotorSpeed[MOTOR_RF].Ki, MotorSpeed[MOTOR_RF].Kd);
Wheel Motor_LB(MotorSpeed[MOTOR_LB].encoder_A, MotorSpeed[MOTOR_LB].encoder_B, false, MotorSpeed[MOTOR_LB].Kp, MotorSpeed[MOTOR_LB].Ki, MotorSpeed[MOTOR_LB].Kd);
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
    pinMode(MotorSpeed[i].encoder_A, INPUT_PULLUP);
    pinMode(MotorSpeed[i].encoder_B, INPUT_PULLUP);  
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
  
  // if(digitalRead(ENCODER_LF_A) && Motor == MOTOR_LB){
  //   MotorSpeed[MOTOR_LB].Speed = 0;
  //   Motor_LB.resetEncoder(); // Reset the encoder to zero
  //   Serial.println("Homing done");
  // }
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
// void WritePWM_BuffSpeed(){

//   WritePWM(Motor_LB.reachAngularPosition(MotorSpeed[MOTOR_LB].Speed), MOTOR_LB);
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
    // digitalWrite(25, HIGH); // Turn on the LED to indicate data received
    // Serial.println("FLAG1:");
    // Serial.println(GC_cmd);
    GC_cmd = GC_base.Gcode_process(GC_cmd, &MotorSpeed->Speed, GC_BOARD, "V");
    // printf("Motor %p position: ", &MotorSpeed->Speed);
    // Serial.println("FLAG2:");
    // Serial.println(GC_cmd);
    // Serial.print(sizeof(MotorSpeed));
    // Serial.print(Motor_LB.getCurrentAngle());
    // Serial.print(Motor_LB.getTicks());
    for (int i = 0; i < MotorNum; i++) {
      // MotorSpeed[i].Speed = Motor_LF.reachAngularVelocity(MotorSpeed[i].Speed, 0.05)*255; // Update the motor speed
      
      // if (i == MOTOR_LB){ // Homing c.
        WritePWM(MotorSpeed[i].Speed, (MotorName)i);
      // }
    }
    Serial.println();
    digitalWrite(25, MotorSpeed[3].Speed); // Turn off the LED after processing
  }
  #endif
}   
// void updateE_LF_A(){ MotorSpeed[MOTOR_LF].isHoming = true;}

// void updateE_LF_B(){Motor_LF.triggerB();}
// void updateE_RF_A(){Motor_RF.triggerA();}
// void updateE_RF_B(){Motor_RF.triggerB();}
// void updateE_LB_A(){
// if (digitalRead(15))
//   {count++;}
// else{
//   count--;
//   // Serial.println(count);
// }
// Motor_LB.triggerA();

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
  // attachInterrupt(14, updateE_LB_A, RISING);
  // attachInterrupt(digitalPinToInterrupt(MotorSpeed[MOTOR_LB].encoder_B), updateE_LB_B, RISING);
  // attachInterrupt(digitalPinToInterrupt(MotorSpeed[MOTOR_RB].encoder_A), updateE_RB_A, RISING);
  // attachInterrupt(digitalPinToInterrupt(MotorSpeed[MOTOR_RB].encoder_B), updateE_RB_B, RISING);
}

void loop() {
  #ifdef SERIAL_COM
  Uart_receive();
  #endif
  delay(10);
}
