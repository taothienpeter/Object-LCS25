#include <Arduino.h>
#include <Wire.h>
#define MotorNum 4
#define MAX_CMD_LEN 32
#define SERIAL_COM // for serial communication
// #define I2C_COM // for I2C communication

UART Serial2(4, 5);
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
typedef struct {
  uint8_t Pin_A, Pin_B;
  short Speed;
  double Kp, Ki, Kd;
} MotorData;
typedef enum { MOTOR_LF, MOTOR_RF, MOTOR_LB, MOTOR_RB } MotorName;
MotorData MotorSpeed[MotorNum];
String GC_cmd;

void initpins(){
  MotorSpeed[MOTOR_LF] = {8, 9, 0};
  MotorSpeed[MOTOR_RF] = {7, 6, 0};
  MotorSpeed[MOTOR_LB] = {3, 2, 0};
  MotorSpeed[MOTOR_RB] = {0, 1, 0};

  for(int i=0; i<MotorNum; i++){
    pinMode(MotorSpeed[i].Pin_A, OUTPUT);
    pinMode(MotorSpeed[i].Pin_B, OUTPUT);  
  }
};
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
void Gcode_process(String command){
        command.trim(); // Remove leading/trailing whitespace
        if (command.startsWith("G1")) {
            // Split the string into tokens
            int spaceIndex;
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

                if (token.startsWith("V")) {
                    char motorIndexChar = token.charAt(1);
                    if (motorIndexChar >= '1' && motorIndexChar <= '4') {
                        int motorIndex = motorIndexChar - '1'; // Convert char to index (0-3)
                        // Get the value after "M<index>"
                        spaceIndex = command.indexOf(' ');
                        String valueStr;
                        if (spaceIndex == -1) {
                            valueStr = command;
                            command = "";
                        } else {
                            valueStr = command.substring(0, spaceIndex);
                            command = command.substring(spaceIndex + 1);
                            command.trim();
                        }
                        int value = valueStr.toInt();
                        if (value != 0 || valueStr == "0") { // Check if it's a valid number
                            MotorSpeed[motorIndex].Speed = value;
                            // Serial.println(value);
                            // Serial.println('\t');
                        } else {
                            // Serial.println(command);
                            Serial.println("Invalid value for motor position.");
                            break;
                        }
                    } else {
                        Serial.println("Invalid motor index.");
                        break;
                    }
                } else {
                    Serial.println("Invalid command format.");
                    break;
                }
            }
        } else {
            Serial.println("Command must start with G1.");
        }
    };
#ifdef I2C_COM
void receiveEvent(int howMany) {
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
}
#endif
#ifdef SERIAL_COM
void Uart_receive(){
  while (Serial2.available() > 0) {
  GC_cmd = Serial2.readStringUntil('t');
  Gcode_process(GC_cmd);
  Serial.println(GC_cmd);
  }
}   
#endif
void setup() {
  // put your setup code here, to run once:
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
  delay(100);
}
