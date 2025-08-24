#include "Motor_test.h"
#define SPEED_MAX 1
#define SPEED_MIN -1


void setup(){
    Serial.begin(9600);
}
void loop(){
 Serial.print(speed); 
 Serial.print("\t");
 motor_test.setMotorControl(accellerateTest(&reverse, &speed));
 Serial.print(motor_test.getPWMMovingThreshold()); 
 Serial.print("\t");
 Serial.println(motor_test.getSendedPWM());
 delay(200);
}

float accellerateTest(bool* reverse, float* speed){
    if(*reverse){
        if(*speed<SPEED_MIN) *reverse = false; 
        (*speed)=*speed-0.05;
    } else{
        if(*speed>SPEED_MAX) *reverse = true;
        (*speed)=*speed+0.05;
    }
    return *speed;
}