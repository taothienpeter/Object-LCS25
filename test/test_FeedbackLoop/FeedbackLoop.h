#include "Precise_Compute.h"
#define ENCODER_LF_A 1
#define ENCODER_LF_B 0
Wheel wheel(ENCODER_LF_A, ENCODER_LF_B, true, 1.0, 0.0, 0.0, 0.05); // Create a Wheel object with the specified parameters
float desiredAngularVelocity = 0.5; 
void WritePWM(short Speed){
    // Serial.print(String(Motor));
    // Serial.println(Speed);
    if(Speed>0){
      analogWrite(21, abs(Speed));
      analogWrite(20, LOW);
    }else if(Speed<0){
      analogWrite(21, LOW);
      analogWrite(20, abs(Speed));
    }else {
      analogWrite(21, 255);
      analogWrite(20, 255);
    }
  };
void setup(){
    Serial.begin(9600); // Initialize serial communication at 9600 baud rate
    wheel.resetEncoder(); // Reset the encoder to zero
    desiredAngularVelocity = 0.0;
}
void loop(){
    float pidOutput = wheel.reachAngularVelocity(desiredAngularVelocity, 0.05); // Calculate the PID output
    WritePWM(pidOutput*255); // Set the motor speed to 0
    Serial.print(">");
    Serial.print("PID Output:"); // Print the PID output to the serial monitor
    Serial.print(pidOutput);
    Serial.print(","); // Print the current angular velocity to the serial monitor
    Serial.print("Set velo:"); // Print the PID output to the serial monitor
    Serial.println(desiredAngularVelocity);
    desiredAngularVelocity = 1.0;
    delay(50); // Delay for 50 milliseconds before the next iteration
}