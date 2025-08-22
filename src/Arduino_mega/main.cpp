/*This code created by Phạm Quang Thiên Tảo, 27/4/2025*/

// #include "test\UNO_FullSystem.h" // The final code for the UNO_FullSystem.h file
// #include "test\UNO_FullSystem_1Pilot.h" // The final code for the UNO_FullSystem.h file
// #include "ROS_control\ROS_Fullsystem.h" // The final code for the UNO_FullSystem.h file
// #include "test\MPU6050_DMP6.h"
// #include "test\MPU6050_DMP6_using_DMP_V6.12.h"


// #include <Arduino_mega.h>


/*====================================TEST module====================================*/
// #include "Gcode.h'" 
/*====================================TEST module====================================*
#include "Thread.h"
#include "Auto_control.h"
Timer timer(0); // Timer for 1 second interval
void setup() {
  Serial.begin(9600); // Initialize serial communication at 115200 baud rate
//   timer.start(); // Start the timer
}
void loop() {
    if(Serial.available() > 0) { // Check if there is data available to read from the serial port
        int period = Serial.readStringUntil('\n').toInt(); // Read the command from the serial port
        timer.start();
        Serial.print("Timer started with period: ");
        Serial.println(period); // Print the period to the serial monitor
    }
       
    if(timer.stop()) Serial.println("Timer elapsed"); // Stop the timer if it has elapsed
    delay(100); // Delay for 100 milliseconds
}
//====================================TEST module====================================*
#include "Automation_Param.h"
float motorSpeed[4] = {0, 0, 0, 0}; 
Semi_Autonomous robot; // Create an instance of the Semi_Autonomous class
void setup() {
  Serial.begin(9600); // Initialize serial communication at 115200 baud rate
//   timer.start(); // Start the timer
}
void loop() {
    if(Serial.available() > 0) { // Check if there is data available to read from the serial port
        int period = Serial.readStringUntil('\n').toInt(); // Read the command from the serial port
        Serial.print("Timer started with period: ");
        Serial.println(period); // Print the period to the serial monitor
    }
    robot.Drivetrain_Kinematic_Compute_Method1(motorSpeed, 0, 0, 0); // Call the function with dummy values
    delay(100); // Delay for 100 milliseconds
}//====================================TEST module====================================*/