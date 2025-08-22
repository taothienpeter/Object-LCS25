#include "Gcode.h" 
#define PICO_BASE // uncomment to upload code to PICO drivetrain
//#define PICO_ARM // 

#ifdef PICO_BASE
Gcode GC_base; // G1 tên board pico 1
#elif PICO_ARM
Gcode gcode("G2"); // G2 tên board pico 2
#endif
float example_array[4] = {0.00, 0.02, 0.03, 0.04}; // Example array to be passed to the Gcode class
void setup(){
    Serial.begin(9600);
}
void loop(){
    while (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        Serial.println("Received: " + command);
        String result = GC_base.Gcode_process(command, example_array, "G1", "M"); // G1 is the command for the base, M1 is the command for the arm
        Serial.println(result);
    }
}