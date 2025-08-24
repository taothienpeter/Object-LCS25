#include <Arduino.h>
#include "Config.h"
#include "Pid.h"
#include <stdio.h>
#include <math.h>
PID pid_test(2.0, 5.0, 1.0);

float SinWave_GenTest(){
    return sin((millis()/1000.0)*2*PI*0.05);
}
void setup(){
    Serial.begin(112500);
    pid_test.setMaxIntegralValue(0.8);
    pid_test.setOutputLimits(-1.0, 1.0);
    while(Serial.available()){
        float kP, kI, kD;
        String string= Serial.readStringUntil('\n');
        sscanf(string.c_str(), "%f %f %f", &kP, &kI, &kD);
        pid_test.setPidParams(kP, kI, kD);
    }
    
}
void loop(){
    pid_test.update(0, SinWave_GenTest());
    
    Serial.print(">Sin:");
    Serial.print(SinWave_GenTest());
    Serial.print(",");

    Serial.print("PID:");
    Serial.println(pid_test.output);
    delay(500);
}
