/*This code created by Phạm Quang Thiên Tảo, 27/4/2025*/
// #include "PICO_Base.h" // The final code for the PCIO Basedrive.h file
// #include "PICO_Arm.h" // The final code for the PCIO Armdrive.h file
// #define TEST_GCODE // uncomment to test Gcode module
// #define TEST_PID // uncomment to test PID module
/*==================TEST module====================================*/
// #include <Pico_base.h>
#include "Encoder.h"
#include "Config.h"
#include <Arduino.h>
// #include <unity.h>

const int pinA = ENCODER_FRONTRIGHT_A;
const int pinB = ENCODER_FRONTRIGHT_B;
Encoder encoder_Frontleft(pinA, pinB, WHEEL_ENC_TICKS_PER_REV, false);

// volatile long position = 0;

void printPosition() {
    // Serial.println("Position: " + (String)position);
    Serial.print('$');
    Serial.print(encoder_Frontleft.getTicks());
    Serial.println(';');
}

void triggerA() {
    encoder_Frontleft.triggerA();
//     if (digitalRead(pinA) != digitalRead(pinB)) {
//         position++;
//     } else {
//         position--;
//     }
}

void triggerB() {
    encoder_Frontleft.triggerB();
//     if (digitalRead(pinA) == digitalRead(pinB)) {
//         position++;
//     } else {
//         position--;
//     }
}

void setup() {
    // pinMode(pinA, INPUT_PULLUP);
    // pinMode(pinB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pinA), triggerA, RISING);
    attachInterrupt(digitalPinToInterrupt(pinB), triggerB, RISING);
    Serial.begin(9600);
}

void loop() {
    printPosition();
}
