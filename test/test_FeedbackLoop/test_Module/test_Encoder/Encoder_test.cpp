// #pragma once

#include "Encoder_test.h"

// volatile long position = 0;

void printPosition() {
    TEST_ASSERT_GREATER_THAN_INT16_MESSAGE(0, encoder_Frontleft.getTicks(), "Ticks are increasing (ticks>0)");
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
    UNITY_BEGIN();
    TEST_MESSAGE("Please move encoder to see the value change!");
    UNITY_END();
}

void loop() {
    printPosition();
}