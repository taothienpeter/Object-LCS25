#include "Encoder.h"
#include "Config.h"
#include <Arduino.h>
#include <unity.h>

const int pinA = ENCODER_FRONTRIGHT_A;
const int pinB = ENCODER_FRONTRIGHT_B;
Encoder encoder_Frontleft(pinA, pinB, WHEEL_ENC_TICKS_PER_REV, false);

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
}

void loop() {
    RUN_TEST(printPosition);
}