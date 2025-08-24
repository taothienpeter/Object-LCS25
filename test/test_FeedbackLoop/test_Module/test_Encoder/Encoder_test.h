#pragma once

#include "Encoder.h"
#include "Config.h"
#include <Arduino.h>
#include <unity.h>

const int pinA = ENCODER_FRONTRIGHT_A;
const int pinB = ENCODER_FRONTRIGHT_B;

Encoder encoder_Frontleft(pinA, pinB, WHEEL_ENC_TICKS_PER_REV, false);

void printPosition();
void triggerA();
void triggerB();
void setup();
void loop();