#pragma once

#ifndef PICO_BASE_CONFIG_H
#define PICO_BASE_CONFIG_H

    #define WHEELBASE_LENGTH 175.0 / 1000     // mm to m by x-axis
    #define WHEELBASE_WIDTH 165.0 / 1000 // mm to m by y-axis
    #define WHEEL_RADIUS 96.0 / 1000 // Wheel radius in mm -> meters

    #define GC_BOARD "G1"
    #define ARM_ENCODER_TPR 8910 // Encoder ticks per revolution for the arm (11 x 270 x 3)
    float PID_ARM[4] = {0.0, 0.0, 0.0};
    float dt;
    float maxIntegralValue;

#endif // PICO_BASE_CONFIG_H