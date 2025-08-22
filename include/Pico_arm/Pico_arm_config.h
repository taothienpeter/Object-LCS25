#pragma once

#ifndef PICO_ARM_CONFIG_H
#define PICO_ARM_CONFIG_H

    #define MOTOR_FRONTLEFT_A 2
    #define MOTOR_FRONTLEFT_B 3
    #define MOTOR_FRONTRIGHT_A 20
    #define MOTOR_FRONTRIGHT_B 21
    #define MOTOR_BACKLEFT_A 7
    #define MOTOR_BACKLEFT_B 6
    #define MOTOR_BACKRIGHT_A 27
    #define MOTOR_BACKRIGHT_B 26

    #define ENCODER_FRONTLEFT_A 0
    #define ENCODER_FRONTLEFT_B 1
    #define ENCODER_FRONTRIGHT_A 29
    #define ENCODER_FRONTRIGHT_B 23
    #define ENCODER_BACKLEFT_A 15
    #define ENCODER_BACKLEFT_B 14
    #define ENCODER_BACKRIGHT_A 16
    #define ENCODER_BACKRIGHT_B 17

    #define GC_BOARD "G2"
    float PID_FRONTLEFT[4] = {0.0, 0.0, 0.0};
    float PID_FRONTRIGHT[4] = {0.0, 0.0, 0.0};
    float PID_BACKLEFT[4] = {0.0, 0.0, 0.0};
    float PID_BACKRIGHT[4] = {0.0, 0.0, 0.0};
    float dt;
    float maxIntegralValue;
#endif // PICO_ARM_CONFIG_H