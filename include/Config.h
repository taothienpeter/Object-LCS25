
#include <math.h>
//=============================DRIVER CONFIG=============================
    // Motor pin definitions
    #define MOTOR_FRONTLEFT_A 21
    #define MOTOR_FRONTLEFT_B 20
    #define MOTOR_FRONTRIGHT_A 7
    #define MOTOR_FRONTRIGHT_B 6
    #define MOTOR_BACKLEFT_A 26
    #define MOTOR_BACKLEFT_B 27
    #define MOTOR_BACKRIGHT_A 2
    #define MOTOR_BACKRIGHT_B 3
    // Encoder pin definitions
    #define ENCODER_FRONTLEFT_A 0
    #define ENCODER_FRONTLEFT_B 1
    #define ENCODER_FRONTRIGHT_A 29
    #define ENCODER_FRONTRIGHT_B 23
    #define ENCODER_BACKLEFT_A 15
    #define ENCODER_BACKLEFT_B 14
    #define ENCODER_BACKRIGHT_A 16
    #define ENCODER_BACKRIGHT_B 17
    
    #define WHEELBASE_LENGTH 250.0 / 1000     // mm to m by x-axis
    #define WHEELBASE_WIDTH 270.0 / 1000 // mm to m by y-axis
    #define WHEEL_RADIUS 96.0 / 1000 // Wheel radius in mm -> meters
    #define WHEEL_CIRCUMFERENCE (2 * M_PI * WHEEL_RADIUS) // Wheel circumference in meters
    
    #define WHEEL_ENC_TICKS_PER_REV 660 // Number of ticks per revolution of the wheel encoder (11 x 30 x 2)
    #define ARM_ENC_TICKS_PER_REV 17820 // Encoder ticks per revolution for the arm (11 x 270 x 3 x 2)
    
    #define BASE_GCODE_COMMUNICATION "G1"
    #define ARM_GCODE_COMMUNICATION "G2"
    #define SPEED_THROTTLE_MAX 0.5 // speed throttle

    // float PID_WHEEL_FRONTLEFT[3] = {0.0, 0.0, 0.0};
    // float PID_WHEEL_FRONTRIGHT[3] = {0.0, 0.0, 0.0};
    // float PID_WHEEL_BACKLEFT[3] = {0.0, 0.0, 0.0};
    // float PID_WHEEL_BACKRIGHT[3] = {0.0, 0.0, 0.0};
    // float PID_ARM[3] = {0.0, 0.0, 0.0};
    #define dt 0.05
    #define maxIntegralValue 0.2

//=============================END OF DRIVER CONFIG=============================
