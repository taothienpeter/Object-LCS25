#ifndef GCODE_H
#define GCODE_H
#include <Arduino.h>
#define Sizeof_Motordata 10

class Gcode {
private:
    // String Mode_cmd = "G"; // Command string
    char TotalMotorChar = '4'; // Total number of motors
    // char GC_cmd_type[6] = { 'M', 'V', 'A', 'P', 'I', 'D' };
public:
    explicit Gcode();
    // void GC_decode(String* GC_return, int spaceIndex, String token, String GC_cmd, short gct, short* input_array);
    // String Gcode_generate(String _GC_cmd);
    String Motion_process(String* command, float* input_array, String Oject_cmd, short* spaceIndex, String token, char TotalMotorChar);
    String Config_process(String* command, float* input_array, String Oject_cmd, short* spaceIndex, String token, char TotalMotorChar);
    String Gcode_process(String command, float* input_array, String Mode_cmd, String Object_cmd);
};
#endif
