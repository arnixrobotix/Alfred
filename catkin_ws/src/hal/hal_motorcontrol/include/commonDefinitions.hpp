#ifndef HAL_MOTOR_CONTROL_COMMON_DEFINITIONS
#define HAL_MOTOR_CONTROL_COMMON_DEFINITIONS

#define MOTOR_LEFT_PWM_A_GPIO 19
#define MOTOR_LEFT_PWM_B_GPIO 16
#define MOTOR_LEFT_ENCODER_CH_A_GPIO 17
#define MOTOR_LEFT_ENCODER_CH_B_GPIO 18

#define MOTOR_RIGHT_PWM_A_GPIO 26
#define MOTOR_RIGHT_PWM_B_GPIO 20
#define MOTOR_RIGHT_ENCODER_CH_A_GPIO 22
#define MOTOR_RIGHT_ENCODER_CH_B_GPIO 23

#define AS_RISING_EDGE 0
#define AS_FALLING_EDGE 1
#define AS_EITHER_EDGE 2

#define FALLING_EDGE 0
#define RISING_EDGE 1
#define NO_CHANGE 2

enum Channel
{ 
    chA, 
    chB
};

#endif