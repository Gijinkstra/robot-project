#pragma once
#include <Arduino.h>

// --------- Hardware Config ---------------
constexpr byte MOTOR_PINS[] = {9, 10, 5, 6}; // Left to right.
constexpr byte AN_SENSOR_PINS[] = {A0, A1, A2, A3, A4};
constexpr byte MOTOR_PIN_COUNT = sizeof(MOTOR_PINS) / 
                                 sizeof(MOTOR_PINS[0]);
constexpr byte AN_SENSOR_COUNT = sizeof(AN_SENSOR_PINS) / 
                                 sizeof(AN_SENSOR_PINS[0]);
constexpr byte DIG_SENSOR_PINS[] = {2, 3, 4};
constexpr byte DIG_SENSOR_COUNT = sizeof(DIG_SENSOR_PINS) / 
                                 sizeof(DIG_SENSOR_PINS[0]);
constexpr unsigned int MIN_SPEED = 0;
constexpr unsigned int MAX_SPEED = 255;   
constexpr unsigned int AN_SENSOR_MAX = 1023;
constexpr unsigned int AN_SENSOR_MIN = 0;
constexpr unsigned int BAUD_RATE = 9600;

// --------- Control Config ---------------
constexpr byte PWM_LEVEL_INCREMENT = 5;
constexpr byte ACCELERATION_INTERVAL = 5;
constexpr unsigned int TURN_DURATION = 1000;
constexpr unsigned int MOTOR_OFFSET = 12;
constexpr unsigned int SET_SPEED = 60;
constexpr unsigned int TURN_SPEED = 100;
constexpr unsigned int SCALING_FACTOR = 1000;
constexpr unsigned int SETPOINT = 2000;
constexpr unsigned int SHARP_LEFT_THR = 3000;
constexpr unsigned int SHARP_RIGHT_THR = 1000;
constexpr unsigned int DEADBAND_ERROR = 20;
constexpr unsigned int WAIT_TIME = 300;

// --------- Control Types ---------------
enum class AutoState {
    Idle,
    Accelerate,
    PIDLoop,
    Stop,
    HardLeft,
    HardRight,
    LineFinish
};

enum class ManualState {
    Idle,
    Forward,
    Backwards,
    Right,
    Left,
    Stop
};

struct MotorSpeeds {
    int leftA;
    int leftB;
    int rightA;
    int rightB;
};

namespace MotorPatterns {
    constexpr MotorSpeeds idle      = {MIN_SPEED, MIN_SPEED, 
                                       MIN_SPEED, MIN_SPEED};
    constexpr MotorSpeeds forward   = {SET_SPEED, MIN_SPEED, 
                                       SET_SPEED - MOTOR_OFFSET, MIN_SPEED};
    constexpr MotorSpeeds backward  = {MIN_SPEED, SET_SPEED, 
                                       MIN_SPEED, SET_SPEED};
    constexpr MotorSpeeds right     = {TURN_SPEED, MIN_SPEED, 
                                       MIN_SPEED, TURN_SPEED};
    constexpr MotorSpeeds left      = {MIN_SPEED, TURN_SPEED, 
                                       TURN_SPEED, MIN_SPEED};
    constexpr MotorSpeeds stop      = {1, 1, 1, 1};
}

namespace PIDGains {
    constexpr float Kp = 0.22f; // SET_SPEED = 60;
    constexpr float Kd = 0.02f; // SET_SPEED = 60;
}