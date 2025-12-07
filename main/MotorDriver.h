#pragma once
#include <Arduino.h>
#include "Config.h"

class MotorDriver {
public:
    MotorDriver(const byte pins[MOTOR_PIN_COUNT]);

    // 
    void init();
    void setSpeed(const MotorSpeeds &speed);

    // Speed patterns
    void idle();
    void forward();
    void backward();
    void left();
    void right();
    void stop();

private:
    byte motorPins[4];
};