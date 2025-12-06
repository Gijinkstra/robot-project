#include "MotorDriver.h"

MotorDriver::MotorDriver(const byte pins[MOTOR_PIN_COUNT]) {
    for (byte i = 0; i < MOTOR_PIN_COUNT; i++) {
        motorPins[i] = pins[i];
    }
}

void MotorDriver::init() {
    for (byte i = 0; i < MOTOR_PIN_COUNT; i++) {
        pinMode(motorPins[i], OUTPUT);
    }
}

void MotorDriver::setSpeed(const MotorSpeeds &speed) {
    analogWrite(motorPins[0], constrain(speed.leftA, MIN_SPEED, SET_SPEED));
    analogWrite(motorPins[1], constrain(speed.leftB, MIN_SPEED, SET_SPEED));
    analogWrite(motorPins[2], constrain(speed.rightA, MIN_SPEED, SET_SPEED));
    analogWrite(motorPins[3], constrain(speed.rightB, MIN_SPEED, SET_SPEED));
}

void MotorDriver::idle()         { setSpeed(MotorPatterns::idle);        }
void MotorDriver::forward()      { setSpeed(MotorPatterns::forward);     }
void MotorDriver::backward()     { setSpeed(MotorPatterns::backward);    }
void MotorDriver::left()         { setSpeed(MotorPatterns::left);        }
void MotorDriver::right()        { setSpeed(MotorPatterns::right);       }
void MotorDriver::stop()         { setSpeed(MotorPatterns::stop);        }