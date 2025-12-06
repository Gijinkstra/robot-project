#include "LineSensors.h"

LineSensors::LineSensors(const byte anPins[], byte anCount,
                         const byte digPins[], byte digCount)
    : analogCount(anCount), digitalCount(digCount)
{
    for (byte i = 0; i < analogCount; i++) {
        analogPins[i] = anPins[i];
    }
    for (byte i = 0; i < digitalCount; i++) {
        digitalPins[i] = digPins[i];
    }
}

void LineSensors::init() {
    for (byte i = 0; i < analogCount; i++) {
        pinMode(analogPins[i], INPUT);
    }
    for (byte i = 0; i < digitalCount; i++) {
        pinMode(digitalPins[i], INPUT);
    }
}

void LineSensors::readAnalog() {
    for (byte i = 0; i < analogCount; i++) {
        analogValues[i] = analogRead(analogPins[i]);
    }
}

void LineSensors::readDigital() {
    for (byte i = 0; i < digitalCount; i++) {
        digitalValues[i] = digitalRead(digitalPins[i]);
    }
}

bool LineSensors::allDigitalOff() const {
    for (byte i = 0; i < digitalCount; ++i) {
        if (digitalValues[i]) {
            return false;
        }
    }
    return true;
}

unsigned int LineSensors::computeLinePosition() {
    long weightedSum = 0;
    unsigned int sum = 0;

    for (byte i = 0; i < analogCount; ++i) {
        int reading = analogValues[i];
        weightedSum += (long)reading * i * SCALING_FACTOR;
        sum += reading;
    }

    if (sum == 0) {
        lastLinePosition = SETPOINT;
    } else {
        lastLinePosition = weightedSum / sum;
    }
    return lastLinePosition;
}

unsigned int LineSensors::linePosition() const {
    return lastLinePosition;
}

int LineSensors::getAnalogReading(byte i) const {
    return analogValues[i];
}

bool LineSensors::getDigitalReading(byte i) const {
    return digitalValues[i];
}