#include "LineSensors.h"

LineSensors::LineSensors(const byte anPins[], byte anCount,
                         unsigned int defaultThreshold)
    : analogCount(anCount), threshold_(defaultThreshold)
{
    for (byte i = 0; i < analogCount; i++) {
        analogPins[i] = anPins[i];
    }
}

void LineSensors::init() {
    for (byte i = 0; i < analogCount; i++) {
        pinMode(analogPins[i], INPUT);
    }
}

void LineSensors::readAnalog() {
    for (byte i = 0; i < analogCount; i++) {
        analogValues[i] = analogRead(analogPins[i]);
    }
}

bool LineSensors::allAnalogAbove() const {
    return allAnalogAbove(threshold_);
}

bool LineSensors::allAnalogAbove(unsigned int customThreshold) const {
    for (byte i = 0; i < analogCount; i++) {
        if (analogValues[i] < customThreshold) {
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