#pragma once
#include <Arduino.h>
#include "Config.h"

class LineSensors {
public:
    LineSensors(const byte anPins[], byte anCount, 
                unsigned int threshold = SENSOR_THR);

    void init();
    void readAnalog();

    bool allAnalogAbove() const;
    bool allAnalogAbove(unsigned int threshold) const;

    unsigned int computeLinePosition();
    unsigned int linePosition() const;

    int  getAnalogReading(byte i) const;

private:
    byte analogPins[AN_SENSOR_COUNT];
    byte analogCount;

    int  analogValues[AN_SENSOR_COUNT];
    unsigned int lastLinePosition = SETPOINT;

    unsigned int threshold_;
};