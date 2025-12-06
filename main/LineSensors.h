#pragma once
#include <Arduino.h>
#include "Config.h"

class LineSensors {
public:
    LineSensors(const byte anPins[], byte anCount,
                const byte digPins[], byte digCount);

    void init();
    void readAnalog();
    void readDigital();
    bool allDigitalOff() const;

    unsigned int computeLinePosition();
    unsigned int linePosition() const;

    int  getAnalogReading(byte i) const;
    bool getDigitalReading(byte i) const;

private:
    byte analogPins[AN_SENSOR_COUNT];
    byte digitalPins[DIG_SENSOR_COUNT];
    byte analogCount;
    byte digitalCount;

    int  analogValues[AN_SENSOR_COUNT];
    bool digitalValues[DIG_SENSOR_COUNT];
    unsigned int lastLinePosition = SETPOINT;
};