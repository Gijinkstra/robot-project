#pragma once

#include <Arduino.h>
#include <ArduinoBLE.h>

#include "Config.h"
#include "RobotController.h"

class BLEComms {
public:
    explicit BLEComms(RobotController &robot);

    void init();
    void monitor();
    void outgoingMessage(const String &msg);

private:
    void incomingMessage(char msg);

    RobotController &robot;

    BLEService terminalService;
    BLECharCharacteristic readCharacteristic;
    BLEStringCharacteristic writeCharacteristic;

    BLEDevice central;
};