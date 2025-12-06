#pragma once
#include <Arduino.h>
#include "Config.h"
#include "MotorDriver.h"
#include "LineSensors.h"

class RobotController {
public:
    RobotController(MotorDriver &motors, LineSensors &sensors);

    void setManualMode();
    void setAutoMode();

    bool isManualMode() const;
    bool isAutoMode() const;

    ManualState getManualState() const;
    void setManualState(ManualState s);

    AutoState getAutoState() const;

    void update(unsigned long now);

private:
    void updateManual();
    void updateAuto();
    void onAutoStateChange(AutoState newState);

    // Auto states
    void stateIdle();
    void stateStop();
    void stateAccelerate();
    void statePIDLoop();
    void stateLineFinish();
    void stateHardLeft();
    void stateHardRight();

    // Members
    MotorDriver &motors;
    LineSensors &sensors;

    bool manualMode = false;
    bool autoMode   = false;

    ManualState currentManualState = ManualState::Idle;
    AutoState   currentAutoState   = AutoState::Idle;
    AutoState   previousAutoState  = AutoState::Idle;

    unsigned long currentTime    = 0;
    unsigned long stateEntryTime = 0;

    int motorInputSignal = 0;
    int lastError        = 0;
};
