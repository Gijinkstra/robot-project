#pragma once
#include <Arduino.h>
#include "Config.h"
#include "MotorDriver.h"
#include "LineSensors.h"

class RobotController {
public:
    RobotController(MotorDriver &motors, LineSensors &sensors);

    void update(unsigned long now);
    void setManualMode();
    void setAutoMode();
    bool isManualMode() const;
    bool isAutoMode() const;
    void setManualState(ManualState s);

    ManualState getManualState() const;
    AutoState getAutoState() const;

private:
    // State machines.
    void updateManual();
    void updateAuto();

    // Helper function to keep track of timers + error on state change.
    void onAutoStateChange(AutoState newState);

    // Auto states
    void stateIdle();
    void stateAccelerate();
    void statePIDLoop();
    void stateHardLeft();
    void stateHardRight();
    void stateStop();
    void stateTurn();

    // PID helper functions
    bool handlePIDTransitions();
    int computeError();
    float computeAdjust(int error);
    int computeBaseSpeed(int error) const;
    MotorSpeeds computePIDSpeeds(int baseSpeed, float adjust) const;

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
