#include "RobotController.h"

RobotController::RobotController(MotorDriver &motors, LineSensors &sensors)
    : motors(motors), sensors(sensors) {}

void RobotController::setManualMode() {
    manualMode = true;
    autoMode   = false;
    setManualState(ManualState::Idle);
}

void RobotController::setAutoMode() {
    manualMode = false;
    autoMode   = true;
    onAutoStateChange(AutoState::Accelerate);
}

bool RobotController::isManualMode() const { return manualMode; }
bool RobotController::isAutoMode()  const  { return autoMode;  }

ManualState RobotController::getManualState() const { return currentManualState; }
void RobotController::setManualState(ManualState s) { currentManualState = s; }

AutoState RobotController::getAutoState() const { return currentAutoState; }

void RobotController::update(unsigned long now) {
    currentTime = now;

    sensors.readAnalog();
    sensors.computeLinePosition();

    if (!manualMode && !autoMode) {
        motors.idle();
        return;
    }

    if (manualMode) {
        updateManual();
    } else {
        updateAuto();
    }
}

void RobotController::updateManual() {
    switch (currentManualState) {
        case ManualState::Idle:      motors.idle();     break;
        case ManualState::Forward:   motors.forward();  break;
        case ManualState::Backwards: motors.backward(); break;
        case ManualState::Left:      motors.left();     break;
        case ManualState::Right:     motors.right();    break;
        case ManualState::Stop:      motors.stop();     break;
        default:                                        break;
    }
}

void RobotController::updateAuto() {
    switch (currentAutoState) {
        case AutoState::Idle:        stateIdle();       break;
        case AutoState::Accelerate:  stateAccelerate(); break;
        case AutoState::PIDLoop:     statePIDLoop();    break;
        case AutoState::HardLeft:    stateHardLeft();   break;
        case AutoState::HardRight:   stateHardRight();  break;
        case AutoState::Turn:        stateTurn();       break;
        case AutoState::Stop:        stateStop();       break;
        default:                                        break;
    }
}

void RobotController::onAutoStateChange(AutoState newState) {
    stateEntryTime    = currentTime;
    previousAutoState = currentAutoState;
    currentAutoState  = newState;

    Serial.print("Auto state -> ");
    Serial.println((int)newState);

    switch (newState) {
        case AutoState::PIDLoop:
            lastError = 0;
            break;
        case AutoState::Accelerate:
            motorInputSignal = 0;
            break;
        default:
            break;
    }
}

void RobotController::stateIdle() {
    motors.idle();
    if ((currentTime - stateEntryTime) > STOP_TIME) {
        onAutoStateChange(AutoState::Accelerate);
    }
}

void RobotController::stateAccelerate() {
    static unsigned long accelerationTimer = 0;

    // bool digitalFlag = sensors.allDigitalOff(); // optional

    if (motorInputSignal >= SET_SPEED) {
        onAutoStateChange(AutoState::PIDLoop);
        return;
    }

    if ((currentTime - accelerationTimer) >= ACCELERATION_INTERVAL &&
        motorInputSignal < SET_SPEED) {

        motorInputSignal += PWM_LEVEL_INCREMENT;

        MotorSpeeds speeds{
            motorInputSignal, MIN_SPEED,
            motorInputSignal, MIN_SPEED
        };

        motors.setSpeed(speeds);
        accelerationTimer = currentTime;
    }
}

void RobotController::statePIDLoop() {
    
    if (handlePIDTransitions()) {
        return;
    }

    int error = computeError();

    float adjust = computeAdjust(error);
    int baseSpeed = computeBaseSpeed(error);
    MotorSpeeds speed = computePIDSpeeds(baseSpeed, adjust);

    motors.setSpeed(speed);
    lastError = error;
}

bool RobotController::handlePIDTransitions() {
    if (sensors.allAnalogAbove()) {
                onAutoStateChange(AutoState::Stop);
                return true;
            }

        unsigned int currentPos = sensors.linePosition();
        if (currentPos > SHARP_LEFT_THR) {
            onAutoStateChange(AutoState::HardLeft);
            return true;
        } else if (currentPos < SHARP_RIGHT_THR) {
            onAutoStateChange(AutoState::HardRight);
            return true;
        }

    return false;
}

int RobotController::computeError() {
    int error = (int)SETPOINT - (int)sensors.linePosition();
    if (abs(error) < (int)DEADBAND_ERROR) error = 0;
    return error;
}

float RobotController::computeAdjust(int error) {
    float adjust = (error * PIDGains::Kp)
                 + (PIDGains::Kd * (error - lastError));
    return adjust;
}

int RobotController::computeBaseSpeed(int error) const {
    const int MIN_CORNER_SPEED = 40;
    const float Kv = 0.12;

    const int bigSpeed = 150;
    int baseSpeed = (int)(bigSpeed - Kv * abs(error));
    return constrain(baseSpeed, MIN_CORNER_SPEED, bigSpeed);
}

MotorSpeeds RobotController::computePIDSpeeds(int baseSpeed, float adjust) const {
    int leftSpeed  = baseSpeed - (int)adjust;
    int rightSpeed = baseSpeed + (int)adjust;

    MotorSpeeds speeds{
        leftSpeed,  MIN_SPEED,
        rightSpeed, MIN_SPEED
    };
    return speeds;
}

void RobotController::stateStop() {
    motors.stop();
    if ((currentTime - stateEntryTime) > STOP_TIME) {
        onAutoStateChange(AutoState::Turn);
    }
}

void RobotController::stateTurn() {
    if ((currentTime - stateEntryTime) > TURN_DURATION) {
        onAutoStateChange(AutoState::Accelerate);
    } else {
        motors.right();
    }
}

void RobotController::stateHardLeft() {
    motors.left();
    if (sensors.linePosition() <= 2500) {
        onAutoStateChange(AutoState::PIDLoop);
    }
}

void RobotController::stateHardRight() {
    motors.right();
    if (sensors.linePosition() > 1500) {
        onAutoStateChange(AutoState::PIDLoop);
    }
}
