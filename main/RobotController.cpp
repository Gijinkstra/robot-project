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
    sensors.readDigital();
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
        default:                                         break;
    }
}

void RobotController::updateAuto() {
    switch (currentAutoState) {
        case AutoState::Idle:        stateIdle();       break;
        case AutoState::Accelerate:  stateAccelerate(); break;
        case AutoState::PIDLoop:     statePIDLoop();    break;
        case AutoState::HardLeft:    stateHardLeft();   break;
        case AutoState::HardRight:   stateHardRight();  break;
        case AutoState::LineFinish:  stateLineFinish(); break;
        case AutoState::Stop:        stateStop();       break;
        default:                                         break;
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
    if ((currentTime - stateEntryTime) > WAIT_TIME) {
        onAutoStateChange(AutoState::Accelerate);
    }
}

void RobotController::stateStop() {
    motors.stop();
    if ((currentTime - stateEntryTime) > TURN_DURATION) {
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
    int error = (int)SETPOINT - (int)sensors.linePosition();
    if (abs(error) < (int)DEADBAND_ERROR) error = 0;

    float adjust = (error * PIDGains::Kp)
                 + (PIDGains::Kd * (error - lastError));

    int leftSpeed  = SET_SPEED - (int)adjust;
    int rightSpeed = SET_SPEED + (int)adjust;

    MotorSpeeds speeds{
        leftSpeed,  MIN_SPEED,
        rightSpeed, MIN_SPEED
    };
    motors.setSpeed(speeds);
    lastError = error;
}

void RobotController::stateLineFinish() {
    if ((currentTime - stateEntryTime) > TURN_DURATION) {
        onAutoStateChange(AutoState::PIDLoop);
    } else {
        motors.right();
    }
}

void RobotController::stateHardLeft() {
    motors.left();
    if (sensors.linePosition() < SHARP_LEFT_THR) {
        onAutoStateChange(AutoState::PIDLoop);
    }
}

void RobotController::stateHardRight() {
    motors.right();
    if (sensors.linePosition() > SHARP_RIGHT_THR) {
        onAutoStateChange(AutoState::PIDLoop);
    }
}
