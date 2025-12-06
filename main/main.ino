// --------- Libraries ---------------------
#include <ArduinoBLE.h>

// --------- Hardware Config ---------------
constexpr byte MOTOR_PINS[] = {9, 10, 5, 6}; // Left to right.
constexpr byte AN_SENSOR_PINS[] = {A0, A1, A2, A3, A4};
constexpr byte MOTOR_PIN_COUNT = sizeof(MOTOR_PINS) / 
                                 sizeof(MOTOR_PINS[0]);
constexpr byte AN_SENSOR_COUNT = sizeof(AN_SENSOR_PINS) / 
                                 sizeof(AN_SENSOR_PINS[0]);
constexpr byte DIG_SENSOR_PINS[] = {2, 3, 4};
constexpr byte DIG_SENSOR_COUNT = sizeof(DIG_SENSOR_PINS) / 
                                 sizeof(DIG_SENSOR_PINS[0]);
constexpr unsigned int MIN_SPEED = 0;
constexpr unsigned int MAX_SPEED = 255;   
constexpr unsigned int AN_SENSOR_MAX = 1023;
constexpr unsigned int AN_SENSOR_MIN = 0;
constexpr unsigned int BAUD_RATE = 9600;

// --------- Control Config ---------------
constexpr byte PWM_LEVEL_INCREMENT = 5;
constexpr byte ACCELERATION_INTERVAL = 5;
constexpr unsigned int TURN_DURATION = 1000;
constexpr unsigned int MOTOR_OFFSET = 12;
constexpr unsigned int SET_SPEED = 100;
constexpr unsigned int TURN_SPEED = 100;
constexpr unsigned int SCALING_FACTOR = 1000;
constexpr unsigned int SETPOINT = 2000;
constexpr unsigned int SHARP_LEFT_THR = 3000;
constexpr unsigned int SHARP_RIGHT_THR = 1000;
constexpr unsigned int DEADBAND_ERROR = 20;
constexpr unsigned int WAIT_TIME = 300;

// --------- Control Types ---------------
enum class AutoState {
    Idle,
    Accelerate,
    PIDLoop,
    Stop,
    HardLeft,
    HardRight,
    LineFinish
};

enum class ManualState {
    Idle,
    Forward,
    Backwards,
    Right,
    Left,
    Stop
};

struct MotorSpeeds {
    int leftA;
    int leftB;
    int rightA;
    int rightB;
};

namespace MotorPatterns {
    constexpr MotorSpeeds idle      = {MIN_SPEED, MIN_SPEED, 
                                       MIN_SPEED, MIN_SPEED};
    constexpr MotorSpeeds forward   = {SET_SPEED, MIN_SPEED, 
                                       SET_SPEED - MOTOR_OFFSET, MIN_SPEED};
    constexpr MotorSpeeds backward  = {MIN_SPEED, SET_SPEED, 
                                       MIN_SPEED, SET_SPEED};
    constexpr MotorSpeeds right     = {TURN_SPEED, MIN_SPEED, 
                                       MIN_SPEED, TURN_SPEED};
    constexpr MotorSpeeds left      = {MIN_SPEED, TURN_SPEED, 
                                       TURN_SPEED, MIN_SPEED};
    constexpr MotorSpeeds stop      = {1, 1, 1, 1};
}

namespace PIDGains {
    constexpr float Kp = 0.80f;
    constexpr float Kd = 0.01f;
}

// Classes
class MotorDriver {
    public:
        MotorDriver(const byte pins[MOTOR_PIN_COUNT]) {
            for (byte i = 0; i < MOTOR_PIN_COUNT; i++) {
                motorPins[i] = pins[i];
            }
        }

        void init() {
            for (byte i = 0; i < MOTOR_PIN_COUNT; i++) {
                pinMode(motorPins[i], OUTPUT);
            }
        }

        void setSpeed(const MotorSpeeds &speed) {
            analogWrite(motorPins[0], constrain(speed.leftA, MIN_SPEED, MAX_SPEED));
            analogWrite(motorPins[1], constrain(speed.leftB, MIN_SPEED, MAX_SPEED));
            analogWrite(motorPins[2], constrain(speed.rightA, MIN_SPEED, MAX_SPEED));
            analogWrite(motorPins[3], constrain(speed.rightB, MIN_SPEED, MAX_SPEED));
        }

        void idle()         { setSpeed(MotorPatterns::idle);        }
        void forward()      { setSpeed(MotorPatterns::forward);     }
        void backward()     { setSpeed(MotorPatterns::backward);    }
        void left()         { setSpeed(MotorPatterns::left);        }
        void right()        { setSpeed(MotorPatterns::right);       }
        void stop()         { setSpeed(MotorPatterns::stop);        }
    
    private:
        byte motorPins[4];
};

class LineSensors {
    public:
        LineSensors(const byte anPins[], byte anCount,
                const byte digPins[], byte digCount)
                : analogCount(anCount), digitalCount(digCount)
            {
                for (byte i = 0; i < analogCount; i++) {
                    analogPins[i] = anPins[i];
                }
                for (byte i = 0; i < digCount; i++) {
                    digitalPins[i] = digPins[i];
                }
            }

            void init() {
                for (byte i = 0; i < analogCount; i++) {
                    pinMode(analogPins[i], INPUT);
                }
                for (byte i = 0; i < digitalCount; i++) {
                    pinMode(digitalPins[i], INPUT);
                }
            }

            void readAnalog() {
                for (byte i = 0; i < analogCount; i++) {
                    analogValues[i] = analogRead(analogPins[i]);
                }
            }

            void readDigital() {
                for (byte i = 0; i < digitalCount; i++) {
                    digitalValues[i] = digitalRead(digitalPins[i]);
                }
            }

            bool allDigitalOff() {
                for (byte i = 0; i < digitalCount; ++i) {
                    if (digitalValues[i]) {
                        // if any are "on", then not all off
                        return false;
                    }
                }
                return true;
            }

            unsigned int computeLinePosition() {
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

            unsigned int linePosition() const { return lastLinePosition; }
            int getAnalogReading(byte i) const { return analogValues[i]; }
            bool getDigitalReading(byte i) const { return digitalValues[i]; }

    private:
        byte analogPins[AN_SENSOR_COUNT];
        byte digitalPins[DIG_SENSOR_COUNT];
        byte analogCount;
        byte digitalCount;

        int analogValues[AN_SENSOR_COUNT];
        bool digitalValues[DIG_SENSOR_COUNT];
        unsigned int lastLinePosition = SETPOINT;
};

class RobotController {
public:
    RobotController(MotorDriver &motors, LineSensors &sensors)
        : motors(motors), sensors(sensors) {}

    void setManualMode() {
        manualMode = true;
        autoMode   = false;
        currentManualState = ManualState::Idle;
    }

    void setAutoMode() {
        manualMode = false;
        autoMode   = true;
        currentAutoState = AutoState::Accelerate;
        previousAutoState = AutoState::Accelerate;
    }

    bool isManualMode() const { return manualMode; }
    bool isAutoMode() const { return autoMode; }
    ManualState getManualState() const { return currentManualState; }
    ManualState setManualState(ManualState s) { currentManualState = s; }
    AutoState getAutoState() const { return currentAutoState; }

    void update(unsigned long now) {
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

private:
    void updateManual() {
        switch (currentManualState) {
            case ManualState::Idle:      motors.idle();         break;
            case ManualState::Forward:   motors.forward();      break;
            case ManualState::Backwards: motors.backward();     break;
            case ManualState::Left:      motors.left();         break;
            case ManualState::Right:     motors.right();        break;
            case ManualState::Stop:      motors.stop();         break;
            default:                                            break;
        }
    }

    void updateAuto() {
        switch (currentAutoState) {
            case AutoState::Idle:        stateIdle();           break;
            case AutoState::Accelerate:  stateAccelerate();     break;
            case AutoState::PIDLoop:     statePIDLoop();        break;
            case AutoState::HardLeft:    stateHardLeft();       break;
            case AutoState::HardRight:   stateHardRight();      break;
            case AutoState::LineFinish:  stateLineFinish();     break;
            case AutoState::Stop:        stateStop();           break;
            default:                                            break;
        }
    }

    void onAutoStateChange(AutoState newState) {
        stateEntryTime = currentTime;
        previousAutoState = currentAutoState;
        currentAutoState = newState;

        Serial.print("Auto state -> "); Serial.println((int)newState);

        switch(newState) {
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

    void stateIdle() {
        motors.idle();
        if ((currentTime - stateEntryTime) > WAIT_TIME) {
            onAutoStateChange(AutoState::Accelerate);
        }
    }

    void stateStop() {
        motors.stop();
        if ((currentTime - stateEntryTime) > TURN_DURATION) {
            onAutoStateChange(AutoState::Accelerate);
        }
    }

    void stateAccelerate() {
        // Static timer for acceleration.
        static unsigned long accelerationTimer = 0;

        bool digitalIRFlag = sensors.allDigitalOff();

        if (motorInputSignal >= SET_SPEED) {
            onAutoStateChange(AutoState::PIDLoop);
            return;
        }

        // Increment the speed according to the delay while the motor is less than the set speed.
        if ((currentTime - accelerationTimer) >= 
             ACCELERATION_INTERVAL && motorInputSignal < SET_SPEED) {

                motorInputSignal += PWM_LEVEL_INCREMENT;

                MotorSpeeds speeds{
                    motorInputSignal, MIN_SPEED,
                    motorInputSignal, MIN_SPEED,
                };

                motors.setSpeed(speeds);
                accelerationTimer = currentTime;
        }
    }

    void statePIDLoop() {
        // Check if the digital pins are active before doing anything else.
        bool digitalIRFlag = sensors.allDigitalOff();

        int error = (int)SETPOINT - (int)sensors.linePosition();
        if (abs(error) < (int)DEADBAND_ERROR) error = 0;        // deadband.

        float adjust = (error * PIDGains::Kp)
                     + (PIDGains::Kd * (error - lastError));

        int leftSpeed = SET_SPEED - (int)adjust;
        int rightSpeed = SET_SPEED + (int)adjust;
        // if (linePosition > SHARP_LEFT_THR) {
        //     setMotorSpeed({TURN_SPEED, MIN_SPEED, MIN_SPEED, MIN_SPEED});
        //     return;
        // }

        // if (linePosition < SHARP_RIGHT_THR) {
        //     setMotorSpeed({MIN_SPEED, MIN_SPEED, TURN_SPEED, MIN_SPEED});
        //     return;
        // }

        // Set the motor speed based on the adjustment.

        MotorSpeeds speeds{
            leftSpeed, MIN_SPEED,
            rightSpeed, MIN_SPEED,
        };
        motors.setSpeed(speeds);
        lastError = error;
    }

    void stateLineFinish() {
        // Accelerate for a fixed time. Could also begin rotation and stop when all sensors light up again.
        if ((currentTime - stateEntryTime) > TURN_DURATION) {
            onAutoStateChange(AutoState::PIDLoop);
        } else {
            motors.right();
        }
    }

    void stateHardLeft() {
        motors.left();
        if (sensors.linePosition() < SHARP_LEFT_THR) {
            onAutoStateChange(AutoState::PIDLoop);
            return;
        }
    }

    void stateHardRight() {
        motors.right();
        if (sensors.linePosition() > SHARP_RIGHT_THR) {
            onAutoStateChange(AutoState::PIDLoop);
            return;
        }
    }


    // Members
    MotorDriver &motors;
    LineSensors &sensors;

    bool manualMode = false;
    bool autoMode   = false;

    ManualState currentManualState = ManualState::Idle;
    AutoState   currentAutoState   = AutoState::Idle;
    AutoState   previousAutoState  = AutoState::Idle;

    unsigned long currentTime = 0;
    unsigned long stateEntryTime = 0;

    int motorInputSignal = 0;
    int lastError = 0;
};

// ---------- Constructors --------------
MotorDriver     motors(MOTOR_PINS);
LineSensors     sensors(AN_SENSOR_PINS, AN_SENSOR_COUNT,
                        DIG_SENSOR_PINS, DIG_SENSOR_COUNT);
RobotController robot(motors, sensors);

// ---------- BLE Service --------------
BLEService terminalService("19B10010-E8F2-537E-4F6C-D104768A1214");
BLECharCharacteristic terminalCharacteristic("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEStringCharacteristic debugCharacteristic("19B10011-E8F2-537E-4F6C-D104768A1215", BLERead | BLENotify, 64);

// ------------ COMMS LAYER ----------
void startBLEModule();
void monitorBLE(RobotController &robot);
void parseBLEMessage(RobotController &robot, char msg);
void writeToBLE(const String &msg);

void setup() {
    Serial.begin(BAUD_RATE);
    motors.init();
    sensors.init();
    startBLEModule();
}

void loop() {
    unsigned long now = millis();

    monitorBLE(robot);
    robot.update(now);
    // ****************** DEBUGGING - REMOVE *****************************
    Serial.print("Line position: "); Serial.println(sensors.linePosition());

    // Serial.println("Output flag: " + String(digitalIRFlag));
    for (int i = 0; i < AN_SENSOR_COUNT; i++) {
        Serial.print("Sensor "); Serial.print(i);
        Serial.print(": "); Serial.println(sensors.getAnalogReading(i));
    }
}

// ------------ HARDWARE LAYER ---------




// ------------ COMMS LAYER ---------

void startBLEModule() {
    // Strictly speaking this would be necessary if the BLE was external and the Arduino didn't come with an ESP32.
    if (!BLE.begin()) {
        Serial.println("Error: BLE startup failed.");
    }

    // Set the name so we know what we're looking for.
    BLE.setLocalName("Chicken Ball Special");
    BLE.setAdvertisedService(terminalService);

    // Following lines allow the reading and writing to the Arduino.
    terminalService.addCharacteristic(terminalCharacteristic);
    terminalService.addCharacteristic(debugCharacteristic);

    BLE.addService(terminalService);
    terminalCharacteristic.writeValue(0);

    // Allow us to view it on the device list.
    BLE.advertise();
}

void monitorBLE(RobotController &robot) {
     BLEDevice central = BLE.central();   // Check for connection or disconnection
    // Checks the central has started correctly and that we are connected.
    if (!central) return;
    if (central.connected()) return;

    // Check if command has been written.
    if (terminalCharacteristic.written()) {
        char msg = terminalCharacteristic.value();
        Serial.print("BLE command: ");
        Serial.println(msg);
        // Separate message parsing module as this fucntion is already triple nested.
        parseBLEMessage(robot, msg);
    }
}

void writeToBLE(const String &msg) {
    debugCharacteristic.writeValue(msg + "\n");
}

void parseBLEMessage(char msg) {
    // Ensure always lower case to cut down on switch cases.
    switch (tolower(msg)) {
        case 'm':
            robot.setManualMode();
            writeToBLE("Switched to MANUAL mode (BLE)");
            break;

        case 'a':
            robot.setAutoMode();
            writeToBLE("Switched to AUTO mode (BLE)");
            break;

        case 'f':
            if (robot.isManualMode()) {
                if (robot.getManualState() == ManualState::Forward) {
                    robot.setManualState(ManualState::Stop);
                    writeToBLE("Forward stopped");
                } else {
                    writeToBLE("Forward command");
                    robot.setManualState(ManualState::Stop);
                }
            }
            break;

        case 'b':
            if (robot.isManualMode()) {
                if (robot.getManualState() == ManualState::Backwards) {
                    robot.setManualState(ManualState::Stop);
                    writeToBLE("Backwards stopped");
                } else {
                    robot.setManualState(ManualState::Backwards);
                    writeToBLE("Backwards command");
                }
            }
            break;

        case 'r':
            if (robot.isManualMode()) {
                if (robot.getManualState() == ManualState::Right) {
                    robot.setManualState(ManualState::Stop);
                    writeToBLE("Turning right stopped");
                } else {
                    robot.setManualState(ManualState::Right);
                    writeToBLE("Turning right");
                }
            }
            break;

        case 'l':
            if (robot.isManualMode()){
                if (robot.getManualState() == ManualState::Left) {
                    robot.setManualState(ManualState::Stop);
                    writeToBLE("Turning left stopped");
                } else {
                    robot.setManualState(ManualState::Left);
                    writeToBLE("Turning left");
                }
            }
            break;

        case 'c':
        // Do not allow sensor calibration outside of idle state.
        if (!robot.isManualMode() && !robot.isAutoMode()) {
            writeToBLE("Starting calibration...");
            // calibrateSensors();
            writeToBLE("Calibration completed.");
            break;
        }
        default:
            writeToBLE("Unknown BLE cmd");
            robot.setManualState(ManualState::Idle);
    }
}

// void calibrateSensors() {
//     const unsigned int CALIBRATION_TIMER = 5000;
//     unsigned long calibrationStartTime = millis();

//     // Initialise all these to 1023 and 0 (default values).
//     for (byte i = 0; i < AN_SENSOR_COUNT; i++) {
//         sensorArrayMax[i] = AN_SENSOR_MIN;
//         sensorArrayMin[i] = AN_SENSOR_MAX;
//     }

//     // Spend 15 seconds in the calibration routine.
//     while ((millis() - calibrationStartTime) < CALIBRATION_TIMER) {
//         for (byte s = 0; s < AN_SENSOR_COUNT; s++) {
//             int currentReading = analogRead(AN_SENSOR_PINS[s]);

//             if (currentReading < sensorArrayMin[s]) {
//                 sensorArrayMin[s] = currentReading;
//             }
//             if (currentReading > sensorArrayMax[s]) {
//                 sensorArrayMax[s] = currentReading;
//             }
//         }
//         delay(5);
//     }
// }

// int readNormalised(int sensorIndex) {
//     // The maximum and minimum values we want to constrain the reading to.
//     int NORMALISED_MIN = 0;
//     int NORMALISED_MAX = 1000;

//     // Read the sensor and map to the sensor arrays.
//     int rawSensorValue = analogRead(AN_SENSOR_PINS[sensorIndex]);
//     int normSensorValue = map(rawSensorValue,
//                               sensorArrayMin[sensorIndex],
//                               sensorArrayMax[sensorIndex],
//                               NORMALISED_MIN,
//                               NORMALISED_MAX);

//     // Logic flip here so high reading means the line is detected. Less confusing.
//     return NORMALISED_MAX - normSensorValue;
// }

// Accept the struct and constrain the values between 0 and 250;



/* In the automatic state machine, we break the behaviour out more explicitly into helper functions as they require timers for each
state, this can get quite messy nested within the switch statement, so it's easier to read if we just separate them.*/