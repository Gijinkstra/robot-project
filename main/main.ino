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

// --------- Control Types ---------------
enum class AutoState {
    Idle,
    Accelerate,
    PIDLoop,
    Stop,
    TurnLeft,
    TurnRight,
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
                    if (digitalRead(digitalPins[i])) {
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
    RobotController(MotorDriver &motors)
        : motors(motors) {}

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

    void update(unsigned long now) {
        currentTime = now;

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
            case ManualState::Idle:      motors.idle();     break;
            case ManualState::Forward:   motors.forward();  break;
            case ManualState::Backwards: motors.backward(); break;
            case ManualState::Left:      motors.left();     break;
            case ManualState::Right:     motors.right();    break;
            case ManualState::Stop:      motors.stop();     break;
            default:                                        break;
        }
    }

    void updateAuto() {
        switch (currentAutoState) {
            case AutoState::Idle:        stateIdle();       break;
            case AutoState::Accelerate:  stateAccelerate(); break;
            case AutoState::PIDLoop:     statePIDLoop();    break;
            case AutoState::LineFinish:  stateLineFinish(); break;
            case AutoState::Stop:        stateStop();       break;
            default:                                        break;
        }
    }

    void onAutoStateChange(AutoState newState) {
        stateEntryTimer = currentTime;
        previousAutoState = currentAutoState;
        currentAutoState = newState;

        Serial.print("Auto state -> ");


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

    void stateAccelerate() {
        
    }

    void statePIDLoop() {
        // compute PID, then motors.setSpeeds(...)
    }

    // Members
    MotorDriver &motors;

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

// --------- Global variables ----------
unsigned long stateEntryTimer;
unsigned long currentTime;

int motorInputSignal = 0;
int lastError = 0;
unsigned int linePosition;
bool manualMode = false;
bool autoMode = false;

AutoState currentAutoState = AutoState::Idle;
AutoState previousAutoState = AutoState::Idle;
ManualState currentManualState = ManualState::Idle;

int sensorArrMax[AN_SENSOR_COUNT];
int sensorArrMin[AN_SENSOR_COUNT];
int sensorArrValues[AN_SENSOR_COUNT];

// ---------- BLE Service --------------
BLEService terminalService("19B10010-E8F2-537E-4F6C-D104768A1214");
BLECharCharacteristic terminalCharacteristic("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEStringCharacteristic debugCharacteristic("19B10011-E8F2-537E-4F6C-D104768A1215", BLERead | BLENotify, 64);

// ------------ HARDWARE LAYER ---------
void initHardware();
void setMotorSpeed(const MotorSpeeds &speed);
void readAnalogSensors();
bool readDigitalSensors();
unsigned int calculateLinePosition();

// ------------ CONTROL LAYER ----------
void manualStateMachine();
void autoStateMachine();
void idleMotors();
void accelerateMotors();
void lineFinish();
void turnMotors();
void turnMotorLeft();
void turnMotorRight();
void stopMotors();
void pidMotors();

// ------------ COMMS LAYER ----------
void startBLEModule();
void monitorBLE();
void parseBLEMessage(char msg);
void writeToBLE(const String &msg);

void setup() {
    initHardware();
    startBLEModule();
}

void loop() {
    currentTime = millis();

    readAnalogSensors();
    monitorBLE();
    
    if (!manualMode && !autoMode) {
        setMotorSpeed(MotorPatterns::idle);
    }
    
    if (manualMode) {
        manualStateMachine();
    } else if (autoMode) {
        autoStateMachine();
    }

    // ****************** DEBUGGING - REMOVE *****************************
    readDigitalSensors();
    Serial.print("Line position: "); Serial.println(linePosition);

    char serialPrint[30];
    // Serial.println("Output flag: " + String(digitalIRFlag));
    for (int i = 0; i < AN_SENSOR_COUNT; i++) {
        sprintf(serialPrint, "Sensor %d value: %d", i, sensorArrValues[i]);
        Serial.println(serialPrint);
    }
}

// ------------ HARDWARE LAYER ---------
void initHardware() {
    Serial.begin(BAUD_RATE);
    for (byte i = 0; i < MOTOR_PIN_COUNT; i++) {
        pinMode(MOTOR_PINS[i], OUTPUT);
    }

    // Allows scaling of the pins easily.
    for (byte i = 0; i < AN_SENSOR_COUNT; i++) {
        pinMode(AN_SENSOR_PINS[i], INPUT);
        pinMode(DIG_SENSOR_PINS[i], INPUT);
    }
}

void readAnalogSensors() {
    for (byte i = 0; i < AN_SENSOR_COUNT; i++) {
        sensorArrValues[i] = analogRead(AN_SENSOR_PINS[i]);
    }
}

bool readDigitalSensors() {
    bool digSensorArray[AN_SENSOR_COUNT];

    for (byte i = 0; i < AN_SENSOR_COUNT; i++) {
        digSensorArray[i] = digitalRead(DIG_SENSOR_PINS[i]);
    }
    // Sensor logic was devised using inverted logic for some reason.
    return (!digSensorArray[0] && !digSensorArray[1] && !digSensorArray[2]);
}

unsigned int calculateLinePosition() {
    long weightedSum = 0;
    int sum = 0;

    for (byte i = 0; i < AN_SENSOR_COUNT; i++) {
        int reading = sensorArrValues[i];
        weightedSum += reading * i * SCALING_FACTOR;
        sum += reading;
    }

    if (sum == 0) {
        linePosition = SETPOINT;
        return;
    }

    linePosition = weightedSum / sum;
    return linePosition;
}

// ------------ CONTROL LAYER ---------

void manualStateMachine() {
    switch (currentManualState) {
        case ManualState::Idle:       setMotorSpeed(MotorPatterns::idle);         break;
        case ManualState::Forward:    setMotorSpeed(MotorPatterns::forward);      break;
        case ManualState::Backwards:  setMotorSpeed(MotorPatterns::backward);     break;
        case ManualState::Right:      setMotorSpeed(MotorPatterns::right);        break;
        case ManualState::Left:       setMotorSpeed(MotorPatterns::left);         break;
        case ManualState::Stop:       setMotorSpeed(MotorPatterns::stop);         break;
        default:                      Serial.println("ERROR: Default case");      break;
    }
}

void autoStateMachine() {
    switch (currentAutoState) {
        case AutoState::Idle:           idleMotors();                    break;
        case AutoState::Accelerate:     accelerateMotors();              break;
        case AutoState::PIDLoop:        pidMotors();                     break;
        // case AutoState::Left:        turnMotorLeft();                break;
        // case AutoState::Right:       turnMotorRight();               break;
        case AutoState::LineFinish:     lineFinish();                    break;
        case AutoState::Stop:           stopMotors();                    break;
        default:            Serial.println("ERROR: Default case");       break;
    }
}




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

void monitorBLE() {
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
        parseBLEMessage(msg);
    }
}

void writeToBLE(const String &msg) {
    debugCharacteristic.writeValue(msg + "\n");
}

void parseBLEMessage(char msg) {
    // Ensure always lower case to cut down on switch cases.
    switch (tolower(msg)) {
        case 'm':
            manualMode = true;
            autoMode = false;
            writeToBLE("Switched to MANUAL mode (BLE)");
            currentManualState = ManualState::Idle;
            break;

        case 'a':
            manualMode = false;
            autoMode = true;
            writeToBLE("Switched to AUTO mode (BLE)");
            onAutoStateChange(AutoState::Idle);
            break;

        case 'f':
            if (manualMode) {
                if (currentManualState == ManualState::Forward) {
                    writeToBLE("Forward stopped");
                    currentManualState = ManualState::Stop;
                } else {
                    writeToBLE("Forward command");
                    currentManualState = ManualState::Forward;
                }
            }
            break;

        case 'b':
            if (manualMode) {
                if (currentManualState == ManualState::Backwards) {
                    writeToBLE("Backwards stopped");
                    currentManualState = ManualState::Stop;
                } else {
                    writeToBLE("Backwards command");
                    currentManualState = ManualState::Backwards;
                }
            }
            break;

        case 'r':
            if (manualMode) {
                if (currentManualState == ManualState::Right) {
                    writeToBLE("Turning right stopped");
                    currentManualState = ManualState::Stop;
                } else {
                    writeToBLE("Turning right");
                    currentManualState = ManualState::Right;
                }
            }
            break;

        case 'l':
            if (manualMode){
                if (currentManualState == ManualState::Left) {
                    writeToBLE("Turning left stopped");
                    currentManualState = ManualState::Stop;
                } else {
                    writeToBLE("Turning left");
                    currentManualState = ManualState::Left;
                }
            }
            break;

        case 'c':
        // Do not allow sensor calibration outside of idle state.
        if (!manualMode && !autoMode) {
            writeToBLE("Starting calibration...");
            // calibrateSensors();
            writeToBLE("Calibration completed.");
            break;
        }
        default:
            writeToBLE("Unknown BLE cmd");
            currentManualState = ManualState::Idle;
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