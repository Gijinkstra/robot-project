// --------- Libraries ---------------------
#include <ArduinoBLE.h>
// --------- Global constants --------------
// Pins
// *********** NOTE - Sort these into arrays once everything is wired up **************
constexpr byte MOTOR_1A_PIN = 5;
constexpr byte MOTOR_1B_PIN = 6;
constexpr byte MOTOR_2A_PIN = 9;
constexpr byte MOTOR_2B_PIN = 10;
constexpr byte AN_SENSOR_PINS[] = {A0, A1, A2, A3, A4};
constexpr byte AN_SENSOR_COUNT = sizeof(AN_SENSOR_PINS) / sizeof(AN_SENSOR_PINS[0]);
constexpr byte DIG_SENSOR_PINS[] = {2, 3, 4, 7, 8}; // There is no digital sensor count as we assume there is 1 digital for every 1 analog pin.

// Motor constants.
constexpr unsigned int MIN_SPEED = 0;               // |
constexpr unsigned int MAX_SPEED = 60;             // |
constexpr unsigned int SET_SPEED = 150;             // | - PWM values.
constexpr unsigned int ROTATION_SPEED = 100;         // |
constexpr unsigned int TURN_DURATION = 1000;       // |
constexpr unsigned int MOTOR_OFFSET = 12;
constexpr byte PWM_LEVEL_INCREMENT = 5;   // [ms]
constexpr byte ACCELERATION_INTERVAL = 12;      // [ms]


// IR sensor constants.
constexpr unsigned int AN_SENSOR_MAX = 1023;
constexpr unsigned int AN_SENSOR_MIN = 0;
constexpr unsigned int TIMEOUT = 2500;  //  [ms]
constexpr unsigned int WAIT_TIME = 300; //  [ms]
constexpr unsigned int SCALING_FACTOR = 1000;
constexpr unsigned int SETPOINT = (AN_SENSOR_COUNT - 1 * AN_SENSOR_MAX) / 2;

// Misc
constexpr int BAUD_RATE = 9600;
enum AutoState {IDLE,
                ACCELERATE,
                PID_LOOP,
                STOP,
                TURN
};

enum ManualState {MANUAL_IDLE,
                  MANUAL_FORWARD,
                  MANUAL_BACKWARDS,
                  MANUAL_RIGHT,
                  MANUAL_LEFT,
                  MANUAL_STOP
};

// This struct method is so cool. Instead of having to pass loads of values into setMotorSpeed(),
// the struct can handle the defaults each time, which is so so handy.
struct MotorSpeeds {
    unsigned int leftA;
    unsigned int leftB;
    unsigned int rightA;
    unsigned int rightB;
};

namespace MotorPatterns {
    constexpr MotorSpeeds idle      = {0, 0, 0, 0};
    constexpr MotorSpeeds forward   = {SET_SPEED, 0, SET_SPEED - MOTOR_OFFSET, 0};
    constexpr MotorSpeeds backward  = {0, SET_SPEED, 0, SET_SPEED};
    constexpr MotorSpeeds right     = {ROTATION_SPEED, 0, 0, ROTATION_SPEED};
    constexpr MotorSpeeds left      = {0, ROTATION_SPEED, ROTATION_SPEED, 0};
    constexpr MotorSpeeds stop      = {1, 1, 1, 1};     // Set theses to 1 to draw the least amount of power.
}

// --------- Global variables ----------
unsigned long stateEntryTimer;
unsigned long currentTime;

unsigned int motorInputSignal = 0;
int lastError = 0;
bool manualMode = false;
bool autoMode = false;

AutoState currentAutoState = IDLE;
AutoState previousAutoState = IDLE;
ManualState currentManualState = MANUAL_IDLE;

int sensorArrayMax[AN_SENSOR_COUNT];
int sensorArrayMin[AN_SENSOR_COUNT];

// ---------- BLE Service --------------
BLEService terminalService("19B10010-E8F2-537E-4F6C-D104768A1214"); // create service

BLECharCharacteristic terminalCharacteristic("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

BLEStringCharacteristic debugCharacteristic("19B10011-E8F2-537E-4F6C-D104768A1215", BLERead | BLENotify, 64);

void setup() {
    Serial.begin(BAUD_RATE);
    pinMode(MOTOR_1A_PIN, OUTPUT);
    pinMode(MOTOR_1B_PIN, OUTPUT);
    pinMode(MOTOR_2A_PIN, OUTPUT);
    pinMode(MOTOR_2B_PIN, OUTPUT);

    // Allows scaling of the pins easily.
    for (byte i = 0; i < AN_SENSOR_COUNT; i++) {
        pinMode(AN_SENSOR_PINS[i], INPUT);
        pinMode(DIG_SENSOR_PINS[i], INPUT);
    }

    startBLEModule();
}

void loop() {
    currentTime = millis();

    // This function contains all the logic for the BLE and parsing the commands. Probably too much.
    monitorBLE();
    
    if (!manualMode && !autoMode) {
        setMotorSpeed(MotorPatterns::idle);
    }
    
    if (manualMode) {
        manualStateMachine();
    } else if (autoMode) {
        autoStateMachine();
    }

    // Timer to determine change of state.
    if (currentAutoState != previousAutoState) {
        stateEntryTimer = currentTime;
        previousAutoState = currentAutoState;
        Serial.println(currentAutoState);

        // Need this to reinitialise whenever the robot turns around and travels back down the course.
        if (currentAutoState == PID_LOOP) {
            lastError = 0;
        }
    }

    // ****************** DEBUGGING - REMOVE *****************************
    readDigitalSensors();
    int linePosition = readAnalogSensors();

    // Serial.println("Output flag: " + String(digitalIRFlag));
    Serial.println("Line value: " + String(linePosition));
    Serial.println("Sensor 1 value: " + String(readNormalised(0)));
    Serial.println("Sensor 2 value: " + String(readNormalised(1)));
    Serial.println("Sensor 3 value: " + String(readNormalised(2)));
    Serial.println("Sensor 4 value: " + String(readNormalised(3)));
    Serial.println("Sensor 5 value: " + String(readNormalised(4)));
}

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
    if (central) {
        if (central.connected()) {

            // Check if command has been written.
            if (terminalCharacteristic.written()) {

                char msg = terminalCharacteristic.value();
                Serial.print("BLE command: ");
                Serial.println(msg);

                // Separate message parsing module as this fucntion is already triple nested.
                parseBLEMessage(msg);
            }
        }
    }
}

void writeToBLE(const String &msg) {
    debugCharacteristic.writeValue(msg + "\n");
}

// 
void parseBLEMessage(char msg) {
    // Ensure always lower case to cut down on switch cases.
    switch (tolower(msg)) {
        case 'm':
            manualMode = true;
            autoMode = false;
            writeToBLE("Switched to MANUAL mode (BLE)");
            currentManualState = MANUAL_IDLE;
            break;

        case 'a':
            manualMode = false;
            autoMode = true;
            writeToBLE("Switched to AUTO mode (BLE)");
            currentAutoState = IDLE;
            previousAutoState = IDLE;
            break;

        case 'f':
            if (manualMode) {
                if (currentManualState == MANUAL_FORWARD) {
                    writeToBLE("Forward stopped");
                    currentManualState = MANUAL_STOP;
                } else {
                    writeToBLE("Forward command");
                    currentManualState = MANUAL_FORWARD;
                }
            }
            break;

        case 'b':
            if (manualMode) {
                if (currentManualState == MANUAL_BACKWARDS) {
                    writeToBLE("Backwards stopped");
                    currentManualState = MANUAL_STOP;
                } else {
                    writeToBLE("Backwards command");
                    currentManualState = MANUAL_BACKWARDS;
                }
            }
            break;

        case 'r':
            if (manualMode) {
                if (currentManualState == MANUAL_RIGHT) {
                    writeToBLE("Turning right stopped");
                    currentManualState = MANUAL_STOP;
                } else {
                    writeToBLE("Turning right");
                    currentManualState = MANUAL_RIGHT;
                }
            }
            break;

        case 'l':
            if (manualMode){
                if (currentManualState == MANUAL_LEFT) {
                    writeToBLE("Turning left stopped");
                    currentManualState = MANUAL_STOP;
                } else {
                    writeToBLE("Turning left");
                    currentManualState = MANUAL_LEFT;
                }
            }
            break;

        case 'c':
        // Do not allow sensor calibration outside of idle state.
        if (!manualMode && !autoMode) {
            writeToBLE("Starting calibration...");
            calibrateSensors();
            writeToBLE("Calibration completed.");
            break;
        }
        default:
            writeToBLE("Unknown BLE cmd");
            currentManualState = MANUAL_IDLE;
    }
}

// Returns the weighted sum of our normalised sensor array values.
int readAnalogSensors() {
    int weightedSum = 0;
    int sum = 0;

    for (byte i = 0; i < AN_SENSOR_COUNT; i++) {
        int reading = readNormalised(i);
        weightedSum += reading * i * SCALING_FACTOR;
        sum += reading;
    }

    if (sum == 0) {
        return SETPOINT;
    }

    return weightedSum / sum;
}

// This function is only here to tell us when all the sensors are false at the end of the course.
bool readDigitalSensors() {
    bool digSensorArray[AN_SENSOR_COUNT];
    bool digitalIRFlag;

    for (byte i = 0; i < AN_SENSOR_COUNT; i++) {
        digSensorArray[i] = digitalRead(DIG_SENSOR_PINS[i]);
    }
    // Sensor logic was devised using inverted logic for some reason.
    return (!digSensorArray[0] && !digSensorArray[1] && !digSensorArray[2]);
}

void calibrateSensors() {
    byte DELAY_TIME = 5;
    unsigned int CALIBRATION_TIMER = 15000;
    static unsigned long calibrationStartTime = millis();

    // Initialise all these to 1023 and 0 (default values).
    for (byte i = 0; i < AN_SENSOR_COUNT; i++) {
        sensorArrayMax[i] = AN_SENSOR_MAX;
        sensorArrayMin[i] = AN_SENSOR_MIN;
    }

    // Spend 15 seconds in the calibration routine.
    while ((millis() - calibrationStartTime) < CALIBRATION_TIMER) {
        for (byte s = 0; s < AN_SENSOR_COUNT; s++) {
            int currentReading = analogRead(AN_SENSOR_PINS[s]);

            if (currentReading < sensorArrayMin[s]) {
                sensorArrayMin[s] = currentReading;
            }
            if (currentReading > sensorArrayMax[s]) {
                sensorArrayMax[s] = currentReading;
            }
        }
    }
}

int readNormalised(int sensorIndex) {
    // The maximum and minimum values we want to constrain the reading to.
    int NORMALISED_MIN = 0;
    int NORMALISED_MAX = 1000;

    // Read the sensor and map to the sensor arrays.
    int rawSensorValue = analogRead(AN_SENSOR_PINS[sensorIndex]);
    int normSensorValue = map(rawSensorValue,
                              sensorArrayMin[sensorIndex],
                              sensorArrayMax[sensorIndex],
                              NORMALISED_MIN,
                              NORMALISED_MAX);

    // Logic flip here so high reading means the line is detected. Less confusing.
    return NORMALISED_MAX - normSensorValue;
}

// Accept the struct and constrain the values between 0 and 250;
void setMotorSpeed(const MotorSpeeds &speed) {
    analogWrite(MOTOR_2A_PIN, constrain(speed.leftA, MIN_SPEED, MAX_SPEED));
    analogWrite(MOTOR_2B_PIN, constrain(speed.leftB, MIN_SPEED, MAX_SPEED));
    analogWrite(MOTOR_1A_PIN, constrain(speed.rightA, MIN_SPEED, MAX_SPEED));
    analogWrite(MOTOR_1B_PIN, constrain(speed.rightB, MIN_SPEED, MAX_SPEED));
}

void manualStateMachine() {
    switch (currentManualState) {
        case MANUAL_IDLE:       setMotorSpeed(MotorPatterns::idle);         break;
        case MANUAL_FORWARD:    setMotorSpeed(MotorPatterns::forward);      break;
        case MANUAL_BACKWARDS:  setMotorSpeed(MotorPatterns::backward);     break;
        case MANUAL_RIGHT:      setMotorSpeed(MotorPatterns::right);        break;
        case MANUAL_LEFT:       setMotorSpeed(MotorPatterns::left);         break;
        case MANUAL_STOP:       setMotorSpeed(MotorPatterns::stop);         break;
        default:                Serial.println("ERROR: Default case");      break;
    }
}

/* In the automatic state machine, we break the behaviour out more explicitly into helper functions as they require timers for each
state, this can get quite messy nested within the switch statement, so it's easier to read if we just separate them.*/
void autoStateMachine() {
    switch (currentAutoState) {
        case IDLE:          idleMotors();                               break;
        case ACCELERATE:    accelerateMotors();                         break;
        case PID_LOOP:      pidMotors();                                break;
        case TURN:          turnMotors();                               break;
        case STOP:          stopMotors();                               break;
        default:            Serial.println("ERROR: Default case");      break;
    }
}

void idleMotors() {
    if ((currentTime - stateEntryTimer) > WAIT_TIME) {
        currentAutoState = ACCELERATE;
    }
    setMotorSpeed(MotorPatterns::idle);
}

void stopMotors() {
    if ((currentTime - stateEntryTimer) > ACCELERATION_INTERVAL) {
        currentAutoState = TURN;
    } else {
        setMotorSpeed(MotorPatterns::stop);
    }
}

void accelerateMotors() {
    // Static timer for acceleration.
    static unsigned long accelerationTimer = 0;

    bool digitalIRFlag = readDigitalSensors();
    // The digital IR Flag here ensures the robot has accelerated past the black line so we don't
    // have any funny behaviour with the sensors.
    // if ((motorInputSignal >= SET_SPEED) && !digitalIRFlag) {
    if (motorInputSignal >= SET_SPEED) {
        currentAutoState = PID_LOOP;
        return;
    }

    // Increment the speed according to the delay while the motor is less than the set speed.
    if ((currentTime - accelerationTimer) >= ACCELERATION_INTERVAL && motorInputSignal < SET_SPEED) {
        motorInputSignal += PWM_LEVEL_INCREMENT;
        setMotorSpeed({.leftA = motorInputSignal, .leftB = 0,
                       .rightA = motorInputSignal, .rightB = 0});
        accelerationTimer = currentTime;
    }
}

void pidMotors() {
    /* Initial value calculation: MAX SPEED VALUE (60) / MAX ERROR (2000) = KP
    Go down in value by 0.01 to check behaviour, then go up and try again. */
    constexpr float KP = 0.03;
    constexpr float KD = 0;

    // Check if the digital pins are active before doing anything else.
    bool digitalIRFlag = readDigitalSensors();

    // Logic to transition out of pid loop based on IR sensor feedback.
    // if (digitalIRFlag) {
    //     currentAutoState = STOP;
    //     return;
    // }

    // Read in the current line position and adjust the error.
    int linePosition = readAnalogSensors();
    int error = SETPOINT - linePosition;
    int adjust = (error * KP) + (KD * (error - lastError));

    // Set the motor speed based on the adjustment.
    setMotorSpeed({.leftA = SET_SPEED + adjust, .leftB = 0, 
                   .rightA = SET_SPEED - adjust, .rightB = 0});
    lastError = error;
}
 
void turnMotors() { 
    // Accelerate for a fixed time. Could also begin rotation and stop when all sensors light up again.
    if ((currentTime - stateEntryTimer) > TURN_DURATION) {
        currentAutoState = ACCELERATE;
    } else {
        setMotorSpeed(MotorPatterns::right);
    }
}