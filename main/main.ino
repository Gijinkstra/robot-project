// --------- Libraries ---------------------
#include <ArduinoBLE.h>
// --------- Global constants --------------
// Pins
// *********** NOTE - Sort these into arrays once everything is wired up **************
constexpr byte MOTOR_1A_PIN = 5;
constexpr byte MOTOR_1B_PIN = 6;
constexpr byte MOTOR_2A_PIN = 9;
constexpr byte MOTOR_2B_PIN = 10;
constexpr byte AN_SENSOR_PINS[] = {A0, A1, A2};
constexpr byte AN_SENSOR_COUNT = sizeof(AN_SENSOR_PINS) / sizeof(AN_SENSOR_PINS[0]);
constexpr byte DIG_SENSOR_PINS[] = {2, 3, 4}; // There is no digital sensor count as we assume there is 1 digital for every 1 analog pin.

// Motor constants.
constexpr unsigned int MIN_SPEED = 0;               // |
constexpr unsigned int MAX_SPEED = 200;             // |
constexpr unsigned int SET_SPEED = 150;             // | - PWM values.
constexpr unsigned int ROTATION_SPEED = 50;         // |
constexpr unsigned int TURN_DURATION = 1000;       // |
constexpr byte PWM_LEVEL_INCREMENT = 5;   // [ms]
constexpr byte ACCELERATION_INTERVAL = 12;      // [ms]

// IR sensor constants.
constexpr unsigned int AN_SENSOR_MAX = 1023;
constexpr int SETPOINT = (AN_SENSOR_COUNT * AN_SENSOR_MAX) / 2;
constexpr unsigned int TIMEOUT = 2500;  //  [ms]
constexpr unsigned int WAIT_TIME = 300; //  [ms]

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
    constexpr MotorSpeeds forward   = {SET_SPEED, 0, SET_SPEED, 0};
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

// ---------- BLE Service --------------
BLEService terminalService("19B10010-E8F2-537E-4F6C-D104768A1214"); // create service

BLECharCharacteristic terminalCharacteristic("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

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
        if (currentAutoState == PID_LOOP) {
            lastError = 0;
        }
    }

    // ****************** DEBUGGING - REMOVE *****************************
    readDigitalSensors();
    int linePosition = readAnalogSensors();

    // Serial.println("Output flag: " + String(digitalIRFlag));
    // Serial.println("Line value: " + String(linePosition));
}

// Execute manual or automatic control scheme based on Serial data.
// ******************* NOTE - DEPRECATED *************************************
void selectControlScheme() {
    if (Serial.available() > 0) {
        char userInput = Serial.read();
        userInput = tolower(userInput);
        
        if (userInput == 'm') {
            Serial.println("User has selected manual control scheme.");
            manualMode = true;
        } else if (userInput == 'a') {
            Serial.println("User has selected automatic control scheme. Initializing...");
            manualMode = false;
        }
    }
}

void startBLEModule() {
    // Strictly speaking this would be necessary if the BLE was external and the Arduino didn't come with an ESP32.
    if (!BLE.begin()) {
        Serial.println("Error: BLE startup failed.");
    }

    // Set the name so we know what we're looking for.
    BLE.setLocalName("Timmy Turner Express");
    BLE.setAdvertisedService(terminalService);

    // Following lines allow the reading and writing to the Arduino.
    terminalService.addCharacteristic(terminalCharacteristic);

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

                // Oh boy. This switch statement handles all the logic for the manual commands. Entering the same command twice will 
                // cause the robot to stop. I think it's a nice QOL addition.
                parseBLEMessage(msg);
            }
        }
    }
}

void parseBLEMessage(char msg) {
    // Ensure always lower case to cut down on switch cases.
    switch (tolower(msg)) {
        case 'm':
            manualMode = true;
            autoMode = false;
            Serial.println("Switched to MANUAL mode (BLE)");
            currentManualState = MANUAL_IDLE;
            break;

        case 'a':
            manualMode = false;
            autoMode = true;
            Serial.println("Switched to AUTO mode (BLE)");
            currentAutoState = IDLE;
            previousAutoState = IDLE;
            break;

        case 'f':
            if (manualMode) {
                if (currentManualState == MANUAL_FORWARD) {
                    Serial.println("Forward stopped");
                    currentManualState = MANUAL_STOP;
                } else {
                    Serial.println("Forward command");
                    currentManualState = MANUAL_FORWARD;
                }
            }
            break;

        case 'b':
            if (manualMode) {
                if (currentManualState == MANUAL_BACKWARDS) {
                    Serial.println("Backwards stopped");
                    currentManualState = MANUAL_STOP;
                } else {
                    Serial.println("Backwards command");
                    currentManualState = MANUAL_BACKWARDS;
                }
            }
            break;
        case 'r':
            if (manualMode) {
                if (currentManualState == MANUAL_RIGHT) {
                    Serial.println("Turning right stopped");
                    currentManualState = MANUAL_STOP;
                } else {
                    Serial.println("Turning right");
                    currentManualState = MANUAL_RIGHT;
                }
            }
                break;
        case 'l':
        if (manualMode){
            if (currentManualState == MANUAL_LEFT) {
                Serial.println("Turning left stopped");
                currentManualState = MANUAL_STOP;
            } else {
                Serial.println("Turning left");
                currentManualState = MANUAL_LEFT;
            }
        }
            break;
        default:
            Serial.println("Unknown BLE cmd");
            currentManualState = MANUAL_IDLE;
    }
}

// This cuts down on code but is it as readable?
int readAnalogSensors() {
    int linePosition = 0;

    for (byte i = 0; i < AN_SENSOR_COUNT; i++) {
        linePosition += analogRead(AN_SENSOR_PINS[i]) * i;
    }

    return linePosition;
}

bool readDigitalSensors() {
    bool digSensorArray[AN_SENSOR_COUNT];
    bool digitalIRFlag;

    for (byte i = 0; i < AN_SENSOR_COUNT; i++) {
        digSensorArray[i] = digitalRead(DIG_SENSOR_PINS[i]);
    }
    // Surely there's a cleaner way of doing this.
    return (!digSensorArray[0] && !digSensorArray[1] && !digSensorArray[2]);
}

// Accept the struct and constrain the values between 0 and 250;
void setMotorSpeed(const MotorSpeeds &speed) {
    analogWrite(MOTOR_1A_PIN, constrain(speed.leftA, MIN_SPEED, MAX_SPEED));
    analogWrite(MOTOR_1B_PIN, constrain(speed.leftB, MIN_SPEED, MAX_SPEED));
    analogWrite(MOTOR_2A_PIN, constrain(speed.rightA, MIN_SPEED, MAX_SPEED));
    analogWrite(MOTOR_2B_PIN, constrain(speed.rightB, MIN_SPEED, MAX_SPEED));
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
    if ((motorInputSignal >= SET_SPEED) && !digitalIRFlag) {
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
    /* Initial value calculation: SPEED VALUE (122) / MAX ERROR (2000) = KP
    Go down in value by 0.01 to check behaviour, then go up and try again. */
    constexpr float KP = 0.061;
    constexpr float KD = 0;

    // Check if the digital pins are active before doing anything else.
    bool digitalIRFlag = readDigitalSensors();

    // Logic to transition out of pid loop based on IR sensor feedback.
    if (digitalIRFlag) {
        currentAutoState = STOP;
        return;
    }

    // Read in the current line position and adjust the error.
    int linePosition = readAnalogSensors();
    int error = SETPOINT - linePosition;
    int adjust = error * KP + KD * (error - lastError);

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