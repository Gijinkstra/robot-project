// --------- Libraries ---------------------
#include <ArduinoBLE.h>

// --------- Global constants --------------
// Pins
constexpr byte MOTOR_1A_PIN = 5;
constexpr byte MOTOR_1B_PIN = 6;
constexpr byte MOTOR_2A_PIN = 9;
constexpr byte MOTOR_2B_PIN = 10;
constexpr byte IR_SENSOR_PIN = A0;
constexpr byte IR_SENSOR_THR = 2;

// Motor constants.
constexpr byte MIN_SPEED = 0;
constexpr byte MAX_SPEED = 200;
constexpr byte SET_SPEED = 200;
constexpr byte ROTATION_SPEED = 50;
constexpr int ROTATION_DELAY = 1000;
constexpr byte INCREMENT = 5;
constexpr byte DELAY = 12;

// IR sensor constants.
constexpr float KP = 0.2;
constexpr byte KD = 0;
constexpr int IR_SENSOR_LOWER_THRESHOLD = 100;
constexpr int IR_SENSOR_UPPER_THRESHOLD = 900;
constexpr int SETPOINT = 500;
constexpr unsigned int TIMEOUT = 2500;
constexpr unsigned int WAIT_TIME = 300;

// Misc
constexpr int BAUD_RATE = 9600;
enum MotorState {STOP,
                 ACCELERATE,
                 PID_LOOP,
                 DECELERATE,
                 COAST,
                 TURN};

// --------- Global variables ----------
unsigned long stateEntryTimer;
unsigned long currentTime;
unsigned int motorInputSignal = 0;
int lastError = 0;
bool manualMode = false;
MotorState currentMotorState = STOP;
MotorState previousMotorState = STOP;

// ---------- BLE Service --------------
BLEService terminalService("19B10010-E8F2-537E-4F6C-D104768A1214"); // create service

BLEByteCharacteristic terminalCharacteristic("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

void setup() {
    Serial.begin(BAUD_RATE);
    pinMode(MOTOR_1A_PIN, OUTPUT);
    pinMode(MOTOR_1B_PIN, OUTPUT);
    pinMode(MOTOR_2A_PIN, OUTPUT);
    pinMode(MOTOR_2B_PIN, OUTPUT);
    pinMode(IR_SENSOR_PIN, INPUT);
    pinMode(IR_SENSOR_THR, INPUT);
    // startBLEModule();
}

void loop() {
    currentTime = millis();

    // BLE.poll();



    selectControlScheme();
    
    if (manualMode) {
        manualControl();
    } else {
        motorStateMachine();
    }

    // Timer to determine change of state.
    if (currentMotorState != previousMotorState) {
        stateEntryTimer = currentTime;
        previousMotorState = currentMotorState;
        if (currentMotorState == PID_LOOP) {
            lastError = 0;
        }
    }
}

// Execute manual or automatic control scheme based on Serial data.
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
    if (!BLE.begin()) {
        Serial.println("Error: BLE startup failed.");
    }

    BLE.setLocalName("Timmy Turner Express");
    BLE.setAdvertisedService(terminalService);

    terminalService.addCharacteristic(terminalCharacteristic);

    BLE.addService(terminalService);

    terminalCharacteristic.writeValue(0);

    BLE.advertise();

    Serial.println("Bluetooth connected. Awaiting commands...");
}

void monitorBLE() {

}

void manualControl() {
    
}

void motorStateMachine() {
    // State machine behaviour.
    switch (currentMotorState) {
        case STOP:          stopMotors();
        Serial.println(currentMotorState);      break;
        case ACCELERATE:    accelerateMotors();
        Serial.println(currentMotorState);      break;
        case PID_LOOP:      pidMotors();
        Serial.println(currentMotorState);      break;
        case DECELERATE:    decelerateMotors();
        Serial.println(currentMotorState);      break;
        case COAST:         coastMotors();
        Serial.println(currentMotorState);      break;
        case TURN:          turnMotors();
        Serial.println(currentMotorState);      break;
    }
}

void setMotorSpeed(int leftMotorSpeed, int rightMotorSpeed) {
    analogWrite(MOTOR_1A_PIN, constrain(leftMotorSpeed, MIN_SPEED, MAX_SPEED));
    analogWrite(MOTOR_1B_PIN, MIN_SPEED);
    analogWrite(MOTOR_2A_PIN, constrain(rightMotorSpeed, MIN_SPEED, MAX_SPEED));
    analogWrite(MOTOR_2B_PIN, MIN_SPEED);
}

void stopMotors() {
    if ((currentTime - stateEntryTimer) > DELAY) {
        currentMotorState = ACCELERATE;
    }
    else {
        analogWrite(MOTOR_1A_PIN, MAX_SPEED);
        analogWrite(MOTOR_1B_PIN, MAX_SPEED);
        analogWrite(MOTOR_2A_PIN, MAX_SPEED);
        analogWrite(MOTOR_2B_PIN, MAX_SPEED);
    }
}

void accelerateMotors() {
    // Static timer for acceleration.
    static unsigned long accelerationTimer = millis();

    if ((currentTime - accelerationTimer) >= DELAY && motorInputSignal < MAX_SPEED) {
        motorInputSignal += INCREMENT;
        setMotorSpeed(motorInputSignal, motorInputSignal);
        accelerationTimer = currentTime;
    }
    
    if (motorInputSignal >= MAX_SPEED) {
        currentMotorState = PID_LOOP;
    }
}

void pidMotors() {
    int sensorValue = analogRead(IR_SENSOR_PIN);
    bool sensorThr = digitalRead(IR_SENSOR_THR);

    int error = SETPOINT - sensorValue;

    int adjust = error * KP + KD * (error - lastError);

    setMotorSpeed(SET_SPEED + adjust, SET_SPEED - adjust);
    lastError = error;

    // Logic to transition out of pid loop based on IR sensor feedback.
    if (sensorValue > IR_SENSOR_UPPER_THRESHOLD) {
        currentMotorState = COAST;
    } 
    else if (sensorThr) {
        currentMotorState = DECELERATE;
    }
}

void coastMotors() {
    setMotorSpeed(MIN_SPEED, MIN_SPEED);

    // Transition back to the PID loop once we pass the line break.
    if ((currentTime - stateEntryTimer) >= WAIT_TIME) {
        currentMotorState = PID_LOOP;
    }
}

void decelerateMotors() {
    static unsigned long decelerationTime = millis();

    if ((currentTime - decelerationTime) >= DELAY && motorInputSignal > MIN_SPEED) {
        motorInputSignal = motorInputSignal - INCREMENT;
        setMotorSpeed(motorInputSignal, motorInputSignal);
        decelerationTime = currentTime;
    }
    
    if (motorInputSignal <= MIN_SPEED) {
        currentMotorState = TURN;
    }
}

void turnMotors() {
    if ((currentTime - stateEntryTimer) > ROTATION_DELAY) {
        currentMotorState = ACCELERATE;
    }
    else {
        analogWrite(MOTOR_1A_PIN, ROTATION_SPEED);
        analogWrite(MOTOR_1B_PIN, MIN_SPEED);
        analogWrite(MOTOR_2A_PIN, MIN_SPEED);
        analogWrite(MOTOR_2B_PIN, ROTATION_SPEED);
    }
}