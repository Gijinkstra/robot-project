// --------- Libraries ---------------------
#include <ArduinoBLE.h>

#include "Config.h"
#include "MotorDriver.h"
#include "LineSensors.h"
#include "RobotController.h"

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

void parseBLEMessage(RobotController &robot, char msg) {
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