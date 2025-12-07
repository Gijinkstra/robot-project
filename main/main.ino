// --------- Libraries ---------------------
#include <ArduinoBLE.h>
#include <Arduino_LED_Matrix.h>
#include <ArduinoGraphics.h>

#include "Config.h"
#include "MotorDriver.h"
#include "LineSensors.h"
#include "RobotController.h"
#include "BLEComms.h"
#include "LEDController.h"

// ---------- Constructors --------------
MotorDriver         motors(MOTOR_PINS);

LineSensors         sensors(AN_SENSOR_PINS, AN_SENSOR_COUNT);

RobotController     robot(motors, sensors);

BLEComms            bleComms(robot);

// ArduinoLEDMatrix    matrix;

// LEDController       ledMatrix(robot, matrix);

// ---------- Startup --------------
void setup() {
    Serial.begin(BAUD_RATE);
    motors.init();
    sensors.init();
    bleComms.init();
    // ledMatrix.init();
}

// ---------- Running --------------
void loop() {
    unsigned long now = millis();

    static unsigned long lastBLECheck = 0;
    if ((now - lastBLECheck) >= BLE_TIMER){
        bleComms.monitor();
        lastBLECheck = now;
    }

    robot.update(now);
    // ledMatrix.update(now);
    // ****************** DEBUGGING - REMOVE *****************************
    Serial.print("Line position: "); Serial.println(sensors.linePosition());

    // Serial.println("Output flag: " + String(digitalIRFlag));
    for (int i = 0; i < AN_SENSOR_COUNT; i++) {
        Serial.print("Sensor "); Serial.print(i);
        Serial.print(": "); Serial.println(sensors.getAnalogReading(i));
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