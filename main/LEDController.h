#pragma once
#include <Arduino.h>
#include "Arduino_LED_Matrix.h"
#include "ArduinoGraphics.h"

#include "Config.h"
#include "RobotController.h"

class LEDController {
public:
    LEDController(RobotController &robot, ArduinoLEDMatrix &matrix);

    void init();

    void update(unsigned long now);

private:
    void drawIdle();
    void drawManualScreen();
    void drawAutoScreen();
    void drawAutoIcon(AutoState state);

    void showFrame(const uint8_t frame[8][12]);

    RobotController &robot;
    ArduinoLEDMatrix &matrix;

    unsigned long lastUpdate = 0;
};