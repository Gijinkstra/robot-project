#include "BLEComms.h"

BLEComms::BLEComms(RobotController &robot)
    : robot(robot),
      terminalService(TERMINAL_ID),
      readCharacteristic(TERMINAL_ID, BLERead | BLEWrite),
      writeCharacteristic(STRING_ID, BLERead | BLENotify, BLE_STRING_SIZE),
      central()  // default-constructed "empty" BLEDevice
{
}

void BLEComms::init() {
    if (!BLE.begin()) {
        Serial.println("Error: BLE startup failed.");
        return;
    }

    BLE.setLocalName(DEVICE_NAME);
    BLE.setAdvertisedService(terminalService);

    terminalService.addCharacteristic(readCharacteristic);
    terminalService.addCharacteristic(writeCharacteristic);

    BLE.addService(terminalService);
    readCharacteristic.writeValue(0);

    BLE.advertise();

    Serial.println("BLE initialised and advertising");
}

void BLEComms::monitor() {
    // Let BLE stack process pending events; non-blocking.
    BLE.poll();

    // If we don't yet have a central, or it got reset, try to get one
    if (!central) {
        central = BLE.central();
        if (central) {
            outgoingMessage("BLE central connected");
        }
        return;
    }

    // We *had* a central; make sure it's still connected
    if (!central.connected()) {
        outgoingMessage("BLE central disconnected");
        central = BLEDevice(); // reset
        return;
    }

    // Central exists and is connected; check if a command was written
    if (readCharacteristic.written()) {
        char msg = readCharacteristic.value();
        Serial.print("BLE command: ");
        Serial.println(msg);
        incomingMessage(msg);
    }
}

void BLEComms::outgoingMessage(const String &msg) {
    writeCharacteristic.writeValue(msg + "\n");
}

void BLEComms::incomingMessage(char msg) {
    switch (tolower(msg)) {
        case 'm':
            robot.setManualMode();
            outgoingMessage("Switched to MANUAL mode (BLE)");
            break;

        case 'a':
            robot.setAutoMode();
            outgoingMessage("Switched to AUTO mode (BLE)");
            break;

        case 'f':
            if (robot.isManualMode()) {
                if (robot.getManualState() == ManualState::Forward) {
                    robot.setManualState(ManualState::Stop);
                    outgoingMessage("Forward stopped");
                } else {
                    robot.setManualState(ManualState::Forward);
                    outgoingMessage("Forward command");
                }
            }
            break;

        case 'b':
            if (robot.isManualMode()) {
                if (robot.getManualState() == ManualState::Backwards) {
                    robot.setManualState(ManualState::Stop);
                    outgoingMessage("Backwards stopped");
                } else {
                    robot.setManualState(ManualState::Backwards);
                    outgoingMessage("Backwards command");
                }
            }
            break;

        case 'r':
            if (robot.isManualMode()) {
                if (robot.getManualState() == ManualState::Right) {
                    robot.setManualState(ManualState::Stop);
                    outgoingMessage("Turning right stopped");
                } else {
                    robot.setManualState(ManualState::Right);
                    outgoingMessage("Turning right");
                }
            }
            break;

        case 'l':
            if (robot.isManualMode()) {
                if (robot.getManualState() == ManualState::Left) {
                    robot.setManualState(ManualState::Stop);
                    outgoingMessage("Turning left stopped");
                } else {
                    robot.setManualState(ManualState::Left);
                    outgoingMessage("Turning left");
                }
            }
            break;

        case 'c':
            // Do not allow sensor calibration outside of idle state.
            if (!robot.isManualMode() && !robot.isAutoMode()) {
                outgoingMessage("Starting calibration...");
                // TODO: call calibration routine here if you expose it via RobotController
                outgoingMessage("Calibration completed.");
            }
            break;

        default:
            outgoingMessage("Unknown BLE cmd");
            robot.setManualState(ManualState::Idle);
            break;
    }
}
