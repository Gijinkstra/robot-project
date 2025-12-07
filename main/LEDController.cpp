#include "LEDController.h"

// ----- Example frames (12x8 matrix) -----
// These are just examples – replace with your own from the LED Matrix editor.

// Idle: a small dot in the centre
static const uint8_t FRAME_IDLE[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },

  { 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0 },

  { 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0 },

  { 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0 },

  { 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0 },

  { 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0 },

  { 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0 },

  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

// Manual mode: "M"-ish shape
static const uint8_t FRAME_MANUAL[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },

  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },

  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },

  { 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0 },

  { 0, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 0 },

  { 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0 },

  { 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0 },

  { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 }
};

// Auto idle tracking: simple bar in the middle
static const uint8_t FRAME_AUTO[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },

  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },

  { 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0 },

  { 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0 },

  { 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0 },

  { 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0 },

  { 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0 },

  { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 }
};

// Auto turning left: arrow left
static const uint8_t FRAME_LEFT[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },

  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },

  { 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 },

  { 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },

  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0 },

  { 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },

  { 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 },

  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

// Auto turning right: arrow right
static const uint8_t FRAME_RIGHT[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },

  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },

  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0 },

  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0 },

  { 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },

  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0 },

  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0 },

  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

// Auto stopped: square
static const uint8_t FRAME_STOP[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },

  { 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0 },

  { 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0 },

  { 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0 },

  { 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0 },

  { 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0 },

  { 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0 },

  { 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0 }
};


// ----- Implementation -----

LEDController::LEDController(RobotController &robot, ArduinoLEDMatrix &matrix)
    : robot(robot), matrix(matrix)
{
}

void LEDController::init() {
    matrix.begin();
    // matrix.textFont(Font_4x6);
    // matrix.beginText(0, 1, 0xFFFFFF);
    // matrix.println(DEVICE_NAME);
    // matrix.endText();
    // matrix.endDraw();
};

void LEDController::update(unsigned long now) {
    // Simple rate limit so we don't spam the matrix
    if (now - lastUpdate < UPDATE_INTERVAL) return;
    lastUpdate = now;

    if (!robot.isManualMode() && !robot.isAutoMode()) {
        drawIdle();
    } else if (robot.isManualMode()) {
        drawManualScreen();
    } else {
        drawAutoScreen();
    }
}

void LEDController::showFrame(const uint8_t frame[8][12]) {
    // The R4 matrix uses 8 rows; each row is a 32-bit value.
    uint32_t packed[8] = {0};

    for (int row = 0; row < 8; ++row) {
        uint32_t value = 0;
        for (int col = 0; col < 12; ++col) {
            if (frame[row][col]) {
                // set bit for this column; using bit (11 - col) so leftmost
                // element in your array is the MSB, tweak if flipped
                value |= (1u << (11 - col));
            }
        }
        packed[row] = value;
    }

    matrix.loadFrame(packed);  // now matches const uint32_t* parameter
}

void LEDController::drawIdle() {
    showFrame(FRAME_IDLE);
}

void LEDController::drawManualScreen() {
    // You could vary the icon based on manual state if you like
    // e.g. different arrows for F/B/L/R
    switch (robot.getManualState()) {
        case ManualState::Forward:
        case ManualState::Backwards:
        case ManualState::Left:
        case ManualState::Right:
            showFrame(FRAME_MANUAL);
            break;
        case ManualState::Idle:
        case ManualState::Stop:
        default:
            showFrame(FRAME_IDLE);
            break;
    }
}

void LEDController::drawAutoScreen() {
    drawAutoIcon(robot.getAutoState());
}

void LEDController::drawAutoIcon(AutoState state) {
    switch (state) {
        case AutoState::PIDLoop:
            showFrame(FRAME_AUTO);
            break;
        case AutoState::HardLeft:
            showFrame(FRAME_LEFT);
            break;
        case AutoState::HardRight:
            showFrame(FRAME_RIGHT);
            break;
        case AutoState::Stop:
        case AutoState::Turn:
            showFrame(FRAME_STOP);
            break;
        case AutoState::Idle:
        case AutoState::Accelerate:
        default:
            showFrame(FRAME_IDLE);
            break;
    }
}
