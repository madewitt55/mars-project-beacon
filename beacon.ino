/**
 * @file beacon.ino
 * @brief Firmware for controlling a Bluetooth compatible beacon
 *
 * @note Beacon firmware connects out to the rover over Bluetooth as the SPP master, dialing the MAC
         address defined as `ROVER_BT_MAC`. Any connected serial monitor should run at 9600 baud. On
         boot, setup() waits 3 seconds to let the serial monitor attach before initialization output
         is printed.
 */

#include "Logger.h" // Local logging helper class
#include "Bluetooth.h" // Local Bluetooth SPP connection and rover command helpers

Logger logger; // Initialize logger on default serial

//---------- Joystick ----------
#define JOYSTICK_VRX 36      // Analog input for joystick X axis 
#define JOYSTICK_VRY 39      // Analog input for joystick Y axis
#define JOYSTICK_ADC_MAX 4095 // Maximum raw reading from the ESP32's 12-bit ADC
#define JOYSTICK_BASE_X -0.08f // Measured resting X value, subtracted out to calibrate center to 0
#define JOYSTICK_BASE_Y -0.1f  // Measured resting Y value, subtracted out to calibrate center to 0
#define JOYSTICK_DEADZONE 0.05f // Normalized magnitude below which axis input is treated as 0 (absorbs center drift/jitter)

void setup() {
    Serial.begin(9600); // Baud rate 9600 Bd
    delay(3000); // Allow serial monitor time to open

    beginBluetooth(); // Registers the BT callback and connects out to the rover
}

void loop() {
    // Read joystick
    float motor_left, motor_right;
    readJoystickMotorValues(motor_left, motor_right);
    Serial.println(motor_left);
    Serial.println(motor_right);

    sendMotorValues(motor_left, motor_right); // Send instructions

    delay(300);
}

/**
 * @brief Reads the joystick and converts it into calibrated left/right motor power levels
 *
 * @note Uses arcade-style controls: y-axis is throttle, x-axis is turn. Clamps raw joystick readings 
         to a -1.0 to 1.0 range after accounting for measured resting values and configured deadzones.
 *
 * @param motor_left Output: desired left motor power, from -1.0 (full reverse) to 1.0 (full forward)
 * @param motor_right Output: desired right motor power, from -1.0 (full reverse) to 1.0 (full forward)
*/
void readJoystickMotorValues(float &motor_left, float &motor_right) {
    // Raw x is read from joystick's y axis, raw y is read from joystick's x axis
    // These are swapped to acheive accurate values with the desired joystick orientation
    uint16_t joystick_raw_x = analogRead(JOYSTICK_VRY);
    uint16_t joystick_raw_y = analogRead(JOYSTICK_VRX);

    // Constrain joystick values to -1 to 1 range, considering measured resting values for each axis
    float joystick_x = constrain((joystick_raw_x * 2.0f / JOYSTICK_ADC_MAX) - 1.0f - JOYSTICK_BASE_Y, -1.0f, 1.0f);
    float joystick_y = constrain((joystick_raw_y * 2.0f / JOYSTICK_ADC_MAX) - 1.0f - JOYSTICK_BASE_X, -1.0f, 1.0f);

    // Apply deadzones
    if (fabsf(joystick_x) < JOYSTICK_DEADZONE) joystick_x = 0.0f;
    if (fabsf(joystick_y) < JOYSTICK_DEADZONE) joystick_y = 0.0f;

    // Arcade-style controls: y-axis is throttle, x-axis is turn
    motor_left = constrain(joystick_y + joystick_x, -1.0f, 1.0f);
    motor_right = constrain(joystick_y - joystick_x, -1.0f, 1.0f);
}

/**
 * @brief Sends the MOTORS command with validated left and right motor power levels
 *
 * @note Clamps `left` and `right` to the valid -1.0 to 1.0 range before sending. Values are
         written as decimal text, e.g. `sendMotorValues(-0.5, 1.0)` sends
         "LEFT\n0.50\nRIGHT\n1.00\n".
 *
 * @param left Desired left motor power, from -1.0 (full reverse) to 1.0 (full forward)
 * @param right Desired right motor power, from -1.0 (full reverse) to 1.0 (full forward)
*/
void sendMotorValues(float left, float right) {
    if (!bt_connected) {
        //logger.warn("Cannot send command, ROVER not connected");
        return;
    }
    // Constrain left and right to float value from -1.0 to 1.0
    left = constrain(left, -1.0f, 1.0f);
    right = constrain(right, -1.0f, 1.0f);
    sendCommand(CMD_LEFT, left);
    sendCommand(CMD_RIGHT, right);
}
