/**
 * @file beacon.ino
 * @brief Firmware for controlling a Bluetooth and UltraWide Band compatible beacon
 *
 * @details Designed to run on a compatible microcontroller in combination with a Bluetooth antenna
            and an UltraWide Band antenna. The prototype uses a MakerFabs ESP32 UWB, which features
            the ESP32-D0WD-V3 microcontroller chip, a DW1000 UWB antenna, and an integrated onboard
            PCB trace WiFi and Bluetooth antenna.
 *
 * @note Beacon firmware connects out to the rover over Bluetooth as the SPP master, dialing the MAC
         address defined as `ROVER_BT_MAC`, and ignores any UWB ranging device other than the short
         hardware address defined as `ROVER_SHORT_ADDRESS`. This value is derived from the first 2
         bytes of the antenna's complete hardware address. Any connected serial monitor should run at
         9600 baud. On boot, setup() waits 3 seconds to let the serial monitor attach before
         initialization output is printed.
 */

#include <SPI.h> // Enables Serial Peripheral Interface communication
#include "DW1000Ranging.h" // MakerFabs DW1000 UWB chip ranging library, imported as ZIP
#include "Logger.h" // Local logging helper class
#include "Bluetooth.h" // Local Bluetooth SPP connection and rover command helpers

Logger logger; // Initialize logger on default serial

//---------- UltraWide Band ----------
// Controller: Microcontroller, Peripheral: DW1000 UWB
const uint16_t DW1000_ANTENNA_DELAY = 16470; // 1 unit = 15.65ps (picoseconds)
#define SPI_SCK 18  // SPI clock, keeps controller and peripheral in sync
#define PIN_PS 4    // SPI peripheral select (controller pulls to LOW to open transaction with controller)
#define SPI_MISO 19 // SPI data in (peripheral -> controller, data received from rover)
#define SPI_MOSI 23 // SPI data out (controller -> peripheral, purely configuration data)
#define PIN_RST 27  // Hardware reset (controller pulls to LOW during init to reboot peripheral)
#define PIN_IRQ 34  // Interrupt (peripheral raises to HIGH when it has a completed measurement)
#define BEACON_HARDWARE_ADDRESS "86:17:5B:D5:A9:9A:E2:9C" // Hardware address of the onboard DW1000 UWB chip
#define ROVER_HARDWARE_ADDRESS "7D:00:22:EA:82:60:3B:9C"  // Hardware address of rover DW1000 UWB chip
#define ROVER_SHORT_ADDRESS 0xA49C                        // Short address of rover, derived from first two bytes of ROVER_HARDWARE_ADDRESS

//---------- Joystick ----------
#define JOYSTICK_VRX 36      // Analog input for joystick X axis
#define JOYSTICK_VRY 39      // Analog input for joystick Y axis
#define JOYSTICK_ADC_MAX 4095 // Maximum raw reading from the ESP32's 12-bit ADC
#define JOYSTICK_BASE_X -0.08f // Measured resting X value, subtracted out to calibrate center to 0
#define JOYSTICK_BASE_Y -0.1f  // Measured resting Y value, subtracted out to calibrate center to 0
#define JOYSTICK_DEADZONE 0.05f // Normalized magnitude below which axis input is treated as 0 (absorbs center drift/jitter)

/**
 * @brief Activates a new connected DW1000 UWB device
 *
 * @note Registered as the callback function for when a previously unseen UWB device
         joins the ranging exchange. Ignores any device that is not recognized as the rover.
 *
 * @param device New device to be activated
*/
void newDeviceCallback(DW1000Device *device) {
    logger.info("New UWB device detected");
    if (device->getShortAddress() != ROVER_SHORT_ADDRESS) {
        logger.warn("New UWB device not recognized as ROVER, ignoring");
        return;
    }

    // Rover activated
    logger.info("New UWB device recognized as ROVER");
}

/**
 * @brief Deactivates a connected DW1000 UWB device
 *
 * @note Registered as the callback function for a DW1000 UWB device that has not been heard from
 *
 * @param device Device being deactivated
*/
void inactiveDeviceCallback(DW1000Device *device) {
    if (device->getShortAddress() != ROVER_SHORT_ADDRESS) {
        logger.info("Unrecognized UWB device disconnected");
        return;
    }

    // Rover deactivated
    logger.error("ROVER disconnected from UWB");
}

void setup() {
    Serial.begin(9600); // Baud rate 9600 Bd
    delay(3000); // Allow serial monitor time to open

    beginBluetooth(); // Registers the BT callback and connects out to the rover

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI); // Initialize SPI bus, hardwired to onboard DW1000 UWB chip
    DW1000Ranging.initCommunication(PIN_RST, PIN_PS, PIN_IRQ); // Initialize DW1000 UWB chip
    DW1000.setAntennaDelay(DW1000_ANTENNA_DELAY); // Delay after UWB signal arrives or before it leaves, fine-tuned for accurate distance readings

    DW1000Ranging.attachNewRange([]() {}); // Called every time a complete ranging exchange between UWB chips occurs; not needed for beacon
    DW1000Ranging.attachBlinkDevice(newDeviceCallback); // Called when an unseen device joins the ranging exchange; only fires on initial detection of rover
    DW1000Ranging.attachInactiveDevice(inactiveDeviceCallback); // Called when a device hasn't been heard from (lost connection, out of range)

    DW1000Ranging.startAsAnchor(BEACON_HARDWARE_ADDRESS, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false); // Start UWB radio as an anchor; long range low power mode
}

void loop() {
    DW1000Ranging.loop(); // Ranging library's main state machine, must be called repeatedly

    // Read joystick
    float motor_left, motor_right;
    readJoystickMotorValues(motor_left, motor_right);

    sendMotorValues(motor_left, motor_right); // Send instructions

    delay(100);
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
