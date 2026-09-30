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
#include <BluetoothSerial.h> // Enables classic Bluetooth Serial Port Profile communication
#include "esp_spp_api.h" // Allows registration of Serial Port Profile Bluetooth callbacks
#include "DW1000Ranging.h" // MakerFabs DW1000 UWB chip ranging library, imported as ZIP
#include "Logger.h" // Local logging helper class

Logger logger; // Initialize logger on default serial

//---------- Bluetooth ----------
BluetoothSerial SerialBT;
volatile bool bt_connected = false;
#define BEACON_BT_NAME "BEACON"
#define BEACON_BT_MAC "B0:A7:32:1B:93:62"
#define ROVER_BT_MAC "64:B7:08:66:8D:56"

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

/**
 * @brief Helper function for parsing a plain text colon-separated MAC address into a raw MAC address
 *
 * @note Used for converting a hardcoded readable MAC into the raw 6-byte form expected by BluetoothSerial::connect.
 *
 * @param plain_text_mac MAC address string, formatted as six colon-separated hex byte pairs (e.g. "AA:BB:CC:DD:EE:FF")
 * @param raw_mac Output buffer to receive the parsed raw 6-byte MAC address
 *
 * @return `true` if `plain_text_mac` was successfully parsed into `raw_mac`, `false` otherwise
*/
bool parseMac(const char* plain_text_mac, uint8_t raw_mac[6]) {
    int parsed = sscanf(plain_text_mac, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                        &raw_mac[0], &raw_mac[1], &raw_mac[2],
                        &raw_mac[3], &raw_mac[4], &raw_mac[5]);
    return parsed == 6;
}

/**
 * @brief Handles Bluetooth connection events for the outbound connection to the rover
 *
 * @note Registered as the callback for the Bluetooth SPP stack. Beacon is the SPP master, so the
         outbound connection it initiates completes as `ESP_SPP_OPEN_EVT` rather than the
         `ESP_SPP_SRV_OPEN_EVT` used by an SPP server.
 *
 * @param event Type of Bluetooth event that occurred (e.g. device connected, device disconnected)
 * @param param Event-specific data, unused here
*/
void bluetoothCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t* param) {
    if (event == ESP_SPP_OPEN_EVT) {
        logger.info("Bluetooth connected to ROVER");
        bt_connected = true;
    } else if (event == ESP_SPP_CLOSE_EVT) {
        if (bt_connected) {
            logger.error("ROVER disconnected from BT");
            bt_connected = false;
        }
    }
}

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

    SerialBT.register_callback(bluetoothCallback); // Callback function fires every time BT stack has an event
    SerialBT.begin(BEACON_BT_NAME, true); // true = master mode; beacon does not listen for connections, only initiates them

    // Bluetooth connection to rover
    logger.info("Attempting Bluetooth connection with ROVER");
    uint8_t rover_mac[6];
    if (!parseMac(ROVER_BT_MAC, rover_mac)) {
        logger.error("ROVER_BT_MAC is malformed, cannot connect");
    } else if (!SerialBT.connect(rover_mac)) {
        logger.warn("Bluetooth connection to ROVER failed");
    }

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
}
