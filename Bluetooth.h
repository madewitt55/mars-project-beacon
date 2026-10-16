/**
 * @file Bluetooth.h
 * @brief Classic Bluetooth Serial Port Profile (SPP) link between the beacon and the rover
 *
 * @note Beacon acts as the SPP master, dialing out to `ROVER_BT_MAC`. Also defines the simple
         newline-terminated command protocol used to send drive signals to the rover.
 */

#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <BluetoothSerial.h> // Enables classic Bluetooth Serial Port Profile communication
#include "esp_spp_api.h" // Allows registration of Serial Port Profile Bluetooth callbacks
#include "esp_gap_bt_api.h" // Allows removal of stale Bluetooth bonds
#include "esp_bt_device.h" // Allows reading the onboard chip's actual Bluetooth MAC address
#include "Logger.h" // Local logging helper class

extern Logger logger; // Defined in beacon.ino

BluetoothSerial SerialBT;
volatile bool bt_connected = false;
#define BEACON_BT_NAME "BEACON"
#define ROVER_BT_MAC "58:2A:BD:D2:E5:96"
#define BEACON_BT_MAC "20:50:0D:E4:5C:92"

// Rover commands
#define CMD_LEFT "LEFT"
#define CMD_RIGHT "RIGHT"
#define CMD_TERMINATOR "\n"

/**
 * @brief Helper function for parsing a plain text colon-separated MAC address into a raw MAC address
 *
 * @note Used for converting a hardcoded readable MAC into the raw 6-byte form
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
 * @note Registered as the callback for the Bluetooth SPP stack. Beacon is the SPP master, so
         it initiates outbound connections and does not listen for inbound ones
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
 * @brief Initializes Bluetooth and attempts to connect to the rover
 *
 * @note Registers `bluetoothCallback` for connection events, starts the Bluetooth stack in SPP
         master mode, removes any stale bond for `ROVER_BT_MAC` so pairing is renegotiated fresh,
         then attempts to connect. Should be called once from `setup()`.
*/
void beginBluetooth() {
    SerialBT.register_callback(bluetoothCallback); // Callback function fires every time BT stack has an event
    SerialBT.begin(BEACON_BT_NAME, true); // true = master mode; beacon does not listen for connections, only initiates them

    uint8_t rover_mac[6];
    if (!parseMac(ROVER_BT_MAC, rover_mac)) {
        logger.error("ROVER_BT_MAC is malformed, cannot connect");
        return;
    }

    esp_bt_gap_remove_bond_device(rover_mac); // Clear any stale bond so pairing is renegotiated fresh

    logger.info("Attempting Bluetooth connection with ROVER");
    if (!SerialBT.connect(rover_mac)) {
        logger.warn("Bluetooth connection to ROVER failed");
    }
}

/**
 * @brief Sends a command string to the rover over the established Bluetooth connection
 *
 * @note Commands are newline-terminated so the rover knows when each command ends regardless
         of number of bytes.
 *
 * @param command Command name to send (e.g. `CMD_FORWARD`)
*/
void sendCommand(const char* command) {
    if (!bt_connected) {
        logger.warn("Cannot send command, ROVER not connected");
        return;
    }
    SerialBT.print(command);
    SerialBT.print(CMD_TERMINATOR);
}

/**
 * @brief Sends a command string followed by an 8-bit integer value to the rover
 *
 * @note Value is written as plain decimal text after `command` and a newline terminator, e.g.
         `sendCommand(CMD_FORWARD, 75)` sends "FORWARD\n75\n".
 *
 * @param command Command name to send (e.g. `CMD_FORWARD`)
 * @param value Float value to send immediately after `command`
*/
void sendCommand(const char* command, float value) {
    if (!bt_connected) {
        logger.warn("Cannot send command, ROVER not connected");
        return;
    }
    SerialBT.print(command);
    SerialBT.print(CMD_TERMINATOR);
    SerialBT.print(value);
    SerialBT.print(CMD_TERMINATOR);
}

#endif
