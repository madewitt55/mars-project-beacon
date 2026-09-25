#include <SPI.h>
#include "DW1000Ranging.h" // MakerFabs DW1000 UWB chip ranging library, imported as ZIP

//---------- DW1000 UWB ----------
// Controller: ESP32-WROOM-32E, Peripheral: DW1000 UWB
const uint16_t DW1000_ANTENNA_DELAY = 16470; // 1 unit = 15.65ps (picoseconds)
#define SPI_SCK 18  // SPI clock, keeps controller and peripheral in sync
#define PIN_PS 4    // SPI peripheral select (controller pulls to LOW to open transaction with controller)
#define SPI_MISO 19 // SPI data in (peripheral -> controller, data recieved from rover)
#define SPI_MOSI 23 // SPI data out (controller -> peripheral, purely configuration data)
#define PIN_RST 27  // Hardware reset (controller pulls to LOW during init to reboot peripheral)
#define PIN_IRQ 34  // Interrupt (peripheral raises to HIGH when it has a completed measurement)
#define BEACON_HARDWARE_ADDRESS "86:17:5B:D5:A9:9A:E2:9C" // Hardware address of the onboard DW1000 UWB chip
#define ROVER_HARDWARE_ADDRESS "7D:00:22:EA:82:60:3B:9C"  // Hardware address of rover DW1000 UWB chip
#define ROVER_SHORT_ADDRESS 0xA49C                        // Short address of rover, derived from first two bytes of ROVER_HARDWARE_ADDRESS

// Logging functions
inline void LOG_DEBUG(const String& msg) { Serial.print("[  DEBUG  ] "); Serial.println(msg); }
inline void LOG_INFO(const String& msg)  { Serial.print("[  INFO   ] "); Serial.println(msg); }
inline void LOG_WARN(const String& msg)  { Serial.print("[! WARN ! ] "); Serial.println(msg); }
inline void LOG_ERROR(const String& msg) { Serial.print("[** ERROR **] "); Serial.println(msg); }

void setup() {
    Serial.begin(9600); // Baud rate 9600 Bd

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI); // Initialize SPI bus, hardwired to oboard DW1000 UWB chip
    DW1000Ranging.initCommunication(PIN_RST, PIN_PS, PIN_IRQ); // Initialize DW1000 UWB chip
    DW1000.setAntennaDelay(DW1000_ANTENNA_DELAY); // Delay after UWB signal arrives or before it leaves, fine-tuned for accurate distance readings
    // Register callback functions
    DW1000Ranging.attachNewRange([]() {}); // Called every time a complete ranging exchange between UWB chips occurs; not needed for beacon
    DW1000Ranging.attachBlinkDevice(newDevice); // Called when an unseen device joins the ranging exchange; only fires on initial detection of rover
    DW1000Ranging.attachInactiveDevice(inactiveDevice); // Called when a device hasn't been heard from (lost connection, out of range)

    DW1000Ranging.startAsAnchor(BEACON_HARDWARE_ADDRESS, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false); // Start UWB radio as an anchor; long range low power mode
}

void loop() {
    DW1000Ranging.loop(); // Ranging library's main state machine, must be called repeatedly
}

/**
 * @brief Registers a new connected DW1000 UWB device
 *
 * @param device New DW1000 UWB device to be registered
*/
void newDevice(DW1000Device *device) {
    LOG_INFO("New device connected");
    if (device->getShortAddress() != ROVER_SHORT_ADDRESS) {
        LOG_WARN("New device not recognized as ROVER");
        return;
    }

    // Rover activated
    LOG_INFO("ROVER connected. Short address: " + device->getShortAddress());
}

/**
 * @brief Deactivates a connected DW1000 UWB device
 *
 * @param device Pointer to connected device
*/
void inactiveDevice(DW1000Device *device) {
    if (device->getShortAddress() != ROVER_SHORT_ADDRESS) {
        LOG_INFO("Unrecognized device disconnected");
        return;
    }

    // Rover deactivated
    LOG_ERROR("ROVER disconnected");
}
