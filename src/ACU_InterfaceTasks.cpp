#include "ACU_InterfaceTasks.h"

void initialize_interfaces() {
    SPI.begin();
    Serial.begin(115200);
    analogReadResolution(12);
    /* BMS Driver */
    BMSDriverInstance<NUM_CHIPS, NUM_CHIP_SELECTS, chip_type::LTC6811_1>::create();
    BMSDriverInstance<NUM_CHIPS, NUM_CHIP_SELECTS, chip_type::LTC6811_1>::instance().init();
    /* Watchdog Interface */
    WatchdogInstance::create();
    WatchdogInstance::instance().init();
}

bool run_kick_watchdog(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    WatchdogInstance::instance().update_watchdog_state(sys_time::hal_millis()));
    return true;
}


void set_bit(uint16_t &value, uint8_t index, bool bitValue) {
    if (index >= 16) return; // Ensure index is within range

    if (bitValue) {
        value |= (1 << index);  // Set the bit
    } else {
        value &= ~(1 << index); // Clear the bit
    }
}