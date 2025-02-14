#include "WatchdogInterface.h"

void pulse_ams_watchdog(bool &current_pulse, int teensy_to_vehicle_watchdog_pin) {
    digitalWrite(teensy_to_vehicle_watchdog_pin, current_pulse ? HIGH : LOW);
    current_pulse = !current_pulse;
}