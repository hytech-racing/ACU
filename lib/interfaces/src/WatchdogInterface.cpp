#include "WatchdogInterface.h"

void pulse_ams_watchdog(bool &current_pulse, int teensy_to_vehicle_watchdog_pulse_pin) {
    digitalWrite(teensy_to_vehicle_watchdog_pulse_pin, current_pulse ? HIGH : LOW);
    current_pulse = !current_pulse;
}

void push_ok_watchdog(int teensy_to_vehicle_watchdog_ok_pin) {
    digitalWrite(teensy_to_vehicle_watchdog_ok_pin, HIGH);
}