#ifndef __WATCHDOG_INTERFACE_H__
#define __WATCHDOG_INTERFACE_H__

#include <Arduino.h>

void pulse_ams_watchdog(bool &current_pulse, int teensy_to_vehicle_watchdog_pulse_pin = 5);

void push_ok_watchdog(bool &current_pulse, int teensy_to_acu_watchdog_ok_pin = 6);

#endif