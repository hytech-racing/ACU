#ifndef __WATCHDOG_INTERFACE_H__
#define __WATCHDOG_INTERFACE_H__

#include <Arduino.h>

void pulse_ams_watchdog(bool &current_pulse, int teensy_to_vehicle_watchdog_pin = 5);

#endif