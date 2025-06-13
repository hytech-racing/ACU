#include "ShutdownInterface.h"

#include <Arduino.h>

ShutdownInterface::ShutdownInterface(int shutdown_pin_number): 
    _shutdown_pin_number(shutdown_pin_number) 
{

    pinMode(shutdown_pin_number, INPUT);
}

void ShutdownInterface::sample_shutdown_state()
{

    if(_updateable) {
        _shutdown_high = digitalRead(_shutdown_pin_number);
        // becomes NOT updateable if the shutdown is LOW
        _updateable = !_shutdown_high;
    }
}

bool ShutdownInterface::get_latest_status()
{
    // reset to where we can sample again
    _updateable = true;
    return _shutdown_high;
}