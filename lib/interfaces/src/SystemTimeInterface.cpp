#include "SystemTimeInterface.h"

#include <Arduino.h>

namespace sys_time
{
    unsigned long hal_millis()
    {
        return millis();
    }

    unsigned long hal_micros()
    {
        return micros();
    }

    unsigned long micros_to_millis(unsigned long micros)
    {
        return micros / MILLIS_TO_MICROS_FACTOR;
    }

    unsigned long millis_to_micros(unsigned long millis)
    {
        return millis * MILLIS_TO_MICROS_FACTOR;
    }
}