#include "SystemTimeInterface.h"


namespace sys_time
{
    unsigned long hal_millis() { return millis(); }
    unsigned long hal_micros() { return micros(); }
}