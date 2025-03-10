#ifndef __SYSTEMTIMEINTERFACE_H__
#define __SYSTEMTIMEINTERFACE_H__

namespace sys_time
{
    unsigned long hal_millis();
    unsigned long hal_micros();
}

#endif // __SYSTEMTIMEINTERFACE_H__