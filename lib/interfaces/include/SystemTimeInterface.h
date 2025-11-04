#ifndef SYSTEMTIMEINTERFACE_H
#define SYSTEMTIMEINTERFACE_H

namespace sys_time
{
    unsigned long hal_millis();
    unsigned long hal_micros();

    unsigned long micros_to_millis(unsigned long micros);
    unsigned long millis_to_micros(unsigned long millis);
};

#endif // SYSTEMTIMEINTERFACE_H