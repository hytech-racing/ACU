#ifndef SHUTDOWNINTERFACE_H
#define SHUTDOWNINTERFACE_H


#include <etl/singleton.h>


class ShutdownInterface
{
    public:
        
        ShutdownInterface(int shutdown_pin_number);
        
        // latching call if this ever goes low from high
        void sample_shutdown_state();

        // resets the ability to sample shutdown state
        bool get_latest_status();
    private:
        int _shutdown_pin_number;
        bool _shutdown_high = false;
        bool _updateable = true;
};

using ShutdownInterfaceInstance = etl::singleton<ShutdownInterface>;
#endif // SHUTDOWNINTERFACE_H