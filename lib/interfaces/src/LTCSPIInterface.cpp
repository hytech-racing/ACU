#include "LTCSPIInterface.h"


namespace ltc_spi_interface
{
    static volatile bool _dma_busy = false;

    bool is_busy()
    {
        return _dma_busy;
    }

    void set_dma_idle()
    {
        _dma_busy = false;
    }

    void write_and_delay_low(int cs, int delay_us)
    {
        digitalWrite(cs, LOW);
        delayMicroseconds(delay_us);
    }

    void write_and_delay_high(int cs, int delay_us)
    {
        digitalWrite(cs, HIGH);
        delayMicroseconds(delay_us);
    }

    void delay_and_write_high(int cs, int delay_us)
    {
        delayMicroseconds(delay_us);
        digitalWrite(cs, HIGH);
    }
}

