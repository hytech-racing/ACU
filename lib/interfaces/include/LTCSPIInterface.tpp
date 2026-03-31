/* Interface Includes */
#include "LTCSPIInterface.h"
#include "SPI.h"
#include <Arduino.h>
#include <array>
#include <cstdint>
#include <EventResponder.h>

namespace ltc_spi_interface 
{
    static volatile bool _dma_busy = false;

    bool is_busy()
    {
        return _dma_busy;
    }

    template <size_t buffer_size>
    void begin_transfer(std::array<uint8_t, buffer_size> tx_buf, std::array<uint8_t, buffer_size> rx_buf, EventResponder& event)
    {
        _dma_busy = true;
        SPI1.transfer(tx_buf.data(), rx_buf.data(), buffer_size, event);
    }

    void _write_and_delay_low(int cs, int delay_us) 
    {
        digitalWrite(cs, LOW);
        delayMicroseconds(delay_us);
    }

    void _write_and_delay_high(int cs, int delay_us) 
    {
        digitalWrite(cs, HIGH);
        delayMicroseconds(delay_us);
    }
}

