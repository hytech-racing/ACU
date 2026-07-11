#include "LTCSPIInterface.h"
#include <EventResponder.h>


namespace ltc_spi_interface
{
    template <size_t buffer_size>
    void begin_transfer(std::array<uint8_t, buffer_size> &tx_buf, std::array<uint8_t, buffer_size> &rx_buf, EventResponder& event)
    {
        _dma_busy = true;
        SPI1.transfer(tx_buf.data(), rx_buf.data(), buffer_size, event);
    }
}

