#ifndef LTCSPIINTERFFACE
#define LTCSPIINTERFFACE

/* Interface Includes */
#include <SPI.h>
#include <stddef.h>
#include <array>

namespace ltc_spi_interface {
    /**
     * @brief initialize Event Responder for DMA
    */
    void init_DMA();

    /**
     * @brief getter function for dma_busy to let the bms driver know if it can initialize a new spi transfer
     * @return bool - is_busy
    */
    bool is_busy();

    /**
     * @brief begin_transfer is a more advanced SPI.transfer wrapper that uses a tx_buf, rx_buf, length, and EventResponder to initiate a callback when finished
     * @param tx_buf is the buffer that needs to be transmitted
     * @param rx_buf is the buffer that will be filled in response to the tx_buf
     * @param event is a reference to a EventResponder that the low-level SPI library uses to keep track of the DMA completion 
     * Note that the tx_buf and rx_buf are the same length, which is recorded in the buffer_size template variable
     * @return void
    */
    template <size_t buffer_size>
    void begin_transfer(std::array<uint8_t, buffer_size> tx_buf, std::array<uint8_t, buffer_size> rx_buf, EventResponder& event);

    /**
     * @brief _write_and_delay_high/low sets the chip select pin HIGH and then delays for a few microseconds
     * @param cs is the chip select pin on the teensy that needs to be digitally written
     * @param delay_us is the number of microseconds to delay for 
     * @return void
    */
    inline void _write_and_delay_high(int cs, int delay_us);
    inline void _write_and_delay_low(int cs, int delay_us);
}

#include <LTCSPIInterface.tpp>
#endif