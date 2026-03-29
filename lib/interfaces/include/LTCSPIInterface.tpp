/* Interface Includes */
#include "LTCSPIInterface.h"
#include "SPI.h"
#include <Arduino.h>
#include <array>
#include <cstdint>
#include <EventResponder.h>

template <size_t data_size>
void _transfer_SPI_data(const std::array<uint8_t, data_size> &data) 
{
    for (size_t i = 0; i < data_size; i++) 
    {
        SPI1.transfer(data[i]);
    }
}

template <size_t data_size>
std::array<uint8_t, data_size> _receive_SPI_data() 
{
    std::array<uint8_t, data_size> data_in;
    for (size_t i = 0; i < data_size; i++) 
    {

        data_in[i] = SPI1.transfer(0);
    }
   
    return data_in;
}

template <size_t data_size>
void _transfer_SPI_data_DMA(const std::array<uint8_t, data_size> &data) 
{
    EventResponder event;
    SPI.transfer(data.data(), nullptr, data_size, event);
}

template <size_t data_size>
std::array<uint8_t, data_size> _receive_SPI_data_DMA() 
{
    EventResponder event;
    std::array<uint8_t, data_size> data_in;
    SPI.transfer(nullptr, data_in.data(), data_size, event);
    return data_in;
}

namespace ltc_spi_interface 
{
    static volatile bool  _dma_busy = false;

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

    void _write_and_delay_low(int cs, int delay_microSeconds) 
    {
        digitalWrite(cs, LOW);
        delayMicroseconds(delay_microSeconds);
    }

    void _write_and_delay_high(int cs, int delay_microSeconds) 
    {
        digitalWrite(cs, HIGH);
        delayMicroseconds(delay_microSeconds);
    }

    template <size_t buffer_size>
    void write_registers_command(int cs, std::array<uint8_t, 4> cmd_and_pec, const std::array<uint8_t, buffer_size> &data) 
    {
        SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
        // Prompting SPI enable
        _write_and_delay_low(cs, 5);

        _transfer_SPI_data_DMA<4>(cmd_and_pec);

        _transfer_SPI_data_DMA<buffer_size>(data);

        _write_and_delay_high(cs, 5);   
        SPI1.endTransaction();
    }

    template <size_t buffer_size>
    std::array<uint8_t, buffer_size> read_registers_command(int cs, std::array<uint8_t, 4> cmd_and_pec) 
    {
        std::array<uint8_t, buffer_size> read_in;
        
        SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
        // Prompts SPI enable
        _write_and_delay_low(cs, 5);
        _transfer_SPI_data_DMA<4>(cmd_and_pec);

        read_in = _receive_SPI_data_DMA<buffer_size>();
        
        _write_and_delay_high(cs, 5); 
        SPI1.endTransaction();
        return read_in;
    }

    void adc_conversion_command(int cs, std::array<uint8_t, 4> cmd_and_pec, size_t num_stacked_devices) 
    {
        SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
        // Prompting SPI enable
        _write_and_delay_low(cs, 5);
        _transfer_SPI_data_DMA<4>(cmd_and_pec);
        for (size_t i = 0; i < num_stacked_devices; i++) 
        {
            SPI1.transfer(0);
        }
        _write_and_delay_high(cs, 5);
        // End Messager
        SPI1.endTransaction();
    }

}

