/* Interface Includes */
#include "LTCSPIInterface.h"
#include "Configuration.h"
#include <Arduino.h>

template <size_t data_size>
void _transfer_SPI_data(const std::array<uint8_t, data_size> &data) {
    for (size_t i = 0; i < data_size; i++) {
        SPI.transfer(data[i]);
    }
}

template <size_t data_size>
std::array<uint8_t, data_size> _receive_SPI_data() {
    std::array<uint8_t, data_size> data_in;
    for (size_t i = 0; i < data_size; i++) {

        data_in[i] = SPI.transfer(0);
    }
   
    return data_in;
}

void ltc_spi_interface::_write_and_delay_low(int cs, int delay_microSeconds) {
    digitalWrite(cs, LOW);
    delayMicroseconds(delay_microSeconds);
}

void ltc_spi_interface::_write_and_delay_high(int cs, int delay_microSeconds) {
    digitalWrite(cs, HIGH);
    delayMicroseconds(delay_microSeconds);
}

template <size_t buffer_size>
void ltc_spi_interface::write_registers_command(int cs, std::array<uint8_t, 4> cmd_and_pec, const std::array<uint8_t, buffer_size> &data) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    // Prompting SPI enable
    _write_and_delay_low(cs, 5);

    _transfer_SPI_data<4>(cmd_and_pec);

    _transfer_SPI_data<buffer_size>(data);

    _write_and_delay_high(cs, 5);   
    SPI.endTransaction();
}

template <size_t buffer_size>
std::array<uint8_t, buffer_size> ltc_spi_interface::read_registers_command(int cs, std::array<uint8_t, 4> cmd_and_pec) {
    std::array<uint8_t, buffer_size> read_in;
    
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    // Prompts SPI enable
    _write_and_delay_low(cs, 5);
    _transfer_SPI_data<4>(cmd_and_pec);

    read_in = _receive_SPI_data<buffer_size>();
    
    _write_and_delay_high(cs, 5); 
    SPI.endTransaction();
    return read_in;
}

void ltc_spi_interface::adc_conversion_command(int cs, std::array<uint8_t, 4> cmd_and_pec, size_t num_stacked_devices) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    // Prompting SPI enable
    _write_and_delay_low(cs, 5);
    _transfer_SPI_data<4>(cmd_and_pec);
    for (size_t i = 0; i < num_stacked_devices; i++) {
        SPI.transfer(0);
    }
    _write_and_delay_high(cs, 5);
    // End Messager
    SPI.endTransaction();
}
