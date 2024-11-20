/* Interface Includes */
#include "LTCSPIInterface.h"
#include "DataContainer.h"
#include <Arduino.h>


void send_SPI_write_registers_command(int cs, uint8_t* cmd_and_pec, uint8_t* buffer_and_pec) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    uint8_t ic_buffer_pec[8];
#ifdef USING_LTC6811_1
    // Prompting SPI enable
    write_and_delay_LOW(cs, 1);
    transfer_SPI_data(cmd_and_pec, 4);
    for (int i = 0; i < TOTAL_IC / 2; i++) { 
        // If it's the _2, we only need to send it once, but the _1 requires the data to be cascaded for each IC on the chip_select line
        std::copy(buffer_and_pec + (i * 8), buffer_and_pec + (i * 8) + 8, ic_buffer_pec);
        transfer_SPI_data(ic_buffer_pec, 8);
    }  
    write_and_delay_HIGH(cs, 1);
#else
    uint8_t ic_cmd_pec[4];
    for (int i = 0; i < TOTAL_IC / 2; i++) { // should loop 6 times for 6 ICs
        write_and_delay_LOW(cs, 1);
        std::copy(cmd_and_pec + (i * 4), cmd_and_pec + (i * 4) + 4, ic_cmd_pec); 
        std::copy(buffer_and_pec + (i * 8), buffer_and_pec + (i * 8) + 8, ic_buffer_pec);   
        transfer_SPI_data(ic_cmd_pec, 4);
        transfer_SPI_data(ic_buffer_pec, 8);
        write_and_delay_HIGH(cs, 1);
    }  
#endif     
    
    SPI.endTransaction();
}

uint8_t* send_SPI_read_registers_command(int cs, uint8_t* cmd_and_pec) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    uint8_t data_in[48];
#ifdef USING_LTC6811_1
    // Prompts SPI enable
    write_and_delay_LOW(cs, 1);
    transfer_SPI_data(cmd_and_pec, 4);
    uint8_t* ic_data;
    for (int i = 0; i < TOTAL_IC / 2; i++) {
        data_in = receive_SPI_data(data_in_count);
    }
    write_and_delay_HIGH(cs, 1);
#else
    uint8_t ic_cmd_pec[4];
    uint8_t* ic_data;
    for (int i = 0; i < TOTAL_IC / 2; i++) {
        // Prompts SPI enable
        write_and_delay_LOW(cs, 1);
        std::copy(cmd_and_pec + (i * 4), cmd_and_pec + (i * 4) + 4, ic_cmd_pec);
        transfer_SPI_data(ic_cmd_pec, 4);
        ic_data = receive_SPI_data(data_in_count);
        std::copy(ic_data, ic_data + 8, data_in + (i * 8));
        write_and_delay_HIGH(cs, 1);
    }
#endif   
    SPI.endTransaction();
    return data_in;
}

void send_SPI_non_register_command(int cs, uint8_t* cmd_and_pec) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
#ifdef USING_LTC6811_1
    // Prompting SPI enable
    write_and_delay_LOW(cs, 1);
    transfer_SPI_data(cmd_and_pec);
    write_and_delay_HIGH(cs, 1);
#else
    uint8_t ic_cmd_pec[4];
    for (int i = 0; i < TOTAL_IC / 2; i++) { 
        write_and_delay_LOW(cs, 1);
        std::copy(cmd_and_pec + (i * 4), cmd_and_pec + (i * 4) + 4, ic_cmd_pec); 
        transfer_SPI_data(ic_cmd_pec, 4);
        write_and_delay_HIGH(cs, 1);
    }  
#endif  
    // End Message
    SPI.endTransaction();
}

void transfer_SPI_data(uint8_t *data, int length) {
    for (int i = 0; i < length; i++) {
        SPI.transfer(data[i]);
    }
}

uint8_t* receive_SPI_data() {
    uint8_t data_in[data_in_count];
    for (int i = 0; i < data_in_count; i++) {
        data_in[i] = SPI.transfer(0); // transfer dummy value over SPI in order to read bytes into data
    }
    return data_in;
}

void write_and_delay_LOW(int cs, int delay_microSeconds) {
    digitalWrite(cs, LOW);
    delayMicroseconds(delay_microSeconds);
}

void write_and_delay_HIGH(int cs, int delay_microSeconds) {
    digitalWrite(cs, HIGH);
    delayMicroseconds(delay_microSeconds);
}

