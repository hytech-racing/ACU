/* Interface Includes */
#include "LTCSPIInterface.h"
#include "Configuration.h"
#include <Arduino.h>


void send_SPI_write_registers_command(int cs, uint8_t* cmd_and_pec, uint8_t* buffer_and_pec, int ic_count) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    // Prompting SPI enable
    write_and_delay_LOW(cs, 2);
    transfer_SPI_data(cmd_and_pec, 4);
    transfer_SPI_data(buffer_and_pec, 8);
    write_and_delay_HIGH(cs, 2);   
    SPI.endTransaction();
}

void send_SPI_read_registers_command(int cs, uint8_t* cmd_and_pec, int ic_count, uint8_t* received) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    // Prompts SPI enable
    write_and_delay_LOW(cs, 2);
    transfer_SPI_data(cmd_and_pec, 4);
    for (int i = 0; i < ic_count; i++) {
        receive_SPI_data(8, received);
    }
    write_and_delay_HIGH(cs, 2); 
    SPI.endTransaction();
}

void send_SPI_non_register_command(int cs, uint8_t* cmd_and_pec, int ic_count) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    // Prompting SPI enable
    write_and_delay_LOW(cs, 2);
    transfer_SPI_data(cmd_and_pec, 4);
    write_and_delay_HIGH(cs, 2);
    // End Message
    SPI.endTransaction();
}

void transfer_SPI_data(uint8_t *data, int length) {
    for (int i = 0; i < length; i++) {
        SPI.transfer(data[i]);
    }
}

void receive_SPI_data(int data_length, uint8_t* received) {
    for (int i = 0; i < data_length; i++) {
        received[i] = SPI.transfer(0); // transfer dummy value over SPI in order to read bytes into data
    }
}

void write_and_delay_LOW(int cs, int delay_microSeconds) {
    digitalWrite(cs, LOW);
    delayMicroseconds(delay_microSeconds);
}

void write_and_delay_HIGH(int cs, int delay_microSeconds) {
    digitalWrite(cs, HIGH);
    delayMicroseconds(delay_microSeconds);
}

