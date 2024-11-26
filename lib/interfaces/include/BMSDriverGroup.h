#ifndef __BMSDriverGroup_H__
#define __BMSDriverGroup_H__
/**
 * PREAMBLE:
 * This file is designed to hold all of the functions that will allow communication with the Teensy.
 * There is a specific command format that needs to be sent on SPI for this device to respond.
 * I will do my best to provide background information to all of these commands as I make them.
 * But for more information, please reference the ACU Firmware Migration on Bookstack
 * https://wiki.hytechracing.org/books/ht09-design/page/acu-firmware-migration
 * Link to the LTC6811 data sheet: https://www.analog.com/media/en/technical-documentation/data-sheets/LTC6811-1-6811-2.pdf
 *
 * NOTE: Because the broadcast command will provide 8 bytes of data per command call from EACH IC,
 * Each instance LTC6811 will represent functionality for all ICs on a particular chip select
 * Therefore, the data stored in one instance will only hold HALF. One for chip_select 9 and the other for 10
 * During instantiation, we need to make sure that the addresses are assigned for each LTC6811
 */

/* Library Includes */
/**
 * INCLUDE: LTC6811-1.h and LTC6811-2.h for model-specific data packaging
 * LTC6811-1 can only function through broadcasted commands and therefore used daisy-chained
 * LTC6811-2 can operate under broadcast and address functionality, but will only perform address-based commands
 * Irregardless of which model we choose to use, we need will need access to Arduino and SPI repositories.
 */
#include <Arduino.h>
#include <SPI.h>
#include <string.h>
#include <stdio.h>
#include "Configuration.h"
#include "LTCSPIInterface.h"

class BMSDriverGroup
{
public:
    BMSDriverGroup() = default;
    BMSDriverGroup(int cs, int ic_num) : chip_select(cs),
                                         ic_count(ic_num) {};

    /* -------------------- SETUP FUNCTIONS -------------------- */

    /**
     * INIT: ONLY CALLED ONCE at initialization. LTC6811 only needs to set up the PEC table
     * @post We should have initialized the PEC by calling generate_PEC_table(), so every other time
     * we need to find the PEC it will be much faster.
     */
    void init();

    /**
     * PEC:
     * The Packet Error Code (PEC) is a Error Checker–like CRC for CAN–to make sure that command and data
     * bytes are communicated to the device correctly.
     * This code was taken directly from the data sheet, page 76
     * @post Instantiate the public pec15Table array of size 256. This variable should never be modified afterwards
     */
    void generate_PEC_table();

    /**
     * WAKEUP:
     * The Wakeup protocol will move the device from the SLEEP to STANDBY / REFUP state
     * Reference pages 51-52 on the data sheet for specific information.
     * @pre we don't care; for us, it's good to perform a wakeup to guarantee proper data propogation
     * @post essentially sets connected pin to LOW for period of time, then to HIGH for period of time
     */
    void start_wakeup_protocol();

    /**
     * Returns the delay time depending on what state we are in: SLEEP, IDLE, or MEASURE
     * @return int number of milliseconds
     */
    int get_delay_time();

    /* -------------------- READING DATA FUNCTIONS -------------------- */

    /**
     * Reads voltage data from ALL 4 Cell Voltage Registers on the device.
     * A register, for those that don't know (I didn't), is memory.
     * @pre in order to actually "read" anything, we need to call wakeup() and send data over SPI
     * @post store all 126 cell voltages into the cell_voltages container
     * AND record the max / minimum values and locations of them
     */
    void read_voltages();

    /**
     * Reads GPIO data from ALL 2 Auxillary GPIO Registers on the device.
     * For each BMS segment, there is 6 board thermistors and 2 humidity sensors.
     * NOTE: Conversions are different depending on which we are reading.
     * @pre in order to actually "read" anything, we need to call wakeup() and send data over SPI
     * @post store all temperature data into the board_temperatures container
     * AND record the maximum value and locations
     */
    void read_GPIOs();

    /* -------------------- WRITING DATA FUNCTIONS -------------------- */

    /**
     * Writes the device configuration
     * @pre needs access to undervoltage, overvoltage, configuration MACROS, and discharge data
     * @post sends packaged data over SPI
     */
    void write_configuration(uint8_t dcto_mode);

    /**
     * Writes command to start cell voltage ADC converion
     * @post packaged data transferred over SPI, need to delay before we can read
     */
    void start_cell_voltage_ADC_conversion();

    /**
     * Writes command to start GPIO ADC conversion
     * @post packaged data transfered over SPI, need to delay before we can read
     * This means that we have to call a delay before continuing.
     */
    void start_GPIO_ADC_conversion();

    /* -------------------- PRINT DATA FUNCTIONS -------------------- */

    /**
     * Prints all the cell voltage information
     * Every cell voltage, the minimum, maximum, and locations of those
     */
    void print_voltage_data();

    /**
     * @post resets the max, min data holders to outside bound
     */
    void reset_voltage_data();

    /**
     * Prints all the GPIO information
     * Every temperature, humidity, the maximum temperature, and locations of those
     */
    void print_GPIO_data();

    /**
     * @post resets the max, min data holders to outside bound
     */
    void reset_GPIO_data();

    /* -------------------- SETTER FUNCTIONS -------------------- */

    /**
     * Sets chip select for this IC
     * @param integer input chip select
     */
    void set_chip_select(int ic_index, int cs) { this->chip_select = cs; }

    /**
     * ADDRESSES: Addresses only pertain to LTC6811-2s which are encoded in hardware through 4 bits
     * These addresses need to be coded into the firmware so that address commands are properly sent
     * This function is a setter for the address
     * @param integer input address
     */
    void set_address(int *addr) { this->address = addr; }

    /* -------------------- GETTER FUNCTIONS -------------------- */

    /**
     * Generates a Packet Error Code
     * @pre PEC table is already generated –> pec15Table has data generated in it
     * @post returns an unsigned 16 bit integer used to check data to and from the device
     * @param data data we are writing PEC for, could be Command Code or Buffer data
     * @param length length of data
     * @return unsigned 16 bit PEC, array of uint8_t of length 2
     */
    uint8_t *calculate_specific_PEC(uint8_t *data, int length);

    /**
     * Generates a formmatted 2 byte array for the Command bytes
     * @return unsigned 8 bit array of length 2
     */
    uint8_t *generate_formatted_CMD(CMD_CODES_e command, int ic_index);

    /**
     * Generates Command and PEC as one byte array of length 4: CMD0, CMD1, PEC0, PEC1
     * @return unsigned 8 bit, length 4
     */
    uint8_t *generate_CMD_PEC(CMD_CODES_e command);

    /**
     * @return total thermistor temperature
     *
     */
    float get_total_thermistor_temperature() { return total_thermistor_temps; }

    /**
     * @return total thermistor temperature
     */
    float get_total_board_temperature() { return total_board_temps; }

    /**
     * @return total voltage
     */
    uint16_t get_total_voltage() { return total_voltage; }

    /**
     * @return a usable command address of the specific LTC6811-2 chip to send command to
     */
    uint8_t get_address(int ic_index) { return address[ic_index]; }

    /**
     * @return IC_data container
     */
    IC_Buffers_s get_IC_buffer() { return *this->IC_buffer; }

    /**
     * @return 2 byte broadcast command
     */
    uint8_t *generate_broadcast_command(CMD_CODES_e cmd_code) {
        uint8_t cmd[2] = {((uint8_t)cmd_code) >> 8, (uint8_t)cmd_code};
        return cmd;
    }

    /**
     * @return 2 byte address command
     */
    uint8_t *generate_address_command(CMD_CODES_e cmd_code, int address) {
        uint8_t cmd[2] = {(uint8_t)(get_cmd_address(address) | (uint8_t)cmd_code >> 8), (uint8_t)cmd_code};
        return cmd;
    }

    /**
     * @return usable command address for LTC6811_2
    */
    uint8_t get_cmd_address(int address) { return 0x80 | (address << 3); }

private:
    /**
     * Pointer to the PEC table we will use to calculate new PEC tables
     */
    uint16_t pec15Table[256];

    /**
     * We will need this for both models of the IC
     * This determines where we get our signals from on the Arduino
     * It can only be 9 or 10
     * NOTE: needs to be initialized
     */
    int chip_select;

    /**
     * Stores number of ICs on the chip select line: 6
     * NOTE: needs to be initialized
     */
    int ic_count;

    /**
     * Stores all 63 cells of data in a 2D array.
     * There are 6 boards with 2 ICs each with a total of 12 ICs on ACU
     * BUT since each instance of LTC6811 will only have half, then we will hold 6 sets of data points
     * NOTE: THIS IS THE PROCESSED DATA
     */
    IC_Voltage_Temp_Data_s IC_data[total_ic / 2];

    /**
     * Stores Register Group Bytes, 6 bytes each
     * There are many other register groups like PWM, S_Control, and COMM bytes,
     * But we don't need that functionality for our purposes
     * NOTE: THIS IS THE RAW DATA
     */
    IC_Buffers_s IC_buffer[total_ic / 2];

    /**
     * Stores where the max voltage location is
     */
    IC_Cell_Location_s max_voltage_location;

    /**
     * Stores where the max voltage location is
     */
    IC_Cell_Location_s min_voltage_location;

    /**
     * Stores locations of max min GPIO data
     */
    IC_Cell_Location_s max_board_temp_location;
    IC_Cell_Location_s min_board_temp_location;
    IC_Cell_Location_s max_thermistor_location;
    IC_Cell_Location_s max_humidity_location;
    IC_Cell_Location_s min_thermistor_location;

    /**
     * We will only end up using the address if this is a LTC6811-2
     * NOTE: But if we are, we need to call a setup function to instatiate each with the correct addresses
     * BMS Segments 1, 4, and 5 are on chip select 9
     * BMS Segments 2, 3, and 6 are on chip select 10
     * Those segments correspond to 2 ICs each, so the instance with chip_select 9
     * Will have IC addresses: 0,1,6,7,8,9 | The rest are for chip_select 10
     */
    int *address;

    /**
     * Stores whether an the AMS system is balancing, true or false.
     * Depends on if we are charging / discharging or in a 0 fault state
     */
    bool currently_balancing;

    /**
     * Stores the total voltage of all 63 cells
     */
    uint16_t total_voltage;

    /**
     * Stores the max voltage of all 63 cells
     */
    uint16_t max_voltage;

    /**
     * Stores the min voltage of all 63 cells
     */
    uint16_t min_voltage;

    /**
     * Stores max, min data for gpio data
     */
    uint16_t max_humidity = 0;
    uint16_t max_thermistor_voltage = 0;
    uint16_t min_thermistor_voltage = 65535;
    uint16_t max_board_temp_voltage = 0;
    uint16_t min_board_temp_voltage = 65535;

    /**
     * Stores the total temperature of the 3 boards
     */
    float total_board_temps;

    /**
     * Stores the total temperature of the thermistors on 3 boards
     */
    float total_thermistor_temps;
};

#endif