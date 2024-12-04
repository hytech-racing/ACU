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
// #include <string.h>
#include <stdio.h>
#include "Configuration.h"
#include "LTCSPIInterface.h"
#include <cstdint>
#include <optional>

template <size_t num_chips, size_t num_chip_selects>
class BMSDriverGroup
{
public:
    using volt = float;
    using celcius = float; 
    struct BMSData
    {
        std::array<std::array<std::optional<volt>, 12>, num_chips> voltages;
        std::array<celcius, 4 * num_chips> cell_temperatures;
        std::array<float, (num_chips + 1) / 2> humidity;
        std::array<float, (num_chips + 1) / 2> board_temperatures;
        float min_voltage;
        float max_voltage;
        size_t min_voltage_cell_id; // 0 - 125
        size_t max_voltage_cell_id; // 0 - 125
        size_t max_board_temperature_segment_id; // 0 - 5
        size_t max_humidity_segment_id; // 0 - 5
        size_t max_cell_temperature_cell_id; // 0 - 47
        float total_voltage;
        float average_cell_temperature;
    };

    BMSDriverGroup() {};

public:
    /* -------------------- SETUP FUNCTIONS -------------------- */

    /**
     * INIT: ONLY CALLED ONCE at initialization. LTC6811 only needs to set up the PEC table
     * @post We should have initialized the PEC by calling generate_PEC_table(), so every other time
     * we need to find the PEC it will be much faster.
     */
    void init();

    /* -------------------- READING DATA FUNCTIONS -------------------- */

    /**
     * Reads Voltage and GPIO data from ALL 12 Cell Voltages and 2 Auxillary GPIO Registers on the device.
     * For each BMS segment, there is 6 board thermistors and 2 humidity sensors.
     * NOTE: Conversions are different depending on which we are reading.
     * @pre in order to actually "read" anything, we need to call wakeup() and send data over SPI
     * @post store all temperature data into the board_temperatures container
     * AND record the maximum value and locations
     */
    // void read_thermistor_and_humidity();
    BMSData read_data(const std::array<std::array<bool, 12>, num_chips> &cell_balance_statuses);

    /* -------------------- WRITING DATA FUNCTIONS -------------------- */

    /**
     * Writes the device configuration
     * @pre needs access to undervoltage, overvoltage, configuration MACROS, and discharge data
     * @post sends packaged data over SPI
     */
    void _write_configuration(uint8_t dcto_mode, const std::array<std::array<bool, 12>, num_chips> &cell_balance_statuses);

private:
    /**
     * PEC:
     * The Packet Error Code (PEC) is a Error Checker–like CRC for CAN–to make sure that command and data
     * bytes are communicated to the device correctly.
     * This code was taken directly from the data sheet, page 76
     * @post Instantiate the public pec15Table array of size 256. This variable should never be modified afterwards
     */
    void _generate_PEC_table();
    /**
     * WAKEUP:
     * The Wakeup protocol will move the device from the SLEEP to STANDBY / REFUP state
     * Reference pages 51-52 on the data sheet for specific information.
     * @pre we don't care; for us, it's good to perform a wakeup to guarantee proper data propogation
     * @post essentially sets connected pin to LOW for period of time, then to HIGH for period of time
     */
    void _start_wakeup_protocol();


    BMSData _read_data_through_broadcast(const std::array<bool, 12> &cell_balance_statuses);

    BMSData _read_data_through_address(const std::array<bool, 12> &cell_balance_statuses);

    /**
     * @post resets the max, min data holders to outside bound
     */
    void _reset_voltage_data();

    /**
     * @post resets the max, min data holders to outside bound
     */
    void _reset_GPIO_data();

    void _write_config_through_broadcast(uint8_t dcto_mode, std::array<uint8_t, 6> buffer_format, const std::array<std::array<bool, 12>, num_chips> &cell_balance_statuses);

    void _write_config_through_address(uint8_t dcto_mode, std::array<uint8_t, 6> buffer_format, const std::array<std::array<bool, 12>, num_chips> &cell_balance_statuses);

    /**
     * Writes command to start cell voltage ADC converion
     * @post packaged data transferred over SPI, need to delay before we can read
     */
    void _start_cell_voltage_ADC_conversion();

    /**
     * Writes command to start GPIO ADC conversion
     * @post packaged data transfered over SPI, need to delay before we can read
     * This means that we have to call a delay before continuing.
     */
    void _start_GPIO_ADC_conversion();

    void _start_ADC_conversion_through_broadcast(std::array<uint8_t, 2> cmd_code);

    void _start_ADC_conversion_through_address(std::array<uint8_t, 2> cmd_code);

    /* -------------------- SETTER FUNCTIONS -------------------- */

    /* -------------------- GETTER FUNCTIONS -------------------- */

    /**
     * Generates a Packet Error Code
     * @pre PEC table is already generated –> pec15Table has data generated in it
     * @post returns an unsigned 16 bit integer used to check data to and from the device
     * @param data data we are writing PEC for, could be Command Code or Buffer data
     * @param length length of data
     * @return unsigned 16 bit PEC, array of uint8_t of length 2
     */
    std::array<uint8_t, 2> _calculate_specific_PEC(uint8_t *data, int length);

    /**
     * Generates a formmatted 2 byte array for the Command bytes
     * @return unsigned 8 bit array of length 2
     */
    std::array<uint8_t, 2> _generate_formatted_CMD(CMD_CODES_e command, int ic_index);

    /**
     * Generates Command and PEC as one byte array of length 4: CMD0, CMD1, PEC0, PEC1
     * @return unsigned 8 bit, length 4
     */
    std::array<uint8_t, 4> _generate_CMD_PEC(CMD_CODES_e command, int ic_index);

    /**
     * @return usable command address for LTC6811_2
     */
    uint8_t _get_cmd_address(int address) { return 0x80 | (address << 3); }

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
    std::array<int, num_chip_selects> chip_select;

    /**
     * We will need this for both models of the IC
     * This determines where we get our signals from on the Arduino
     * It can only be 9 or 10
     * NOTE: needs to be initialized
     */
    std::array<int, num_chips> chip_select_per_chip;

    /**
     * Stores number of ICs on the chip select line: 6
     * NOTE: needs to be initialized
     */
    int ic_count;

    /**
     * We will only end up using the address if this is a LTC6811-2
     * NOTE: But if we are, we need to call a setup function to instatiate each with the correct addresses
     * BMS Segments 1, 4, and 5 are on chip select 9
     * BMS Segments 2, 3, and 6 are on chip select 10
     * Those segments correspond to 2 ICs each, so the instance with chip_select 9
     * Will have IC addresses: 0,1,6,7,8,9 | The rest are for chip_select 10
     */
    std::array<int, num_chips> address;

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
    // uint16_t min_thermistor_voltage = 65535;
    uint16_t max_board_temp_voltage = 0;
    uint16_t min_board_temp_voltage = 65535;

    /**
     * Stores the total temperature of the 3 boards
     */
    //float total_board_temps;

    /**
     * Stores the total temperature of the thermistors on 3 boards
     */
    float total_thermistor_temps;
};

#include <BMSDriverGroup.tpp>

#endif