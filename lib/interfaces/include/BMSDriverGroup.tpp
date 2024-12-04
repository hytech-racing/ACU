/* Library Includes */
#include "Configuration.h"
#include "BMSDriverGroup.h"
#include "LTCSPIInterface.h"
#include <string.h>
#include <stdio.h>
#include <string>
#include <optional>

/* -------------------- SETUP FUNCTIONS -------------------- */

template <size_t num_chips, size_t num_chip_selects>
void BMSDriverGroup<num_chips, num_chip_selects>::init()
{
    // We only call this once during setup to instantiate the variable pec15Table, a pointer to an array
    _generate_PEC_table();
    int cs = 9;
    for (size_t i = 0; i < num_chip_selects; i++) {
        chip_select[i] = cs;
        //chip select defines
        pinMode(cs, OUTPUT); 
        digitalWrite(cs, HIGH);  
        cs++;
    }
    for (size_t j = 0; j < num_chips; j++)
    {
        switch (j)
        {
        case 2:
        case 3:
        case 4:
        case 5:
        case 10:
        case 11:
            chip_select_per_chip[j] = 10;
            break;

        case 0:
        case 1:
        case 6:
        case 7:
        case 8:
        case 9:
            chip_select_per_chip[j] = 10;
            break;
        }
    }
}

// this implementation is straight from: https://www.analog.com/media/en/technical-documentation/data-sheets/LTC6811-1-6811-2.pdf
// on page <76>, section: Applications Information
template <size_t num_chips, size_t num_chip_selects>
void BMSDriverGroup<num_chips, num_chip_selects>::_generate_PEC_table()
{
    for (int i = 0; i < 256; i++)
    {
        uint16_t remainder = i << 7;
        for (int bit = 8; bit > 0; --bit)
        {
            if (remainder & 0x4000)
            {
                remainder = ((remainder << 1));
                remainder = (remainder ^ CRC15_POLY);
            }
            else
            {
                remainder = ((remainder << 1));
            }
        }
        pec15Table[i] = remainder & 0xFFFF;
    }
}

template <size_t num_chips, size_t num_chip_selects>
void BMSDriverGroup<num_chips, num_chip_selects>::_start_wakeup_protocol()
{   
    for (int cs = 0; cs < num_chip_selects; cs++) {
        _write_and_delay_LOW(chip_select[cs], 1);
        SPI.transfer(0);
        _write_and_delay_HIGH(chip_select[cs], 400); // t_wake is 400 microseconds; wait that long to ensure device has turned on.
    }
}

/* -------------------- READING DATA FUNCTIONS -------------------- */

template <size_t num_chips, size_t num_chip_selects>
typename BMSDriverGroup<num_chips, num_chip_selects>::BMSData
BMSDriverGroup<num_chips, num_chip_selects>::read_data(const std::array<std::array<bool, 12>, num_chips> &cell_balance_statuses)
{
    _reset_voltage_data();
    _reset_GPIO_data();
#ifdef USING_LTC6811_1
    return _read_data_through_broadcast(cell_balance_statuses);
#else
    return _read_data_through_address(cell_balance_statuses);
#endif
}

template <size_t num_chips, size_t num_chip_selects>
typename BMSDriverGroup<num_chips, num_chip_selects>::BMSData
BMSDriverGroup<num_chips, num_chip_selects>::_read_data_through_broadcast(const std::array<bool, 12> &cell_balance_statuses) {
    BMSData data_in;
    _start_wakeup_protocol(); // wakes all of the ICs on the chip select line
    _write_configuration(dcto_read, cell_balance_statuses);
    _start_cell_voltage_ADC_conversion(); // Gets the ICs ready to be read, must delay afterwards by ? us
    std::array<uint8_t, 8 * num_chips> data_in_cell_voltages_1_to_3;
    std::array<uint8_t, 8 * num_chips> data_in_cell_voltages_4_to_6;
    std::array<uint8_t, 8 * num_chips> data_in_cell_voltages_7_to_9;
    std::array<uint8_t, 8 * num_chips> data_in_cell_voltages_10_to_12;

    return data_in;
}

template <size_t num_chips, size_t num_chip_selects>
typename BMSDriverGroup<num_chips, num_chip_selects>::BMSData
BMSDriverGroup<num_chips, num_chip_selects>::_read_data_through_address(const std::array<bool, 12> &cell_balance_statuses) {
    BMSData bms_data;
    _start_wakeup_protocol(); // wakes all of the ICs on the chip select line
    _write_configuration(dcto_read, cell_balance_statuses);
    _start_cell_voltage_ADC_conversion(); // Gets the ICs ready to be read, must delay afterwards by ? us
    // _start_wakeup_protocol(); // wakes all of the ICs on the chip select line
    // _write_configuration(dcto_read, cell_balance_statuses);
    _start_GPIO_ADC_conversion();
    std::array<uint8_t, 8> data_in_cell_voltages_1_to_3;
    std::array<uint8_t, 8> data_in_cell_voltages_4_to_6;
    std::array<uint8_t, 8> data_in_cell_voltages_7_to_9;
    std::array<uint8_t, 8> data_in_cell_voltages_10_to_12;
    std::array<uint8_t, 8> data_in_auxillaries_1_to_3;
    std::array<uint8_t, 8> data_in_auxillaries_4_to_6;
    std::array<uint8_t, 4> cmd_pec;
    std::array<uint8_t, 8> buffer_in;
    int count = 0;
    for (int chip = 0; chip < num_chips; chip++) {
        _start_wakeup_protocol();
        cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_A, chip);
        data_in_cell_voltages_1_to_3 = read_registers_command<8>(chip_select_per_chip[chip], cmd_pec);
        cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_B, chip);
        data_in_cell_voltages_4_to_6 = read_registers_command<8>(chip_select_per_chip[chip], cmd_pec);
        cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_C, chip);
        data_in_cell_voltages_7_to_9 = read_registers_command<8>(chip_select_per_chip[chip], cmd_pec);
        cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_D, chip);
        data_in_cell_voltages_10_to_12 = read_registers_command<8>(chip_select_per_chip[chip], cmd_pec);
        cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_GPIO_VOLTAGE_GROUP_A, chip);
        data_in_auxillaries_1_to_3 = read_registers_command<8>(chip_select_per_chip[chip], cmd_pec);
        cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_GPIO_VOLTAGE_GROUP_A, chip);
        data_in_auxillaries_4_to_6 = read_registers_command<8>(chip_select_per_chip[chip], cmd_pec);

        // DEBUG: Check to see that the PEC is what we expect it to be

        int cell_count = (chip % 2 == 0) ? 12 : 9; // Even indexed ICs have 12 cells, odd have 9
        std::array<uint16_t, 12> chip_voltages_in;
        for (int cell_Index = 0; cell_Index < cell_count; cell_Index++)
        {
            std::array<uint8_t, 2> data_in_cell_voltage;
            int sub_index = cell_Index / 3;
            switch (sub_index) // There are 4 groups of CELL VOLTAGES, each with 3 cells
            {
            case 0:
                std::copy(data_in_cell_voltages_1_to_3.data() + ((cell_Index % 3) * 2), data_in_cell_voltages_1_to_3.data() + 2 + ((cell_Index % 3) * 2), data_in_cell_voltage.data());
                break;
            case 1:
                std::copy(data_in_cell_voltages_4_to_6.data() + ((cell_Index % 3) * 2), data_in_cell_voltages_4_to_6.data() + 2 + ((cell_Index % 3) * 2), data_in_cell_voltage.data());
                break;
            case 2:
                std::copy(data_in_cell_voltages_7_to_9.data() + ((cell_Index % 3) * 2), data_in_cell_voltages_7_to_9.data() + 2 + ((cell_Index % 3) * 2), data_in_cell_voltage.data());
                break;
            case 3:
                std::copy(data_in_cell_voltages_10_to_12.data() + ((cell_Index % 3) * 2), data_in_cell_voltages_10_to_12.data() + 2 + ((cell_Index % 3) * 2), data_in_cell_voltage.data());
                break; // We will never get here for odd indexed ICs
            }
            
            uint16_t voltage_in = (uint16_t) data_in_cell_voltage.data();
            chip_voltages_in[cell_Index] = voltage_in;
            total_voltage += voltage_in;
            if (voltage_in < min_voltage)
            {
                min_voltage = voltage_in;
                bms_data.min_voltage_cell_id = count;
            }
            if (voltage_in > max_voltage)
            {
                max_voltage = voltage_in;
                bms_data.max_voltage_cell_id = count;
            }
            count++;
            bms_data.voltages[chip_voltages_in];
        }

        // HUMIDITY AND TEMPERATURES

        count = 0; // reset count var
        for (int gpio_Index = 0; gpio_Index < 6; gpio_Index++)
        {
            std::array<uint8_t, 2> data_in_gpio_voltage;
            int sub_index = gpio_Index / 3;
            switch (sub_index) // There are 2 groups of AUXILLARIES, each with 3 2-byte gpio data
            {
            case 0:
                std::copy(data_in_auxillaries_1_to_3.data() + ((gpio_Index % 3) * 2), data_in_auxillaries_1_to_3.data() + 2 + ((gpio_Index % 3) * 2), data_in_gpio_voltage.data());
                break;
            case 1:
                std::copy(data_in_auxillaries_4_to_6.data() + ((gpio_Index % 3) * 2), data_in_auxillaries_4_to_6.data() + 2 + ((gpio_Index % 3) * 2), data_in_gpio_voltage.data());
                break;
            }
            
            uint16_t gpio_in = (uint16_t) data_in_gpio_voltage.data();

            if ((chip % 2) && gpio_Index == 4)
            {   
                bms_data.board_temperatures[(chip + 2) / 2] = -66.875 + 218.75 * (gpio_in / 50000.0); // caculation for SHT31 temperature in C
                if (gpio_in > max_board_temp_voltage)
                {
                    max_board_temp_voltage = gpio_in;
                    bms_data.max_board_temperature_segment_id = (chip + 2) / 2; // Because each segment only has 1 humidity and 1 board temp sensor
                }
            }
            else if (gpio_Index == 4)
            {
                bms_data.humidity[(chip + 2) / 2] = -12.5 + 125 * (gpio_in) / 50000.0; // humidity calculation
                if (gpio_in > max_humidity)
                {
                    max_humidity = gpio_in;
                    bms_data.max_board_temperature_segment_id = (chip + 2) / 2;
                }
            }
            else if (gpio_Index < 4)
            {
                float thermistor_resistance = (2740 / (gpio_in / 50000.0)) - 2740;
                bms_data.cell_temperatures[count] = 1 / ((1 / 298.15) + (1 / 3984.0) * log(thermistor_resistance / 10000.0)) - 273.15; // calculation for thermistor temperature in C
                total_thermistor_temps += gpio_in;
                if (gpio_in > max_thermistor_voltage)
                {
                    max_thermistor_voltage = gpio_in;
                    bms_data.max_cell_temperature_cell_id = count;
                }
            }
            count++;
        }
    }

    bms_data.min_voltage = this->min_voltage;
    bms_data.max_voltage = this->max_voltage;
    bms_data.total_voltage = this->total_voltage;
    bms_data.average_cell_temperature = this->total_thermistor_temps / count;
    return bms_data;
}

template <size_t num_chips, size_t num_chip_selects>
void BMSDriverGroup<num_chips, num_chip_selects>::_reset_voltage_data()
{
    total_voltage = 0;
    max_voltage = 0;
    min_voltage = 65535;
}

template <size_t num_chips, size_t num_chip_selects>
void BMSDriverGroup<num_chips, num_chip_selects>::_reset_GPIO_data()
{
    max_humidity = 0;
    max_thermistor_voltage = 0;
    max_board_temp_voltage = 0;
    min_board_temp_voltage = 65535;
    total_thermistor_temps = 0;
}

/* -------------------- WRITING DATA FUNCTIONS -------------------- */

template <size_t num_chips, size_t num_chip_selects>
void BMSDriverGroup<num_chips, num_chip_selects>::_write_configuration(uint8_t dcto_mode, const std::array<std::array<bool, 12>, num_chips> &cell_balance_statuses)
{   
    std::array<uint8_t, 6> buffer_format; // This buffer processing can be seen in more detail on page 62 of the data sheet
    buffer_format[0] = (gpios_enabled << 3) | (static_cast<int>(device_refup_mode) << 2) | static_cast<int>(adcopt);
    buffer_format[1] = (under_voltage_threshold & 0x0FF);
    buffer_format[2] = ((over_voltage_threshold & 0x00F) << 4) | ((under_voltage_threshold & 0xF00) >> 8);
    buffer_format[3] = ((over_voltage_threshold & 0xFF0) >> 4);
    _start_wakeup_protocol();
#ifdef USING_LTC6811_1
    _write_config_through_broadcast(dcto_mode, buffer_format, cell_balance_statuses);
#else
    _write_config_through_address(dcto_mode, buffer_format, cell_balance_statuses);
#endif
}

template <size_t num_chips, size_t num_chip_selects>
void BMSDriverGroup<num_chips, num_chip_selects>::_write_config_through_broadcast(uint8_t dcto_mode, std::array<uint8_t, 6> buffer_format, const std::array<std::array<bool, 12>, num_chips> &cell_balance_statuses) {
    std::array<uint8_t, 4> cmd_and_pec;
    std::array<uint8_t, (1 + num_chips) / 2 * 8> full_buffer;
    std::array<uint8_t, 2> temp_pec;
    
    // Needs to be sent on each chip select line
    for (int cs = 0; cs < num_chip_selects; cs++) {
        cmd_and_pec = _generate_CMD_PEC(CMD_CODES_e::WRITE_CONFIG, -1);
        int j = 0;
        for (int i = 0; i < num_chips; i++) { // Find chips with the same CS
            if (chip_select_per_chip[i] == chip_select[cs] && j < (num_chips + 1) / 2) {
                buffer_format[4] = ((reinterpret_cast<uint16_t>(cell_balance_statuses[i]) & 0x0FF));
                buffer_format[5] = ((dcto_mode & 0x0F) << 4) | ((reinterpret_cast<uint16_t>(cell_balance_statuses[i]) & 0xF00) >> 8);
                temp_pec = _calculate_specific_PEC(buffer_format, 6);
                std::copy(buffer_format.data(), buffer_format.data() + 6, full_buffer.data() + (j * 8));
                std::copy(temp_pec.data(), temp_pec.data() + 2, full_buffer.data() + 6 + (j * 8));
            }
        }
        write_registers_command<(sizeof(full_buffer))>(chip_select[cs], cmd_and_pec, full_buffer); 
    }
}

template <size_t num_chips, size_t num_chip_selects>
void BMSDriverGroup<num_chips, num_chip_selects>::_write_config_through_address(uint8_t dcto_mode, std::array<uint8_t, 6> buffer_format, const std::array<std::array<bool, 12>, num_chips> &cell_balance_statuses) {
    // Need to manipulate the command code to have address, therefore have to send command num_chips times
    std::array<uint8_t, 4> cmd_and_pec;
    std::array<uint8_t, 8> full_buffer;
    std::array<uint8_t, 2> temp_pec;
    for (int i = 0; i < num_chips; i++) {
        cmd_and_pec = _generate_CMD_PEC(CMD_CODES_e::WRITE_CONFIG, i);
        buffer_format[4] = ((reinterpret_cast<uint16_t>(cell_balance_statuses[i]) & 0x0FF));
        buffer_format[5] = ((dcto_mode & 0x0F) << 4) | ((reinterpret_cast<uint16_t>(cell_balance_statuses[i]) & 0xF00) >> 8);
        temp_pec = _calculate_specific_PEC(buffer_format, 6);
        std::copy(buffer_format.data(), buffer_format.data() + 6, full_buffer.data());
        std::copy(temp_pec.data(), temp_pec.data() + 2, full_buffer.data() + 6);
        write_registers_command<8>(chip_select_per_chip[i], cmd_and_pec, full_buffer);
    }
}

template <size_t num_chips, size_t num_chip_selects>
void BMSDriverGroup<num_chips, num_chip_selects>::_start_cell_voltage_ADC_conversion()
{
    uint16_t adc_cmd = (uint16_t) CMD_CODES_e::START_CV_ADC_CONVERSION | (adc_mode_cv_conversion << 7) | (discharge_permitted << 4) | static_cast<uint8_t>(adc_conversion_cell_select_mode);
    std::array<uint8_t, 2> cmd = {(adc_cmd >> 8) & 0xFF, adc_cmd & 0xFF};
#ifdef USING_LTC6811_1
    _start_ADC_conversion_through_broadcast(cmd);
#else
    _start_ADC_conversion_through_address(cmd);
#endif
    delay(cv_adc_conversion_time_us); // us
}

template <size_t num_chips, size_t num_chip_selects>
void BMSDriverGroup<num_chips, num_chip_selects>::_start_GPIO_ADC_conversion()
{
    uint16_t adc_cmd = (uint16_t) CMD_CODES_e::START_GPIO_ADC_CONVERSION | (adc_mode_gpio_conversion << 7) | static_cast<uint8_t>(adc_conversion_gpio_select_mode);
    std::array<uint8_t, 2> cmd = {(adc_cmd >> 8) & 0xFF, adc_cmd & 0xFF};
#ifdef USING_LTC6811_1
    _start_ADC_conversion_through_broadcast(cmd);
#else
    _start_ADC_conversion_through_address(cmd);
#endif
    delay(gpio_adc_conversion_time_us); // us
}

template <size_t num_chips, size_t num_chip_selects>
void BMSDriverGroup<num_chips, num_chip_selects>::_start_ADC_conversion_through_broadcast(std::array<uint8_t, 2> cmd_code) {
    // Leave the command code as is
    std::array<uint8_t, 2> pec = _calculate_specific_PEC(cmd_code, 2);
    std::array<uint8_t, 4> cmd_and_pec;
    std::copy(cmd_code.data(), cmd_code.data() + 2, cmd_and_pec.data());     // Copy first two bytes (cmd)
    std::copy(pec.data(), pec.data() + 2, cmd_and_pec.data() + 2); // Copy next two bytes (pec)
    
    // Needs to be sent on each chip select line
    for (int cs = 0; cs < num_chip_selects; cs++) {
        adc_conversion_command<4>(chip_select[cs], cmd_and_pec); 
    }
}

template <size_t num_chips, size_t num_chip_selects>
void BMSDriverGroup<num_chips, num_chip_selects>::_start_ADC_conversion_through_address(std::array<uint8_t, 2> cmd_code) {
    // Need to manipulate the command code to have address, therefore have to send command num_chips times
    for (int i = 0; i < num_chips; i++) {
        cmd_code[0] = _get_cmd_address(i) | cmd_code[0]; // Make sure address is embedded in each cmd_code send
        std::array<uint8_t, 2> pec = _calculate_specific_PEC(cmd_code, 2);
        std::array<uint8_t, 4> cmd_and_pec;
        std::copy(cmd_code.data(), cmd_code.data() + 2, cmd_and_pec.data());     // Copy first two bytes (cmd)
        std::copy(pec.data(), pec.data() + 2, cmd_and_pec.data() + 2); // Copy next two bytes (pec)
        adc_conversion_command<4>(chip_select_per_chip[i], cmd_and_pec);
    }
}

/* -------------------- GETTER FUNCTIONS -------------------- */

// This implementation is taken directly from the data sheet linked here: https://www.analog.com/media/en/technical-documentation/data-sheets/LTC6811-1-6811-2.pdf
template <size_t num_chips, size_t num_chip_selects>
std::array<uint8_t, 2> BMSDriverGroup<num_chips, num_chip_selects>::_calculate_specific_PEC(uint8_t *data, int length)
{
    std::array<uint8_t, 2> pec;
    uint16_t remainder;
    uint16_t addr;
    remainder = 0x10; // PEC seed
    for (int i = 0; i < length; i++)
    {
        addr = ((remainder >> 7) ^ data[i]) & 0xff; // calculate PEC table address
        remainder = (remainder << 8) ^ (uint16_t)pec15Table[addr];
    }
    remainder = remainder * 2; // The CRC15 has a 0 in the LSB so the final value must be multiplied by 2
    pec[0] = (uint8_t)((remainder >> 8) & 0xFF);
    pec[1] = (uint8_t)(remainder & 0xFF);
    return pec;
}

template <size_t num_chips, size_t num_chip_selects>
std::array<uint8_t, 2> BMSDriverGroup<num_chips, num_chip_selects>::_generate_formatted_CMD(CMD_CODES_e command, int ic_index)
{
    std::array<uint8_t, 2> cmd;
#ifdef USING_LTC6811_1
    cmd[0] = (uint8_t) command >> 8;
    cmd[1] = (uint8_t) command;
#else
    cmd[0] = (uint8_t) get_cmd_address(address) | (uint8_t)command >> 8;
    cmd[1] = (uint8_t) command;
#endif
    return cmd;
}

template <size_t num_chips, size_t num_chip_selects>
std::array<uint8_t, 4> BMSDriverGroup<num_chips, num_chip_selects>::_generate_CMD_PEC(CMD_CODES_e command, int ic_index)
{
    std::array<uint8_t, 4> cmd_pec;
    std::array<uint8_t, 2> cmd = _generate_formatted_CMD(command, ic_index);
    std::array<uint8_t, 2> pec = _calculate_specific_PEC(cmd, 2);
    std::copy(cmd.data(), cmd.data() + 2, cmd_pec.data()); // Copy first two bytes (cmd)
    std::copy(pec.data(), pec.data() + 2, cmd_pec.data() + 2);           // Copy next two bytes (pec)
    return cmd_pec;
}
