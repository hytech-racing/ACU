/* Library Includes */
#include "Configuration.h"
#include "BMSDriverGroup.h"
#include "LTCSPIInterface.h"
#include <string.h>
#include <stdio.h>
#include <array>
#include <string>
#include <optional>

/* -------------------- SETUP FUNCTIONS -------------------- */

template <size_t num_chips, size_t num_chip_selects>
void BMSDriverGroup<num_chips, num_chip_selects>::manual_send_and_print() {
    std::array<uint8_t, 4> cmd_pec;
    
    // Wake up protocol
    _write_and_delay_LOW(10, 400);
    //SPI.transfer(0);
    _write_and_delay_HIGH(10, 400); // t_wake is 400 microseconds; wait that long to ensure device has turned on.
    

    //read_registers_command<8>(10, cmd_pec);
    
    // Send ADC conversion command which we know works
    _start_cell_voltage_ADC_conversion();

    // Another way is to do a dummy command like read configuration
    //cmd_pec = _generate_CMD_PEC(CMD_CODES_e::POLL_ADC_STATUS, -1);
    //cmd_pec[0] = (_address[0] << 3) | cmd_pec[0];
    //adc_conversion_command(10, cmd_pec);
    // Wake up again
    // _write_and_delay_LOW(10, 400);
    // SPI.transfer(0);
    // _write_and_delay_HIGH(10, 400); // t_wake is 400 mic roseconds; wait that long to ensure device has turned on.
    // cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_CONFIG, -1);
    // read_registers_command<8>(10, cmd_pec);
    
    // Now we read
    // cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_A, -1);

    // std::array<uint8_t, 8> data_in = read_registers_command<8>(10, cmd_pec);
}

template <size_t num_chips, size_t num_chip_selects>
void BMSDriverGroup<num_chips, num_chip_selects>::init()
{
    // We only call this once during setup to instantiate the variable pec15Table, a pointer to an array
    _generate_PEC_table();
    int cs = 10; // 9;
    for (size_t i = 0; i < num_chip_selects; i++)
    {
        _chip_select[i] = cs;
        // chip select defines
        pinMode(cs, OUTPUT);
        digitalWrite(cs, HIGH);
        cs++;
    }
    for (size_t j = 0; j < num_chips; j++)
    {
        _address[j] = j; // For the most part
        switch (j)
        {
        case 2:
        case 3:
        case 4:
        case 5:
        case 10:
        case 11:
            _chip_select_per_chip[j] = 10;
            break;

        case 0:
        case 1:
        case 6:
        case 7:
        case 8:
        case 9:
            _chip_select_per_chip[j] = 9;
            break;
        default: 
            break;
        }
    }
    std::array<int, num_chips> test_address = {4}; // For testing only
    _override_default_address(test_address);       // For testing only

    _chip_select_per_chip[0] = 10; // For testing only
    _chip_select[0] = 10;          // For testing only
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
        _pec15Table[i] = remainder & 0xFFFF;
    }
}

template <size_t num_chips, size_t num_chip_selects>
void BMSDriverGroup<num_chips, num_chip_selects>::_start_wakeup_protocol()
{
    for (size_t cs = 0; cs < num_chip_selects; cs++)
    {   
        if (_chip_type == LTC6811_Type_e::LTC6811_1) {
            _write_and_delay_LOW(_chip_select[cs], 450);
            SPI.transfer16(0);
            _write_and_delay_HIGH(_chip_select[cs], 450); // t_wake is 400 microseconds; wait that long to ensure device has turned on.
        } else {
            _write_and_delay_LOW(_chip_select[cs], 450);
            SPI.transfer(0);
            _write_and_delay_HIGH(_chip_select[cs], 450); // t_wake is 400 microseconds; wait that long to ensure device has turned on.
        }
    }
}

/* -------------------- READING DATA FUNCTIONS -------------------- */

template <size_t num_chips, size_t num_chip_selects>
typename BMSDriverGroup<num_chips, num_chip_selects>::BMSDriverData
BMSDriverGroup<num_chips, num_chip_selects>::read_data()
{
    _start_wakeup_protocol(); // wakes all of the ICs on the chip select line
    // read configuration? 
    // write_configuration(dcto_read, cell_discharge_en);
    _start_cell_voltage_ADC_conversion(); // Gets the ICs ready to be read, must delay afterwards by ? us
    _start_GPIO_ADC_conversion();
    
    if (_chip_type == LTC6811_Type_e::LTC6811_1)
    {
        return _read_data_through_broadcast();
    }
    else
    {
        return _read_data_through_address();
    }
}

template <size_t num_chips, size_t num_chip_selects>
typename BMSDriverGroup<num_chips, num_chip_selects>::BMSDriverData
BMSDriverGroup<num_chips, num_chip_selects>::_read_data_through_broadcast()
{
    ReferenceMaxMin max_min_reference;
    BMSDriverData bms_data;
    constexpr size_t data_size = 8 * (num_chips / num_chip_selects);
    // constexpr size_t voltage_data_size = 4 * data_size;
    // std::array<uint8_t, voltage_data_size> data_in_cell_voltages_1_to_12;
    std::array<uint8_t, data_size> data_in_cell_voltages_1_to_3;
    std::array<uint8_t, data_size> data_in_cell_voltages_4_to_6;
    std::array<uint8_t, data_size> data_in_cell_voltages_7_to_9;
    std::array<uint8_t, data_size> data_in_cell_voltages_10_to_12;
    std::array<uint8_t, data_size> data_in_auxillaries_1_to_3;
    std::array<uint8_t, data_size> data_in_auxillaries_4_to_6;
    std::array<uint8_t, 4> cmd_pec;

    size_t battery_cell_count = 0;
    size_t gpio_count = 0;
    for (size_t cs = 0; cs < num_chip_selects; cs++)
    {
        _start_wakeup_protocol();
        //write_configuration(dcto_read, cell_discharge_en);

        cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_A, -1); // The address should never be used here
        data_in_cell_voltages_1_to_3 = read_registers_command<data_size>(_chip_select[cs], cmd_pec);

        cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_B, -1);
        data_in_cell_voltages_4_to_6 = read_registers_command<data_size>(_chip_select[cs], cmd_pec);

        cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_C, -1);
        data_in_cell_voltages_7_to_9 = read_registers_command<data_size>(_chip_select[cs], cmd_pec);

        cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_D, -1);
        data_in_cell_voltages_10_to_12 = read_registers_command<data_size>(_chip_select[cs], cmd_pec);

        cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_GPIO_VOLTAGE_GROUP_A, -1);
        data_in_auxillaries_1_to_3 = read_registers_command<data_size>(_chip_select[cs], cmd_pec);

        cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_GPIO_VOLTAGE_GROUP_A, -1);
        data_in_auxillaries_4_to_6 = read_registers_command<data_size>(_chip_select[cs], cmd_pec);

        // DEBUG: Check to see that the PEC is what we expect it to be

        for (size_t chip = 0; chip < num_chips / num_chip_selects; chip++)
        {
            std::array<uint16_t, 12> chip_voltages_in;
            int cell_count = (_address[chip] % 2 == 0) ? 12 : 9; // Even indexed ICs have 12 cells, odd have 9
            for (int cell_Index = 0; cell_Index < cell_count; cell_Index++)
            {
                std::array<uint8_t, 2> data_in_cell_voltage;
                int sub_index = chip * 8 + (cell_Index % 3) * 2;
                switch (cell_Index / 3) // There are 4 groups of CELL VOLTAGES, each with 3 cells
                {
                case 0:
                    std::copy(data_in_cell_voltages_1_to_3.data() + (sub_index), data_in_cell_voltages_1_to_3.data() + 2 + (sub_index), data_in_cell_voltage.data());
                    break;
                case 1:
                    std::copy(data_in_cell_voltages_4_to_6.data() + (sub_index), data_in_cell_voltages_4_to_6.data() + 2 + (sub_index), data_in_cell_voltage.data());
                    break;
                case 2:
                    std::copy(data_in_cell_voltages_7_to_9.data() + (sub_index), data_in_cell_voltages_7_to_9.data() + 2 + (sub_index), data_in_cell_voltage.data());
                    break;
                case 3:
                    std::copy(data_in_cell_voltages_10_to_12.data() + (sub_index), data_in_cell_voltages_10_to_12.data() + 2 + (sub_index), data_in_cell_voltage.data());
                    break; // We will never get here for odd indexed ICs
                }

                uint16_t voltage_in = data_in_cell_voltage[1] << 8 | data_in_cell_voltage[0];
                chip_voltages_in[cell_Index] = voltage_in;
                Serial.print(voltage_in / 10000.0, 4);
                Serial.print("\t");
                _store_voltage_data(bms_data, max_min_reference, chip_voltages_in, voltage_in, battery_cell_count);
            }
            std::copy(chip_voltages_in.begin(), chip_voltages_in.end() + cell_count, bms_data.voltages[cs * (num_chips / num_chip_selects) + chip].data());

            // Humidity and Temperature Data

            for (int gpio_Index = 0; gpio_Index < 5; gpio_Index++) // There are only five Auxillary ports
            {
                std::array<uint8_t, 2> data_in_gpio_voltage;
                int sub_index = chip * 8 + (gpio_Index % 3) * 2;
                switch (gpio_Index / 3) // There are 2 groups of AUXILLARIES, each with 3 2-byte gpio data
                {
                case 0:
                    std::copy(data_in_auxillaries_1_to_3.data() + (sub_index), data_in_auxillaries_1_to_3.data() + 2 + (sub_index), data_in_gpio_voltage.data());
                    break;
                case 1:
                    std::copy(data_in_auxillaries_4_to_6.data() + (sub_index), data_in_auxillaries_4_to_6.data() + 2 + (sub_index), data_in_gpio_voltage.data());
                    break;
                }
                uint16_t gpio_in = data_in_gpio_voltage[0] << 8 | data_in_gpio_voltage[1];
                _store_temperature_humidity_data(bms_data, max_min_reference, gpio_in, gpio_Index, gpio_count, chip);
            }
        }
    }

    bms_data.min_voltage = max_min_reference.min_voltage;
    bms_data.max_voltage = max_min_reference.max_voltage;
    bms_data.total_voltage = max_min_reference.total_voltage;
    bms_data.average_cell_temperature = max_min_reference.total_thermistor_temps / gpio_count;
    return bms_data;
}

template <size_t num_chips, size_t num_chip_selects>
typename BMSDriverGroup<num_chips, num_chip_selects>::BMSDriverData
BMSDriverGroup<num_chips, num_chip_selects>::_read_data_through_address()
{
    ReferenceMaxMin max_min_reference;
    BMSDriverData bms_data;
    std::array<uint8_t, 24> data_in_cell_voltages_1_to_12;
    std::array<uint8_t, 10> data_in_auxillaries_1_to_5;
    std::array<uint8_t, 4> cmd_pec;
    size_t battery_cell_count = 0;
    size_t gpio_count = 0;
    for (size_t chip = 0; chip < num_chips; chip++)
    {
        _start_wakeup_protocol();
        cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_A, chip);
        auto data_in_3_cell_voltages = read_registers_command<8>(_chip_select_per_chip[chip], cmd_pec);
        std::copy(data_in_3_cell_voltages.begin(), data_in_3_cell_voltages.begin() + 6, data_in_cell_voltages_1_to_12.begin());

        cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_B, chip);
        data_in_3_cell_voltages = read_registers_command<8>(_chip_select_per_chip[chip], cmd_pec);
        std::copy(data_in_3_cell_voltages.begin(), data_in_3_cell_voltages.begin() + 6, data_in_cell_voltages_1_to_12.begin() + 6);

        cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_C, chip);
        data_in_3_cell_voltages = read_registers_command<8>(_chip_select_per_chip[chip], cmd_pec);
        std::copy(data_in_3_cell_voltages.begin(), data_in_3_cell_voltages.begin() + 6, data_in_cell_voltages_1_to_12.begin() + 12);

        cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_D, chip);
        data_in_3_cell_voltages = read_registers_command<8>(_chip_select_per_chip[chip], cmd_pec);
        std::copy(data_in_3_cell_voltages.begin(), data_in_3_cell_voltages.begin() + 6, data_in_cell_voltages_1_to_12.begin() + 18);

        cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_GPIO_VOLTAGE_GROUP_A, chip);
        auto data_in_3_auxillaries = read_registers_command<8>(_chip_select_per_chip[chip], cmd_pec);
        std::copy(data_in_3_cell_voltages.begin(), data_in_3_cell_voltages.begin() + 6, data_in_auxillaries_1_to_5.begin());

        cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_GPIO_VOLTAGE_GROUP_A, chip);
        data_in_3_auxillaries = read_registers_command<8>(_chip_select_per_chip[chip], cmd_pec);
        std::copy(data_in_3_cell_voltages.begin(), data_in_3_cell_voltages.begin() + 4, data_in_auxillaries_1_to_5.begin() + 6);

        // DEBUG: Check to see that the PEC is what we expect it to be

        int cell_count = (chip % 2 == 0) ? 12 : 9; // Even indexed ICs have 12 cells, odd have 9
        std::array<uint16_t, 12> chip_voltages_in;
        for (int cell_Index = 0; cell_Index < cell_count; cell_Index++)
        {
            std::array<uint8_t, 2> data_in_cell_voltage;
            auto start = data_in_cell_voltages_1_to_12.begin() + (cell_Index * 2);
            auto end = start + 2;
            std::copy(start, end, data_in_cell_voltage.begin());

            uint16_t voltage_in = data_in_cell_voltage[1] << 8 | data_in_cell_voltage[0];
            chip_voltages_in[cell_Index] = voltage_in;
            Serial.print(voltage_in / 10000.0, 4); Serial.print("V\t");
            _store_voltage_data(bms_data, max_min_reference, chip_voltages_in, voltage_in, battery_cell_count);
        }
        std::copy(chip_voltages_in.data(), chip_voltages_in.data() + cell_count, bms_data.voltages[chip].data());

        // HUMIDITY AND TEMPERATURES

        for (int gpio_Index = 0; gpio_Index < 5; gpio_Index++) // There are only five Auxillary ports
        {
            std::array<uint8_t, 2> data_in_gpio_voltage;
            auto start = data_in_auxillaries_1_to_5.begin() + (gpio_Index * 2);
            auto end = start + 2;
            std::copy(start, end, data_in_gpio_voltage.begin());

            uint16_t gpio_in = data_in_gpio_voltage[0] << 8 | data_in_gpio_voltage[1];
            _store_temperature_humidity_data(bms_data, max_min_reference, gpio_in, gpio_Index, gpio_count, chip);
        }
    }

    bms_data.min_voltage = max_min_reference.min_voltage;
    bms_data.max_voltage = max_min_reference.max_voltage;
    bms_data.total_voltage = max_min_reference.total_voltage;
    bms_data.average_cell_temperature = max_min_reference.total_thermistor_temps / gpio_count;
    return bms_data;
}

template <size_t num_chips, size_t num_chip_selects>
void BMSDriverGroup<num_chips, num_chip_selects>::_store_voltage_data(BMSDriverData &bms_data, ReferenceMaxMin &max_min_reference, std::array<uint16_t, 12> &chip_voltages_in, const uint16_t &voltage_in, size_t &cell_count)
{
    max_min_reference.total_voltage += voltage_in;
    if (voltage_in < max_min_reference.min_voltage)
    {
        max_min_reference.min_voltage = voltage_in;
        bms_data.min_voltage_cell_id = cell_count;
    }
    if (voltage_in > max_min_reference.max_voltage)
    {
        max_min_reference.max_voltage = voltage_in;
        bms_data.max_voltage_cell_id = cell_count;
    }
    cell_count++;
}

template <size_t num_chips, size_t num_chip_selects>
void BMSDriverGroup<num_chips, num_chip_selects>::_store_temperature_humidity_data(BMSDriverData &bms_data, ReferenceMaxMin &max_min_reference, const uint16_t &gpio_in, size_t gpio_Index, size_t &gpio_count, size_t chip_num)
{
    if (gpio_Index < 4)
    {
        float thermistor_resistance = (2740 / (gpio_in / 50000.0)) - 2740;
        bms_data.cell_temperatures[gpio_count] = 1 / ((1 / 298.15) + (1 / 3984.0) * log(thermistor_resistance / 10000.0)) - 273.15; // calculation for thermistor temperature in C
        max_min_reference.total_thermistor_temps += gpio_in;
        if (gpio_in > max_min_reference.max_thermistor_voltage)
        {
            max_min_reference.max_thermistor_voltage = gpio_in;
            bms_data.max_cell_temperature_cell_id = gpio_count;
        }
        gpio_count++;
    }
    else if ((chip_num % 2) && gpio_Index == 4)
    {
        bms_data.board_temperatures[(chip_num + 2) / 2] = -66.875 + 218.75 * (gpio_in / 50000.0); // caculation for SHT31 temperature in C
        if (gpio_in > max_min_reference.max_board_temp_voltage)
        {
            max_min_reference.total_voltage = gpio_in;
            bms_data.max_board_temperature_segment_id = (chip_num + 2) / 2; // Because each segment only has 1 humidity and 1 board temp sensor
        }
    }
    else
    {
        bms_data.humidity[(chip_num + 2) / 2] = -12.5 + 125 * (gpio_in) / 50000.0; // humidity calculation
        if (gpio_in > max_min_reference.max_humidity)
        {
            max_min_reference.max_humidity = gpio_in;
            bms_data.max_board_temperature_segment_id = (chip_num + 2) / 2;
        }
    }
}

/* -------------------- WRITING DATA FUNCTIONS -------------------- */

template <size_t num_chips, size_t num_chip_selects>
void BMSDriverGroup<num_chips, num_chip_selects>::write_configuration(uint8_t dcto_mode, const std::array<uint16_t, num_chips> &cell_balance_statuses)
{
    std::copy(cell_balance_statuses.begin(), cell_balance_statuses.end(), cell_discharge_en.begin());

    std::array<uint8_t, 6> buffer_format; // This buffer processing can be seen in more detail on page 62 of the data sheet
    buffer_format[0] = (gpios_enabled << 3) | (static_cast<int>(device_refup_mode) << 2) | static_cast<int>(adcopt);
    buffer_format[1] = (under_voltage_threshold & 0x0FF);
    buffer_format[2] = ((over_voltage_threshold & 0x00F) << 4) | ((under_voltage_threshold & 0xF00) >> 8);
    buffer_format[3] = ((over_voltage_threshold & 0xFF0) >> 4);
    
    if (_chip_type == LTC6811_Type_e::LTC6811_1)
    {
        _write_config_through_broadcast(dcto_mode, buffer_format, cell_balance_statuses);
    }
    else
    {
        _write_config_through_address(dcto_mode, buffer_format, cell_balance_statuses);
    }
}

template <size_t num_chips, size_t num_chip_selects>
void BMSDriverGroup<num_chips, num_chip_selects>::_write_config_through_broadcast(uint8_t dcto_mode, std::array<uint8_t, 6> buffer_format, const std::array<uint16_t, num_chips> &cell_balance_statuses)
{   
    constexpr size_t data_size = 8 * (num_chips / num_chip_selects);
    std::array<uint8_t, 4> cmd_and_pec = _generate_CMD_PEC(CMD_CODES_e::WRITE_CONFIG, -1);
    std::array<uint8_t, data_size> full_buffer;
    std::array<uint8_t, 2> temp_pec;

    // Needs to be sent on each chip select line
    for (size_t cs = 0; cs < num_chip_selects; cs++)
    {
        size_t j = 0;
        for (int i = num_chips - 1; i >= 0; i--)           // This needs to be flipped because when writing a command, primary device holds the last bytes
        {                                                     // Find chips with the same CS
            if (_chip_select_per_chip[i] == _chip_select[cs]) // This could be an optimization:  && j < (num_chips + 1) / 2)
            {
                buffer_format[4] = ((cell_balance_statuses[i] & 0x0FF));
                buffer_format[5] = ((dcto_mode & 0x0F) << 4) | ((cell_balance_statuses[i] & 0xF00) >> 8);
                temp_pec = _calculate_specific_PEC(buffer_format.data(), 6);
                std::copy(buffer_format.begin(), buffer_format.end(), full_buffer.data() + (j * 8));
                std::copy(temp_pec.begin(), temp_pec.end(), full_buffer.data() + 6 + (j * 8));
                j++;
            }
        }
        write_registers_command<data_size>(_chip_select[cs], cmd_and_pec, full_buffer);
    }
}

template <size_t num_chips, size_t num_chip_selects>
void BMSDriverGroup<num_chips, num_chip_selects>::_write_config_through_address(uint8_t dcto_mode, std::array<uint8_t, 6> buffer_format, const std::array<uint16_t, num_chips> &cell_balance_statuses)
{
    // Need to manipulate the command code to have address, therefore have to send command num_chips times
    std::array<uint8_t, 4> cmd_and_pec;
    std::array<uint8_t, 8> full_buffer;
    std::array<uint8_t, 2> temp_pec;
    for (size_t i = 0; i < num_chips; i++)
    {
        cmd_and_pec = _generate_CMD_PEC(CMD_CODES_e::WRITE_CONFIG, i);
        buffer_format[4] = ((cell_balance_statuses[i] & 0x0FF));
        buffer_format[5] = ((dcto_mode & 0x0F) << 4) | ((cell_balance_statuses[i] & 0xF00) >> 8);
        temp_pec = _calculate_specific_PEC(buffer_format.data(), 6);
        std::copy(buffer_format.data(), buffer_format.data() + 6, full_buffer.data());
        std::copy(temp_pec.data(), temp_pec.data() + 2, full_buffer.data() + 6);
        write_registers_command<8>(_chip_select_per_chip[i], cmd_and_pec, full_buffer);
    }
}

template <size_t num_chips, size_t num_chip_selects>
void BMSDriverGroup<num_chips, num_chip_selects>::_start_cell_voltage_ADC_conversion()
{
    uint16_t adc_cmd = (uint16_t) CMD_CODES_e::START_CV_ADC_CONVERSION | (adc_mode_cv_conversion << 7) | (discharge_permitted << 4) | static_cast<uint8_t>(adc_conversion_cell_select_mode);
    std::array<uint8_t, 2> cmd;
    cmd[0] = (adc_cmd >> 8) & 0xFF;
    cmd[1] = adc_cmd & 0xFF;
    if (_chip_type == LTC6811_Type_e::LTC6811_1)
    {
        _start_ADC_conversion_through_broadcast(cmd);
    }
    else
    {
        _start_ADC_conversion_through_address(cmd);
    }

    delay(cv_adc_conversion_time_us); // us
}

template <size_t num_chips, size_t num_chip_selects>
void BMSDriverGroup<num_chips, num_chip_selects>::_start_GPIO_ADC_conversion()
{
    uint16_t adc_cmd = (uint16_t)CMD_CODES_e::START_GPIO_ADC_CONVERSION | (adc_mode_gpio_conversion << 7) | static_cast<uint8_t>(adc_conversion_gpio_select_mode);
    std::array<uint8_t, 2> cmd;
    cmd[0] = (adc_cmd >> 8) & 0xFF;
    cmd[1] = adc_cmd & 0xFF;

    if (_chip_type == LTC6811_Type_e::LTC6811_1)
    {
        _start_ADC_conversion_through_broadcast(cmd);
    }
    else
    {
        _start_ADC_conversion_through_address(cmd);
    }

    delay(gpio_adc_conversion_time_us); // us
}

template <size_t num_chips, size_t num_chip_selects>
void BMSDriverGroup<num_chips, num_chip_selects>::_start_ADC_conversion_through_broadcast(const std::array<uint8_t, 2> &cmd_code)
{
    // Leave the command code as is
    uint8_t cc[2] = {cmd_code[0], cmd_code[1]};
    std::array<uint8_t, 2> pec = _calculate_specific_PEC(cc, 2);
    std::array<uint8_t, 4> cmd_and_pec;
    std::copy(cmd_code.begin(), cmd_code.end(), cmd_and_pec.begin()); // Copy first two bytes (cmd)
    std::copy(pec.begin(), pec.end(), cmd_and_pec.begin() + 2);       // Copy next two bytes (pec)

    // Needs to be sent on each chip select line
    for (size_t cs = 0; cs < num_chip_selects; cs++)
    {   
        adc_conversion_command(_chip_select[cs], cmd_and_pec, (num_chips / num_chip_selects));
    }
}

template <size_t num_chips, size_t num_chip_selects>
void BMSDriverGroup<num_chips, num_chip_selects>::_start_ADC_conversion_through_address(std::array<uint8_t, 2> cmd_code)
{
    // Need to manipulate the command code to have address, therefore have to send command num_chips times
    for (size_t i = 0; i < num_chips; i++)
    {
        cmd_code[0] = _get_cmd_address(_address[i]) | cmd_code[0]; // Make sure address is embedded in each cmd_code send
        std::array<uint8_t, 2> pec = _calculate_specific_PEC(cmd_code.data(), 2);
        std::array<uint8_t, 4> cmd_and_pec;
        std::copy(cmd_code.data(), cmd_code.data() + 2, cmd_and_pec.data()); // Copy first two bytes (cmd)
        std::copy(pec.data(), pec.data() + 2, cmd_and_pec.data() + 2);       // Copy next two bytes (pec)
        adc_conversion_command(_chip_select_per_chip[i], cmd_and_pec, 0);
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
        remainder = (remainder << 8) ^ (uint16_t)_pec15Table[addr];
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

    if (_chip_type == LTC6811_Type_e::LTC6811_1)
    {
        cmd[0] = (uint8_t)command >> 8;
        cmd[1] = (uint8_t)command;
    }
    else
    {
        cmd[0] = (uint8_t)_get_cmd_address(_address[ic_index]) | (uint8_t)command >> 8;
        cmd[1] = (uint8_t)command;
    }
    return cmd;
}

template <size_t num_chips, size_t num_chip_selects>
std::array<uint8_t, 4> BMSDriverGroup<num_chips, num_chip_selects>::_generate_CMD_PEC(CMD_CODES_e command, int ic_index)
{
    std::array<uint8_t, 4> cmd_pec;
    std::array<uint8_t, 2> cmd = _generate_formatted_CMD(command, ic_index);
    std::array<uint8_t, 2> pec = _calculate_specific_PEC(cmd.data(), 2);
    std::copy(cmd.data(), cmd.data() + 2, cmd_pec.data());     // Copy first two bytes (cmd)
    std::copy(pec.data(), pec.data() + 2, cmd_pec.data() + 2); // Copy next two bytes (pec)
    return cmd_pec;
}

template <size_t num_chips, size_t num_chip_selects>
void BMSDriverGroup<num_chips, num_chip_selects>::_override_default_address(std::array<int, num_chips> addr)
{
    std::copy(addr.data(), addr.data() + num_chips, _address.data());
}