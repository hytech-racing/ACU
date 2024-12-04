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
    for (int i = 0; i < num_chip_selects; i++)
    {
        switch (i)
        {
        case 2:
        case 3:
        case 4:
        case 5:
        case 10:
        case 11:
            chip_select[i] = 10;
            break;

        case 0:
        case 1:
        case 6:
        case 7:
        case 8:
        case 9:
            chip_select[i] = 10;
            break;
        }
        pinMode(chip_select[i], OUTPUT);
        digitalWrite(chip_select[i], HIGH);
    }
}

// this implementation is straight from:
// on page <>, section:
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
    write_and_delay_LOW(this->chip_select, 1);
    SPI.transfer(0);
    write_and_delay_HIGH(this->chip_select, 400); // t_wake is 400 microseconds; wait that long to ensure device has turned on.
}

/* -------------------- READING DATA FUNCTIONS -------------------- */

template <size_t num_chips, size_t num_chip_selects>
typename BMSDriverGroup<num_chips, num_chip_selects>::BMSData
BMSDriverGroup<num_chips, num_chip_selects>::read_data(const std::array<bool, 12> &cell_balance_statuses)
{

    _start_wakeup_protocol(); // wakes all of the ICs on the chip select line
    __write_configuration(dcto_read);
    _start_cell_voltage_ADC_conversion(); // Gets the ICs ready to be read, must delay afterwards by ? ms
    uint8_t data_in_cell_voltages_0_to_3[num_chips * 8];
    uint8_t data_in_cell_voltages_4_to_6[num_chips * 8];
    uint8_t data_in_cell_voltages_7_to_9[num_chips * 8];
    uint8_t data_in_cell_voltages_10_to_12[num_chips * 8];
#ifdef USING_LTC6811_1
    uint8_t cmd_and_pec[4];
#else
    uint8_t cmd_and_pec[4 * num_chips];
#endif

    // Every time we call a READ command, we will take in 48 bytes->(6 buffer + 2 pec)* #ICs(6) = 48
    _start_wakeup_protocol();
    _generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_A, cmd_and_pec);                          // 24 bytes
    read_registers_command(this->chip_select, cmd_and_pec, num_chips, data_in_cell_voltages_0_to_3); // 48 bytes
    _generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_B, cmd_and_pec);
    read_registers_command(this->chip_select, cmd_and_pec, num_chips, data_in_cell_voltages_4_to_6); // 48 bytes
    _generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_C, cmd_and_pec);
    read_registers_command(this->chip_select, cmd_and_pec, num_chips, data_in_cell_voltages_7_to_9); // 48 bytes
    _generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_D, cmd_and_pec);
    read_registers_command(this->chip_select, cmd_and_pec, num_chips, data_in_cell_voltages_10_to_12); // 48 bytes

    for (int i = 0; i < num_chips; i++)
    {
        std::copy(data_in_cell_voltages_0_to_3 + (i * 8), data_in_cell_voltages_0_to_3 + (i * 8) + 6, IC_buffer[i].cell_voltage_A);
        std::copy(data_in_cell_voltages_4_to_6 + (i * 8), data_in_cell_voltages_4_to_6 + (i * 8) + 6, IC_buffer[i].cell_voltage_B);
        std::copy(data_in_cell_voltages_7_to_9 + (i * 8), data_in_cell_voltages_7_to_9 + (i * 8) + 6, IC_buffer[i].cell_voltage_C);
        std::copy(data_in_cell_voltages_10_to_12 + (i * 8), data_in_cell_voltages_10_to_12 + (i * 8) + 6, IC_buffer[i].cell_voltage_D);
        // Should check for PEC error, this is just for A
        uint8_t data_A[6];
        std::copy(data_in_cell_voltages_0_to_3 + (i * 8), data_in_cell_voltages_0_to_3 + (i * 8) + 6, data_A);
        uint8_t pec_A[2];
        _calculate_specific_PEC(data_A, 6, pec_A);
        if (pec_A[0] != data_in_cell_voltages_0_to_3[i * 8 + 6] || pec_A[0] != data_in_cell_voltages_0_to_3[i * 8 + 7])
        {
            // Serial.print("There is a pec error!");
        }
        // Now we're going to convert the buffers into usable data
        _reset_voltage_data();
        int cell_count = (i % 2 == 0) ? 12 : 9; // Even indexed ICs have 12 cells, odd have 9
        for (int cell_Index = 0; cell_Index < cell_count; cell_Index++)
        {
            uint16_t *cell_voltage_buffer;
            switch (cell_Index / 3) // There are 4 groups of CELL VOLTAGES, each with 3 cells
            {
            case 0:
                cell_voltage_buffer = IC_buffer[i].cell_voltage_A;
                break;
            case 1:
                cell_voltage_buffer = IC_buffer[i].cell_voltage_B;
                break;
            case 2:
                cell_voltage_buffer = IC_buffer[i].cell_voltage_C;
                break;
            case 3:
                cell_voltage_buffer = IC_buffer[i].cell_voltage_D;
                break; // We will never get here for odd indexed ICs
            }

            data_to_return.cell_voltage[cell_Index] = cell_voltage_buffer[cell_Index % 3];
            total_voltage += IC_data[i].cell_voltage[cell_Index];
            if (IC_data[i].cell_voltage[cell_Index] < min_voltage)
            {
                min_voltage = IC_data[i].cell_voltage[cell_Index];
                min_voltage_location.icIndex = i;
                min_voltage_location.cellIndex = cell_Index;
            }
            if (IC_data[i].cell_voltage[cell_Index] > max_voltage)
            {
                max_voltage = IC_data[i].cell_voltage[cell_Index];
                max_voltage_location.icIndex = i;
                max_voltage_location.cellIndex = cell_Index;
            }
        }
    }

    // Reading Thermistor and Humidity Data

    _start_wakeup_protocol();
    _write_configuration(dcto_read);
    _start_cell_voltage_ADC_conversion();
    std::array<uint8_t, 4> cmd_and_pec;
    std::array<uint8_t, num_chips * 8> data_in_A;
    std::array<uint8_t, num_chips * 8> data_in_B;
    // Every time we call a READ command, we will take in 48 bytes->(6 buffer + 2 pec)* #ICs(6) = 48
    _generate_CMD_PEC(CMD_CODES_e::READ_GPIO_VOLTAGE_GROUP_A, cmd_and_pec, 0);    // 24 bytes
    read_registers_command(this->chip_select, cmd_and_pec, num_chips, data_in_A); // 48 bytes
    _generate_CMD_PEC(CMD_CODES_e::READ_GPIO_VOLTAGE_GROUP_B, cmd_and_pec, 0);
    read_registers_command(this->chip_select, cmd_and_pec, num_chips, data_in_B); // 48 bytes
    for (int i = 0; i < num_chips; i++)
    {
        std::copy(data_in_A + (i * 8), data_in_A + (i * 8) + 6, IC_buffer[i].gpio_voltage_A);
        std::copy(data_in_B + (i * 8), data_in_B + (i * 8) + 6, IC_buffer[i].gpio_voltage_B);
        // Should check for PEC error
        uint8_t *data_A;
        std::copy(data_in_A + (i * 8), data_in_A + (i * 8) + 6, data_A);
        uint8_t pec_A[2];
        _calculate_specific_PEC(data_A, 6, pec_A);
        if (pec_A[0] != data_in_A[i * 8 + 6] || pec_A[0] != data_in_A[i * 8 + 7])
        {
            Serial.print("There is a pec error!");
        }

        // At this point, all of the data has been read and put into the IC_buffer container
        // Now we have to convert it into tangible data
        _reset_GPIO_data();
        for (int gpio_Index = 0; gpio_Index < 6; gpio_Index++)
        {
            uint16_t *gpio_voltage_buf;
            switch (gpio_Index / 3) // There are 4 groups of CELL VOLTAGES, each with 3 cells
            {
            case 0:
                gpio_voltage_buf = IC_buffer[i].gpio_voltage_A;
                break;
            case 1:
                gpio_voltage_buf = IC_buffer[i].gpio_voltage_B;
                break;
            }
            IC_data[i].gpio_voltage[gpio_Index] = gpio_voltage_buf[gpio_Index % 3];
            if ((i % 2) && gpio_Index == 4)
            {
                IC_data[i].gpio_temperatures[gpio_Index] = -66.875 + 218.75 * (IC_data[i].gpio_voltage[gpio_Index] / 50000.0); // caculation for SHT31 temperature in C
                if (IC_data[i].gpio_voltage[gpio_Index] > max_board_temp_voltage)
                {
                    max_board_temp_voltage = IC_data[i].gpio_voltage[gpio_Index];
                    max_board_temp_location.icIndex = i;
                    max_board_temp_location.cellIndex = gpio_Index;
                }
                if (IC_data[i].gpio_voltage[gpio_Index] < min_board_temp_voltage)
                {
                    min_board_temp_voltage = IC_data[i].gpio_voltage[gpio_Index];
                    min_board_temp_location.icIndex = i;
                    min_board_temp_location.cellIndex = gpio_Index;
                }
            }
            else if (gpio_Index == 4)
            {
                IC_data[i].gpio_temperatures[gpio_Index] = -12.5 + 125 * (IC_data[i].gpio_voltage[gpio_Index]) / 50000.0; // humidity calculation
                if (IC_data[i].gpio_voltage[gpio_Index] > max_humidity)
                {
                    max_humidity = IC_data[i].gpio_voltage[gpio_Index];
                    max_humidity_location.icIndex = i;
                    max_humidity_location.cellIndex = gpio_Index;
                }
            }
            else if (gpio_Index < 4)
            {
                float thermistor_resistance = (2740 / (IC_data[i].gpio_voltage[gpio_Index] / 50000.0)) - 2740;
                IC_data[i].gpio_temperatures[gpio_Index] = 1 / ((1 / 298.15) + (1 / 3984.0) * log(thermistor_resistance / 10000.0)) - 273.15; // calculation for thermistor temperature in C
                total_thermistor_temps += IC_data[i].gpio_temperatures[gpio_Index];
                if (IC_data[i].gpio_voltage[gpio_Index] > max_thermistor_voltage)
                {
                    max_thermistor_voltage = IC_data[i].gpio_voltage[gpio_Index];
                    max_thermistor_location.icIndex = i;
                    max_thermistor_location.cellIndex = gpio_Index;
                }
                if (IC_data[i].gpio_voltage[gpio_Index] < min_thermistor_voltage)
                {
                    min_thermistor_voltage = IC_data[i].gpio_voltage[gpio_Index];
                    min_thermistor_location.icIndex = i;
                    min_thermistor_location.cellIndex = gpio_Index;
                }
            }
        }
    }

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
    BMSData data_to_return;
    
    return bms_return;
}

template <size_t num_chips, size_t num_chip_selects>
typename BMSDriverGroup<num_chips, num_chip_selects>::BMSData
BMSDriverGroup<num_chips, num_chip_selects>::_read_data_through_address(const std::array<bool, 12> &cell_balance_statuses) {
    BMSData data_to_return;

    return bms_return;
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
    min_thermistor_voltage = 65535;
    max_board_temp_voltage = 0;
    min_board_temp_voltage = 65535;
    total_board_temps = 0;
    total_thermistor_temps = 0;
}

/* -------------------- WRITING DATA FUNCTIONS -------------------- */

template <size_t num_chips, size_t num_chip_selects>
void BMSDriverGroup<num_chips, num_chip_selects>::_write_configuration(uint8_t dcto_mode, const std::array<bool, 12> &cell_balance_statuses)
{
    uint8_t cmd_and_pec[4];
    _generate_CMD_PEC(CMD_CODES_e::WRITE_CONFIG, cmd_and_pec);
    uint8_t buffer_format[6]; // This buffer processing can be seen in more detail on page 62 of the data sheet
    buffer_format[0] = (gpios_enabled << 3) | (static_cast<int>(device_refup_mode) << 2) | static_cast<int>(adcopt);
    buffer_format[1] = (under_voltage_threshold & 0x0FF);
    buffer_format[2] = ((over_voltage_threshold & 0x00F) << 4) | ((under_voltage_threshold & 0xF00) >> 8);
    buffer_format[3] = ((over_voltage_threshold & 0xFF0) >> 4);
    uint8_t buffer_and_pec[num_chips * 8]; // 48 bytes, 8 bytes per IC(6)
    // Regardless of IC model, we will need to send 6 x 6 bytes of buffer and 2 x 6 bytes of buffer pec
    uint8_t ic_buffer_pec[2];

    for (int i = 0; i < num_chips; i++)
    {
        buffer_format[4] = ((reinterpret_cast<uint16_t>(cell_balance_statuses) & 0x0FF));
        buffer_format[5] = ((dcto_mode & 0x0F) << 4) | ((reinterpret_cast<uint16_t>(cell_balance_statuses) & 0xF00) >> 8);
        _calculate_specific_PEC(buffer_format, 6, ic_buffer_pec);
        std::copy(buffer_format, buffer_format + 6, buffer_and_pec + (i * 8));
        std::copy(ic_buffer_pec, ic_buffer_pec + 2, buffer_and_pec + (i * 8) + 6);
    }

    // send message on SPI
    write_registers_command(this->chip_select, cmd_and_pec, buffer_and_pec, num_chips);
}

template <size_t num_chips, size_t num_chip_selects>
void BMSDriverGroup<num_chips, num_chip_selects>::_start_cell_voltage_ADC_conversion()
{
    uint16_t adc_cmd = (uint16_t)CMD_CODES_e::START_CV_ADC_CONVERSION | (adc_mode_cv_conversion << 7) | (discharge_permitted << 4) | static_cast<uint8_t>(adc_conversion_cell_select_mode);
    uint8_t cmd[2];
    cmd[0] = (adc_cmd >> 8) & 0xFF;
    cmd[1] = adc_cmd && 0xFF;
    uint8_t pec[2];
    _calculate_specific_PEC(cmd, 2, pec);
    // SPI function will take care of how many times we need to send this message
    uint8_t cmd_and_pec[4];
    std::copy(cmd, cmd + 2, cmd_and_pec);     // Copy first two bytes (cmd)
    std::copy(pec, pec + 2, cmd_and_pec + 2); // Copy next two bytes (pec)
    non_register_command(this->chip_select, cmd_and_pec, num_chips);
    delay(cv_adc_conversion_time_ms); // us
}

template <size_t num_chips, size_t num_chip_selects>
void BMSDriverGroup<num_chips, num_chip_selects>::_start_GPIO_ADC_conversion()
{
    uint16_t adc_cmd = (uint16_t)CMD_CODES_e::START_GPIO_ADC_CONVERSION | (adc_mode_gpio_conversion << 7) | static_cast<uint8_t>(adc_conversion_gpio_select_mode);
    std::array<uint8_t, 2> cmd;
    cmd[0] = (adc_cmd >> 8) & 0xFF;
    cmd[1] = adc_cmd && 0xFF;
    uint8_t pec[2];
    _calculate_specific_PEC(cmd, 2, pec);
    // SPI function will take care of how many times we need to send this message
    uint8_t cmd_and_pec[4];
    std::copy(cmd.data(), cmd.data() + 2, cmd_and_pec);     // Copy first two bytes (cmd)
    std::copy(pec, pec + 2, cmd_and_pec + 2); // Copy next two bytes (pec)
    non_register_command(this->chip_select, cmd_and_pec, num_chips);
    delay(gpio_adc_conversion_time_ms); // us
}

/* -------------------- GETTER FUNCTIONS -------------------- */

// This implementation is taken directly from the data sheet linked here: https://www.analog.com/media/en/technical-documentation/data-sheets/LTC6811-1-6811-2.pdf
template <size_t num_chips, size_t num_chip_selects>
void BMSDriverGroup<num_chips, num_chip_selects>::_calculate_specific_PEC(uint8_t *data, int length, uint8_t *pec)
{
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
std::array<uint8_t, 4 * num_chips> BMSDriverGroup<num_chips, num_chip_selects>::_generate_CMD_PEC(CMD_CODES_e command, int ic_index)
{
    std::array<uint8_t, 4 * num_chips> cmd_pec;
    std::array<uint8_t, 2> cmd;
    uint8_t pec[2];
    for (int i = 0; i < num_chips; i++)
    {
        cmd = _generate_formatted_CMD(command, i);
        _calculate_specific_PEC(cmd, 2, pec);
        std::copy(cmd.data(), cmd.data() + 2, cmd_pec.data() + (i * 4)); // Copy first two bytes (cmd)
        std::copy(pec, pec + 2, cmd_pec.data() + 2 + (i * 4));           // Copy next two bytes (pec)
    }
    return cmd_pec;
}
