/* Library Includes */
#include "DataContainer.h"
#include "LTC6811.h"
#include "LTCSPIInterface.h"
#include "LTC6811_1.h"
#include "LTC6811_2.h"
#include <string.h>
#include <stdio.h>
#include <string>
/* -------------------- SETUP FUNCTIONS -------------------- */

void LTC6811::setup()
{
    // We only call this once during setup to instantiate the variable pec15Table, a pointer to an array
    generate_PEC_table();
}

void LTC6811::generate_PEC_table()
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

void LTC6811::start_wakeup_protocol()
{
    write_and_delay_LOW(this->chip_select, 1);
    SPI.transfer(0);
    write_and_delay_HIGH(this->chip_select, 400); // t_wake is 400 microseconds; wait that long to ensure device has turned on.
}

int LTC6811::get_delay_time()
{
    return 0;
}

/* -------------------- READING DATA FUNCTIONS -------------------- */

void LTC6811::read_voltages()
{
    start_wakeup_protocol();             // wakes all of the ICs on the chip select line
    write_configuration(DCTO_READ);      // part of the wake up
    start_cell_voltage_ADC_conversion(); // Gets the ICs ready to be read, must delay afterwards by ? ms
    uint8_t *cmd_and_pec;
    uint8_t *data_in_A;
    uint8_t *data_in_B;
    uint8_t *data_in_C;
    uint8_t *data_in_D;
    // Every time we call a READ command, we will take in 48 bytes->(6 buffer + 2 pec)* #ICs(6) = 48
    start_wakeup_protocol();
    cmd_and_pec = generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_A);      // 24 bytes
    data_in_A = send_SPI_read_registers_command(this->chip_select, cmd_and_pec); // 48 bytes
    cmd_and_pec = generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_B);
    data_in_B = send_SPI_read_registers_command(this->chip_select, cmd_and_pec); // 48 bytes
    cmd_and_pec = generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_C);
    data_in_C = send_SPI_read_registers_command(this->chip_select, cmd_and_pec); // 48 bytes
    cmd_and_pec = generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_D);
    data_in_D = send_SPI_read_registers_command(this->chip_select, cmd_and_pec); // 48 bytes
    for (int i = 0; i < TOTAL_IC / 2; i++)
    {
        std::copy(data_in_A + (i * 8), data_in_A + (i * 8) + 6, IC_buffer[i].cell_voltage_A);
        std::copy(data_in_B + (i * 8), data_in_B + (i * 8) + 6, IC_buffer[i].cell_voltage_B);
        std::copy(data_in_C + (i * 8), data_in_C + (i * 8) + 6, IC_buffer[i].cell_voltage_C);
        std::copy(data_in_D + (i * 8), data_in_D + (i * 8) + 6, IC_buffer[i].cell_voltage_D);
        // Should check for PEC error, this is just for A
        uint8_t *data_A;
        std::copy(data_in_A + (i * 8), data_in_A + (i * 8) + 6, data_A);
        uint8_t *pec_A = calculate_specific_PEC(data_A, 6);
        if (pec_A[0] != data_in_A[i * 8 + 6] || pec_A[0] != data_in_A[i * 8 + 7])
        {
            Serial.print("There is a pec error!");
        }
        // Now we're going to convert the buffers into usable data
        int cell_count = (i % 2 == 0) ? 12 : 9; // Even indexed ICs have 12 cells, odd have 9
        for (int cell_Index = 0; cell_Index < cell_count; cell_Index++)
        {
            uint16_t* cell_voltage_buffer;
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
            IC_data[i].cell_voltage[cell_Index] = cell_voltage_buffer[cell_Index % 3];
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
}

void LTC6811::read_GPIOs()
{
    start_wakeup_protocol();
    write_configuration(DCTO_READ);
    start_cell_voltage_ADC_conversion();
    uint8_t *cmd_and_pec;
    uint8_t *data_in_A;
    uint8_t *data_in_B;
    // Every time we call a READ command, we will take in 48 bytes->(6 buffer + 2 pec)* #ICs(6) = 48
    cmd_and_pec = generate_CMD_PEC(CMD_CODES_e::READ_GPIO_VOLTAGE_GROUP_A);      // 24 bytes
    data_in_A = send_SPI_read_registers_command(this->chip_select, cmd_and_pec); // 48 bytes
    cmd_and_pec = generate_CMD_PEC(CMD_CODES_e::READ_GPIO_VOLTAGE_GROUP_B);
    data_in_B = send_SPI_read_registers_command(this->chip_select, cmd_and_pec); // 48 bytes
    for (int i = 0; i < TOTAL_IC / 2; i++)
    {
        std::copy(data_in_A + (i * 8), data_in_A + (i * 8) + 6, IC_buffer[i].gpio_voltage_A);
        std::copy(data_in_B + (i * 8), data_in_B + (i * 8) + 6, IC_buffer[i].gpio_voltage_B);
        // Should check for PEC error
        uint8_t *data_A;
        std::copy(data_in_A + (i * 8), data_in_A + (i * 8) + 6, data_A);
        uint8_t *pec_A = calculate_specific_PEC(data_A, 6);
        if (pec_A[0] != data_in_A[i * 8 + 6] || pec_A[0] != data_in_A[i * 8 + 7])
        {
            Serial.print("There is a pec error!");
        }
    }
    // At this point, all of the data has been read and put into the IC_buffer container
    // Now we have to convert it into tangible data
}

/* -------------------- WRITING DATA FUNCTIONS -------------------- */

void LTC6811::write_configuration(uint8_t dcto_mode)
{
    uint8_t *cmd_and_pec = generate_CMD_PEC(CMD_CODES_e::WRITE_CONFIG);
    uint8_t buffer_format[6]; // This buffer processing can be seen in more detail on page 62 of the data sheet
    buffer_format[0] = (GPIOS_ENABLED << 3) | (static_cast<int>(DEVICE_REFUP_MODE) << 2) | static_cast<int>(ADCOPT);
    buffer_format[1] = (under_voltage_threshold & 0x0FF);
    buffer_format[2] = ((over_voltage_threshold & 0x00F) << 4) | ((under_voltage_threshold & 0xF00) >> 8);
    buffer_format[3] = ((over_voltage_threshold & 0xFF0) >> 4);
    uint8_t buffer_and_pec[TOTAL_IC / 2 * 8]; // 48 bytes, 8 bytes per IC(6)
    // Regardless of IC model, we will need to send 6 x 6 bytes of buffer and 2 x 6 bytes of buffer pec
    uint8_t *ic_buffer_pec;
    for (int i = 0; i < TOTAL_IC / 2; i++)
    {
        buffer_format[4] = ((IC_data->balance_status[i] & 0x0FF));
        buffer_format[5] = ((dcto_mode & 0x0F) << 4) | ((IC_data->balance_status[i] & 0xF00) >> 8);
        ic_buffer_pec = calculate_specific_PEC(buffer_format, 6);
        std::copy(buffer_format, buffer_format + 6, buffer_and_pec + (i * 8));
        std::copy(ic_buffer_pec, ic_buffer_pec + 2, buffer_and_pec + (i * 8) + 6);
    }
    // send message on SPI
    send_SPI_write_registers_command(this->chip_select, cmd_and_pec, buffer_and_pec);
}

void LTC6811::start_cell_voltage_ADC_conversion()
{
    uint16_t adc_cmd = (uint16_t)CMD_CODES_e::START_CV_ADC_CONVERSION | (ADC_MODE_CV_CONVERSION << 7) | (DISCHARGE_PERMITTED << 4) | static_cast<uint8_t>(ADC_CONVERSION_CELL_SELECT_MODE);
    uint8_t cmd[2];
    cmd[0] = (adc_cmd >> 8) & 0xFF;
    cmd[1] = adc_cmd && 0xFF;
    uint8_t *pec = calculate_specific_PEC(cmd, 2);
    // SPI function will take care of how many times we need to send this message
    uint8_t cmd_and_pec[4];
    std::copy(cmd, cmd + 2, cmd_and_pec);     // Copy first two bytes (cmd)
    std::copy(pec, pec + 2, cmd_and_pec + 2); // Copy next two bytes (pec)
    send_SPI_non_register_command(this->chip_select, cmd_and_pec);
    delay(cv_adc_conversion_time_ms); // ms
}

void LTC6811::start_GPIO_ADC_conversion()
{
    uint16_t adc_cmd = (uint16_t)CMD_CODES_e::START_GPIO_ADC_CONVERSION | (ADC_MODE_GPIO_CONVERSION << 7) | static_cast<uint8_t>(ADC_CONVERSION_GPIO_SELECT);
    uint8_t cmd[2];
    cmd[0] = (adc_cmd >> 8) & 0xFF;
    cmd[1] = adc_cmd && 0xFF;
    uint8_t *pec = calculate_specific_PEC(cmd, 2);
    // SPI function will take care of how many times we need to send this message
    uint8_t cmd_and_pec[4];
    std::copy(cmd, cmd + 2, cmd_and_pec);     // Copy first two bytes (cmd)
    std::copy(pec, pec + 2, cmd_and_pec + 2); // Copy next two bytes (pec)
    send_SPI_non_register_command(this->chip_select, cmd_and_pec);
    delay(gpio_adc_conversion_time_ms); // ms
}

/* -------------------- PRINT DATA FUNCTIONS -------------------- */

void LTC6811::print_voltage_data()
{
    // 6 ICs, 21 Cell voltages for every 2 ICs, 63 Cells per chip select
}

void LTC6811::print_GPIO_data()
{
    // 6 ICs, 21 Cell voltages for every 2 ICs, 63 Cells per chip select
}

/* -------------------- GETTER FUNCTIONS -------------------- */

uint8_t *LTC6811::calculate_specific_PEC(uint8_t *data, int length)
{
    uint16_t remainder;
    uint16_t addr;
    uint8_t pec[2];
    remainder = 16; // PEC seed
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

uint8_t *LTC6811::generate_formatted_CMD(CMD_CODES_e command, int ic_index)
{
    uint8_t *cmd;
#ifdef USING_LTC6811_1
    cmd = generate_broadcast_command(command);
#else
    cmd = generate_address_command(command, address[ic_index]);
#endif
    return cmd;
}

uint8_t *LTC6811::generate_CMD_PEC(CMD_CODES_e command)
{
    uint8_t *cmd_pec;
#ifdef USING_LTC6811_1
    uint8_t *cmd = generate_formatted_CMD(command, -1);
    uint8_t *pec = calculate_specific_PEC(reinterpret_cast<uint8_t *>(command), 2);
    std::copy(cmd, cmd + 2, cmd_pec);     // Copy first two bytes (cmd)
    std::copy(pec, pec + 2, cmd_pec + 2); // Copy next two bytes (pec)
#else
    uint8_t *cmd;
    uint8_t *pec;
    for (int i = 0; i < TOTAL_IC / 2; i++)
    {
        cmd = generate_formatted_CMD(command, address[i]);
        pec = calculate_specific_PEC(reinterpret_cast<uint8_t *>(command), 2);
        std::copy(cmd, cmd + 2, cmd_pec + (i * 4));     // Copy first two bytes (cmd)
        std::copy(pec, pec + 2, cmd_pec + (i * 4) + 2); // Copy next two bytes (pec)
    }
#endif
    return cmd_pec; // Will be 4 bytes if _1, otherwise 24 bytes b/c 4 bytes x 6 ICs
}
