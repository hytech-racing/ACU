/* Library Includes */
#include "Configuration.h"
#include "BMSDriverGroup.h"
#include "LTCSPIInterface.h"
#include <string.h>
#include <stdio.h>
#include <string>
/* -------------------- SETUP FUNCTIONS -------------------- */

void BMSDriverGroup::init()
{
    // We only call this once during setup to instantiate the variable pec15Table, a pointer to an array
    generate_PEC_table();
    pinMode(chip_select, OUTPUT);
    digitalWrite(chip_select, HIGH);
}

void BMSDriverGroup::generate_PEC_table()
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

void BMSDriverGroup::start_wakeup_protocol()
{
    write_and_delay_LOW(this->chip_select, 1);
    SPI.transfer(0);
    write_and_delay_HIGH(this->chip_select, 400); // t_wake is 400 microseconds; wait that long to ensure device has turned on.
}

int BMSDriverGroup::get_delay_time()
{
    return 0;
}

/* -------------------- READING DATA FUNCTIONS -------------------- */

void BMSDriverGroup::read_voltages()
{
    start_wakeup_protocol();             // wakes all of the ICs on the chip select line
    write_configuration(dcto_read);      // part of the wake up
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
    for (int i = 0; i < ic_count; i++)
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
        reset_voltage_data();
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

void BMSDriverGroup::reset_voltage_data()
{
    total_voltage = 0;
    max_voltage = 0;
    min_voltage = 65535;
}

void BMSDriverGroup::read_GPIOs()
{
    start_wakeup_protocol();
    write_configuration(dcto_read);
    start_cell_voltage_ADC_conversion();
    uint8_t *cmd_and_pec;
    uint8_t *data_in_A;
    uint8_t *data_in_B;
    // Every time we call a READ command, we will take in 48 bytes->(6 buffer + 2 pec)* #ICs(6) = 48
    cmd_and_pec = generate_CMD_PEC(CMD_CODES_e::READ_GPIO_VOLTAGE_GROUP_A);      // 24 bytes
    data_in_A = send_SPI_read_registers_command(this->chip_select, cmd_and_pec); // 48 bytes
    cmd_and_pec = generate_CMD_PEC(CMD_CODES_e::READ_GPIO_VOLTAGE_GROUP_B);
    data_in_B = send_SPI_read_registers_command(this->chip_select, cmd_and_pec); // 48 bytes
    for (int i = 0; i < ic_count; i++)
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

        // At this point, all of the data has been read and put into the IC_buffer container
        // Now we have to convert it into tangible data
        reset_GPIO_data();
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
}

void BMSDriverGroup::reset_GPIO_data()
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

void BMSDriverGroup::write_configuration(uint8_t dcto_mode)
{
    uint8_t *cmd_and_pec = generate_CMD_PEC(CMD_CODES_e::WRITE_CONFIG);
    uint8_t buffer_format[6]; // This buffer processing can be seen in more detail on page 62 of the data sheet
    buffer_format[0] = (gpios_enabled << 3) | (static_cast<int>(device_refup_mode) << 2) | static_cast<int>(adcopt);
    buffer_format[1] = (under_voltage_threshold & 0x0FF);
    buffer_format[2] = ((over_voltage_threshold & 0x00F) << 4) | ((under_voltage_threshold & 0xF00) >> 8);
    buffer_format[3] = ((over_voltage_threshold & 0xFF0) >> 4);
    uint8_t buffer_and_pec[ic_count * 8]; // 48 bytes, 8 bytes per IC(6)
    // Regardless of IC model, we will need to send 6 x 6 bytes of buffer and 2 x 6 bytes of buffer pec
    uint8_t *ic_buffer_pec;
    for (int i = 0; i < ic_count; i++)
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

void BMSDriverGroup::start_cell_voltage_ADC_conversion()
{
    uint16_t adc_cmd = (uint16_t)CMD_CODES_e::START_CV_ADC_CONVERSION | (adc_mode_cv_conversion << 7) | (discharge_permitted << 4) | static_cast<uint8_t>(adc_conversion_cell_select_mode);
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

void BMSDriverGroup::start_GPIO_ADC_conversion()
{
    uint16_t adc_cmd = (uint16_t)CMD_CODES_e::START_GPIO_ADC_CONVERSION | (adc_mode_gpio_conversion << 7) | static_cast<uint8_t>(adc_conversion_gpio_select_mode);
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

void BMSDriverGroup::print_voltage_data()
{
    // 6 ICs, 21 Cell voltages for every 2 ICs, 63 Cells per chip select
    Serial.println("------------------------------------------------------------------------------------------------------------------------------------------------------------");
    if (min_voltage < minimum_voltage)
    {
        Serial.print("UNDERVOLTAGE FAULT: ");
        Serial.print("IC #: ");
        Serial.print(min_voltage_location.icIndex);
        Serial.print("\tCell #: ");
        Serial.print(min_voltage_location.cellIndex);
        Serial.print("\tFault Voltage: ");
        Serial.print(min_voltage / 10000.0, 4);
        // Serial.print("\tConsecutive fault #: ");
        // Serial.println(uv_fault_counter);
    }
    if (max_voltage > maximum_voltage)
    {
        Serial.print("OVERVOLTAGE FAULT: ");
        Serial.print("IC #: ");
        Serial.print(max_voltage_location.icIndex);
        Serial.print("\tCell #: ");
        Serial.println(max_voltage_location.cellIndex);
        Serial.print("\tFault Voltage: ");
        Serial.print(max_voltage / 10000.0, 4);
        // Serial.print("\tConsecutive fault #: ");
        // Serial.println(ov_fault_counter);
    }
    if (total_voltage > maximum_total_voltage)
    {
        Serial.print("PACK OVERVOLTAGE:");
        // Serial.print("\tConsecutive fault #: ");
        // Serial.println(pack_ov_fault_counter);
    }

    Serial.print("Total pack voltage: ");
    Serial.print(total_voltage / 10000.0, 4);
    Serial.print("V\t");
    Serial.print("Max voltage differential: ");
    Serial.print(max_voltage / 10000.0 - min_voltage / 10000.0, 4);
    Serial.println("V");
    // Serial.print("AMS status: ");
    // Serial.println(bms_status.get_state() == BMS_STATE_DISCHARGING ? "Discharging" : "Charging");

    Serial.println("------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.print("Max Voltage: ");
    Serial.print(IC_data[max_voltage_location.icIndex].cell_voltage[max_voltage_location.cellIndex] / 10000.0, 4);
    Serial.print("V \t ");
    Serial.print("Min Voltage: ");
    Serial.print(IC_data[min_voltage_location.icIndex].cell_voltage[min_voltage_location.cellIndex] / 10000.0, 4);
    Serial.print("V \t");
    Serial.print("Avg Voltage: ");
    Serial.print(total_voltage / 1260000.0, 4);
    Serial.println("V \t");
    Serial.println("------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.println("Raw Cell Voltages\t\t\t\t\t\t\t\t\t\t\t\t\tBalancing Status");
    Serial.print("\tC0\tC1\tC2\tC3\tC4\tC5\tC6\tC7\tC8\tC9\tC10\tC11\t\t");
    Serial.println(currently_balancing ? "\tC0\tC1\tC2\tC3\tC4\tC5\tC6\tC7\tC8\tC9\tC10\tC11" : "");

    for (int ic = 0; ic < ic_count; ic++)
    {
        Serial.print("IC");
        Serial.print(ic);
        Serial.print("\t");
        for (int cell = 0; cell < even_ic_cells; cell++)
        {
            Serial.print(IC_data[ic].cell_voltage[cell] / 10000.0, 4);
            Serial.print("V\t");
        }
        if (currently_balancing)
        {
            Serial.print("\t\t");
            for (int cell = 0; cell < even_ic_cells; cell++)
            {
                Serial.print(IC_data[ic].balance_status[cell]);
                Serial.print("\t");
            }
        }
        Serial.println();
    }
}

void BMSDriverGroup::print_GPIO_data()
{
    // 6 ICs, 6 GPIO voltages for every IC, 36 Temp / Humidity information per chip select
    Serial.println("------------------------------------------------------------------------------------------------------------------------------------------------------------");
    if (max_thermistor_voltage > maximum_thermistor_voltage)
    {
        Serial.print("OVERTEMP FAULT: ");
        //Serial.print("\tConsecutive fault #: ");
        //Serial.println(overtemp_fault_counter);
    }
    Serial.print("Max Board Temp: ");
    Serial.print(IC_data[max_board_temp_location.icIndex].gpio_temperatures[max_board_temp_location.cellIndex], 3);
    Serial.print("C \t ");
    Serial.print("Min Board Temp: ");
    Serial.print(IC_data[min_board_temp_location.icIndex].gpio_temperatures[min_board_temp_location.cellIndex], 3);
    Serial.print("C \t");
    Serial.print("Avg Board Temp: ");
    Serial.print(total_board_temps / 6, 3);
    Serial.println("C \t");
    Serial.print("Max Thermistor Temp: ");
    Serial.print(IC_data[max_thermistor_location.icIndex].gpio_temperatures[max_thermistor_location.cellIndex], 3);
    Serial.print("C \t");
    Serial.print("Min Thermistor Temp: ");
    Serial.print(IC_data[min_thermistor_location.icIndex].gpio_temperatures[min_thermistor_location.cellIndex], 3);
    Serial.print("C \t");
    Serial.print("Avg Thermistor Temp: ");
    Serial.print(total_thermistor_temps / 48, 3);
    Serial.println("C \t");
    Serial.print("Max Humidity: ");
    Serial.print(IC_data[max_humidity_location.icIndex].gpio_temperatures[max_humidity_location.cellIndex], 3);
    Serial.println("% \t ");
    Serial.println("------------------------------------------------------------------------------------------------------------------------------------------------------------");
    Serial.println("Raw Segment Temperatures");
    Serial.println("                  \tT0\tT1\tT2\tT3");
    for (int ic = 0; ic < ic_count; ic++)
    {
        Serial.print("Cell Temperatures");
        Serial.print(ic);
        Serial.print("\t");
        for (int cell = 0; cell < 4; cell++)
        {
            Serial.print(IC_data[ic].gpio_temperatures[cell], 3);
            Serial.print("C\t");
        }
        if ((ic % 2))
        {
            Serial.print("PCB Temps: ");
            Serial.print(IC_data[ic].gpio_temperatures[4], 3);
            Serial.print("C\t");
        }
        else
        {
            Serial.print("PCB Humidity: ");
            Serial.print(IC_data[ic].gpio_temperatures[4], 3);
            Serial.print("%\t");
        }
        Serial.print("\t");
        Serial.println();
    }
    Serial.println("------------------------------------------------------------------------------------------------------------------------------------------------------------");
}

/* -------------------- GETTER FUNCTIONS -------------------- */

uint8_t *BMSDriverGroup::calculate_specific_PEC(uint8_t *data, int length)
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

uint8_t *BMSDriverGroup::generate_formatted_CMD(CMD_CODES_e command, int ic_index)
{
    uint8_t *cmd;
#ifdef USING_LTC6811_1
    cmd = generate_broadcast_command(command);
#else
    cmd = generate_address_command(command, address[ic_index]);
#endif
    return cmd;
}

uint8_t *BMSDriverGroup::generate_CMD_PEC(CMD_CODES_e command)
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
    for (int i = 0; i < ic_count; i++)
    {
        cmd = generate_formatted_CMD(command, address[i]);
        pec = calculate_specific_PEC(reinterpret_cast<uint8_t *>(command), 2);
        std::copy(cmd, cmd + 2, cmd_pec + (i * 4));     // Copy first two bytes (cmd)
        std::copy(pec, pec + 2, cmd_pec + (i * 4) + 2); // Copy next two bytes (pec)
    }
#endif
    return cmd_pec; // Will be 4 bytes if _1, otherwise 24 bytes b/c 4 bytes x 6 ICs
}
