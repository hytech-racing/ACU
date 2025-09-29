/* Library Includes */
#include "Configuration.h"
#include "BMSDriverGroup.h"
#include "LTCSPIInterface.h"
#include <string.h>
#include <stdio.h>
#include <array>
#include <string>
#include <optional>

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
BMSDriverGroup<num_chips, num_chip_selects, chip_type>::BMSDriverGroup(std::array<int, num_chip_selects> cs, std::array<int, num_chips> cs_per_chip, std::array<int, num_chips> addr, const BMSDriverGroupConfig_s config = {}) : _pec15Table(_initialize_Pec_Table()),
                                                                                                                                                                                        _chip_select(cs),
                                                                                                                                                                                        _chip_select_per_chip(cs_per_chip),
                                                                                                                                                                                        _address(addr),
                                                                                                                                                                                        _config(config) {};

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
void BMSDriverGroup<num_chips, num_chip_selects, chip_type>::init()
{
    // We initialized the pec table during beginning of runtime which allows _pec15table to be const -> no need to call in init()
    for (size_t i = 0; i < num_chip_selects; i++)
    {
        int cs = _chip_select[i];
        // chip select defines
        pinMode(cs, OUTPUT);
        digitalWrite(cs, HIGH);
    }
}

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
void BMSDriverGroup<num_chips, num_chip_selects, chip_type>::_start_wakeup_protocol()
{
    for (size_t cs = 0; cs < num_chip_selects; cs++)
    {
        _start_wakeup_protocol(cs);
    }
}

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
void BMSDriverGroup<num_chips, num_chip_selects, chip_type>::_start_wakeup_protocol(size_t cs)
{
    if constexpr (chip_type == LTC6811_Type_e::LTC6811_1)
    {
        ltc_spi_interface::_write_and_delay_low(_chip_select[cs], 400);
        SPI.transfer16(0);
        ltc_spi_interface::_write_and_delay_high(_chip_select[cs], 400);
    }
    else
    {
        ltc_spi_interface::_write_and_delay_low(_chip_select[cs], 400);
        SPI.transfer(0);
        ltc_spi_interface::_write_and_delay_high(_chip_select[cs], 400); // t_wake is 400 microseconds; wait that long to ensure device has turned on.
    }
}

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
constexpr std::array<uint16_t, 256> BMSDriverGroup<num_chips, num_chip_selects, chip_type>::_initialize_Pec_Table()
{
    std::array<uint16_t, 256> temp{};
    // Logic to fill temp
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
        temp[i] = remainder & 0xFFFF;
    }
    return temp;
}

/* -------------------- READING DATA FUNCTIONS -------------------- */

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
typename BMSDriverGroup<num_chips, num_chip_selects, chip_type>::BMSDriverData
BMSDriverGroup<num_chips, num_chip_selects, chip_type>::read_data()
{
    BMSDriverData bms_data;
    if constexpr (chip_type == LTC6811_Type_e::LTC6811_1)
    {
        bms_data = _read_data_through_broadcast();
    }
    else
    {
        bms_data = _read_data_through_address();
    }

    _start_cell_voltage_ADC_conversion(); // Gets the ICs ready to be read, must delay afterwards by ? us
    _start_GPIO_ADC_conversion();

    return bms_data;
}

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
typename BMSDriverGroup<num_chips, num_chip_selects, chip_type>::BMSDriverData
BMSDriverGroup<num_chips, num_chip_selects, chip_type>::_read_data_through_broadcast()
{
    ReferenceMaxMin max_min_reference;
    ValidPacketData_s clean_valid_packet_data;                  // should be all reset to true
    _bms_data.valid_read_packets.fill(clean_valid_packet_data); // reset
    constexpr size_t data_size = 8 * (num_chips / num_chip_selects);
    size_t battery_cell_count = 0;
    size_t gpio_count = 0;
    for (size_t cs = 0; cs < num_chip_selects; cs++)
    {
        write_configuration(_config.dcto_read, _cell_discharge_en);
    
        std::array<uint8_t, 4> cmd_pec;
        std::array<uint8_t, data_size> spi_data;

        // Get buffers for each group we care about, all at once for ONE chip select line
        _start_wakeup_protocol(cs);

        switch (this->current_read_group) {
            case CurrentGroup_e::CURRENT_GROUP_A:
                cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_A, -1); // The address should never be used here
                spi_data = ltc_spi_interface::read_registers_command<data_size>(_chip_select[cs], cmd_pec);
                break;
            case CurrentGroup_e::CURRENT_GROUP_B:
                cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_B, -1);
                spi_data = ltc_spi_interface::read_registers_command<data_size>(_chip_select[cs], cmd_pec);
                break;
            case CurrentGroup_e::CURRENT_GROUP_C:
                cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_C, -1);
                spi_data = ltc_spi_interface::read_registers_command<data_size>(_chip_select[cs], cmd_pec);
                break;
            case CurrentGroup_e::CURRENT_GROUP_D:
                cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_D, -1);
                spi_data = ltc_spi_interface::read_registers_command<data_size>(_chip_select[cs], cmd_pec);
                break;
            case CurrentGroup_e::CURRENT_GROUP_AUX_A:
                cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_GPIO_VOLTAGE_GROUP_A, -1);
                spi_data = ltc_spi_interface::read_registers_command<data_size>(_chip_select[cs], cmd_pec);
                break;
            case CurrentGroup_e::CURRENT_GROUP_AUX_B:
                cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_GPIO_VOLTAGE_GROUP_B, -1);
                spi_data = ltc_spi_interface::read_registers_command<data_size>(_chip_select[cs], cmd_pec);
                break;
        }

        // WHAT HAPPENS FOR EACH CHIP:
        // 1) Packaging: copy it to combined
       // 2) Set valid_read_packetts for specific chip for cell group 
       // 3) Packaging returns all cell_voltages 
    // 4) load cell takes specific chip 
    // it uses valid_read_packets from its index 
    // check invalidity and skip if detected (the chip assignment)
    // copies to imd voltage by chip
        for (size_t chip = 0; chip < num_chips / num_chip_selects; chip++) {
            size_t chip_index = chip + (cs * (num_chips / num_chip_selects));

            // relevant for cell voltage reading
            int cell_count = (chip_index % 2 == 0) ? 12 : 9; // Even indexed ICs have 12 cells, odd have 9

            std::array<uint8_t, 2> data_in_cell_voltage;
            std::array<uint8_t, 2> data_in_gpio_voltage;

            uint8_t start_cell_index, end_cell_index;
            bool valid_data_packet;

            //relevant for GPIO reading


            switch(this->current_read_group) {
                case CurrentGroup_e::CURRENT_GROUP_A:
                    _bms_data.valid_read_packets[chip_index].valid_read_cells_1_to_3 = valid_data_packet = _check_if_valid_packet(spi_data, 8 * chip);
                    start_cell_index = 0;
                    end_cell_index = (cell_count >= 3) ? 3 : cell_count;
                    break;
                case CurrentGroup_e::CURRENT_GROUP_B:
                    _bms_data.valid_read_packets[chip_index].valid_read_cells_4_to_6 = valid_data_packet = _check_if_valid_packet(spi_data, 8 * chip);
                    start_cell_index = 3;
                    end_cell_index = 6;
                    break;
                case CurrentGroup_e::CURRENT_GROUP_C:
                    _bms_data.valid_read_packets[chip_index].valid_read_cells_7_to_9 = valid_data_packet = _check_if_valid_packet(spi_data, 8 * chip);
                    start_cell_index = 6;
                    end_cell_index = 9;
                    break;
                case CurrentGroup_e::CURRENT_GROUP_D:
                    _bms_data.valid_read_packets[chip_index].valid_read_cells_10_to_12 = valid_data_packet = _check_if_valid_packet(spi_data, 8 * chip);
                    start_cell_index = 9;
                    end_cell_index = (cell_count >= 12) ? 12 : cell_count;
                    break;
                case CurrentGroup_e::CURRENT_GROUP_AUX_A:
                    _bms_data.valid_read_packets[chip_index].valid_read_gpios_1_to_3 = valid_data_packet = _check_if_valid_packet(spi_data, 8 * chip);
                    // valid_data_packet = _bms_data.valid_read_packets[chip_index].valid_read_gpios_1_to_3;
                    break;
                case CurrentGroup_e::CURRENT_GROUP_AUX_B:
                    _bms_data.valid_read_packets[chip_index].valid_read_gpios_4_to_6 = valid_data_packet = _check_if_valid_packet(spi_data, 8 * chip);
                    // valid_data_packet = _bms_data.valid_read_packets[chip_index].valid_read_gpios_4_to_6;
                    break;  
            }

            if (this->current_read_group <= CurrentGroup_e::CURRENT_GROUP_D) {
                std::copy(spi_data.begin() + (8 * chip), spi_data.begin() + (8 * chip) + 6, this->cell_voltages_1_12_buffer[chip].begin() + start_cell_index * 2);
            } else {
                if (this->current_read_group == CurrentGroup_e::CURRENT_GROUP_AUX_A) {
                    std::copy(spi_data.begin() + (8 * chip), spi_data.begin() + (8 * chip) + 6, this->auxillary_1_5_buffer[chip_index].begin());
                } else if (this->current_read_group == CurrentGroup_e::CURRENT_GROUP_AUX_B) {
                    std::copy(spi_data.begin() + (8 * chip), spi_data.begin() + (8 * chip) + 4, this->auxillary_1_5_buffer[chip_index].begin() + 6);
                }
            }
        }
        
        // store the data for all chips on a chip select into one array, no PEC included


        // DEBUG: Check to see that the PEC is what we expect it to be
        if (this->current_read_group == CurrentGroup_e::CURRENT_GROUP_D) {
            for (size_t chip = 0; chip < num_chips / num_chip_selects; chip++)
            {
                size_t chip_index = chip + (cs * (num_chips / num_chip_selects));


                _bms_data.valid_read_packets[chip_index].all_invalid_reads = _check_if_all_invalid(chip_index);
                // if (_check_if_all_invalid(chip_index))
                // {
                //     continue;
                // }

                // load cell voltages function
                std::array<uint8_t, 24> cv_1_to_12_for_one_chip;
                auto start = this->cell_voltages_1_12_buffer[chip].begin();
                auto end = start + 24;
                std::copy(start, end, cv_1_to_12_for_one_chip.begin());
                _bms_data = _load_cell_voltages(_bms_data, max_min_reference, cv_1_to_12_for_one_chip, chip_index, battery_cell_count);
            }
        } 
        if (this->current_read_group == CurrentGroup_e::CURRENT_GROUP_AUX_B) {
            for (size_t chip = 0; chip < num_chips / num_chip_selects; chip++) {
                size_t chip_index = chip + (cs * (num_chips / num_chip_selects));
                // load humidity / temperatures function
                std::array<uint8_t, 10> gpio_1_to_5_for_one_chip;
                start = data_in_temps_1_to_5.begin() + (chip * 10);
                end = start + 10;
                std::copy(start, end, gpio_1_to_5_for_one_chip.begin());
                _bms_data = _load_auxillaries(_bms_data, max_min_reference, gpio_1_to_5_for_one_chip, chip_index, gpio_count);
            }
        }
    }


        
    
    _bms_data.min_cell_voltage = max_min_reference.min_cell_voltage;
    _bms_data.max_cell_voltage = max_min_reference.max_cell_voltage;
    _bms_data.total_voltage = _sum_cell_voltages();
    _bms_data.average_cell_temperature = max_min_reference.total_thermistor_temps / gpio_count;
    _bms_data.max_cell_temp = _bms_data.cell_temperatures[_bms_data.max_cell_temperature_cell_id];
    _bms_data.min_cell_temp = _bms_data.cell_temperatures[_bms_data.min_cell_temperature_cell_id];
    _bms_data.max_board_temp = _bms_data.board_temperatures[_bms_data.max_board_temperature_segment_id];
    return _bms_data;
}

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
typename BMSDriverGroup<num_chips, num_chip_selects, chip_type>::BMSDriverData
BMSDriverGroup<num_chips, num_chip_selects, chip_type>::_read_data_through_address()
{
    ReferenceMaxMin max_min_reference;
    ValidPacketData_s clean_valid_packet_data;                  // should be all reset to true
    _bms_data.valid_read_packets.fill(clean_valid_packet_data); // reset
    std::array<uint8_t, 24> data_in_cell_voltages_1_to_12;
    std::array<uint8_t, 10> data_in_auxillaries_1_to_5;
    std::array<uint8_t, 4> cmd_pec;
    size_t battery_cell_count = 0;
    size_t gpio_count = 0;
    for (size_t chip = 0; chip < num_chips; chip++)
    {
        _start_wakeup_protocol();

        cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_A, chip);
        auto data_in_3_cell_voltages = ltc_spi_interface::read_registers_command<8>(_chip_select_per_chip[chip], cmd_pec);
        std::copy(data_in_3_cell_voltages.begin(), data_in_3_cell_voltages.begin() + 6, data_in_cell_voltages_1_to_12.begin());

        cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_B, chip);
        data_in_3_cell_voltages = ltc_spi_interface::read_registers_command<8>(_chip_select_per_chip[chip], cmd_pec);
        std::copy(data_in_3_cell_voltages.begin(), data_in_3_cell_voltages.begin() + 6, data_in_cell_voltages_1_to_12.begin() + 6);

        cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_C, chip);
        data_in_3_cell_voltages = ltc_spi_interface::read_registers_command<8>(_chip_select_per_chip[chip], cmd_pec);
        std::copy(data_in_3_cell_voltages.begin(), data_in_3_cell_voltages.begin() + 6, data_in_cell_voltages_1_to_12.begin() + 12);

        cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_D, chip);
        data_in_3_cell_voltages = ltc_spi_interface::read_registers_command<8>(_chip_select_per_chip[chip], cmd_pec);
        std::copy(data_in_3_cell_voltages.begin(), data_in_3_cell_voltages.begin() + 6, data_in_cell_voltages_1_to_12.begin() + 18);

        cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_GPIO_VOLTAGE_GROUP_A, chip);
        auto data_in_3_auxillaries = ltc_spi_interface::read_registers_command<8>(_chip_select_per_chip[chip], cmd_pec);
        std::copy(data_in_3_auxillaries.begin(), data_in_3_auxillaries.begin() + 6, data_in_auxillaries_1_to_5.begin());

        cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_GPIO_VOLTAGE_GROUP_B, chip);
        data_in_3_auxillaries = ltc_spi_interface::read_registers_command<8>(_chip_select_per_chip[chip], cmd_pec);
        std::copy(data_in_3_auxillaries.begin(), data_in_3_auxillaries.begin() + 4, data_in_auxillaries_1_to_5.begin() + 6);

        // DEBUG: Check to see that the PEC is what we expect it to be

        _bms_data = _load_cell_voltages(_bms_data, max_min_reference, data_in_cell_voltages_1_to_12, chip, battery_cell_count);
        _bms_data = _load_auxillaries(_bms_data, max_min_reference, data_in_auxillaries_1_to_5, chip, gpio_count);
    }

    _bms_data.min_cell_voltage = max_min_reference.min_cell_voltage;
    _bms_data.max_cell_voltage = max_min_reference.max_cell_voltage;
    _bms_data.total_voltage = _sum_cell_voltages();
    _bms_data.average_cell_temperature = max_min_reference.total_thermistor_temps / gpio_count;
    _bms_data.max_cell_temp = _bms_data.cell_temperatures[_bms_data.max_cell_temperature_cell_id];
    _bms_data.max_board_temp = _bms_data.board_temperatures[_bms_data.max_board_temperature_segment_id];
    return _bms_data;
}

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
typename BMSDriverGroup<num_chips, num_chip_selects, chip_type>::BMSDriverData
BMSDriverGroup<num_chips, num_chip_selects, chip_type>::_load_cell_voltages(BMSDriverData bms_data, ReferenceMaxMin &max_min_ref, const std::array<uint8_t, 24> &data_in_cv_1_to_12,
                                                                            size_t chip_index, size_t &battery_cell_count)
{
    int cell_count = (chip_index % 2 == 0) ? 12 : 9; // Even indexed ICs have 12 cells, odd have 9
    std::array<volt, 12> chip_voltages_in;
    std::array<volt, 12> imd_chip_voltages_in;
    
    ValidPacketData_s packet_data = bms_data.valid_read_packets[chip_index];
    bool invalid_A = !packet_data.valid_read_cells_1_to_3;
    bool invalid_B = !packet_data.valid_read_cells_4_to_6;
    bool invalid_C = !packet_data.valid_read_cells_7_to_9;
    bool invalid_D = !packet_data.valid_read_cells_10_to_12;
    for (int cell_Index = 0; cell_Index < cell_count; cell_Index++)
    {
        std::array<uint8_t, 2> data_in_cell_voltage;
        auto start = data_in_cv_1_to_12.begin() + (cell_Index * 2);
        auto end = start + 2;
        std::copy(start, end, data_in_cell_voltage.begin());

        uint16_t voltage_in = data_in_cell_voltage[1] << 8 | data_in_cell_voltage[0];

        imd_chip_voltages_in[cell_Index] = voltage_in / 10000.0;
        bms_data.imd_voltages[battery_cell_count] = imd_chip_voltages_in[cell_Index];

        if (_check_specific_packet_group_is_invalid(cell_Index, invalid_A, invalid_B, invalid_C, invalid_D)) {
            battery_cell_count++;
            continue;
        }

        chip_voltages_in[cell_Index] = imd_chip_voltages_in[cell_Index];
        bms_data.voltages[battery_cell_count] = chip_voltages_in[cell_Index];
        // _store_voltage_data(bms_data, max_min_ref, chip_voltages_in, chip_voltages_in[cell_Index], battery_cell_count); since changed signature

        battery_cell_count++;
    }
    std::copy(chip_voltages_in.data(), chip_voltages_in.data() + cell_count, bms_data.voltages_by_chip[chip_index].data());
    std::copy(imd_chip_voltages_in.data(), imd_chip_voltages_in.data() + cell_count, bms_data.imd_voltages_by_chip[chip_index].data());

    return bms_data;
}

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
typename BMSDriverGroup<num_chips, num_chip_selects, chip_type>::BMSDriverData
BMSDriverGroup<num_chips, num_chip_selects, chip_type>::_load_auxillaries(BMSDriverData bms_data, ReferenceMaxMin &max_min_ref, const std::array<uint8_t, 10> &data_in_gpio_1_to_5,
                                                                          size_t chip_index, size_t &gpio_count)
{
    ValidPacketData_s packet_data = bms_data.valid_read_packets[chip_index];
    bool invalid_A = !packet_data.valid_read_gpios_1_to_3;
    bool invalid_B = !packet_data.valid_read_gpios_4_to_6;
    bool invalid_C = false;
    bool invalid_D = false;

    for (int gpio_Index = 0; gpio_Index < 5; gpio_Index++) // There are only five Auxillary ports
    {
        bool imd_only = true;
        std::array<uint8_t, 2> data_in_gpio_voltage;
        auto start = data_in_gpio_1_to_5.begin() + (gpio_Index * 2);
        auto end = start + 2;
        std::copy(start, end, data_in_gpio_voltage.begin());

        uint16_t gpio_in = data_in_gpio_voltage[1] << 8 | data_in_gpio_voltage[0];
        imd_only = _check_specific_packet_group_is_invalid(gpio_Index, invalid_A, invalid_B, invalid_C, invalid_D); // if any group invalid, we only save to imd data
        _store_temperature_humidity_data(bms_data, max_min_ref, gpio_in, gpio_Index, gpio_count, chip_index, imd_only);
        if (gpio_Index < 4) {
            gpio_count++; // TODO: Check with David why we have multiple gpio_count++'s, and why the different conditions
        }
    }
    return bms_data;
}

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
void BMSDriverGroup<num_chips, num_chip_selects, chip_type>::_store_voltage_data(BMSDriverData &bms_data, ReferenceMaxMin &max_min_reference, const float &voltage_in, size_t &cell_count)
{
    max_min_reference.total_voltage += voltage_in;
    if (voltage_in <= max_min_reference.min_cell_voltage)
    {
        max_min_reference.min_cell_voltage = voltage_in;
        bms_data.min_cell_voltage_id = cell_count;
    }
    if (voltage_in >= max_min_reference.max_cell_voltage)
    {
        max_min_reference.max_cell_voltage = voltage_in;
        bms_data.max_cell_voltage_id = cell_count;
    }
}

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
void BMSDriverGroup<num_chips, num_chip_selects, chip_type>::_store_temperature_humidity_data(BMSDriverData &bms_data, ReferenceMaxMin &max_min_reference, const uint16_t &gpio_in, size_t gpio_Index, size_t &gpio_count, size_t chip_num, bool imd_only)
{
    if (gpio_Index < 4) // These are all thermistors [0,1,2,3]
    {
        float thermistor_resistance = (2740 / (gpio_in / 50000.0)) - 2740;
        bms_data.imd_cell_temperatures[gpio_count] = 1 / ((1 / 298.15) + (1 / 3984.0) * log(thermistor_resistance / 10000.0)) - 272.15; // calculation for thermistor temperature in C
        if(imd_only){ // we do not want non-PEC check data to be saved
            return;
        }
        bms_data.cell_temperatures[gpio_count] = bms_data.imd_cell_temperatures[gpio_count];
        max_min_reference.total_thermistor_temps += bms_data.cell_temperatures[gpio_count];
        if (gpio_in > max_min_reference.max_cell_temp_voltage)
        {
            max_min_reference.max_cell_temp_voltage = gpio_in;
            bms_data.max_cell_temperature_cell_id = gpio_count;
        }
        if (gpio_in < max_min_reference.min_cell_temp_voltage)
        {
            max_min_reference.min_cell_temp_voltage = gpio_in;
            bms_data.min_cell_temperature_cell_id = gpio_count;
        }
    }
    else
    {
        constexpr float mcp_9701_temperature_coefficient = 0.0195f;
        constexpr float mcp_9701_output_v_at_0c = 0.4f;
        bms_data.imd_board_temperatures[chip_num] = ((gpio_in / 10000.0f) - mcp_9701_output_v_at_0c) / mcp_9701_temperature_coefficient;
        if(imd_only){ // we do not want non-PEC check data to be saved
            return;
        }
        bms_data.board_temperatures[chip_num] = bms_data.imd_board_temperatures[chip_num];
        // bms_data.board_temperatures[(chip_num +2)/2] = 0;
        if (gpio_in > max_min_reference.max_board_temp_voltage)
        {
            max_min_reference.max_board_temp_voltage = gpio_in;

            bms_data.max_board_temperature_segment_id = chip_num; // Because each segment only has 1 humidity and 1 board temp sensor
        }
    }
}

/* -------------------- WRITING DATA FUNCTIONS -------------------- */

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
void BMSDriverGroup<num_chips, num_chip_selects, chip_type>::write_configuration(uint8_t dcto_mode, const std::array<bool, num_cells> &cell_balance_statuses)
{
    std::array<uint16_t, num_chips> cb;
    size_t cell = 0;
    for (size_t chip = 0; chip < num_chips; chip++)
    {
        uint16_t chip_cb = 0;
        size_t cell_count = (chip % 2 == 0) ? 12 : 9;
        for (size_t cell_i = 0; cell_i < cell_count; cell_i++)
        {
            if (cell_balance_statuses[cell])
            {
                chip_cb = (0b1 << cell_i) | chip_cb;
            }
            cell++;
        }
        cb[chip] = chip_cb;
    }

    write_configuration(dcto_mode, cb);
}

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
void BMSDriverGroup<num_chips, num_chip_selects, chip_type>::write_configuration(uint8_t dcto_mode, const std::array<uint16_t, num_chips> &cell_balance_statuses)
{
    std::copy(cell_balance_statuses.begin(), cell_balance_statuses.end(), _cell_discharge_en.begin());

    std::array<uint8_t, 6> buffer_format; // This buffer processing can be seen in more detail on page 62 of the data sheet
    buffer_format[0] = (gpios_enabled << 3) | (static_cast<int>(device_refup_mode) << 2) | static_cast<int>(adcopt);
    buffer_format[1] = (under_voltage_threshold & 0x0FF);
    buffer_format[2] = ((over_voltage_threshold & 0x00F) << 4) | ((under_voltage_threshold & 0xF00) >> 8);
    buffer_format[3] = ((over_voltage_threshold & 0xFF0) >> 4);

    _start_wakeup_protocol();

    if constexpr (chip_type == LTC6811_Type_e::LTC6811_1)
    {
        _write_config_through_broadcast(dcto_mode, buffer_format, cell_balance_statuses);
    }
    else
    {
        _write_config_through_address(dcto_mode, buffer_format, cell_balance_statuses);
    }
}

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
void BMSDriverGroup<num_chips, num_chip_selects, chip_type>::_write_config_through_broadcast(uint8_t dcto_mode, std::array<uint8_t, 6> buffer_format, const std::array<uint16_t, num_chips> &cell_balance_statuses)
{
    constexpr size_t data_size = 8 * (num_chips / num_chip_selects);
    std::array<uint8_t, 4> cmd_and_pec = _generate_CMD_PEC(CMD_CODES_e::WRITE_CONFIG, -1);
    std::array<uint8_t, data_size> full_buffer;
    std::array<uint8_t, 2> temp_pec;

    // Needs to be sent on each chip select line
    for (size_t cs = 0; cs < num_chip_selects; cs++)
    {
        size_t j = 0;
        for (int i = num_chips - 1; i >= 0; i--)              // This needs to be flipped because when writing a command, primary device holds the last bytes
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
        ltc_spi_interface::write_registers_command<data_size>(_chip_select[cs], cmd_and_pec, full_buffer);
    }
}

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
void BMSDriverGroup<num_chips, num_chip_selects, chip_type>::_write_config_through_address(uint8_t dcto_mode, std::array<uint8_t, 6> buffer_format, const std::array<uint16_t, num_chips> &cell_balance_statuses)
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
        ltc_spi_interface::write_registers_command<8>(_chip_select_per_chip[i], cmd_and_pec, full_buffer);
    }
}

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
void BMSDriverGroup<num_chips, num_chip_selects, chip_type>::_start_cell_voltage_ADC_conversion()
{
    uint16_t adc_cmd = (uint16_t)CMD_CODES_e::START_CV_ADC_CONVERSION | (adc_mode_cv_conversion << 7) | (discharge_permitted << 4) | static_cast<uint8_t>(adc_conversion_cell_select_mode);
    std::array<uint8_t, 2> cmd;
    cmd[0] = (adc_cmd >> 8) & 0xFF;
    cmd[1] = adc_cmd & 0xFF;
    if constexpr (chip_type == LTC6811_Type_e::LTC6811_1)
    {
        _start_ADC_conversion_through_broadcast(cmd);
    }
    else
    {
        _start_ADC_conversion_through_address(cmd);
    }
}

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
void BMSDriverGroup<num_chips, num_chip_selects, chip_type>::_start_GPIO_ADC_conversion()
{
    uint16_t adc_cmd = (uint16_t)CMD_CODES_e::START_GPIO_ADC_CONVERSION | (adc_mode_gpio_conversion << 7); // | static_cast<uint8_t>(adc_conversion_gpio_select_mode);
    std::array<uint8_t, 2> cmd;
    cmd[0] = (adc_cmd >> 8) & 0xFF;
    cmd[1] = adc_cmd & 0xFF;

    if constexpr (chip_type == LTC6811_Type_e::LTC6811_1)
    {
        _start_ADC_conversion_through_broadcast(cmd);
    }
    else
    {
        _start_ADC_conversion_through_address(cmd);
    }
}

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
void BMSDriverGroup<num_chips, num_chip_selects, chip_type>::_start_ADC_conversion_through_broadcast(const std::array<uint8_t, 2> &cmd_code)
{
    // Leave the command code as is
    uint8_t cc[2] = {cmd_code[0], cmd_code[1]};
    std::array<uint8_t, 2> pec = _calculate_specific_PEC(cc, 2);
    std::array<uint8_t, 4> cmd_and_pec;
    std::copy(cmd_code.begin(), cmd_code.end(), cmd_and_pec.begin()); // Copy first two bytes (cmd)
    std::copy(pec.begin(), pec.end(), cmd_and_pec.begin() + 2);       // Copy next two bytes (pec)

    // Needs to be sent on each chip select line
    for (size_t cs = 0; cs < num_chip_selects; cs++) {
        _start_wakeup_protocol(cs);
        ltc_spi_interface::adc_conversion_command(_chip_select[cs], cmd_and_pec, (num_chips / num_chip_selects));
    }
}

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
void BMSDriverGroup<num_chips, num_chip_selects, chip_type>::_start_ADC_conversion_through_address(std::array<uint8_t, 2> cmd_code)
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
template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
std::array<uint8_t, 2> BMSDriverGroup<num_chips, num_chip_selects, chip_type>::_calculate_specific_PEC(uint8_t *data, int length)
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

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
std::array<uint8_t, 2> BMSDriverGroup<num_chips, num_chip_selects, chip_type>::_generate_formatted_CMD(CMD_CODES_e command, int ic_index)
{
    std::array<uint8_t, 2> cmd;

    if constexpr (chip_type == LTC6811_Type_e::LTC6811_1)
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

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
std::array<uint8_t, 4> BMSDriverGroup<num_chips, num_chip_selects, chip_type>::_generate_CMD_PEC(CMD_CODES_e command, int ic_index)
{
    std::array<uint8_t, 4> cmd_pec;
    std::array<uint8_t, 2> cmd = _generate_formatted_CMD(command, ic_index);
    std::array<uint8_t, 2> pec = _calculate_specific_PEC(cmd.data(), 2);
    std::copy(cmd.data(), cmd.data() + 2, cmd_pec.data());     // Copy first two bytes (cmd)
    std::copy(pec.data(), pec.data() + 2, cmd_pec.data() + 2); // Copy next two bytes (pec)
    return cmd_pec;
}



template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
bool BMSDriverGroup<num_chips, num_chip_selects, chip_type>::_check_if_valid_packet(const std::array<uint8_t, 8 * (num_chips / num_chip_selects)> &data, size_t param_iterator)
{
    uint8_t sample_packet[6];
    uint8_t sample_pec[2];
    for (int packet = 0; packet < 6; packet++)
    {
        sample_packet[packet] = data[param_iterator + packet];
    }
    for (int packet = 0; packet < 2; packet++)
    {
        sample_pec[packet] = data[param_iterator + packet + 6];
    }
    std::array<uint8_t, 2> calculated_pec = _calculate_specific_PEC(sample_packet, 6);

    return calculated_pec[0] == sample_pec[0] && calculated_pec[1] == sample_pec[1];
}

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
bool BMSDriverGroup<num_chips, num_chip_selects, chip_type>::_check_if_all_valid(size_t chip_index)
{
    ValidPacketData_s data = _bms_data.valid_read_packets[chip_index];
    return data.valid_read_cells_1_to_3 && data.valid_read_cells_4_to_6 && data.valid_read_cells_7_to_9 && data.valid_read_cells_10_to_12 && data.valid_read_gpios_1_to_3 && data.valid_read_gpios_4_to_6;
}

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
bool BMSDriverGroup<num_chips, num_chip_selects, chip_type>::_check_if_all_invalid(size_t chip_index)
{
    ValidPacketData_s data = _bms_data.valid_read_packets[chip_index];
    return !(data.valid_read_cells_1_to_3 || data.valid_read_cells_4_to_6 || data.valid_read_cells_7_to_9 || data.valid_read_cells_10_to_12 || data.valid_read_gpios_1_to_3 || data.valid_read_gpios_4_to_6);
}

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
volt BMSDriverGroup<num_chips, num_chip_selects, chip_type>::_sum_cell_voltages()
{
    volt sum = 0;
    for (volt v : _bms_data.voltages) {
        sum += v;
    }
    return sum;
}

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
bool BMSDriverGroup<num_chips, num_chip_selects, chip_type>::_check_specific_packet_group_is_invalid(size_t index, bool group_A_invalid, bool group_B_invalid, bool group_C_invalid, bool group_D_invalid) {
    if (index < 3) {
        return group_A_invalid;
    } else if (index < 6) {
        return group_B_invalid;
    } else if (index < 9) {
        return group_C_invalid;
    } else { // index < 12
        return group_D_invalid;
    }
}