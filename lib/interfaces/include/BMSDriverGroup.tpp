/* Library Includes */
#include "Configuration.h"
#include "BMSDriverGroup.h"
#include "LTCSPIInterface.h"
#include <array>
#include <string>
#include <optional>

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
BMSDriverGroup<num_chips, num_chip_selects, chip_type>::BMSDriverGroup(std::array<int, num_chip_selects> cs, std::array<int, num_chips> cs_per_chip, std::array<int, num_chips> addr, const BMSDriverGroupConfig_s config) : _pec15Table(_initialize_Pec_Table()),
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
    // Trigger ADC conversions at the start of each complete 6-group read cycle
    // This ensures all groups (A, B, C, D, AUX_A, AUX_B) read from the same timestamp
    if (_current_read_group == CurrentReadGroup_e::CURRENT_GROUP_A)
    {
        _start_cell_voltage_ADC_conversion();
        _start_GPIO_ADC_conversion();
    }

    BMSDriverData bms_data;
    if constexpr (chip_type == LTC6811_Type_e::LTC6811_1)
    {
        bms_data = _read_data_through_broadcast();
    }
    else
    {
        bms_data = _read_data_through_address();
    }

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

        switch (_current_read_group) {
            case CurrentReadGroup_e::CURRENT_GROUP_A:
                cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_A, -1); // The address should never be used here
                spi_data = ltc_spi_interface::read_registers_command<data_size>(_chip_select[cs], cmd_pec);
                break;
            case CurrentReadGroup_e::CURRENT_GROUP_B:
                cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_B, -1);
                spi_data = ltc_spi_interface::read_registers_command<data_size>(_chip_select[cs], cmd_pec);
                break;
            case CurrentReadGroup_e::CURRENT_GROUP_C:
                cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_C, -1);
                spi_data = ltc_spi_interface::read_registers_command<data_size>(_chip_select[cs], cmd_pec);
                break;
            case CurrentReadGroup_e::CURRENT_GROUP_D:
                cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_D, -1);
                spi_data = ltc_spi_interface::read_registers_command<data_size>(_chip_select[cs], cmd_pec);
                break;
            case CurrentReadGroup_e::CURRENT_GROUP_AUX_A:
                cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_GPIO_VOLTAGE_GROUP_A, -1);
                spi_data = ltc_spi_interface::read_registers_command<data_size>(_chip_select[cs], cmd_pec);
                break;
            case CurrentReadGroup_e::CURRENT_GROUP_AUX_B:
                cmd_pec = _generate_CMD_PEC(CMD_CODES_e::READ_GPIO_VOLTAGE_GROUP_B, -1);
                spi_data = ltc_spi_interface::read_registers_command<data_size>(_chip_select[cs], cmd_pec);
                break;
        }

        
        for (size_t chip = 0; chip < num_chips / num_chip_selects; chip++) {
            size_t chip_index = chip + (cs * (num_chips / num_chip_selects));

            // relevant for cell voltage reading
            int cell_count = (chip_index % 2 == 0) ? 12 : 9; // Even indexed ICs have 12 cells, odd have 9

            std::array<uint8_t, 2> data_in_cell_voltage;
            std::array<uint8_t, 2> data_in_gpio_voltage;

            uint8_t start_cell_index = 0;
            uint8_t start_gpio_index = 0;
            std::array<uint8_t, 6> spi_response;

            //relevant for GPIO reading
            bool current_group_valid = false;
            switch(_current_read_group) {
                case CurrentReadGroup_e::CURRENT_GROUP_A:
                    current_group_valid = _check_if_valid_packet(spi_data, 8 * chip);
                    _bms_data.valid_read_packets[chip_index].valid_read_cells_1_to_3 = current_group_valid;
                    start_cell_index = 0;
                    break;
                case CurrentReadGroup_e::CURRENT_GROUP_B:
                    current_group_valid = _check_if_valid_packet(spi_data, 8 * chip);
                    _bms_data.valid_read_packets[chip_index].valid_read_cells_4_to_6 = current_group_valid;
                    start_cell_index = 3;
                    break;
                case CurrentReadGroup_e::CURRENT_GROUP_C:
                    current_group_valid = _check_if_valid_packet(spi_data, 8 * chip);
                    _bms_data.valid_read_packets[chip_index].valid_read_cells_7_to_9 = current_group_valid;
                    start_cell_index = 6;
                    break;
                case CurrentReadGroup_e::CURRENT_GROUP_D:
                    current_group_valid = _check_if_valid_packet(spi_data, 8 * chip);
                    _bms_data.valid_read_packets[chip_index].valid_read_cells_10_to_12 = current_group_valid;
                    start_cell_index = 9;
                    break;
                case CurrentReadGroup_e::CURRENT_GROUP_AUX_A:
                    current_group_valid = _check_if_valid_packet(spi_data, 8 * chip);
                    _bms_data.valid_read_packets[chip_index].valid_read_gpios_1_to_3 = current_group_valid;
                    start_gpio_index = 0;
                    break;
                case CurrentReadGroup_e::CURRENT_GROUP_AUX_B:
                    current_group_valid = _check_if_valid_packet(spi_data, 8 * chip);
                    _bms_data.valid_read_packets[chip_index].valid_read_gpios_4_to_6 = current_group_valid;
                    start_gpio_index = 3;
                    break;
            }

            // Skip processing if current group packet is invalid
            if (!current_group_valid)
            {
                continue;
            }

            // don't do calculation for cells that don't exist. not putting this above so that package is declared as valid
            if (_current_read_group == CurrentReadGroup_e::CURRENT_GROUP_D && cell_count == 9) {
                continue;
            }

            if (_current_read_group == CurrentReadGroup_e::CURRENT_GROUP_AUX_B) {
                    std::copy_n(spi_data.begin() + (8 * chip), 4, spi_response.begin());
                    std::fill(spi_response.begin() + 4, spi_response.end(), 0); // padding to make it 6 bytes
            } else {
                std::copy_n(spi_data.begin() + (8 * chip), 6, spi_response.begin());
            }

            if (_current_read_group <= CurrentReadGroup_e::CURRENT_GROUP_D) {
                _bms_data = _load_cell_voltages(_bms_data, max_min_reference, spi_response, chip_index, battery_cell_count, start_cell_index);
            } else {
                _bms_data = _load_auxillaries(_bms_data, max_min_reference, spi_response, chip_index, gpio_count, start_gpio_index);
            }
        }
    }

    _bms_data.min_cell_voltage = max_min_reference.min_cell_voltage;
    _bms_data.max_cell_voltage = max_min_reference.max_cell_voltage;
    _bms_data.total_voltage = _sum_cell_voltages();
    
    // Avoid divide by zero - skip calculation if no GPIOs were read
    if (gpio_count > 0) {
        _bms_data.average_cell_temperature = max_min_reference.total_thermistor_temps / gpio_count;
    }

    _bms_data.max_cell_temp = _bms_data.cell_temperatures[_bms_data.max_cell_temperature_cell_id];
    _bms_data.min_cell_temp = _bms_data.cell_temperatures[_bms_data.min_cell_temperature_cell_id];
    _bms_data.max_board_temp = _bms_data.board_temperatures[_bms_data.max_board_temperature_segment_id];

    _current_read_group = advance_read_group(_current_read_group);
    return _bms_data;
}

/* UNUSED: LTC6811-2 ADDRESS MODE - REFERENCE ONLY
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

    // Avoid divide by zero - skip calculation if no GPIOs were read
    if (gpio_count > 0) {
        _bms_data.average_cell_temperature = max_min_reference.total_thermistor_temps / gpio_count;
    }

    _bms_data.max_cell_temp = _bms_data.cell_temperatures[_bms_data.max_cell_temperature_cell_id];
    _bms_data.max_board_temp = _bms_data.board_temperatures[_bms_data.max_board_temperature_segment_id];
    return _bms_data;
}
*/

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
typename BMSDriverGroup<num_chips, num_chip_selects, chip_type>::BMSDriverData
BMSDriverGroup<num_chips, num_chip_selects, chip_type>::_load_cell_voltages(BMSDriverData bms_data, ReferenceMaxMin &max_min_ref, const std::array<uint8_t, 6> &data_in_cv_group,
                                                                            size_t chip_index, size_t &battery_cell_count, uint8_t start_cell_index)
{
    // remove validity check. if invalid - won't get to here     
    // remove 9 cell check, same reason

    std::array<uint8_t, 2> data_in_cell_voltage;

    for (int cell_Index = start_cell_index; cell_Index < start_cell_index+3; cell_Index++)
    {
        // not checking for invalidity again

        std::copy_n(data_in_cv_group.begin() + (cell_Index - start_cell_index) * 2, 2, data_in_cell_voltage.begin());

        uint16_t voltage_in = data_in_cell_voltage[1] << 8 | data_in_cell_voltage[0];

        float voltage_converted = voltage_in / 10000.0;
        bms_data.voltages[battery_cell_count] = voltage_converted;
        _store_voltage_data(bms_data, max_min_ref, voltage_converted, battery_cell_count);

        battery_cell_count++;
    }
    std::copy_n(data_in_cv_group.begin(), 6, bms_data.voltages_by_chip[chip_index].begin() + start_cell_index * 2);
    return bms_data;
}

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
typename BMSDriverGroup<num_chips, num_chip_selects, chip_type>::BMSDriverData
BMSDriverGroup<num_chips, num_chip_selects, chip_type>::_load_auxillaries(BMSDriverData bms_data, ReferenceMaxMin &max_min_ref, const std::array<uint8_t, 6> &data_in_gpio_group,
                                                                            size_t chip_index, size_t &gpio_count, uint8_t start_gpio_index)
{
    // invalid packets handled beforehand

    for (int gpio_Index = start_gpio_index; gpio_Index < start_gpio_index + 3 && gpio_Index != 5; gpio_Index++) // There are only five Auxillary ports
    {

        std::array<uint8_t, 2> data_in_gpio_voltage;
        std::copy_n(data_in_gpio_group.begin() + (gpio_Index - start_gpio_index) * 2, 2, data_in_gpio_voltage.begin());

        uint16_t gpio_in = data_in_gpio_voltage[1] << 8 | data_in_gpio_voltage[0];
        _store_temperature_humidity_data(bms_data, max_min_ref, gpio_in, gpio_Index, gpio_count, chip_index);
        if (gpio_Index < 4) {
            gpio_count++; // we are using gpio count for calculation of average cell temperature, 
            // however on each chip we have 4 cell temps + 1 board temp, which doesn't need to go into calc
            // so we're ignoring counter increment for board temp
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
void BMSDriverGroup<num_chips, num_chip_selects, chip_type>::_store_temperature_humidity_data(BMSDriverData &bms_data, ReferenceMaxMin &max_min_reference, const uint16_t &gpio_in, size_t gpio_Index, size_t &gpio_count, size_t chip_num)
{
    // there is 8 cell temperatures per chip, and 2 board temperatures per board, so 4+1 per chip
    if (gpio_Index < 4) // These are all thermistors [0,1,2,3]. 
    {
        float thermistor_resistance = (2740 / (gpio_in / 50000.0)) - 2740;
        bms_data.cell_temperatures[gpio_count] = 1 / ((1 / 298.15) + (1 / 3984.0) * std::log(thermistor_resistance / 10000.0)) - 272.15; // calculation for thermistor temperature in C
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
    else // this is apparently the case for temperature sensor for the BOARD, not the cells. There is 2 per segment
    {
        constexpr float mcp_9701_temperature_coefficient = 0.0195f;
        constexpr float mcp_9701_output_v_at_0c = 0.4f;
        bms_data.board_temperatures[chip_num] = ((gpio_in / 10000.0f) - mcp_9701_output_v_at_0c) / mcp_9701_temperature_coefficient; // 2 per board = 1 per chip
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
        for (size_t i = num_chips - 1; i >= 0; i--)              // This needs to be flipped because when writing a command, primary device holds the last bytes
        {                                                     // Find chips with the same CS
            if (_chip_select_per_chip[i] == _chip_select[cs]) // This could be an optimization:  && j < (num_chips + 1) / 2)
            {
                buffer_format[4] = ((cell_balance_statuses[i] & 0x0FF));
                buffer_format[5] = ((dcto_mode & 0x0F) << 4) | ((cell_balance_statuses[i] & 0xF00) >> 8);
                temp_pec = _calculate_specific_PEC(buffer_format.data(), 6);
                std::copy_n(buffer_format.begin(), 6, full_buffer.data() + (j * 8));
                std::copy_n(temp_pec.begin(), 2, full_buffer.data() + 6 + (j * 8));
                j++;
            }
        }
        ltc_spi_interface::write_registers_command<data_size>(_chip_select[cs], cmd_and_pec, full_buffer);
    }
}

/* UNUSED: LTC6811-2 ADDRESS MODE - REFERENCE ONLY
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
*/

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
    std::array<uint8_t, 2> cc = {cmd_code[0], cmd_code[1]};
    std::array<uint8_t, 2> pec = _calculate_specific_PEC(cc.data(), 2);
    std::array<uint8_t, 4> cmd_and_pec;
    std::copy_n(cmd_code.begin(), 2, cmd_and_pec.begin()); // Copy first two bytes (cmd)
    std::copy_n(pec.begin(), 2, cmd_and_pec.begin() + 2);  // Copy next two bytes (pec)

    // Needs to be sent on each chip select line
    for (size_t cs = 0; cs < num_chip_selects; cs++) {
        _start_wakeup_protocol(cs);
        ltc_spi_interface::adc_conversion_command(_chip_select[cs], cmd_and_pec, (num_chips / num_chip_selects));
    }
}

/* UNUSED: LTC6811-2 ADDRESS MODE - REFERENCE ONLY
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
*/

/* -------------------- GETTER FUNCTIONS -------------------- */

// This implementation is taken directly from the data sheet linked here: https://www.analog.com/media/en/technical-documentation/data-sheets/LTC6811-1-6811-2.pdf
template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
std::array<uint8_t, 2> BMSDriverGroup<num_chips, num_chip_selects, chip_type>::_calculate_specific_PEC(const uint8_t *data, int length)
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
    const uint16_t cmd_val = static_cast<uint16_t>(command);

    if constexpr (chip_type == LTC6811_Type_e::LTC6811_1)
    {
        cmd[0] = static_cast<uint8_t>(cmd_val >> 8);
        cmd[1] = static_cast<uint8_t>(cmd_val);
    }
    else
    {
        cmd[0] = static_cast<uint8_t>(_get_cmd_address(_address[ic_index]) | (cmd_val >> 8));
        cmd[1] = static_cast<uint8_t>(cmd_val);
    }
    return cmd;
}

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
std::array<uint8_t, 4> BMSDriverGroup<num_chips, num_chip_selects, chip_type>::_generate_CMD_PEC(CMD_CODES_e command, int ic_index)
{
    std::array<uint8_t, 4> cmd_pec;
    std::array<uint8_t, 2> cmd = _generate_formatted_CMD(command, ic_index);
    std::array<uint8_t, 2> pec = _calculate_specific_PEC(cmd.data(), 2);
    std::copy_n(cmd.data(), 2, cmd_pec.data());     // Copy first two bytes (cmd)
    std::copy_n(pec.data(), 2, cmd_pec.data() + 2); // Copy next two bytes (pec)
    return cmd_pec;
}



template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
bool BMSDriverGroup<num_chips, num_chip_selects, chip_type>::_check_if_valid_packet(const std::array<uint8_t, 8 * (num_chips / num_chip_selects)> &data, size_t param_iterator)
{
    std::array<uint8_t, 6> sample_packet;
    std::array<uint8_t, 2> sample_pec;
    for (int packet = 0; packet < 6; packet++)
    {
        sample_packet[packet] = data[param_iterator + packet];
    }
    for (int packet = 0; packet < 2; packet++)
    {
        sample_pec[packet] = data[param_iterator + packet + 6];
    }
    std::array<uint8_t, 2> calculated_pec = _calculate_specific_PEC(sample_packet.data(), 6);

    return calculated_pec[0] == sample_pec[0] && calculated_pec[1] == sample_pec[1];
}

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
bool BMSDriverGroup<num_chips, num_chip_selects, chip_type>::_check_if_all_valid(size_t chip_index)
{
    ValidPacketData_s data = _bms_data.valid_read_packets[chip_index];
    return data.valid_read_cells_1_to_3 && data.valid_read_cells_4_to_6 && data.valid_read_cells_7_to_9 && data.valid_read_cells_10_to_12 && data.valid_read_gpios_1_to_3 && data.valid_read_gpios_4_to_6;
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