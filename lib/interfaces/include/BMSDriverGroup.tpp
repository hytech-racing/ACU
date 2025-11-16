/* Library Includes */
#include "BMSDriverGroup.h"
#include "LTCSPIInterface.h"
#include <array>
#include <string>
#include <cmath>
#include <algorithm>
#include <optional>

template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::BMSDriverGroup(const ChipSelectConfig_t& chip_select_config = ACUConstants::BMS_CHIP_SELECTS,
                                                                        const BMSDriverGroupConfig_s default_params = {
                                                                            .device_refup_mode = bms_driver_defaults::DEVICE_REFUP_MODE,
                                                                            .adcopt = bms_driver_defaults::ADCOPT,
                                                                            .gpios_enabled = bms_driver_defaults::GPIOS_ENABLED,
                                                                            .dcto_read = bms_driver_defaults::DCTO_READ,
                                                                            .dcto_write = bms_driver_defaults::DCTO_WRITE,
                                                                            .adc_conversion_cell_select_mode = bms_driver_defaults::ADC_CONVERSION_CELL_SELECT_MODE,
                                                                            .adc_conversion_gpio_select_mode = bms_driver_defaults::ADC_CONVERSION_GPIO_SELECT_MODE,
                                                                            .discharge_permitted = bms_driver_defaults::DISCHARGE_PERMITTED,
                                                                            .adc_mode_cv_conversion = bms_driver_defaults::ADC_MODE_CV_CONVERSION,
                                                                            .adc_mode_gpio_conversion = bms_driver_defaults::ADC_MODE_GPIO_CONVERSION,
                                                                            .under_voltage_threshold = bms_driver_defaults::UNDER_VOLTAGE_THRESHOLD,
                                                                            .over_voltage_threshold = bms_driver_defaults::OVER_VOLTAGE_THRESHOLD,
                                                                            .gpio_enable = bms_driver_defaults::GPIO_ENABLE,
                                                                            .CRC15_POLY = bms_driver_defaults::CRC15_POLY,
                                                                            .cv_adc_conversion_time_ms = bms_driver_defaults::CV_ADC_CONVERSION_TIME_MS,
                                                                            .gpio_adc_conversion_time_ms = bms_driver_defaults::GPIO_ADC_CONVERSION_TIME_MS,
                                                                            .cv_adc_lsb_voltage = bms_driver_defaults::CV_ADC_LSB_VOLTAGE                                                                        
                                                                        }
                                                                ) : _chip_select_config(chip_select_config),
                                                                    _config(default_params),
                                                                    _pec15Table(_initialize_Pec_Table()) {}

template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
void BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::init()
{
    // We initialized the pec table during beginning of runtime which allows _pec15table to be const -> no need to call in init()
    for (const ChipSelect_t& cs : _chip_select_config.chip_selects)
    {
        pinMode(cs.cs_pin, OUTPUT);
        digitalWrite(cs.cs_pin, HIGH);
    }
    _bms_data.voltages.fill(0);
    _bms_data.cell_temperatures.fill(0);
    _bms_data.board_temperatures.fill(0);
    _bms_data.valid_read_packets.fill({false});
    _bms_data.total_voltage = 0;
    _max_min_reference = {
                            .total_voltage = ref_max_min_defaults::TOTAL_VOLTAGE,
                            .max_cell_voltage = ref_max_min_defaults::MAX_CELL_VOLTAGE,
                            .min_cell_voltage = ref_max_min_defaults::MIN_CELL_VOLTAGE,
                            .min_cell_temp = ref_max_min_defaults::MIN_CELL_TEMP,
                            .max_cell_temp = ref_max_min_defaults::MAX_CELL_TEMP,
                            .max_board_temp = ref_max_min_defaults::MAX_BOARD_TEMP,
                        };
}

template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
void BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::_start_wakeup_protocol()
{
    for (const ChipSelect_t& chip_select : _chip_select_config.chip_selects)
    {
        _start_wakeup_protocol(chip_select.cs_pin);
    }
}

template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
void BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::_start_wakeup_protocol(size_t cs_pin)
{
    if constexpr (chip_type == LTC6811_Type_e::LTC6811_1)
    {
        ltc_spi_interface::_write_and_delay_low(cs_pin, 400);
        SPI.transfer16(0);
        ltc_spi_interface::_write_and_delay_high(cs_pin, 400);
    }
    else
    {
        ltc_spi_interface::_write_and_delay_low(cs_pin, 400);
        SPI.transfer(0);
        ltc_spi_interface::_write_and_delay_high(cs_pin, 400); // t_wake is 400 microseconds; wait that long to ensure device has turned on.
    }
}

template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
constexpr std::array<uint16_t, 256> BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::_initialize_Pec_Table()
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
                remainder = (remainder ^ _config.CRC15_POLY);
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

template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
typename BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::BMSCoreData_t 
BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::get_bms_core_data()
    {
        BMSCoreData_t out{};

        // Basic voltages
        out.min_cell_voltage = _bms_data.min_cell_voltage;
        out.max_cell_voltage = _bms_data.max_cell_voltage;
        out.pack_voltage = _bms_data.total_voltage; 

        // Per-cell array (sizes must match)
        out.voltages = _bms_data.voltages;

        // Temps
        out.max_cell_temp  = _bms_data.max_cell_temp;
        out.min_cell_temp  = _bms_data.min_cell_temp;
        out.max_board_temp = _bms_data.max_board_temp;

        return out;
    }

template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
typename BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::BMSDriverData_t
BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::get_bms_data()
{
    return _bms_data;
}

template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
typename BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::BMSDriverData_t
BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::read_data()
{
    BMSDriverData_t bms_data;
    if constexpr (chip_type == LTC6811_Type_e::LTC6811_1)
    {
        bms_data = _read_data_through_broadcast();
    }
    else
    {
        bms_data = _read_data_through_address();
    }
    
    // Trigger ADC conversions at the start of each complete 6-group read cycle
    // This ensures all groups (A, B, C, D, AUX_A, AUX_B) read from the same timestamp
    if (_current_read_group == ReadGroup_e::AUX_GROUP_A)
    {
        _start_cell_voltage_ADC_conversion();
    }
    if (_current_read_group == ReadGroup_e::CV_GROUP_A)
    {
        _start_GPIO_ADC_conversion();
    }
    
    return bms_data;
}

template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
typename BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::BMSDriverData_t
BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::_read_data_through_broadcast()
{
    
    constexpr size_t chip_select_packet_size = _total_packet_size_bytes * num_chips_per_chip_select;
    size_t chip_start_voltage_cell_index = 0;
    size_t chip_start_temperature_cell_index = 0;
    size_t chip_start_board_temperature_index = 0;
    for (size_t chip_select_index = 0; chip_select_index < num_chip_selects; chip_select_index++)
    {
        const ChipSelect_t& chip_select = _chip_select_config.chip_selects[chip_select_index];
        
        write_configuration(_config.dcto_read, _cell_discharge_en);
    
        std::array<uint8_t, 4> cmd_pec;
        std::array<uint8_t, chip_select_packet_size> spi_data;

        // Get buffers for each group we care about, all at once for ONE chip select line
        _start_wakeup_protocol(chip_select.cs_pin);

        cmd_pec = _generate_CMD_PEC(_read_group_to_cmd[_current_read_group], -1); // The address should never be used here
        spi_data = ltc_spi_interface::read_registers_command<chip_select_packet_size>(chip_select.cs_pin, cmd_pec);

        for (size_t chip_index = 0; chip_index < num_chips_per_chip_select; chip_index++) {
            Chip_t chip = chip_select.chips[chip_index];
            size_t global_chip_index = _chip_select_config.global_chip_index(chip_select_index, chip_index);  

            uint8_t start_index;
            std::array<uint8_t, _total_packet_size_bytes> spi_response;

            //relevant for GPIO reading
            bool current_group_valid = false;
            current_group_valid = _check_if_valid_packet(spi_data, _total_packet_size_bytes * chip_index);
            _bms_data.valid_read_packets[chip_index][static_cast<size_t>(_current_read_group)] = current_group_valid;

            // Skip processing if current group packet is invalid and skip cells 9-12 for group D cuz they don't exist

            size_t chip_packet_num_values = chip.read_map.get_num_values_in_group(_current_read_group);
            size_t chip_packet_size_byte = _total_packet_size_bytes * chip_packet_num_values;

            // Skip processing if current group packet is invalid and updates indexes accordingly
            if (!current_group_valid) {
                continue;
            }


            std::copy_n(spi_data.begin() + (_total_packet_size_bytes * chip_index), chip_packet_size_byte, spi_response.begin());
            std::fill(spi_response.begin() + chip_packet_size_byte, spi_response.end(), 0); // padding

            std::array<uint8_t, size_of_packet_value_bytes> value_buffer;
            size_t start_voltage_index = chip_start_voltage_cell_index + chip.read_map.get_group_start_cell_voltage_index(_current_read_group);
            size_t start_temperature_index = chip_start_temperature_cell_index + chip.read_map.get_group_start_cell_temperature_index(_current_read_group);
            size_t start_board_temperature_index = chip_start_board_temperature_index + chip.read_map.get_group_start_board_temperature_index(_current_read_group);
            auto read_group_data_types = chip.read_map.group_data_types[_current_read_group];
            for (size_t i = 0; i < chip_packet_num_values; i++) {
                std::copy_n(spi_response.begin() + (i * size_of_packet_value_bytes), size_of_packet_value_bytes, value_buffer.begin());
                switch (read_group_data_types[i]){
                    case CELL_VOLTAGE:
                        _load_cell_voltages(_bms_data, _max_min_reference, value_buffer, start_voltage_index++);
                        break;
                    case CELL_TEMPERATURE:
                        _load_cell_temps(_bms_data, _max_min_reference, value_buffer, start_temperature_index++);
                        break;
                    case BOARD_TEMPERATURE:
                        _load_board_temps(_bms_data, _max_min_reference, value_buffer, start_board_temperature_index++);
                        break;
                }
            }
            chip_start_voltage_cell_index += chip.read_map.get_num_cell_voltages();
            chip_start_temperature_cell_index += chip.read_map.get_num_cell_temps();
            chip_start_board_temperature_index += chip.read_map.get_num_board_temps();
        }
    }

    _bms_data.total_voltage = _max_min_reference.total_voltage;
    _bms_data.avg_cell_voltage = _bms_data.total_voltage / num_voltage_cells;

    _bms_data.average_cell_temperature = _max_min_reference.total_thermistor_temps / num_temp_cells;

    _bms_data.max_cell_temp = _max_min_reference.max_cell_temp;
    _bms_data.min_cell_temp = _max_min_reference.min_cell_temp;
    _bms_data.max_board_temp = _max_min_reference.max_board_temp;

    _current_read_group = advance_read_group(_current_read_group);
    return _bms_data;
}

template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
typename BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::BMSDriverData_t
BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::_read_data_through_address()
{
    ReferenceMaxMin_s max_min_reference;
    std::array<bool, ReadGroup_e::NUM_GROUPS> clean_valid_packet_data = {true};                  // should be all reset to true
    _bms_data.valid_read_packets.fill(clean_valid_packet_data); // reset
    // std::array<uint8_t, 24> data_in_cell_voltages_1_to_12;
    std::array<uint8_t, ReadGroup_e::NUM_GROUPS * _total_packet_size_bytes> chip_data;
    // std::array<uint8_t, 10> data_in_auxillaries_1_to_5;
    std::array<uint8_t, 4> cmd_pec;
    size_t battery_cell_count = 0;
    size_t gpio_count = 0;

    size_t start_cell_voltage_index = 0;
    size_t start_cell_temp_index = 0;
    size_t start_board_temp_index = 0;
    size_t chip_data_index = 0;
    std::array<uint8_t, _total_packet_size_bytes> buffer;
    std::array<uint8_t, size_of_packet_value_bytes> value_buffer;
    for (const ChipSelect_t& chip_select : _chip_select_config.chip_selects){
        _start_wakeup_protocol(chip_select.cs_pin);
        for (const Chip_t& chip : chip_select.chips){
            size_t chip_packet_num_values = chip.read_map.get_num_values_in_each_group(_current_read_group);
            size_t chip_packet_size_byte = _total_packet_size_bytes * chip_packet_num_values;

            for (size_t group = 0; group < ReadGroup_e::NUM_GROUPS; group ++){
                cmd_pec = _generate_CMD_PEC(_read_group_to_cmd[group], chip);
                buffer = ltc_spi_interface::read_registers_command<_total_packet_size_bytes>(chip_select.cs_pin, cmd_pec);
                auto read_group_data_types = chip.read_map.group_data_types[group];
                for (size_t i = 0; i < chip_packet_num_values; i++){
                    std::copy_n(buffer.begin() + (i * size_of_packet_value_bytes), size_of_packet_value_bytes, value_buffer.begin());
                    switch (read_group_data_types[i]){
                        case CELL_VOLTAGE:
                            _load_cell_voltages(_bms_data, _max_min_reference, value_buffer, start_cell_voltage_index++);
                            break;
                        case CELL_TEMPERATURE:
                            _load_cell_temps(_bms_data, _max_min_reference, value_buffer, start_cell_temp_index++);
                            break;
                        case BOARD_TEMPERATURE:
                            _load_board_temps(_bms_data, _max_min_reference, value_buffer, start_board_temp_index++);
                            break;
                    }
                }
            }
        }
    }

    //Because increments everytime cell is read and all of them are read
    size_t total_num_voltage_cells = start_cell_voltage_index;
    size_t total_num_temp_cells = start_cell_temp_index;

    _bms_data.min_cell_voltage = _max_min_reference.min_cell_voltage;
    _bms_data.max_cell_voltage = _max_min_reference.max_cell_voltage;
    _bms_data.total_voltage = _max_min_reference.total_voltage;
    _bms_data.avg_cell_voltage = _bms_data.total_voltage / num_voltage_cells;

    _bms_data.average_cell_temperature = max_min_reference.total_thermistor_temps / total_num_temp_cells;

    _bms_data.max_cell_temp = _bms_data.cell_temperatures[_bms_data.max_cell_temperature_cell_id];
    _bms_data.max_board_temp = _bms_data.board_temperatures[_bms_data.max_board_temperature_segment_id];

    
    return _bms_data;
}

template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
void BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::_load_cell_voltages(BMSDriverData_t &bms_data, ReferenceMaxMin_s &max_min_ref, const std::array<uint8_t, size_of_packet_value_bytes> &data_in_cell_voltage, uint8_t cell_index)
{
        uint16_t voltage_in = data_in_cell_voltage[1] << 8 | data_in_cell_voltage[0];

        float voltage_converted = voltage_in * _config.cv_adc_lsb_voltage;

        _store_voltage_data(bms_data, max_min_ref, voltage_converted, cell_index);
}

template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
void BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::_load_cell_temps(BMSDriverData_t& bms_data, ReferenceMaxMin_s &max_min_ref, const std::array<uint8_t, size_of_packet_value_bytes> &data_in_temp,
                                                                            uint8_t cell_index)
{
        uint16_t temp_in = data_in_temp[1] << 8 | data_in_temp[0];
        _store_cell_temperature_data(bms_data, max_min_ref, temp_in, cell_index);
}

template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
void BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::_load_board_temps(BMSDriverData_t& bms_data, ReferenceMaxMin_s &max_min_ref, const std::array<uint8_t, size_of_packet_value_bytes> &data_in_temp,
                                                                            uint8_t cell_index)
{
        uint16_t temp_in = data_in_temp[1] << 8 | data_in_temp[0];
        _store_board_temperature_data(bms_data, max_min_ref, temp_in, cell_index);
}

template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
void BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::_store_voltage_data(BMSDriverData_t &bms_data, ReferenceMaxMin_s &max_min_reference, volt voltage_in, uint8_t cell_index)
{
    max_min_reference.total_voltage -= bms_data.voltages[cell_index];
    bms_data.voltages[cell_index] = voltage_in;
    max_min_reference.total_voltage += bms_data.voltages[cell_index];

    if (voltage_in <= max_min_reference.min_cell_voltage)
    {
        max_min_reference.min_cell_voltage = voltage_in;
        bms_data.min_cell_voltage_id = cell_index;
    }
    if (voltage_in >= max_min_reference.max_cell_voltage)
    {
        max_min_reference.max_cell_voltage = voltage_in;
        bms_data.max_cell_voltage_id = cell_index;
    }
}

template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
void BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::_store_cell_temperature_data(BMSDriverData_t &bms_data, ReferenceMaxMin_s &max_min_reference, const uint16_t &temp_in, uint8_t cell_index)
{

    max_min_reference.total_thermistor_temps -= bms_data.cell_temperatures[cell_index];
    float thermistor_resistance = (2740 / (temp_in / 50000.0)) - 2740;
    bms_data.cell_temperatures[cell_index] = 1 / ((1 / 298.15) + (1 / 3984.0) * std::log(thermistor_resistance / 10000.0)) - 272.15; // calculation for thermistor temperature in C
    max_min_reference.total_thermistor_temps += bms_data.cell_temperatures[cell_index];

    if (bms_data.cell_temperatures[cell_index] > max_min_reference.max_cell_temp)
    {
        max_min_reference.max_cell_temp = bms_data.cell_temperatures[cell_index];
        bms_data.max_cell_temperature_cell_id = cell_index;
    }
    if (bms_data.cell_temperatures[cell_index] < max_min_reference.min_cell_temp)
    {
        max_min_reference.min_cell_temp = bms_data.cell_temperatures[cell_index];
        bms_data.min_cell_temperature_cell_id = cell_index;
    }
}
template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
void BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::_store_board_temperature_data(BMSDriverData_t &bms_data, ReferenceMaxMin_s &max_min_reference, const uint16_t &temp_in, uint8_t board_index)
{
        constexpr float mcp_9701_temperature_coefficient = 0.0195f;
        constexpr float mcp_9701_output_v_at_0c = 0.4f;
        bms_data.board_temperatures[board_index] = ((temp_in / 10000.0f) - mcp_9701_output_v_at_0c) / mcp_9701_temperature_coefficient; // 2 per board = 1 per chip, calculation for bord temps
        if (bms_data.board_temperatures[board_index] > max_min_reference.max_board_temp)
        {
            max_min_reference.max_board_temp = bms_data.board_temperatures[board_index];

            bms_data.max_board_temperature_segment_id = board_index; // Because each segment only has 1 humidity and 1 board temp sensor
        }
}

/* -------------------- WRITING DATA FUNCTIONS -------------------- */

template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
void BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::write_configuration(const bool* cell_balance_statuses, size_t cell_balance_statuses_size)
{
    std::array<uint16_t, num_chip_selects * num_chips_per_chip_select> cb;
    size_t global_cell_index = 0;
    for (size_t chip_select_index = 0; chip_select_index < num_chip_selects; chip_select_index++){
        ChipSelect_t chip_select = _chip_select_config.chip_selects[chip_select_index];
        for (size_t chip_index = 0; chip_index < num_chips_per_chip_select; chip_index++){
            Chip_t chip = chip_select.chips[chip_index];
            uint16_t chip_cb = 0;
            size_t cells_per_chip = chip.read_map.get_num_cell_voltages();
            for (size_t cell_i = 0; cell_i < cells_per_chip; cell_i++)
            {
                if (cell_balance_statuses[global_cell_index])
                {
                    chip_cb = (0b1 << cell_i) | chip_cb;
                }
                global_cell_index++;
            }
            if (cell_balance_statuses_size > _chip_select_config.global_chip_index(chip_select_index, chip_index)){ //Redudanancy
                cb[_chip_select_config.global_chip_index(chip_select_index, chip_index)] = chip_cb;
            }
        }
    }
    write_configuration(_config.dcto_write, cb);
}

template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
void BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::write_configuration(uint8_t dcto_mode, const std::array<uint16_t, num_chip_selects * num_chips_per_chip_select> &cell_balance_statuses)
{
    std::copy(cell_balance_statuses.begin(), cell_balance_statuses.end(), _cell_discharge_en.begin());

    std::array<uint8_t, 6> buffer_format; // This buffer processing can be seen in more detail on page 62 of the data sheet
    buffer_format[0] = (_config.gpios_enabled << 3) | (static_cast<int>(_config.device_refup_mode) << 2) | static_cast<int>(_config.adcopt);
    buffer_format[1] = (_config.under_voltage_threshold & 0x0FF);
    buffer_format[2] = ((_config.over_voltage_threshold & 0x00F) << 4) | ((_config.under_voltage_threshold & 0xF00) >> 8);
    buffer_format[3] = ((_config.over_voltage_threshold & 0xFF0) >> 4);

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

template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
void BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::_write_config_through_broadcast(uint8_t dcto_mode, std::array<uint8_t, 6> buffer_format, const std::array<uint16_t, num_chip_selects * num_chips_per_chip_select> &cell_balance_statuses)
{
    constexpr size_t data_size = _total_packet_size_bytes * num_chips_per_chip_select;
    std::array<uint8_t, 4> cmd_and_pec = _generate_CMD_PEC(CMD_CODES_e::WRITE_CONFIG, -1);
    std::array<uint8_t, data_size> full_buffer;
    std::array<uint8_t, 2> temp_pec;

    // Needs to be sent on each chip select line
    for (size_t chip_select_index = 0; chip_select_index < num_chip_selects; chip_select_index++)
    {
        for (size_t chip_index = num_chips_per_chip_select - 1; chip_index >= 0; chip_index--)              // This needs to be flipped because when writing a command, primary device holds the last bytes
        {
            size_t global_chip_index = _chip_select_config.global_chip_index(chip_select_index, chip_index);
            buffer_format[4] = ((cell_balance_statuses[global_chip_index] & 0x0FF));
            buffer_format[5] = ((dcto_mode & 0x0F) << 4) | ((cell_balance_statuses[global_chip_index] & 0xF00) >> 8);
            temp_pec = _calculate_specific_PEC(buffer_format.data(), _total_packet_size_bytes);
            std::copy_n(buffer_format.begin(), _total_packet_size_bytes, full_buffer.data() + (chip_select_index * _total_packet_size_bytes));
            std::copy_n(temp_pec.begin(), _pec_size_bytes, full_buffer.data() + _total_packet_size_bytes + (chip_select_index * _total_packet_size_bytes));
        }
        ltc_spi_interface::write_registers_command<data_size>(_chip_select_config.chip_selects[chip_select_index].cs_pin, cmd_and_pec, full_buffer);
    }
}

/* UNUSED: LTC6811-2 ADDRESS MODE - REFERENCE ONLY
template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
void BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::_write_config_through_address(uint8_t dcto_mode, const std::array<uint8_t, 6>& buffer_format, const std::array<uint16_t, num_chip_selects * num_chips_per_chip_select> &cell_balance_statuses)
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

template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
void BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::_start_cell_voltage_ADC_conversion()
{
    uint16_t adc_cmd = (uint16_t)CMD_CODES_e::START_CV_ADC_CONVERSION | (_config.adc_mode_cv_conversion << 7) | (_config.discharge_permitted << 4) | static_cast<uint8_t>(_config.adc_conversion_cell_select_mode);
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

template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
void BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::_start_GPIO_ADC_conversion()
{
    uint16_t adc_cmd = (uint16_t)CMD_CODES_e::START_GPIO_ADC_CONVERSION | (_config.adc_mode_gpio_conversion << 7); // | static_cast<uint8_t>(_config.adc_conversion_gpio_select_mode);
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

template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
void BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::_start_ADC_conversion_through_broadcast(const std::array<uint8_t, 2> &cmd_code)
{
    // Leave the command code as is
    std::array<uint8_t, 2> cc = {cmd_code[0], cmd_code[1]};
    std::array<uint8_t, 2> pec = _calculate_specific_PEC(cc.data(), 2);
    std::array<uint8_t, 4> cmd_and_pec;
    std::copy_n(cmd_code.begin(), 2, cmd_and_pec.begin()); // Copy first two bytes (cmd)
    std::copy_n(pec.begin(), 2, cmd_and_pec.begin() + 2);  // Copy next two bytes (pec)

    // Needs to be sent on each chip select line
    for (const ChipSelect_t chip_select : _chip_select_config.chip_selects) {
        _start_wakeup_protocol(chip_select.cs_pin);
        ltc_spi_interface::adc_conversion_command(chip_select.cs_pin, cmd_and_pec, num_chips_per_chip_select);
    }
}

/* UNUSED: LTC6811-2 ADDRESS MODE - REFERENCE ONLY
template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
void BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::_start_ADC_conversion_through_address(const std::array<uint8_t, 2>& cmd_code)
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
template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
std::array<uint8_t, 2> BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::_calculate_specific_PEC(const uint8_t *data, int length)
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

template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
std::array<uint8_t, 2> BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::_generate_formatted_CMD(CMD_CODES_e command, size_t chip_addr)
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
        cmd[0] = static_cast<uint8_t>(_get_cmd_address(chip_addr) | (cmd_val >> 8));
        cmd[1] = static_cast<uint8_t>(cmd_val);
    }
    return cmd;
}

template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
std::array<uint8_t, 4> BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::_generate_CMD_PEC(CMD_CODES_e command, size_t chip_addr)
{
    std::array<uint8_t, 4> cmd_pec;
    std::array<uint8_t, 2> cmd = _generate_formatted_CMD(command, chip_addr);
    std::array<uint8_t, 2> pec = _calculate_specific_PEC(cmd.data(), 2);
    std::copy_n(cmd.data(), 2, cmd_pec.data());     // Copy first two bytes (cmd)
    std::copy_n(pec.data(), 2, cmd_pec.data() + 2); // Copy next two bytes (pec)
    return cmd_pec;
}



template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
bool BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::_check_if_valid_packet(const std::array<uint8_t, _total_packet_size_bytes * num_chips_per_chip_select> &data, size_t param_iterator)
{
    std::array<uint8_t, 6> sample_packet;
    std::array<uint8_t, 2> sample_pec;
    std::copy_n(data.begin() + param_iterator, 6, sample_packet.begin());
    std::copy_n(data.begin() + param_iterator + 6, 2, sample_pec.begin());
    std::array<uint8_t, 2> calculated_pec = _calculate_specific_PEC(sample_packet.data(), 6);

    return calculated_pec[0] == sample_pec[0] && calculated_pec[1] == sample_pec[1];
}

/* -------------------- OBSERVABILITY FUNCTIONS -------------------- */

template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
const char* BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::get_current_read_group_name()
{
    switch (_current_read_group) {
        case ReadGroup_e::CV_GROUP_A:
            return "GROUP_A";
        case ReadGroup_e::CV_GROUP_B:
            return "GROUP_B";
        case ReadGroup_e::CV_GROUP_C:
            return "GROUP_C";
        case ReadGroup_e::CV_GROUP_D:
            return "GROUP_D";
        case ReadGroup_e::AUX_GROUP_A:
            return "AUX_A";
        case ReadGroup_e::AUX_GROUP_B:
            return "AUX_B";
        default:
            return "UNKNOWN";
    }
}

template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
bool BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::last_read_all_valid()
{
    // Check validity for the specific group that was just read (current state before advancing)
    for (size_t chip = 0; chip < num_chip_selects * num_chips_per_chip_select; chip++) {
        const auto& validity = _bms_data.valid_read_packets[chip];

        switch (_current_read_group) {
            case ReadGroup_e::CV_GROUP_A:
                if (!validity[ReadGroup_e::CV_GROUP_A]) return false;
                break;
            case ReadGroup_e::CV_GROUP_B:
                if (!validity[ReadGroup_e::CV_GROUP_B]) return false;
                break;
            case ReadGroup_e::CV_GROUP_C:
                if (!validity[ReadGroup_e::CV_GROUP_C]) return false;
                break;
            case ReadGroup_e::CV_GROUP_D:
                // Skip 9-cell chips (odd indices)
                if (chip % 2 == 0 && !validity[ReadGroup_e::CV_GROUP_D]) return false;
                break;
            case ReadGroup_e::AUX_GROUP_A:
                if (!validity[ReadGroup_e::AUX_GROUP_A]) return false;
                break;
            case ReadGroup_e::AUX_GROUP_B:
                if (!validity[ReadGroup_e::AUX_GROUP_B]) return false;
                break;
            default:
                return false;
        }
    }
    return true;
}

template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
size_t BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, size_of_packet_value_bytes>::count_invalid_packets()
{
    size_t invalid_count = 0;

    // Count invalidity for the specific group that was just read
    for (size_t chip = 0; chip < num_chip_selects * num_chips_per_chip_select; chip++) {
        const auto& validity = _bms_data.valid_read_packets[chip];

        switch (_current_read_group) {
            case ReadGroup_e::CV_GROUP_A:
                if (!validity[ReadGroup_e::CV_GROUP_A]) invalid_count++;
                break;
            case ReadGroup_e::CV_GROUP_B:
                if (!validity[ReadGroup_e::CV_GROUP_B]) invalid_count++;
                break;
            case ReadGroup_e::CV_GROUP_C:
                if (!validity[ReadGroup_e::CV_GROUP_C]) invalid_count++;
                break;
            case ReadGroup_e::CV_GROUP_D:
                // Skip 9-cell chips (odd indices) when counting
                if (chip % 2 == 0 && !validity[ReadGroup_e::CV_GROUP_D]) invalid_count++;
                break;
            case ReadGroup_e::AUX_GROUP_A:
                if (!validity[ReadGroup_e::AUX_GROUP_A]) invalid_count++;
                break;
            case ReadGroup_e::AUX_GROUP_B:
                if (!validity[ReadGroup_e::AUX_GROUP_B]) invalid_count++;
                break;
            default:
                break;
        }
    }
    return invalid_count;
}