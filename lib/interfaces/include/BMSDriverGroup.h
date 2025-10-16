#ifndef BMSDriverGroup_H
#define BMSDriverGroup_H

#include "LTCSPIInterface.h"
#include <Arduino.h>
#include <SPI.h>
#include <stdio.h>
#include "Configuration.h"
#include "LTCSPIInterface.h"
#include <cstdint>
#include "etl/optional.h"
#include <numeric>

#include "etl/singleton.h"

#include "SharedFirmwareTypes.h"
#include "shared_types.h"

enum class LTC6811_Type_e
{
    LTC6811_1 = 0,
    LTC6811_2
};

// Command Codes
enum class CMD_CODES_e
{
    // WRITES
    WRITE_CONFIG = 0x1,
    WRITE_S_CONTROL = 0x14,
    WRITE_PWM = 0x20,
    WRITE_COMM = 0x721,
    // READS
    READ_CONFIG = 0x2,
    READ_CELL_VOLTAGE_GROUP_A = 0x4,
    READ_CELL_VOLTAGE_GROUP_B = 0x6,
    READ_CELL_VOLTAGE_GROUP_C = 0x8,
    READ_CELL_VOLTAGE_GROUP_D = 0xA,
    READ_GPIO_VOLTAGE_GROUP_A = 0xC,
    READ_GPIO_VOLTAGE_GROUP_B = 0xE,
    READ_STATUS_GROUP_A = 0x10,
    READ_STATUS_GROUP_B = 0x12,
    READ_S_CONTROL = 0x16,
    READ_PWM = 0x22,
    READ_COMM = 0x722,
    // STARTS
    START_S_CONTROL = 0x19,
    START_CV_ADC_CONVERSION = 0x260,
    START_GPIO_ADC_CONVERSION = 0x460,
    START_CV_GPIO_ADC_CONVERSION = 0x46F,
    START_CV_SC_CONVERSION = 0x467,
    START_COMM = 0x723,
    // CLEARS
    CLEAR_S_CONTROL = 0x18,
    CLEAR_GPIOS = 0x712,
    CLEAR_STATUS = 0x713,
    // POLL ADC STATUS, DIAGNOSE MUX
    POLL_ADC_STATUS = 0x714,
    DIAGNOSE_MUX_POLL_STATUS = 0x715
};

enum class ADC_MODE_e : uint8_t
{
    MODE_ZERO = 0x0,
    FAST = 0x1,
    NORMAL = 0x2,
    FILTERED = 0x3
};

struct ValidPacketData_s
{
    bool valid_read_cells_1_to_3 = true;
    bool valid_read_cells_4_to_6 = true;
    bool valid_read_cells_7_to_9 = true;
    bool valid_read_cells_10_to_12 = true;
    bool valid_read_gpios_1_to_3 = true;
    bool valid_read_gpios_4_to_6 = true;
    bool all_invalid_reads = true;
};

struct BMSFaultCountData_s {
    uint8_t invalid_cell_1_to_3_count;
    uint8_t invalid_cell_4_to_6_count;
    uint8_t invalid_cell_7_to_9_count;
    uint8_t invalid_cell_10_to_12_count;
    uint8_t invalid_gpio_1_to_3_count;
    uint8_t invalid_gpio_4_to_6_count;
};

template <size_t num_chips, size_t num_cells, size_t num_board_thermistors>
struct BMSData
{
    std::array<ValidPacketData_s, num_chips> valid_read_packets;
    std::array<size_t, num_chips> consecutive_invalid_packet_counts;
    float valid_packet_rate;
    size_t max_consecutive_invalid_packet_count;
    std::array<BMSFaultCountData_s, num_chips> chip_invalid_cmd_counts;
    
    std::array<std::array<etl::optional<volt>, 12>, num_chips> voltages_by_chip;
    std::array<volt, num_cells> voltages;
    std::array<celsius, 4 * num_chips> cell_temperatures;
    std::array<celsius, num_board_thermistors> board_temperatures;
    volt min_cell_voltage;
    volt max_cell_voltage;
    celsius max_cell_temp;
    celsius min_cell_temp;
    celsius max_board_temp;
    size_t min_cell_voltage_id;              // 0 - 125
    size_t max_cell_voltage_id;              // 0 - 125
    size_t max_board_temperature_segment_id; // 0 - 11
    size_t max_humidity_segment_id;          // DNP
    size_t max_cell_temperature_cell_id;     // 0 - 47
    size_t min_cell_temperature_cell_id;     // 0 - 47
    volt total_voltage;
    volt avg_cell_voltage;
    celsius average_cell_temperature;
};

struct ReferenceMaxMin
{
    volt total_voltage = 0;
    volt max_cell_voltage = 0;
    volt min_cell_voltage = 65535;
    celsius min_cell_temp_voltage = 65535;
    celsius max_cell_temp_voltage = 0;
    celsius max_board_temp_voltage = 0;
    celsius total_thermistor_temps = 0;
};

struct BMSDriverGroupConfig_s
{
    const bool device_refup_mode = true;
    const bool adcopt = false;
    const uint16_t gpios_enabled = 0x1F; // There are 5 GPIOs, we are using all 5 so they are all given a 1
    const bool dcto_read = 0x1;
    const bool dcto_write = 0x0;
    const int adc_conversion_cell_select_mode = 0;
    const int adc_conversion_gpio_select_mode = 0;
    const uint8_t discharge_permitted = 0x0;
    const uint8_t adc_mode_cv_conversion = 0x1;
    const uint8_t adc_mode_gpio_conversion = 0x1;
    const uint16_t under_voltage_threshold = 1874; // 3.0V  // Minimum voltage value following datasheet formula: Comparison Voltage = (VUV + 1) • 16 • 100μV
    const uint16_t over_voltage_threshold = 2625;  // 4.2V  // Maximum voltage value following datasheet formula: Comparison Voltage = VOV • 16 • 100μV
    const uint16_t gpio_enable = 0x1F;
    const uint16_t CRC15_POLY = 0x4599; // Used for calculating the PEC table for LTC6811
    const float cv_adc_conversion_time_us = 13;
    const float gpio_adc_conversion_time_us = 3.1;
};

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
class BMSDriverGroup
{
public:
    constexpr static size_t num_cells = (num_chips / 2) * 21;
    using ACUDataDTO = ACUData_s<ACUConstants::NUM_CELLS, ACUConstants::NUM_CELL_TEMPS, ACUConstants::NUM_BOARD_TEMPS>;
    using BMSDriverData = BMSData<num_chips, num_cells, num_chips>;

    BMSDriverGroup(std::array<int, num_chip_selects> cs, std::array<int, num_chips> cs_per_chip, std::array<int, num_chips> addr, const BMSDriverGroupConfig_s config = {});

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
    BMSDriverData read_data();

    ACUDataDTO get_acu_data() const
{
    ACUDataDTO out{};

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

    // Invalid-packet max (derive from fault counters you already maintain)
    out.max_consecutive_invalid_packet_count = _bms_data.max_consecutive_invalid_packet_count;

    return out;
}

    BMSDriverData get_data() const
    {
        return _bms_data;
    }

    /* -------------------- WRITING DATA FUNCTIONS -------------------- */

    /**
     * Writes the device configuration
     * @pre needs access to undervoltage, overvoltage, configuration MACROS, and discharge data
     * @post sends packaged data over SPI
     */
    void write_configuration(uint8_t dcto_mode, const std::array<uint16_t, num_chips> &cell_balance_statuses);

    /**
     * Alternative header for configuration function call
     */
    void write_configuration(uint8_t dcto_mode, const std::array<bool, num_cells> &cell_balance_statuses);

private:
    void _generate_fault_data();
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
     * Using the "more robust wakeup" method of sending pair of long isoSPI pulses
     * Reference pages 51-52 on the data sheet for specific information.
     * @pre we don't care; for us, it's good to perform a wakeup to guarantee proper data propogation
     * @post essentially sets connected pin to LOW for period of time, then to HIGH for period of time
     */
    void _start_wakeup_protocol();

    void _start_wakeup_protocol(size_t cs);

    BMSDriverData _read_data_through_broadcast();

    BMSDriverData _read_data_through_address();

    void _store_temperature_humidity_data(BMSDriverData &bms_data, ReferenceMaxMin &max_min_reference, const uint16_t &gpio_in, size_t gpio_Index, size_t &gpio_count, size_t chip_num);

    void _store_voltage_data(BMSDriverData &bms_data, ReferenceMaxMin &max_min_reference, std::array<volt, 12> &chip_voltages_in, const float &voltage_in, size_t &cell_count);

    void _write_config_through_broadcast(uint8_t dcto_mode, std::array<uint8_t, 6> buffer_format, const std::array<uint16_t, num_chips> &cell_balance_statuses);

    void _write_config_through_address(uint8_t dcto_mode, std::array<uint8_t, 6> buffer_format, const std::array<uint16_t, num_chips> &cell_balance_statuses);

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

    void _start_ADC_conversion_through_broadcast(const std::array<uint8_t, 2> &cmd_code);

    void _start_ADC_conversion_through_address(std::array<uint8_t, 2> cmd_code);


    std::array<uint8_t, 24 * (num_chips / num_chip_selects)> _package_cell_voltages(BMSDriverData &bms_data, size_t chip_select_index,
                                                                                    const std::array<uint8_t, 8 * (num_chips / num_chip_selects)> &cv_1_to_3,
                                                                                    const std::array<uint8_t, 8 * (num_chips / num_chip_selects)> &cv_4_to_6,
                                                                                    const std::array<uint8_t, 8 * (num_chips / num_chip_selects)> &cv_7_to_9,
                                                                                    const std::array<uint8_t, 8 * (num_chips / num_chip_selects)> &cv_10_to_12);

    std::array<uint8_t, 10 * (num_chips / num_chip_selects)> _package_auxillary_data(BMSDriverData &bms_data, size_t chip_select_index,
                                                                                     const std::array<uint8_t, 8 * (num_chips / num_chip_selects)> &aux_1_to_3,
                                                                                     const std::array<uint8_t, 8 * (num_chips / num_chip_selects)> &aux_4_to_6);

    BMSDriverData _load_cell_voltages(BMSDriverData bms_data, ReferenceMaxMin &max_min_ref, const std::array<uint8_t, 24> &data_in_cv_1_to_12,
                                      size_t chip_index, size_t &battery_cell_count);

    BMSDriverData _load_auxillaries(BMSDriverData bms_data, ReferenceMaxMin &max_min_ref, const std::array<uint8_t, 10> &data_in_gpio_1_to_5,
                                    size_t chip_index, size_t &gpio_count);

    /* -------------------- GETTER FUNCTIONS -------------------- */

    /**
     * @brief When the inverters are idle, comms get funky from EMI. This function allows us to determine if the acu reads valid packets
     * @return bool of whether the PEC correctly reflects the buffer being given. If no, then we know that EMI (likely) is causing invalid reads
     */
    bool _check_if_valid_packet(const std::array<uint8_t, 8 * (num_chips / num_chip_selects)> &data, size_t param_iterator);

    /**
     * @return bool of whether valid read packets are all 1
     */
    bool _check_if_all_valid(size_t chip_index);

    /**
     * @return bool of whether valid read packets are all 0
     */
    bool _check_if_all_invalid(size_t chip_index);

    /**
     * @return sum of all cell voltages
     */
    volt _sum_cell_voltages();

    /**
     * @return bool whether the specific packet group (ex: cell voltages group B) should be updated, specifically if it's invalid
     */
    bool _check_specific_packet_group_is_invalid(size_t index, bool group_A_invalid, bool group_B_invalid, bool group_C_invalid, bool group_D_invalid);

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

private:

    /**
     * initializes PEC table
     * Made static so that it can be called in constructor -> _pec15table is made const
     * This implementation is straight from: https://www.analog.com/media/en/technical-documentation/data-sheets/LTC6811-1-6811-2.pdf
     * On page <76>, section: Applications Information
     */
    constexpr std::array<uint16_t, 256> _initialize_Pec_Table();

    /* MEMBER VARIABLES */
    BMSDriverData _bms_data;

    /**
     * Pointer to the PEC table we will use to calculate new PEC tables
     */
    // uint16_t _pec15Table[256];
    const std::array<uint16_t, 256> _pec15Table;

    /**
     * We will need this for both models of the IC
     * This determines where we get our signals from on the Arduino
     * It can only be 9 or 10
     * NOTE: needs to be initialized
     */
    const std::array<int, num_chip_selects> _chip_select;

    /**
     * We will need this for both models of the IC
     * This determines where we get our signals from on the Arduino
     * It can only be 9 or 10
     * NOTE: needs to be initialized
     */
    const std::array<int, num_chips> _chip_select_per_chip;

    /**
     * We will only end up using the address if this is a LTC6811-2
     * NOTE: But if we are, we need to call a setup function to instatiate each with the correct addresses
     * EX) BMS Segments 1, 4, and 5 are on chip select 9
     * BMS Segments 2, 3, and 6 are on chip select 10
     * Those segments correspond to 2 ICs each, so the instance with chip_select 9
     * Will have IC addresses: 0,1,6,7,8,9 | The rest are for chip_select 10
     */
    const std::array<int, num_chips> _address; // constant

    const BMSDriverGroupConfig_s _config;

    /**
     * Stores the balance statuses for all the chips
     * We only use 12 bits to represent a 1 (discharge) or 0 (charge)
     * out of the 16 bits
     */
    std::array<uint16_t, num_chips> _cell_discharge_en = {}; // not const  
};

template <size_t num_chips, size_t num_chip_selects, LTC6811_Type_e chip_type>
using BMSDriverInstance = etl::singleton<BMSDriverGroup<num_chips, num_chip_selects, chip_type>>;

#include <BMSDriverGroup.tpp>

#endif