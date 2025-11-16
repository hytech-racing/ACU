#ifndef BMSDriverGroup_H
#define BMSDriverGroup_H

#include "LTCSPIInterface.h"

#include <Arduino.h>
#include <SPI.h>
#include <stdio.h>
#include <cstdint>
#include "etl/optional.h"
#include <numeric>

#include "etl/singleton.h"

#include "SharedFirmwareTypes.h"
#include "shared_types.h"

enum class LTC6811_Type_e
{
    LTC6811_1 = 0,  ///< Broadcast mode (used in production)
    LTC6811_2       ///< Address mode (reference only)
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

namespace bms_driver_defaults
{
    constexpr const bool DEVICE_REFUP_MODE = true;
    constexpr const bool ADCOPT = false;
    constexpr const uint16_t GPIOS_ENABLED = 0x1F; // 5 GPIOs, all used
    constexpr const bool DCTO_READ = true;
    constexpr const bool DCTO_WRITE = false;
    constexpr const int ADC_CONVERSION_CELL_SELECT_MODE = 0;
    constexpr const int ADC_CONVERSION_GPIO_SELECT_MODE = 0;
    constexpr const uint8_t DISCHARGE_PERMITTED = 0x0;
    constexpr const uint8_t ADC_MODE_CV_CONVERSION = 0x1;
    constexpr const uint8_t ADC_MODE_GPIO_CONVERSION = 0x1;
    constexpr const uint16_t UNDER_VOLTAGE_THRESHOLD = 1874; // 3.0V (datasheet formula) Comparison Voltage = (VUV + 1) • 16 • 100μV
    constexpr const uint16_t OVER_VOLTAGE_THRESHOLD = 2625;  // 4.2V (datasheet formula) Comparison Voltage = VOV • 16 • 100μV
    constexpr const uint16_t GPIO_ENABLE = 0x1F;
    constexpr const uint16_t CRC15_POLY = 0x4599; // Used for calculating the PEC table for LTC6811
    constexpr const float CV_ADC_CONVERSION_TIME_MS = 1.2f;
    constexpr const float GPIO_ADC_CONVERSION_TIME_MS = 1.2f;
    constexpr const float CV_ADC_LSB_VOLTAGE = 0.0001f; // Cell voltage ADC resolution: 100μV per LSB (1/10000 V)
    constexpr const size_t SIZE_OF_PACKET_VALUE_BYTES = 2;
}
namespace ref_max_min_defaults
{
    constexpr const volt TOTAL_VOLTAGE = 0;
    constexpr const volt MAX_CELL_VOLTAGE = 0;
    constexpr const volt MIN_CELL_VOLTAGE = 10;
    constexpr const celsius MIN_CELL_TEMP = 80;
    constexpr const celsius MAX_CELL_TEMP = 0;
    constexpr const celsius MAX_BOARD_TEMP = 0;
};



template <size_t num_chips, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps>
struct BMSData_s
{
    std::array<std::array<bool, ReadGroup_e::NUM_GROUPS>, num_chips> valid_read_packets;
    std::array<volt, num_voltage_cells> voltages;
    std::array<celsius, num_temp_cells> cell_temperatures;
    std::array<celsius, num_board_temps> board_temperatures;
    volt min_cell_voltage;
    volt max_cell_voltage;
    celsius max_cell_temp;
    celsius min_cell_temp;
    celsius max_board_temp;
    size_t min_cell_voltage_id;              // 0 - 125
    size_t max_cell_voltage_id;              // 0 - 125
    size_t max_board_temperature_segment_id; // 0 - 11
    size_t max_cell_temperature_cell_id;     // 0 - 47
    size_t min_cell_temperature_cell_id;     // 0 - 47
    volt total_voltage;
    volt avg_cell_voltage;
    celsius average_cell_temperature;
};

struct ReferenceMaxMin_s
{
    volt total_voltage;
    volt max_cell_voltage;
    volt min_cell_voltage;
    celsius min_cell_temp;
    celsius max_cell_temp;
    celsius max_board_temp;
    celsius total_thermistor_temps;
};

struct BMSDriverGroupConfig_s
{
    bool device_refup_mode;
    bool adcopt;
    uint16_t gpios_enabled;
    bool dcto_read;
    bool dcto_write;
    int adc_conversion_cell_select_mode;
    int adc_conversion_gpio_select_mode;
    uint8_t discharge_permitted;
    uint8_t adc_mode_cv_conversion;
    uint8_t adc_mode_gpio_conversion;
    uint16_t under_voltage_threshold;
    uint16_t over_voltage_threshold;
    uint16_t gpio_enable;
    uint16_t CRC15_POLY;
    float cv_adc_conversion_time_ms;
    float gpio_adc_conversion_time_ms;
    float cv_adc_lsb_voltage;
};


/**
 * @brief Advances to the next read group in the 6-state cycle (A → B → C → D → AUX_A → AUX_B → A)
 * @param current The current read group state
 * @return The next read group, wrapping from CURRENT_GROUP_AUX_B back to CURRENT_GROUP_A
 */
constexpr bool advance_read_group(ReadGroup_e* current)
{
    // Get current as int
    int curr = static_cast<int>(*current);

    // Compute next value with wrap-around
    int next = (curr + 1) % static_cast<int>(ReadGroup_e::NUM_GROUPS);

    // Detect wrap: we wrapped if next is 0
    bool wrapped = (next == 0);

    // Write back through the pointer
    *current = static_cast<ReadGroup_e>(next);

    return wrapped;
}


template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type, size_t size_of_packet_value_bytes>
class BMSDriverGroup
{
public:
    static constexpr size_t _data_size_bytes = 6;
    static constexpr size_t _pec_size_bytes = 2;
    static constexpr size_t _total_packet_size_bytes = _data_size_bytes + _pec_size_bytes;
    static constexpr std::array<CMD_CODES_e, ReadGroup_e::NUM_GROUPS> _read_group_to_cmd = {
        CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_A,
        CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_B,
        CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_C,
        CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_D,
        CMD_CODES_e::READ_GPIO_VOLTAGE_GROUP_A,
        CMD_CODES_e::READ_GPIO_VOLTAGE_GROUP_B
    };

    // constexpr static size_t num_cell_temps = (num_chips * 4);
    // constexpr static size_t num_board_temps = num_chips;

    using BMSCoreData_t = BMSCoreData_s<num_voltage_cells, num_temp_cells, num_board_temps>;
    using BMSDriverData_t = BMSData_s<num_chip_selects * num_chips_per_chip_select, num_voltage_cells, num_temp_cells, num_board_temps>;
    using ChipSelectConfig_t = ChipSelectConfig<num_chip_selects, num_chips_per_chip_select, _data_size_bytes / size_of_packet_value_bytes>;
    using ChipSelect_t = ChipSelect<num_chips_per_chip_select, _data_size_bytes / size_of_packet_value_bytes>;
    using Chip_t = Chip<_data_size_bytes / size_of_packet_value_bytes>;
    BMSDriverGroup(
        const ChipSelectConfig_t& chip_select_config,
        const BMSDriverGroupConfig_s default_params
    );
    

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
    BMSDriverData_t read_data();

    /**
     * Getter function to retrieve the ACUData structure
     */
    BMSCoreData_t get_bms_core_data();

    /**
     * Getter function to retrieve the BMSDriverData structure
     */
    BMSDriverData_t get_bms_data();

    /* -------------------- WRITING DATA FUNCTIONS -------------------- */

    /**
     * Writes the device configuration
     * @pre needs access to undervoltage, overvoltage, configuration MACROS, and discharge data
     * @post sends packaged data over SPI
     */
    void write_configuration(uint8_t dcto_mode, const std::array<uint16_t, num_chip_selects * num_chips_per_chip_select> &cell_balance_statuses);

    /**
     * Alternative header for configuration function call
     */
    void write_configuration(const bool* cell_balance_statuses, size_t cell_balance_statuses_size);

    /* -------------------- OBSERVABILITY FUNCTIONS -------------------- */

    /**
     * @brief Get the current read group state in the 6-state cycle
     * @return Current read group (CURRENT_GROUP_A through CURRENT_GROUP_AUX_B)
     * @note Useful for verifying state machine advancement and cycle tracking
     */
    ReadGroup_e get_current_read_group() {
        return _current_read_group;
    }

    /**
     * @brief Check if the next read_data() call will start a new cycle
     * @return true if next call reads GROUP_A (starts new ADC conversion cycle)
     * @note Useful for detecting cycle boundaries and synchronization points
     */
    bool is_cycle_start() {
        return _current_read_group == ReadGroup_e::CV_GROUP_A;
    }

    /**
     * @brief Get human-readable name of current read group
     * @return String name (e.g., "GROUP_A", "AUX_B")
     * @note Useful for logging and debugging
     */
    const char* get_current_read_group_name();

    /**
     * @brief Get validity status for all chips from last read
     * @return Const reference to validity data array (no copy overhead)
     * @note Each chip has 6 validity flags (cells 1-3, 4-6, 7-9, 10-12, GPIO 1-3, 4-6)
     * @note Useful for fault detection and EMI resilience monitoring
     */
    const std::array<std::array<bool, ReadGroup_e::NUM_GROUPS>, num_chip_selects * num_chips_per_chip_select>& get_validity_data() {
        return _bms_data.valid_read_packets;
    }

    /**
     * @brief Check if all packets were valid in last read_data() call
     * @return true if all chips returned valid PEC for the last group read
     * @note Useful for quick fault checks without iterating through all chips
     */
    bool last_read_all_valid();

    /**
     * @brief Count total invalid packets across all chips from last read
     * @return Number of chips that had invalid PEC in last group read (0 to num_chips)
     * @note Only counts invalidity for the specific group that was just read
     */
    size_t count_invalid_packets();

    /**
     * @brief Get current cell discharge enable statuses
     * @return Const reference to discharge enable array (one uint16_t per chip)
     * @note Each bit represents one cell's balance enable status
     * @note Useful for verifying write_configuration() worked correctly
     */
    const std::array<uint16_t, num_chip_selects * num_chips_per_chip_select>& get_cell_discharge_enable() {
        return _cell_discharge_en;  
    }

    /**
     * @brief Get configuration parameters (read-only)
     * @return Const reference to driver config struct
     * @note Useful for verifying hardware settings match expectations
     */
    const BMSDriverGroupConfig_s& get_config() {
        return _config;
    }

private:
    ReadGroup_e _current_read_group = ReadGroup_e::CV_GROUP_A;

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

    void _start_wakeup_protocol(size_t chip_select_pin);

    BMSDriverData_t _read_data_through_broadcast();

    /**
     * REFERENCE ONLY: LTC6811-2 ADDRESS MODE IS BROKEN AND UNUSED
     *
     * This implementation has major bugs and is NOT used in production (it wont compile anyway):
     *
     * BUG #1 (Line 318 in .tpp): Function signature mismatch
     *   - Calls: _load_cell_voltages(..., data_in_cell_voltages_1_to_12, chip, battery_cell_count)
     *   - Expects: _load_cell_voltages(..., std::array<uint8_t, 6>, chip_index, start_cell_index)
     *   - Problem: Passing 24-byte array where 6-byte array expected, wrong parameters
     *
     * BUG #2 (Line 319 in .tpp): Wrong function called
     *   - Calls: _load_auxillaries() which depends on _current_read_group state
     *   - Should call: _load_auxillaries_address_mode() instead
     *   - Problem: State variable not updated in address mode, causes incorrect GPIO indexing
     *
     * BUG #3: Stale variable names
     *   - Variables battery_cell_count and gpio_count (lines 286-287) are misleading
     *   - In broadcast mode, global cell indices are calculated geometrically per-chip, not accumulated
     *
     * PRODUCTION: All production code uses LTC6811_1 (broadcast mode) exclusively.
     * See ACU_InterfaceTasks.cpp lines 27-28.
     */
    BMSDriverData_t _read_data_through_address();

    void _store_cell_temperature_data(BMSDriverData_t &bms_data, ReferenceMaxMin_s &max_min_reference, const uint16_t &temp_in, uint8_t cell_temp_index);
    void _store_board_temperature_data(BMSDriverData_t &bms_data, ReferenceMaxMin_s &max_min_reference, const uint16_t &temp_in, uint8_t board_index);
    void _store_voltage_data(BMSDriverData_t &bms_data, ReferenceMaxMin_s &max_min_reference, volt voltage_in, uint8_t cell_index);

    void _write_config_through_broadcast(uint8_t dcto_mode, std::array<uint8_t, 6> buffer_format, const std::array<uint16_t, num_chip_selects * num_chips_per_chip_select> &cell_balance_statuses);

    // void _write_config_through_address(uint8_t dcto_mode, const std::array<uint8_t, 6>& buffer_format, const std::array<uint16_t, num_chip_selects * num_chips_per_chip_select> &cell_balance_statuses);

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

    // void _start_ADC_conversion_through_address(const std::array<uint8_t, 2>& cmd_code);

    void _load_cell_voltages(BMSDriverData_t &bms_data, ReferenceMaxMin_s &max_min_ref, const std::array<uint8_t, size_of_packet_value_bytes> &data_in_cell_voltage, uint8_t cell_index);
    void _load_cell_temps(BMSDriverData_t &bms_data, ReferenceMaxMin_s &max_min_ref, const std::array<uint8_t, size_of_packet_value_bytes> &data_in_temp, uint8_t cell_index);
    void _load_board_temps(BMSDriverData_t &bms_data, ReferenceMaxMin_s &max_min_ref, const std::array<uint8_t, size_of_packet_value_bytes> &data_in_temp, uint8_t board_index);
    /* -------------------- GETTER FUNCTIONS -------------------- */

    /**
     * @brief When the inverters are idle, comms get funky from EMI. This function allows us to determine if the acu reads valid packets
     * @return bool of whether the PEC correctly reflects the buffer being given. If no, then we know that EMI (likely) is causing invalid reads
     */
    bool _check_if_valid_packet(const std::array<uint8_t, _total_packet_size_bytes * num_chips_per_chip_select> &data, size_t param_iterator);

    /**
     * Generates a Packet Error Code
     * @pre PEC table is already generated –> pec15Table has data generated in it
     * @post returns an unsigned 16 bit integer used to check data to and from the device
     * @param data data we are writing PEC for, could be Command Code or Buffer data
     * @param length length of data
     * @return unsigned 16 bit PEC, array of uint8_t of length 2
     */
    std::array<uint8_t, 2> _calculate_specific_PEC(const uint8_t *data, int length);

    /**
     * Generates a formmatted 2 byte array for the Command bytes
     * @return unsigned 8 bit array of length 2
     */
    std::array<uint8_t, 2> _generate_formatted_CMD(CMD_CODES_e command, size_t chip_addr);

    /**
     * Generates Command and PEC as one byte array of length 4: CMD0, CMD1, PEC0, PEC1
     * @return unsigned 8 bit, length 4
     */
    std::array<uint8_t, 4> _generate_CMD_PEC(CMD_CODES_e command, size_t chip_addr);

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
    BMSDriverData_t _bms_data;

    /**
     * Tracks min/max/sum values across all 6 read groups within a single timestamp cycle.
     * Reset only at the start of each new cycle (when _current_read_group == CURRENT_GROUP_A).
     */
    ReferenceMaxMin_s _max_min_reference;
    
    /**
     * We will need this for both models of the IC
     * This determines where we get our signals from on the Arduino
     * It can only be 9 or 10
     * NOTE: needs to be initialized
     */
    const ChipSelectConfig_t _chip_select_config;
    

    
    /**
     * REPLACING SEPARATE CONFIGURATION FILE
     * NOTE: THIS SHOULD BE TREATED AS THE INTERFACE'S DEFAULT PARAMTERS    
     */
    const BMSDriverGroupConfig_s _config;
    
    /**
     * Pointer to the PEC table we will use to calculate new PEC tables
     */
    // uint16_t _pec15Table[256];
    const std::array<uint16_t, 256> _pec15Table; //must be below _config to be initialized after it
    
    /**
     * Stores the balance statuses for all the chips
     * We only use 12 bits to represent a 1 (discharge) or 0 (charge)
     * out of the 16 bits
     */
    std::array<uint16_t, num_chip_selects * num_chips_per_chip_select> _cell_discharge_en = {}; // not const  
};

template <size_t num_chips_per_chip_select, size_t num_chip_selects, size_t num_voltage_cells, size_t num_temp_cells, size_t num_board_temps, LTC6811_Type_e chip_type>
using BMSDriverInstance = etl::singleton<BMSDriverGroup<num_chips_per_chip_select, num_chip_selects, num_voltage_cells, num_temp_cells, num_board_temps, chip_type, bms_driver_defaults::SIZE_OF_PACKET_VALUE_BYTES>>;

#include <BMSDriverGroup.tpp>

#endif