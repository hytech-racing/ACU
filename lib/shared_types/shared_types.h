#ifndef SHAREDTYPES_H
#define SHAREDTYPES_H

#include "SharedFirmwareTypes.h"

struct BMSCoreData_s {
    volt min_cell_voltage;
    volt max_cell_voltage;
    volt pack_voltage;
    celsius max_cell_temp; 
    celsius min_cell_temp; 
    celsius max_board_temp;
};

/**
 * CurrentReadGroup_e - State machine for incremental BMS register group reading
 *
 * This enum defines the 6-state read cycle used to distribute voltage and GPIO readings
 * across multiple read_data() calls. Instead of reading all registers at once (which takes
 * significant time), each call reads ONE register group, cycling through all 6 groups.
 *
 * STATE MACHINE CYCLE:
 *
 *   Call 1 → CURRENT_GROUP_A     → Read cells 0-2   (CV Group A)
 *   Call 2 → CURRENT_GROUP_B     → Read cells 3-5   (CV Group B)
 *   Call 3 → CURRENT_GROUP_C     → Read cells 6-8   (CV Group C)
 *   Call 4 → CURRENT_GROUP_D     → Read cells 9-11  (CV Group D)  [9-cell chips: skip]
 *   Call 5 → CURRENT_GROUP_AUX_A → Read GPIO 0-2    (Aux Group A: thermistors 0-2)
 *   Call 6 → CURRENT_GROUP_AUX_B → Read GPIO 3-5    (Aux Group B: thermistors 3-4, board temp)
 *   [GOTO Call 1]
 *
 * HARDWARE MAPPING:
 * - Each LTC6811 chip has 12 cell voltage registers divided into 4 groups (A-D)
 * - Each group contains 3 consecutive cell readings (6 bytes = 3 × 16-bit values)
 * - GPIO registers are divided into 2 groups (AUX_A, AUX_B)
 * - GPIO 0-3: Cell thermistors (temperature sensors)
 * - GPIO 4: Board temperature sensor (MCP9701)
 * - GPIO 5: Not used (padding in AUX_B)
 *
 * TIMING & FREQUENCY:
 * - Original (main): Read all 6 groups at 50 Hz (20ms period)
 * - Optimized (this branch): Read 1 group per call at 300 Hz (3ms period)
 * - Effective sampling rate per cell: 300 Hz / 6 groups = 50 Hz (same as before)
 *
 * CHIP ARCHITECTURE NOTES:
 * - System has 12 ICs total in pairs: 6 × (12-cell + 9-cell) = 126 cells total
 * - Even-indexed chips (0, 2, 4, 6, 8, 10): 12 cells each
 * - Odd-indexed chips  (1, 3, 5, 7, 9, 11): 9 cells each
 * - For 9-cell chips, GROUP_D read is skipped (no cells 10-12)
 *
 * VALIDATION:
 * Each group read has its own validity flag in ValidPacketData_s:
 *   - CURRENT_GROUP_A     → valid_read_cells_1_to_3
 *   - CURRENT_GROUP_B     → valid_read_cells_4_to_6
 *   - CURRENT_GROUP_C     → valid_read_cells_7_to_9
 *   - CURRENT_GROUP_D     → valid_read_cells_10_to_12
 *   - CURRENT_GROUP_AUX_A → valid_read_gpios_1_to_3
 *   - CURRENT_GROUP_AUX_B → valid_read_gpios_4_to_6
 */

enum ReadGroup_e {
    CV_GROUP_A = 0, CV_GROUP_B, CV_GROUP_C, CV_GROUP_D,
    AUX_GROUP_A,   AUX_GROUP_B, NUM_GROUPS
};

enum ReadDataResultType_e { CELL_VOLTAGE, CELL_TEMPERATURE, BOARD_TEMPERATURE, NUM_DATA_TYPES, NO_DATA };

template<std::size_t DATA_PER_GROUP>
class ReadGroupResultMap {
public:
    const std::array<std::array<ReadDataResultType_e, DATA_PER_GROUP>, NUM_GROUPS> group_data_types;

    // constexpr ctor so the object can be constexpr if inputs are constexpr
    constexpr explicit ReadGroupResultMap(const std::array<std::array<ReadDataResultType_e, DATA_PER_GROUP>, NUM_GROUPS>& gdt)
        : group_data_types(gdt)
        , num_values_in_each_group(make_per_group_counts(gdt))
        , num_data_type_counts(count_type(gdt))
    {}

    constexpr std::size_t get_num_values_in_group(ReadGroup_e group) const {
        return num_values_in_each_group[static_cast<std::size_t>(group)];
    }
    constexpr std::size_t get_num_cell_voltages() const { return num_data_type_counts[CELL_VOLTAGE]; }
    constexpr std::size_t get_num_cell_temps()   const { return num_data_type_counts[CELL_TEMPERATURE]; }
    constexpr std::size_t get_num_board_temps()  const { return num_data_type_counts[BOARD_TEMPERATURE]; }
    constexpr std::size_t get_group_start_cell_voltage_index(ReadGroup_e group){
        std::size_t index = 0;
        for (std::size_t g = 0; g < static_cast<std::size_t>(group); ++g) {
            for (std::size_t k = 0; k < DATA_PER_GROUP; ++k) {
                if (group_data_types[g][k] == CELL_VOLTAGE) {
                    ++index;
                }
            }
        }
        return index;
    }
    constexpr std::size_t get_group_start_cell_temperature_index(ReadGroup_e group){
        std::size_t index = 0;
        for (std::size_t g = 0; g < static_cast<std::size_t>(group); ++g) {
            for (std::size_t k = 0; k < DATA_PER_GROUP; ++k) {
                if (group_data_types[g][k] == CELL_TEMPERATURE) {
                    ++index;
                }
            }
        }
        return index;
    }
    constexpr std::size_t get_group_start_board_temperature_index(ReadGroup_e group){
        std::size_t index = 0;
        for (std::size_t g = 0; g < static_cast<std::size_t>(group); ++g) {
            for (std::size_t k = 0; k < DATA_PER_GROUP; ++k) {
                if (group_data_types[g][k] == BOARD_TEMPERATURE) {
                    ++index;
                }
            }
        }
        return index;
    }

private:
    const std::array<std::size_t, NUM_GROUPS> num_values_in_each_group;
    const std::array<size_t, NUM_DATA_TYPES> num_data_type_counts;

    static constexpr std::array<std::size_t, NUM_GROUPS> make_per_group_counts(const std::array<std::array<ReadDataResultType_e, DATA_PER_GROUP>, NUM_GROUPS>& gdt) {
        std::array<std::size_t, NUM_GROUPS> out{}; // zero-initialized
        for (std::size_t g = 0; g < out.size(); ++g) {
            std::size_t c = 0;
            for (std::size_t k = 0; k < DATA_PER_GROUP; ++k)
                if (gdt[g][k] != NO_DATA) ++c;
            out[g] = c;
        }
        return out;
    }

    static constexpr std::array<std::size_t, NUM_DATA_TYPES> count_type(const std::array<std::array<ReadDataResultType_e, DATA_PER_GROUP>, NUM_GROUPS>& gdt) {
        std::array<std::size_t, NUM_DATA_TYPES> counts{}; // zero-initialized
        for (std::size_t g = 0; g < gdt.size(); ++g)
            for (std::size_t k = 0; k < DATA_PER_GROUP; ++k)
                switch (gdt[g][k]) {
                    case CELL_VOLTAGE:
                        ++counts[CELL_VOLTAGE];
                        break;
                    case CELL_TEMPERATURE:
                        ++counts[CELL_TEMPERATURE];
                        break;
                    case BOARD_TEMPERATURE:
                        ++counts[BOARD_TEMPERATURE];
                        break;
                    default:
                        break; // NO_DATA or unrecognized
                }

        return counts;
    }
};

// ----- chip (immutable) -----
template<size_t DATA_PER_GROUP>
struct Chip {
    const size_t addr_pin;
    const ReadGroupResultMap<DATA_PER_GROUP> read_map;
};

// ----- chip-select (immutable) -----
template <std::size_t NUM_CHIPS_PER_CS, std::size_t DATA_PER_GROUP>
class ChipSelect {
public:
    const std::size_t cs_pin;
    const std::array<Chip<DATA_PER_GROUP>, NUM_CHIPS_PER_CS> chips;


    constexpr std::size_t get_num_cell_voltages() const { return num_data_type_counts[CELL_VOLTAGE]; }
    constexpr std::size_t get_num_cell_temps()   const { return num_data_type_counts[CELL_TEMPERATURE]; }
    constexpr std::size_t get_num_board_temps()  const { return num_data_type_counts[BOARD_TEMPERATURE]; }

    // Constructor: initialize all const members
    constexpr ChipSelect(
        std::size_t cs,
        const std::array<Chip<DATA_PER_GROUP>, NUM_CHIPS_PER_CS>& chips_in
    )
        : cs_pin(cs)
        , chips(chips_in)
        , num_data_type_counts(count_type(chips_in)) // <-- uses static helper
    {}

    

private:
    const std::array<std::size_t, NUM_DATA_TYPES> num_data_type_counts;
    // Static helper: computes counts from the given chips array
    static constexpr std::array<std::size_t, NUM_DATA_TYPES>
    count_type(const std::array<Chip<DATA_PER_GROUP>, NUM_CHIPS_PER_CS>& chips_in)
    {
        std::array<std::size_t, NUM_DATA_TYPES> counts{}; // zero-initialized

        for (const auto& chip : chips_in) {
            counts[CELL_VOLTAGE]     += chip.read_map.get_num_cell_voltages();
            counts[CELL_TEMPERATURE] += chip.read_map.get_num_cell_temps();
            counts[BOARD_TEMPERATURE]+= chip.read_map.get_num_board_temps();
        }

        return counts;
    }
};


// ----- full config (immutable) + constexpr derived views -----
template<size_t NUM_CS, size_t NUM_CHIPS_PER_CS, size_t DATA_PER_GROUP>
class ChipSelectConfig {
public:
    const std::array<ChipSelect<NUM_CHIPS_PER_CS, DATA_PER_GROUP>, NUM_CS> chip_selects;

    constexpr ChipSelectConfig(
        const std::array<ChipSelect<NUM_CHIPS_PER_CS, DATA_PER_GROUP>, NUM_CS>& chip_selects
    ) :
        chip_selects(chip_selects)
        , num_data_type_counts(count_type(chip_selects)) // <-- uses static helper
    {}

    constexpr std::size_t get_num_cell_voltages() const { return num_data_type_counts[CELL_VOLTAGE]; }
    constexpr std::size_t get_num_cell_temps()   const { return num_data_type_counts[CELL_TEMPERATURE]; }
    constexpr std::size_t get_num_board_temps()  const { return num_data_type_counts[BOARD_TEMPERATURE]; }

    size_t constexpr global_chip_index(size_t cs_index, size_t chip_index) const noexcept {
        return cs_index * NUM_CHIPS_PER_CS + chip_index;
    }     
    
    private:
        const std::array<std::size_t, NUM_DATA_TYPES> num_data_type_counts;

        // Static helper: computes counts from the given chips array
        static constexpr std::array<std::size_t, NUM_DATA_TYPES>
        count_type(const std::array<ChipSelect<NUM_CHIPS_PER_CS, DATA_PER_GROUP>, NUM_CS>& chip_selects)
        {
            std::array<std::size_t, NUM_DATA_TYPES> counts{}; // zero-initialized

            for (const auto& cs : chip_selects) {
                counts[CELL_VOLTAGE]     += cs.get_num_cell_voltages();
                counts[CELL_TEMPERATURE] += cs.get_num_cell_temps();
                counts[BOARD_TEMPERATURE]+= cs.get_num_board_temps();
            }

            return counts;
        }
};
#endif
