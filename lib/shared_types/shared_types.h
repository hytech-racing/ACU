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
    size_t max_consecutive_invalid_packet_count;
    bool charging_enabled;
};

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