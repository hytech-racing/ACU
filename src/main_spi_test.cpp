/* ACU Dependent */
#include "ACU_Constants.h"
#include "ACU_Globals.h"
#include "ACU_InterfaceTasks.h"
#include "ACU_SystemTasks.h"

/* Interface Includes */
#include <Arduino.h>
#include "BMSDriverGroup.h"
#include "WatchdogInterface.h"
#include "SystemTimeInterface.h"

/* System Includes */
#include "ACUController.h"
#include "ACUStateMachine.h"

/* Schedular Dependencies */
#include "ht_sched.hpp"
#include "ht_task.hpp"

elapsedMillis timer = 0;
elapsedMicros read_timer = 0;

using chip_type = LTC6811_Type_e;

const size_t sample_period_ms = 3; // 300 Hz - reads one group per call
const uint8_t spi_baudrate = 115200;
const uint8_t num_cells_per_board = 21;

// Initialize chip_select, chip_select_per_chip, and address
const constexpr int num_chips = 12; 
const constexpr int num_chip_selects = 1;
const std::array<int, num_chip_selects> cs = {10};
const std::array<int, num_chips> cs_per_chip = {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10};
const std::array<int, num_chips> addr = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

// Instantiate BMS Driver Group
const BMSDriverGroup<num_chips, num_chip_selects, chip_type::LTC6811_1> BMSGroup = BMSDriverGroup<num_chips, num_chip_selects, chip_type::LTC6811_1>(cs, cs_per_chip, addr);

std::array<BMSFaultCountData_s, num_chips> chip_invalid_cmd_counts;

// Tracking variables for optimized read testing
struct ReadGroupStats {
    uint32_t read_count = 0;
    uint32_t total_duration_us = 0;
    uint32_t min_duration_us = UINT32_MAX;
    uint32_t max_duration_us = 0;
};

std::array<ReadGroupStats, 6> group_stats;
uint32_t cycle_count = 0;
uint32_t current_group_index = 0;
bool cycle_complete = false;

const char* get_group_name(uint32_t group_index) {
    switch(group_index) {
        case 0: return "GROUP_A (Cells 0-2)";
        case 1: return "GROUP_B (Cells 3-5)";
        case 2: return "GROUP_C (Cells 6-8)";
        case 3: return "GROUP_D (Cells 9-11)";
        case 4: return "AUX_A (GPIO 0-2)";
        case 5: return "AUX_B (GPIO 3-4)";
        default: return "UNKNOWN";
    }
}

template <typename driver_data>
void print_voltages(driver_data data, uint32_t read_duration_us)
{
    Serial.println("========================================");
    Serial.print("Read Group: ");
    Serial.print(get_group_name(current_group_index));
    Serial.print(" | Duration: ");
    Serial.print(read_duration_us);
    Serial.println("us");

    if (cycle_complete) {
        Serial.println("*** CYCLE COMPLETE ***");
        Serial.print("Cycle #");
        Serial.println(cycle_count);
    }
    Serial.println("========================================");

    Serial.print("Total Voltage: ");
    Serial.print(data.total_voltage, 4);
    Serial.println("V");

    Serial.print("Minimum Voltage: ");
    Serial.print(data.min_cell_voltage, 4);
    Serial.print("V\tLocation of Minimum Voltage: ");
    Serial.println(data. min_cell_voltage_id);

    Serial.print("Maxmimum Voltage: ");
    Serial.print(data.max_cell_voltage, 4);
    Serial.print("V\tLocation of Maximum Voltage: ");
    Serial.println(data.max_cell_voltage_id);

    Serial.print("Average Voltage: ");
    Serial.print(data.total_voltage / ((num_chips / 2) * num_cells_per_board), 4);
    Serial.println("V");

    // BUG TEST: Check for division by zero in temperature calculation
    Serial.print("Average Cell Temperature: ");
    Serial.print(data.average_cell_temperature, 2);
    Serial.print("°C");
    if (isinf(data.average_cell_temperature)) {
        Serial.println(" *** WARNING: Temperature average is INFINITY (division by zero bug!) ***");
    } else if (isnan(data.average_cell_temperature)) {
        Serial.println(" *** WARNING: Temperature average is NaN! ***");
    } else {
        Serial.println();
    }

    Serial.print("Max Cell Temp: ");
    Serial.print(data.max_cell_temp, 2);
    Serial.print("°C (ID: ");
    Serial.print(data.max_cell_temperature_cell_id);
    Serial.print(") | Min Cell Temp: ");
    Serial.print(data.min_cell_temp, 2);
    Serial.print("°C (ID: ");
    Serial.print(data.min_cell_temperature_cell_id);
    Serial.println(")");

    Serial.println();

    size_t chip_index = 1;
    for(auto chip_voltages : data.voltages_by_chip)
    {
        Serial.print("Chip ");
        Serial.println(chip_index);
        for(auto voltage : chip_voltages)
        {
            if(voltage)
            {
                Serial.print((*voltage), 4);
                Serial.print("V\t");
            }
        }
        chip_index++;
        Serial.println();
    }

    int cti = 0;
    for(auto temp : data.cell_temperatures)
    {
        Serial.print("temp id ");
        Serial.print(cti);
        Serial.print(" val \t");
        Serial.print(temp);
        Serial.print("\t");
        if (cti % 4 == 3) Serial.println();
        cti++;
    }
    Serial.println();

    int temp_index = 0;
    for(auto bt : data.board_temperatures)
    {
        Serial.print("board temp id ");
        Serial.print(temp_index);
        Serial.print(" val ");
        Serial.print(bt);
        Serial.print("\t");
        if (temp_index % 4 == 3) Serial.println();
        temp_index++;
    }
    Serial.println();
    Serial.println();

    // Validity tracking - show which groups were read this call
    Serial.println("--- Validity Status (Current Read) ---");
    for (size_t chip = 0; chip < num_chips; chip++)
    {
        chip_invalid_cmd_counts[chip].invalid_cell_1_to_3_count = (!data.valid_read_packets[chip].valid_read_cells_1_to_3) ? chip_invalid_cmd_counts[chip].invalid_cell_1_to_3_count+1 : 0;
        chip_invalid_cmd_counts[chip].invalid_cell_4_to_6_count = (!data.valid_read_packets[chip].valid_read_cells_4_to_6) ? chip_invalid_cmd_counts[chip].invalid_cell_4_to_6_count+1 : 0;
        chip_invalid_cmd_counts[chip].invalid_cell_7_to_9_count = (!data.valid_read_packets[chip].valid_read_cells_7_to_9) ? chip_invalid_cmd_counts[chip].invalid_cell_7_to_9_count+1 : 0;
        chip_invalid_cmd_counts[chip].invalid_cell_10_to_12_count = (!data.valid_read_packets[chip].valid_read_cells_10_to_12) ? chip_invalid_cmd_counts[chip].invalid_cell_10_to_12_count+1 : 0;
        chip_invalid_cmd_counts[chip].invalid_gpio_1_to_3_count = (!data.valid_read_packets[chip].valid_read_gpios_1_to_3) ? chip_invalid_cmd_counts[chip].invalid_gpio_1_to_3_count+1 : 0;
        chip_invalid_cmd_counts[chip].invalid_gpio_4_to_6_count = (!data.valid_read_packets[chip].valid_read_gpios_4_to_6) ? chip_invalid_cmd_counts[chip].invalid_gpio_4_to_6_count+1 : 0;

        Serial.print("Chip ");
        Serial.print(chip);
        Serial.print(": ");

        // Show validity for current group being read
        bool has_invalid = false;
        switch(current_group_index) {
            case 0:
                if (!data.valid_read_packets[chip].valid_read_cells_1_to_3) {
                    Serial.print("INVALID_GROUP_A ");
                    has_invalid = true;
                }
                break;
            case 1:
                if (!data.valid_read_packets[chip].valid_read_cells_4_to_6) {
                    Serial.print("INVALID_GROUP_B ");
                    has_invalid = true;
                }
                break;
            case 2:
                if (!data.valid_read_packets[chip].valid_read_cells_7_to_9) {
                    Serial.print("INVALID_GROUP_C ");
                    has_invalid = true;
                }
                break;
            case 3:
                if (!data.valid_read_packets[chip].valid_read_cells_10_to_12) {
                    Serial.print("INVALID_GROUP_D ");
                    has_invalid = true;
                } else if (chip % 2 == 1) {
                    Serial.print("SKIPPED_9CELL ");
                }
                break;
            case 4:
                if (!data.valid_read_packets[chip].valid_read_gpios_1_to_3) {
                    Serial.print("INVALID_AUX_A ");
                    has_invalid = true;
                }
                break;
            case 5:
                if (!data.valid_read_packets[chip].valid_read_gpios_4_to_6) {
                    Serial.print("INVALID_AUX_B ");
                    has_invalid = true;
                }
                break;
        }

        if (!has_invalid && !(current_group_index == 3 && chip % 2 == 1)) {
            Serial.print("VALID");
        }

        Serial.print(" | Consecutive Invalids: c13:");
        Serial.print(chip_invalid_cmd_counts[chip].invalid_cell_1_to_3_count);
        Serial.print(" c46:");
        Serial.print(chip_invalid_cmd_counts[chip].invalid_cell_4_to_6_count);
        Serial.print(" c79:");
        Serial.print(chip_invalid_cmd_counts[chip].invalid_cell_7_to_9_count);
        Serial.print(" c1012:");
        Serial.print(chip_invalid_cmd_counts[chip].invalid_cell_10_to_12_count);
        Serial.print(" g13:");
        Serial.print(chip_invalid_cmd_counts[chip].invalid_gpio_1_to_3_count);
        Serial.print(" g46:");
        Serial.print(chip_invalid_cmd_counts[chip].invalid_gpio_4_to_6_count);
        Serial.println();
    }

    Serial.println();
    Serial.println();
}

void print_performance_stats() {
    Serial.println("========================================");
    Serial.println("=== PERFORMANCE STATISTICS ===");
    Serial.println("========================================");

    for (size_t i = 0; i < 6; i++) {
        Serial.print(get_group_name(i));
        Serial.println(":");
        Serial.print("  Reads: ");
        Serial.print(group_stats[i].read_count);

        if (group_stats[i].read_count > 0) {
            uint32_t avg = group_stats[i].total_duration_us / group_stats[i].read_count;
            Serial.print(" | Avg: ");
            Serial.print(avg);
            Serial.print("us | Min: ");
            Serial.print(group_stats[i].min_duration_us);
            Serial.print("us | Max: ");
            Serial.print(group_stats[i].max_duration_us);
            Serial.println("us");

            if (group_stats[i].max_duration_us > 3000) {
                Serial.println("  *** WARNING: Max duration exceeds 3ms budget! ***");
            }
        } else {
            Serial.println(" (no data)");
        }
    }

    Serial.println();
    Serial.print("Total Cycles Completed: ");
    Serial.println(cycle_count);
    Serial.print("Expected Frequency per Cell: ");
    Serial.print(300.0 / 6.0, 1);
    Serial.println(" Hz");
    Serial.println("========================================");
    Serial.println();
}

void setup()
{
    Serial.begin(spi_baudrate);
    SPI.begin();
    BMSGroup.init();
    Serial.println("Setup Finished!");
    Serial.println();

    /* ACU Data Struct */
    ACUDataInstance::create();

    /* Watchdog Interface */
    WatchdogInstance::create();
    WatchdogInstance::instance().init();
}

void loop()
{
    WatchdogInstance::instance().update_watchdog_state(sys_time::hal_millis()); // verified

    if (timer >= sample_period_ms) // 300 Hz - reads one group per call
    {
        // Reset timer
        timer = 0;

        // Track which group we're about to read
        uint32_t previous_group = current_group_index;

        // Start timing the read
        read_timer = 0;

        // Read one group from the BMS Driver
        auto bms_data = BMSGroup.read_data();

        // Capture read duration
        uint32_t read_duration_us = read_timer;

        // Update statistics for this group
        group_stats[current_group_index].read_count++;
        group_stats[current_group_index].total_duration_us += read_duration_us;
        if (read_duration_us < group_stats[current_group_index].min_duration_us) {
            group_stats[current_group_index].min_duration_us = read_duration_us;
        }
        if (read_duration_us > group_stats[current_group_index].max_duration_us) {
            group_stats[current_group_index].max_duration_us = read_duration_us;
        }

        // Detect cycle completion (we just read AUX_B and will wrap to GROUP_A next)
        cycle_complete = (current_group_index == 5);
        if (cycle_complete) {
            cycle_count++;
        }

        // Print detailed output
        print_voltages(bms_data, read_duration_us);

        // Print performance stats every 10 complete cycles
        if (cycle_complete && (cycle_count % 10 == 0)) {
            print_performance_stats();
        }

        // Advance to next group for next iteration
        current_group_index = (current_group_index + 1) % 6;
    }
}
