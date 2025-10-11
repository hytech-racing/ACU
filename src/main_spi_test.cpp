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
// #include "ACUEthernetInterface.h"

/* System Includes */
#include "ACUController.h"
#include "ACUStateMachine.h"

/* Schedular Dependencies */
#include "ht_sched.hpp"
#include "ht_task.hpp"

const elapsedMillis timer = 0;

using chip_type = LTC6811_Type_e;

const size_t sample_period_ms = 250;
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

template <typename driver_data>
void print_voltages(driver_data data)
{
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
        Serial.print("WARNING: Temperature average is INFINITY (division by zero bug!)");
    } else if (isnan(data.average_cell_temperature)) {
        Serial.print("WARNING: Temperature average is NaN!");
    }
    Serial.println();

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

    for (size_t chip = 0; chip < num_chips; chip++)
    {
        chip_invalid_cmd_counts[chip].invalid_cell_1_to_3_count = (!data.valid_read_packets[chip].valid_read_cells_1_to_3) ? chip_invalid_cmd_counts[chip].invalid_cell_1_to_3_count+1 : 0;
        chip_invalid_cmd_counts[chip].invalid_cell_4_to_6_count = (!data.valid_read_packets[chip].valid_read_cells_4_to_6) ? chip_invalid_cmd_counts[chip].invalid_cell_4_to_6_count+1 : 0;
        chip_invalid_cmd_counts[chip].invalid_cell_7_to_9_count = (!data.valid_read_packets[chip].valid_read_cells_7_to_9) ? chip_invalid_cmd_counts[chip].invalid_cell_7_to_9_count+1 : 0;
        chip_invalid_cmd_counts[chip].invalid_cell_10_to_12_count = (!data.valid_read_packets[chip].valid_read_cells_10_to_12) ? chip_invalid_cmd_counts[chip].invalid_cell_10_to_12_count+1 : 0;
        chip_invalid_cmd_counts[chip].invalid_gpio_1_to_3_count = (!data.valid_read_packets[chip].valid_read_gpios_1_to_3) ? chip_invalid_cmd_counts[chip].invalid_gpio_1_to_3_count+1 : 0;
        chip_invalid_cmd_counts[chip].invalid_gpio_4_to_6_count = (!data.valid_read_packets[chip].valid_read_gpios_4_to_6) ? chip_invalid_cmd_counts[chip].invalid_gpio_4_to_6_count+1 : 0;

        Serial.print("Chip ");
        Serial.println(chip);
        Serial.print("Invalids: c13 ");
        Serial.print(chip_invalid_cmd_counts[chip].invalid_cell_1_to_3_count);
        Serial.print("\t c46 ");
        Serial.print(chip_invalid_cmd_counts[chip].invalid_cell_4_to_6_count);
        Serial.print("\t c79 ");
        Serial.print(chip_invalid_cmd_counts[chip].invalid_cell_7_to_9_count);
        Serial.print("\t c1012 ");
        Serial.print(chip_invalid_cmd_counts[chip].invalid_cell_10_to_12_count);
        Serial.print("\t g13 ");
        Serial.print(chip_invalid_cmd_counts[chip].invalid_gpio_1_to_3_count);
        Serial.print("\t g46 ");
        Serial.print(chip_invalid_cmd_counts[chip].invalid_gpio_4_to_6_count);
        Serial.println();
    }
    
    Serial.println();
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

    if (timer > sample_period_ms) // Need an actual schedular
    {   
        // reset timer
        timer = 0;

        // Read cell and auxiliary data from the BMS Driver
        auto bms_data = BMSGroup.read_data();
        print_voltages(bms_data);
    }

}
