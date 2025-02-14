#include <SPI.h>
#include "BMSDriverGroup.h"
#include "WatchdogInterface.h"
#include <LTCSPIInterface.h>
#include "Configuration.h"
#include "ACUController.h"

#include <array>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <string>

elapsedMillis timer = 0;

using chip_type = LTC6811_Type_e;

// Initialize chip_select, chip_select_per_chip, and address
const int num_chips = 1;
const int num_chip_selects = 1;
std::array<int, num_chip_selects> cs = {10};
std::array<int, num_chips> cs_per_chip = {10};
std::array<int, num_chips> addr = {4};
ACU_State_s<num_chips> acu_state = {};

// Instantiate BMS Driver Group
BMSDriverGroup<num_chips, num_chip_selects, chip_type::LTC6811_1> BMSGroup = BMSDriverGroup<num_chips, num_chip_selects, chip_type::LTC6811_1>(cs, cs_per_chip, addr);



void print_voltages(auto data)
{
    Serial.print("Total Voltage: ");
    Serial.print(data.total_voltage / 10000.0, 4); Serial.println("V");

    Serial.print("Minimum Voltage: ");
    Serial.print(data.min_voltage / 10000.0, 4);
    Serial.print("V\tLocation of Minimum Voltage: ");
    Serial.println(data.min_voltage_cell_id);

    Serial.print("Maxmimum Voltage: ");
    Serial.print(data.max_voltage / 10000.0, 4);
    Serial.print("V\tLocation of Maximum Voltage: ");
    Serial.println(data.max_voltage_cell_id);

    Serial.print("Average Voltage: ");
    Serial.print(data.total_voltage / 120000.0, 4);
    Serial.println("V");

    Serial.println();
}

void setup()
{
    Serial.begin(115200);
    SPI.begin();
    BMSGroup.init();
    Serial.println("Setup Finished!");
    Serial.println();
}

void loop()
{
    if (timer > 300)
    {   
        Serial.println("Enter looped!");
        // Can't be more than 1500 or t sleep will disable itself -> will notice initial update, but that's it.
        timer = 0;
        // Reading in voltage / GPIO data
        auto bms_data = BMSGroup.read_data();
        print_voltages(bms_data);

        // Calculate cell_balance_statuses based on data.voltages + updating faults
        update_acu_state<num_chips>(acu_state, bms_data.voltages, bms_data.min_voltage, bms_data.max_voltage);
        
        // Toggle pin 5 on teensy if there are no voltage faults <- AMS watchdog
        if (!acu_state.has_voltage_fault) {
            pulse_ams_watchdog(acu_state.current_pulse);
        }
        
        // Update cell balances to the cells
        BMSGroup.write_configuration(dcto_write, acu_state.cell_balance_statuses); // cell_balance_statuses is updated at this point

        // Send bms_data through message interface here
    }
}
