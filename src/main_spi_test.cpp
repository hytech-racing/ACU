#include <SPI.h>
#include "BMSDriverGroup.h"
#include <LTCSPIInterface.h>
#include "Configuration.h"
// #include "ACUController.h"

#include <array>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <string>

elapsedMillis timer = 0;

using chip_type = LTC6811_Type_e;

// Initialize chip_select, chip_select_per_chip, and address
const int num_chips = 4;
const int num_chip_selects = 1;
std::array<int, num_chip_selects> cs = {10};
std::array<int, num_chips> cs_per_chip = {10, 10, 10, 10};
std::array<int, num_chips> addr = {4, 4, 4, 4};
ACU_State_s<num_chips> acu_state = {};

// Instantiate BMS Driver Group
BMSDriverGroup<num_chips, num_chip_selects, chip_type::LTC6811_1> BMSGroup = BMSDriverGroup<num_chips, num_chip_selects, chip_type::LTC6811_1>(cs, cs_per_chip, addr);

template <typename driver_data>
void print_voltages(driver_data data)
{
    Serial.print("Total Voltage: ");
    Serial.print(data.total_voltage / 10000.0, 4); 
    Serial.println("V");

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

    size_t chip_index = 1;
    for(auto chip_voltages : data.voltages )
    {
        Serial.print("chip ");
        Serial.println(chip_index);
        for(auto voltage : chip_voltages)
        {
            if(voltage)
            {
                Serial.print((*voltage) / 10000.0, 4);
                Serial.print("V\t");
            }
            
        }
        chip_index++;
        Serial.println();
    }
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
        // Serial.println("Enter looped!");
        // Can't be more than 1500 or t sleep will disable itself -> will notice initial update, but that's it.
        timer = 0;
        // BMSGroup.manual_send_and_print();
        auto bms_data = BMSGroup.read_data();
        print_voltages(bms_data);

        // Calculate cell_balance_statuses based on data.voltages
        // Passing in voltages, min_voltage, max_voltage; Returns cell_balance_statuses, 
        // update_acu_state<num_chips>(acu_state, bms_data.voltages, bms_data.min_voltage, bms_data.max_voltage);

        BMSGroup.write_configuration(dcto_write, acu_state.cell_balance_statuses); // cell_balance_statuses is updated at this point
    }
}
