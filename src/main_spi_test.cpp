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

#include <cstdint>

elapsedMillis timer = 0;

using chip_type = LTC6811_Type_e;

// Initialize chip_select, chip_select_per_chip, and address
const int num_chips = 2;
const int num_chip_selects = 1;
std::array<int, num_chip_selects> cs = {10};
std::array<int, num_chips> cs_per_chip = {10, 10};
std::array<int, num_chips> addr = {4, 5};
ACU_State_s<num_chips> acu_state = {};

// Instantiate BMS Driver Group
BMSDriverGroup<num_chips, num_chip_selects, chip_type::LTC6811_1> BMSGroup = BMSDriverGroup<num_chips, num_chip_selects, chip_type::LTC6811_1>(cs, cs_per_chip, addr);

void setBit(uint16_t &value, uint8_t index, bool bitValue) {
    if (index >= 16) return; // Ensure index is within range

    if (bitValue) {
        value |= (1 << index);  // Set the bit
    } else {
        value &= ~(1 << index); // Clear the bit
    }
}

template <typename driver_data>
void print_voltages(driver_data data)
{
    Serial.print("Total Voltage: ");
    Serial.print(data.total_voltage / 10000.0, 4);
    Serial.println("V");

    Serial.print("Minimum Voltage: ");
    Serial.print(data.min_voltage, 4);
    Serial.print("V\tLocation of Minimum Voltage: ");
    Serial.println(data.min_voltage_cell_id);

    Serial.print("Maxmimum Voltage: ");
    Serial.print(data.max_voltage, 4);
    Serial.print("V\tLocation of Maximum Voltage: ");
    Serial.println(data.max_voltage_cell_id);

    Serial.print("Average Voltage: ");
    Serial.print(data.total_voltage / (num_chips * 21), 4);
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
        Serial.print(" temp id");
        Serial.print(cti);
        Serial.print(" val \t");
        Serial.print("");
        Serial.print(temp);
        cti++;
    }
    Serial.println();
    Serial.println();
    int temp_index = 0;
    for(auto bt : data.board_temperatures)
    {
        Serial.print("board temp id");
        Serial.print(temp_index);
        Serial.print(" val ");
        Serial.print("");
        Serial.print(bt);
        temp_index++;
    }
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
        // Serial.println("Enter looped!");
        // Can't be more than 1500 or t sleep will disable itself -> will notice initial update, but that's it.
        timer = 0;
        // Reading in voltage / GPIO data
        auto bms_data = BMSGroup.read_data();
        print_voltages(bms_data);
    
        // Calculate cell_balance_statuses based on data.voltages
        // Passing in voltages, min_voltage, max_voltage; Returns cell_balance_statuses,
        update_acu_state<num_chips>(acu_state, bms_data.voltages, bms_data.min_voltage, bms_data.max_voltage);
        
        

        for(size_t chip_index = 0; chip_index < bms_data.voltages.size(); chip_index++)
        {
            auto segment_voltages = bms_data.voltages[chip_index];
            for(size_t j = 0; j < segment_voltages.size(); j++)
            {
                auto opt_volt = segment_voltages[j];
                if(opt_volt)
                {
                    if(((*opt_volt) / 10000.0) > 3.8)
                    {
                        setBit(acu_state.cell_balance_statuses[chip_index], j, true);
                    } else {
                        setBit(acu_state.cell_balance_statuses[chip_index], j, false);
                    }
                }
            }
        }

        BMSGroup.write_configuration(dcto_write, acu_state.cell_balance_statuses); // cell_balance_statuses is updated at this point

        // Send bms_data through message interface here
    }
}
