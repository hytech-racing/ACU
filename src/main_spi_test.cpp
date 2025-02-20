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
constexpr int num_chips = 2;
constexpr int num_chip_selects = 1;
std::array<int, num_chip_selects> cs = {10};
std::array<int, num_chips> cs_per_chip = {10, 10};
std::array<int, num_chips> addr = {0, 1};

// Instantiate BMS Driver Group
BMSDriverGroup<num_chips, num_chip_selects, chip_type::LTC6811_1> BMSGroup = BMSDriverGroup<num_chips, num_chip_selects, chip_type::LTC6811_1>(cs, cs_per_chip, addr);

// Instantiate ACU Controller
ACUController<num_chips> controller = ACUController<num_chips>();

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
    Serial.print(data.total_voltage, 4);
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
    Serial.print(data.total_voltage / ((num_chips / 2) * 21), 4);
    Serial.println("V");

    Serial.println();

    size_t chip_index = 1;
    for(auto chip_voltages : data.voltages )
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
    int cti = 1;
    for(auto temp : data.cell_temperatures) // Should be 4 per chip
    {
        Serial.print("temp id ");
        Serial.print(cti);
        Serial.print(" val ");
        Serial.print(temp);
        Serial.print("\t");
        if (cti % 4 == 0) Serial.println();
        cti++;
    }
    int temp_index = 1;
    for(auto bt : data.board_temperatures) // Should be 1 per chip
    {
        Serial.print("board temp id ");
        Serial.print(temp_index);
        Serial.print(" val ");
        Serial.print(bt);
        Serial.print("\t");
        if (temp_index % 4 == 0) Serial.println();
        temp_index++;
    }
    Serial.println();
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
    if (timer > 300) // Need an actual schedular
    {   
        // reset timer
        timer = 0;

        // Read cell and auxillary data from the BMS Driver
        auto bms_data = BMSGroup.read_data();
        print_voltages(bms_data);

        // Calculate cell_balance_statuses based on data.voltages
        // Passing in voltages, min_voltage, max_voltage; Returns cell_balance_statuses,
        // controller.update_acu_state(bms_data.voltages, bms_data.min_voltage, bms_data.max_voltage);
    

        // Retrieve the cell balance status array from the controller
        //std::array<uint16_t, num_chips> cell_balance_config = controller.get_cell_balance_params();

        // Rewrite the configuration for the chip
        //BMSGroup.write_configuration(dcto_write, cell_balance_config); 

        // Send bms_data through message interface here

    }
}
