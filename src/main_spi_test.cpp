#include <SPI.h>
#include "BMSDriverGroup.h"

#include "LTCSPIInterface.h"
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
std::array<int, num_chips> addr = {6, 7};

// Instantiate BMS Driver Group
BMSDriverGroup<num_chips, num_chip_selects, chip_type::LTC6811_1> BMSGroup = BMSDriverGroup<num_chips, num_chip_selects, chip_type::LTC6811_1>(cs, cs_per_chip, addr);

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

    Serial.print(data.total_voltage / ((num_chips / 2) * 21), 4);

    Serial.println("V");

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
}

void setup()
{
    Serial.begin(115200);
    SPI.begin();
    BMSGroup.init();
    Serial.println("Setup Finished!");
    Serial.println();
}

int ci = 0;
int ji = 0;
int cell = 0;
void loop()
{

    if (timer > 250) // Need an actual schedular
    {   
        // reset timer
        timer = 0;

        // Read cell and auxiliary data from the BMS Driver
        auto bms_data = BMSGroup.read_data();
        print_voltages(bms_data);
    
    }
}
