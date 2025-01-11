#include <SPI.h>
#include "BMSDriverGroup.h"
#include <LTCSPIInterface.h>
#include "Configuration.h"
#include <array>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <string>

elapsedMillis timer = 0;
using ltc_type = LTC6811_Type_e;

// Initialize chip_select, chip_select_per_chip, and address
constexpr int num_chips = 1;
constexpr int num_chip_selects = 1;
std::array<int, num_chip_selects> cs = {10};
std::array<int, num_chips> cs_per_chip = {10};
std::array<int, num_chips> addr = {4};

BMSDriverGroup<num_chips, num_chip_selects, ltc_type::LTC6811_1> BMSGroup = BMSDriverGroup<num_chips, num_chip_selects, ltc_type::LTC6811_1>(cs, cs_per_chip, addr);
std::array<uint16_t, 1> cell_balance_statuses;

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
    cell_balance_statuses[0] = 0x0;
    if (timer > 500)
    {   
        Serial.println("Enter looped!");
        // Can't be more than 1500 or t sleep will disable itself -> will notice initial update, but that's it.
        timer = 0;
        BMSGroup.manual_send_and_print();
        //auto data = BMSGroup.read_data();
        //print_voltages(data);
    }
}
