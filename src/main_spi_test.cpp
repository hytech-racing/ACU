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

BMSDriverGroup<1, 1, ltc_type::LTC6811_1> BMSGroup = BMSDriverGroup<1, 1, ltc_type::LTC6811_1>();
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
    std::array<int, 1> test_address = {4}; // For testing only
    BMSGroup.set_addresses(test_address);
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
