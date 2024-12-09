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
BMSDriverGroup<1,1> BMSGroup = BMSDriverGroup<1,1>(LTC6811_Type_e::LTC6811_1);

void print_voltages() {
    
}

void setup() {
    Serial.begin(115200);
    SPI.begin();
    BMSGroup.init();
}

void loop() {
    //Serial.print("looped");
    //Serial.print("Timer:");
    //Serial.println(timer.can_bms_voltages_timer);
    std::array<uint16_t, 1> cell_balance_statuses;
    cell_balance_statuses[0] = 0x0;
    if (timer > 1000) {
        Serial.println("Entered loop!");
        Serial.println();
        timer = 0;
        auto data = BMSGroup.read_data(cell_balance_statuses);
        Serial.print("Total Voltage: ");
        Serial.println(data.total_voltage);
        // Serial.print("Minimum Voltage: ");
        // Serial.println(data.min_voltage);
        // Serial.print("Maxmimum Voltage: ");
        // Serial.println(data.max_voltage);
    }    
}

