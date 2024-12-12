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
    Serial.println("Setup Finished!");
    Serial.println(); 
}

void loop() {
    //Serial.print("looped");
    //Serial.print("Timer:");
    //Serial.println(timer.can_bms_voltages_timer);
    std::array<uint16_t, 1> cell_balance_statuses;
    cell_balance_statuses[0] = 0x0;
    if (timer > 1000) {  // Can't be more than 1500 or t sleep will disable itself -> will notice initial update, but that's it.
        Serial.println("Entered loop!");
        //Serial.println();
        timer = 0;
        auto data = BMSGroup.read_data();
        //   delay(100000); 
        Serial.print("Total Voltage: ");
        Serial.println(data.total_voltage);

        Serial.print("Minimum Voltage: ");
        Serial.println(data.min_voltage); 
        Serial.print("Location of Minimum Voltage: ");
        Serial.println(data.min_voltage_cell_id);

        Serial.print("Maxmimum Voltage: ");
        Serial.println(data.max_voltage);
        Serial.print("Location of Maximum Voltage: ");
        Serial.println(data.max_voltage_cell_id);
        Serial.println();
    }    
}

