#include <SPI.h>
#include "BMSDriverGroup.h"

Elapsed_Timers_s timer;
BMSDriverGroup<1, 2> BMSGroup;

void setup() {
    Serial.begin(115200);
    SPI.begin();
    BMSGroup = BMSDriverGroup<1, 2>();
    BMSGroup.init();
}

void loop() {
    //Serial.print("looped");
    //Serial.print("Timer:");
    //Serial.println(timer.can_bms_voltages_timer);
    std::array<bool, 12> cell_balance_statuses;
    if (timer.can_bms_voltages_timer > 1000) {
        Serial.println("Entered loop!");
        timer.can_bms_voltages_timer = 0;
        //BMSGroup.read_data(dcto_read, cell_balance_statuses);
    }    
}