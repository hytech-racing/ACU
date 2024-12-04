#include <SPI.h>
#include "BMSDriverGroup.h"

elapsedMillis timer = 0;
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
    if (timer > 1000) {
        Serial.println("Entered loop!");
        timer = 0;
        //BMSGroup.read_data(dcto_read, cell_balance_statuses);
        
    }    
}