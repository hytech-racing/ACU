#include <Arduino.h>
#include "DS2480B.h"
#include <AltSoftSerial.h>

AltSoftSerial dsSerial;

// Pass it into DS2480B constructor
DS2480B ds(dsSerial);


void setup()
{   Serial.begin(9600);
    // dsSerial.begin(9600);

    dsSerial.end();
    dsSerial.flush();
    // Serial.begin(19200);
    dsSerial.begin(19200);
    // uint8_t reset_result2 = ds.reset();
    // if (reset_result2){ 
    //     Serial.println(19200);
    
    // }
}


void loop()
{
    uint8_t reset_result = ds.reset();
    Serial.println(reset_result);
    delay(1000);
}


 
