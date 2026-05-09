#include "TempSensorDriver.h"

void TempSensorDriver::init() {
    //search for each id
    oneWire.reset_search(); //clear the search state so that it starts from the beginning
    oneWire.target_search(); //set up the search to find device type
}

TempSensorData_s TempSensorDriver::get_temps() {

    oneWire.beginTransaction(); // enter Data Mode, where you can send reset, pulse, configuraiton, and search commands

    // every transaction must follow the sequence reset - ROM command - function, and repeat the cycle

    oneWire.reset(); // every transaction requires a reset
    oneWire.skip(); // skip ROM command - address all devices on the bus
    oneWire.write(0x44); // temperature conversion byte (page 23 os DS2480B datasheet) - tell all devices on the bus to convert temperatures

    //need to include a delay in between (?)

    // need to address each sensor individually in order to read each of their data

    for (int i = 0; i < 7; i++) { 
        oneWire.reset();
        oneWire.select(SensorIDs[i]);
        oneWire.write(0xBE); // read scratchpad (where all of the data is stored) - must do this for each sensor
    }

    /*  address 1: 40 166 245 16 17 0 0 93
        address 2: 40 249 90 113 17 0 0 20 
        address 3: 40 117 66 17 17 0 0 81
        address 4: 40 107 206 112 17 0 0 236
        address 5: 40 224 244 112 17 0 0 199
        address 6: 40 200 146 112 17 0 0 93
    */



    for (int i = 0; i < 9; i++) {
        raw_temp_data[i] = oneWire.read();
    }

    converted_data = (raw_temp_data[1] << 8) | raw_temp_data[0];

    oneWire.endTransaction(); // back to Command Mode


}