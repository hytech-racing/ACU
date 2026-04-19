#include "TempSensorDriver.h"

void TempSensorDriver::init() {
    //search for each id
    oneWire.reset_search(); //clear the search state so that it starts from the beginning
    oneWire.target_search(); //set up the search to find device type
}

TempSensorData_s TempSensorDriver::get_temps() {

    oneWire.beginTransaction();

    oneWire.reset(); // every transaction requires a reset
    oneWire.skip(); // skip ROM command - address all devices on the bus
    oneWire.write(0x44); // temperature conversion byte (page 23 os DS2480B datasheet)

    //include a delay in between 

    oneWire.reset(); 
    // need to address each sensor individually in order to read each of their data
    oneWire.write(0xBE); // read scratchpad (where all of the data is stored)

    for (int i = 0; i < 9; i++) {
        raw_temp_data[i] = oneWire.read();
    }

    converted_data = (raw_temp_data[1] << 8) | raw_temp_data[0];


}