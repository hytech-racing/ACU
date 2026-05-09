#ifndef TempSensorDriver_H
#define TempSensorDriver_H
#include "DS2480B.h"
#include <Arduino.h>
#include <etl/singleton.h>


struct TempSensorData_s {
    float temp_1 = 0;
    float temp_2 = 0;
    float temp_3 = 0;
    float temp_4 = 0;
    float temp_5 = 0;
    float temp_6 = 0;
};

// hard coded ROM IDs of DS1820Bs based on addresses collected with an arduino when validating the harneses in the accumulator 
uint8_t SensorIDs [6][8] = {
    {0x28, 0xA6, 0xF5, 0x10, 0x11, 0x00, 0x00, 0x5D}, 
    {0x28, 0xF9, 0x5A, 0x71, 0x11, 0x00, 0x00, 0x14}, 
    {0x28, 0x75, 0x42, 0x11, 0x11, 0x00, 0x00, 0x51},
    {0x28, 0x6B, 0xCE, 0x70, 0x11, 0x00, 0x00, 0xEC},
    {0x28, 0xE0, 0xF4, 0x70, 0x11, 0x00, 0x00, 0xC7},
    {0x28, 0xC8, 0x92, 0x70, 0x11, 0x00, 0x00, 0x5D}
};

    /*  address 1: 40 166 245 16 17 0 0 93
        address 2: 40 249 90 113 17 0 0 20 
        address 3: 40 117 66 17 17 0 0 81
        address 4: 40 107 206 112 17 0 0 236
        address 5: 40 224 244 112 17 0 0 199
        address 6: 40 200 146 112 17 0 0 93

        first byte 
    */


class TempSensorDriver {
public: 

    TempSensorDriver();

    void init(); // searches for each id

    TempSensorData_s get_temps();

    DS2480B oneWire; //set port??
    uint8_t raw_temp_data[9];
    int16_t converted_data = 0;

private:
    uint8_t temp_sensor_data;
};


using TempSensorDriverInstance = etl::singleton<TempSensorDriver>;

#endif