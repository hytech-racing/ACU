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

struct SensorIDs_s {
    uint8_t addr_1;
    uint8_t addr_2;
    uint8_t addr_3;
    uint8_t addr_4;
    uint8_t addr_5;
    uint8_t addr_6;
};
class TempSensorDriver {
public: 

    TempSensorDriver();

    void init(); // searches for each id

    TempSensorData_s get_temps();

    DS2480B oneWire; //set port??
    uint8_t raw_temp_data[9];
    int16_t converted_data = 0;
};


using TempSensorDriverInstance = etl::singleton<TempSensorDriver>;

#endif