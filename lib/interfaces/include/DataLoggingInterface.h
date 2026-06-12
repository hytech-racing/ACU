#ifndef DATA_LOGGING_INTERFACE_H
#define DATA_LOGGING_INTERFACE_H

#include <etl/singleton.h>
#include <Arduino.h>
#include "SD.h"
#include <EEPROM.h>
#include <string>

#include "BMSDriverGroup.h"
using BMSDriverInstance_t = BMSDriverInstance<12, 2, LTC6811_Type_e::LTC6811_1>;

class DataLoggingInterface
{
public:
    DataLoggingInterface() = default;

    bool init();
    void log_data();

private:
    File data_file;
    std::string file_name;

    static const int eeprom_counter_address = 0;
    static const int eeprom_magic_address = 4;
    static const uint32_t magic_number = 0x12345678;
};

using DataLoggingInterfaceInstance = etl::singleton<DataLoggingInterface>;

#endif /* DATA_LOGGING_INTERFACE_H */