#include "DataLoggingInterface.h"
#include "ACUController.h"
#include "EMInterface.h"
#include "SystemTimeInterface.h"

const int DataLoggingInterface::eeprom_counter_address;
const int DataLoggingInterface::eeprom_magic_address;
const uint32_t DataLoggingInterface::magic_number;

static const char* CSV_HEADER = "timestamp_ms,em_current_A,min_cell_v," "soc_pct,lifetime_ah,soh,soe_pct,remaining_wh";

bool DataLoggingInterface::init()
{
    if (!SD.begin(BUILTIN_SDCARD))
    {
        return false;
    }

    uint32_t file_counter = 0;
    uint32_t magic_check  = 0;

    EEPROM.get(eeprom_magic_address, magic_check);
    if (magic_check != magic_number) {

        file_counter = 0;
        EEPROM.put(eeprom_magic_address,   magic_number);
        EEPROM.put(eeprom_counter_address, file_counter);
    } else
    {
        EEPROM.get(eeprom_counter_address, file_counter);
    }

    file_name = std::string("soc_log_") + std::to_string(file_counter) + std::string(".csv");

    data_file = SD.open(file_name.c_str(), FILE_WRITE);
    if (!data_file)
    {
        return false;
    }
    if (data_file.size() == 0)
    {
        data_file.println(CSV_HEADER);
    }
    data_file.close();

    EEPROM.put(eeprom_counter_address, file_counter + 1);
    return true;
}

void DataLoggingInterface::log_data()
{
    // Only write a row when the BMS has completed a fresh full voltage cycle.
    // check_clear_voltage_ready() atomically reads-and-clears the flag so
    // calling log_data() every task tick will not produce duplicate rows.
    if (!BMSDriverInstance_t::instance().check_clear_voltage_ready())
    {
        return;
    }

    auto status = ACUControllerInstance::instance().get_status();
    auto em_data = EMInterfaceInstance::instance().get_latest_data(sys_time::hal_millis());
    auto bms_data = BMSDriverInstance_t::instance().get_bms_data();

    data_file = SD.open(file_name.c_str(), FILE_WRITE);
    if (!data_file)
    {
        return;
    }

    data_file.print(sys_time::hal_millis()); data_file.print(",");
    data_file.print(em_data.em_current, 4); data_file.print(",");
    data_file.print(bms_data.min_cell_voltage, 4); data_file.print(",");
    data_file.print(status.SoC * 100.0f, 2); data_file.print(",");
    data_file.print((float)status.lifetime_ah_throughput, 2); data_file.print(",");
    data_file.print(status.SoH, 4); data_file.print(",");
    data_file.print(status.SoE_percentage, 2); data_file.print(",");
    data_file.println(status.remaining_pack_wh, 1);

    data_file.close();
}