#include "ACU_InterfaceTasks.h"

void initialize_all_interfaces() {
    SPI.begin();
    Serial.begin(115200);
    analogReadResolution(12);
    /* ACU Data Struct */
    ACUDataInstance::create();
    /* BMS Driver */
    BMSDriverInstance<NUM_CHIPS, NUM_CHIP_SELECTS, chip_type::LTC6811_1>::create(CS, CS_PER_CHIP, ADDR);
    BMSDriverInstance<NUM_CHIPS, NUM_CHIP_SELECTS, chip_type::LTC6811_1>::instance().init();
    /* Watchdog Interface */
    WatchdogInstance::create();
    WatchdogInstance::instance().init();
}

bool run_kick_watchdog(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    WatchdogInstance::instance().update_watchdog_state(sys_time::hal_millis());
    return true;
}

void get_bms_data() {
    auto data = BMSDriverInstance<NUM_CHIPS, NUM_CHIP_SELECTS, chip_type::LTC6811_1>::instance().read_data();
    handle_bms_data(data);
}


void write_cell_balancing_config() {
    BMSDriverInstance<NUM_CHIPS, NUM_CHIP_SELECTS, chip_type::LTC6811_1>::instance().write_configuration(dcto_write, ACUDataInstance::instance().cb);
}

template <typename bms_data>
void handle_bms_data(bms_data data) {
    /* Store into ACUDataInstance */
    ACUDataInstance::instance().voltages = data.voltages;
    ACUDataInstance::instance().min_cell_voltage = data.min_cell_voltage;
    ACUDataInstance::instance().max_cell_voltage = data.max_cell_voltage;
    ACUDataInstance::instance().pack_voltage = data.total_voltage;
    ACUDataInstance::instance().max_board_temp = data.max_board_temp;
    ACUDataInstance::instance().max_cell_temp = data.max_cell_temp;
}

/* Print Functions */
template <typename bms_data>
void print_bms_data(bms_data data)
{
    Serial.print("Total Voltage: ");
    Serial.print(data.total_voltage, 4);
    Serial.println("V");

    Serial.print("Minimum Voltage: ");
    Serial.print(data.min_cell_voltage, 4);
    Serial.print("V\tLocation of Minimum Voltage: ");
    Serial.println(data.min_cell_voltage_id);

    Serial.print("Maxmimum Voltage: ");
    Serial.print(data.max_cell_voltage, 4);
    Serial.print("V\tLocation of Maximum Voltage: ");
    Serial.println(data.max_cell_voltage_id);

    Serial.print("Average Voltage: ");

    Serial.print(data.total_voltage / ((NUM_CHIPS / 2) * 21), 4);

    Serial.println("V");

    Serial.println();

    size_t chip_index = 1;
    for(auto chip_voltages : data.voltages_by_chip )

    {
        Serial.print("Chip ");
        Serial.println(chip_index);
        for(auto voltage : chip_voltages)
        {
            if(voltage)
            {
                Serial.print((*voltage), 4);
                Serial.print("V\t");
            }
        }
        chip_index++;
        Serial.println();
    }

    int cti = 0;
    for(auto temp : data.cell_temperatures)
    {
        Serial.print("temp id ");
        Serial.print(cti);
        Serial.print(" val ");
        Serial.print(temp);
        Serial.print("\t");
        if (cti % 4 == 3) Serial.println();
        cti++;
    }
    Serial.println();

    int temp_index = 0;
    for(auto bt : data.board_temperatures)
    {
        Serial.print("board temp id");
        Serial.print(temp_index);
        Serial.print(" val ");
        Serial.print("");
        Serial.print(bt);
        Serial.print("\t");
        if (temp_index % 4 == 3) Serial.println();
        temp_index++;
    }
    Serial.println();
    Serial.println();
}

void print_watchdog_data() {
    Serial.printf("IMD OK: %d\n", WatchdogInstance::instance().read_imd_ok());
    Serial.printf("SHDN OUT: %d\n", WatchdogInstance::instance().read_shdn_out());

    Serial.print("TS OUT Filtered: ");
    Serial.println(WatchdogInstance::instance().read_ts_out_filtered(), 4);
    Serial.print("PACK OUT Filtered: ");
    Serial.println(WatchdogInstance::instance().read_pack_out_filtered(), 4);

    Serial.println();
}

void set_bit(uint16_t &value, uint8_t index, bool bitValue) {
    if (index >= 16) return; // Ensure index is within range

    if (bitValue) {
        value |= (1 << index);  // Set the bit
    } else {
        value &= ~(1 << index); // Clear the bit
    }
}