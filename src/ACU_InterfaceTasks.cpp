#include "ACU_InterfaceTasks.h"

using chip_type = LTC6811_Type_e;

void initialize_all_interfaces()
{
    SPI.begin();
    Serial.begin(115200);
    analogReadResolution(12);

    /* ACU Data Struct */
    ACUDataInstance::create();
    ACUCoreDataInstance::create();

    /* BMS Driver */
    BMSDriverInstance<ACUConstants::NUM_CHIPS, ACUConstants::NUM_CHIP_SELECTS, chip_type::LTC6811_1>::create(ACUConstants::CS, ACUConstants::CS_PER_CHIP, ACUConstants::ADDR);
    BMSDriverInstance<ACUConstants::NUM_CHIPS, ACUConstants::NUM_CHIP_SELECTS, chip_type::LTC6811_1>::instance().init();

    /* Watchdog Interface */
    WatchdogInstance::create();
    WatchdogInstance::instance().init(); 

    /* Ethernet Interface */
    ACUEthernetInterfaceInstance::create();
    ACUEthernetInterfaceInstance::instance().init_ethernet_device();

    /* CCU Interface */
    CCUInterfaceInstance::create(sys_time::hal_millis(), 500);

    /* CAN Interfaces Construct */
    CANInterfacesInstance::create(CCUInterfaceInstance::instance());

    handle_CAN_setup(ACUCANInterfaceImpl::CCU_CAN, ACUConstants::CAN_baudrate, &ACUCANInterfaceImpl::on_ccu_can_receive);
}

bool run_kick_watchdog(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
{
    WatchdogInstance::instance().update_watchdog_state(sys_time::hal_millis());
    return true;
}

bool sample_bms_data(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
{
    auto data = BMSDriverInstance<ACUConstants::NUM_CHIPS, ACUConstants::NUM_CHIP_SELECTS, chip_type::LTC6811_1>::instance().read_data();
    
    /* Store into ACUDataInstance */
    ACUDataInstance::instance().voltages = data.voltages;
    ACUDataInstance::instance().min_cell_voltage = data.min_cell_voltage;
    ACUDataInstance::instance().max_cell_voltage = data.max_cell_voltage;
    ACUDataInstance::instance().pack_voltage = data.total_voltage;
    ACUDataInstance::instance().avg_cell_voltage = data.total_voltage / ACUConstants::NUM_CELLS;
    ACUDataInstance::instance().max_board_temp = data.max_board_temp;
    ACUDataInstance::instance().max_cell_temp = data.max_cell_temp;
    ACUDataInstance::instance().cell_temps = data.cell_temperatures;

    /* Store into ACUCoreDataInstance */
    ACUCoreDataInstance::instance().avg_cell_voltage = ACUDataInstance::instance().avg_cell_voltage;
    ACUCoreDataInstance::instance().max_cell_voltage = ACUDataInstance::instance().max_cell_voltage;
    ACUCoreDataInstance::instance().min_cell_voltage = ACUDataInstance::instance().min_cell_voltage;
    ACUCoreDataInstance::instance().pack_voltage = ACUDataInstance::instance().pack_voltage;
    ACUCoreDataInstance::instance().max_cell_temp = ACUDataInstance::instance().max_cell_temp;


    for (size_t chip = 0; chip < data.valid_read_packets.size() ; chip++)
    {
        if (!data.valid_read_packets[chip].all_valid_reads) // only if all 6 read commands fail for this chip
        {
            ACUFaultDataInstance::instance().consecutive_invalid_packet_counts[chip]++;
        }
        else
        {
            ACUFaultDataInstance::instance().consecutive_invalid_packet_counts[chip] = 0;
        }
    }
    auto start = ACUFaultDataInstance::instance().consecutive_invalid_packet_counts.begin();
    auto end = ACUFaultDataInstance::instance().consecutive_invalid_packet_counts.end();
    ACUDataInstance::instance().max_consecutive_invalid_packet_count = *etl::max_element(start, end);
    //print_bms_data(data);

    return true;
}

bool write_cell_balancing_config(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
{
    BMSDriverInstance<ACUConstants::NUM_CHIPS, ACUConstants::NUM_CHIP_SELECTS, chip_type::LTC6811_1>::instance().write_configuration(dcto_write, ACUDataInstance::instance().cell_balancing_statuses);
    return true;
}

bool handle_send_ACU_core_ethernet_data(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
{
    ACUCoreData_s data = {.pack_voltage = ACUDataInstance::instance().pack_voltage,
                          .min_cell_voltage = ACUDataInstance::instance().min_cell_voltage,
                          .avg_cell_voltage = ACUDataInstance::instance().pack_voltage / ACUConstants::NUM_CELLS,
                          .max_cell_temp = ACUDataInstance::instance().max_cell_temp };
    ACUEthernetInterfaceInstance::instance().handle_send_ethernet_acu_core_data(ACUEthernetInterfaceInstance::instance().make_acu_core_data_msg(data));
    
    return true;
}

bool handle_send_ACU_all_ethernet_data(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
{
    ACUAllDataType_s data = {};
    for (size_t cell = 0; cell < ACUConstants::NUM_CELLS; cell++)
    {
        data.cell_voltages[cell] = ACUDataInstance::instance().voltages[cell];
    }
    for (size_t temp = 0; temp < ACUConstants::NUM_CELL_TEMPS; temp++)
    {
        data.cell_temps[temp] = ACUDataInstance::instance().cell_temps[temp];
    }
    ACUEthernetInterfaceInstance::instance().handle_send_ethernet_acu_all_data(ACUEthernetInterfaceInstance::instance().make_acu_all_data_msg(data));

    return true;
}

bool handle_send_all_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    ACUCANInterfaceImpl::send_all_CAN_msgs(ACUCANInterfaceImpl::ccu_can_tx_buffer, &ACUCANInterfaceImpl::CCU_CAN);
    return true;
}

bool enqueue_CCU_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo) {
    CCUInterfaceInstance::instance().set_ACU_core_data(ACUCoreDataInstance::instance());
    CCUInterfaceInstance::instance().handle_enqueue_acu_status_CAN_message();
    CCUInterfaceInstance::instance().handle_enqueue_acu_voltages_CAN_message();
    return true;
}

bool sample_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo) {
    etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)> main_can_recv = etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)>::create<ACUCANInterfaceImpl::acu_CAN_recv>();
    process_ring_buffer(ACUCANInterfaceImpl::ccu_can_rx_buffer, CANInterfacesInstance::instance(), sys_time::hal_millis(), main_can_recv); 
    return true;
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
    Serial.print(data.total_voltage / ACUConstants::NUM_CELLS, 4);
    Serial.println("V");
    Serial.println();

    size_t chip_index = 1;
    for (auto chip_voltages : data.voltages_by_chip)
    {
        Serial.print("Chip ");
        Serial.println(chip_index);
        for (auto voltage : chip_voltages)
        {
            if (voltage)
            {
                Serial.print((*voltage), 4);
                Serial.print("V\t");
            }
        }
        chip_index++;
        Serial.println();
    }

    int cti = 0;
    for (auto temp : data.cell_temperatures)
    {
        Serial.print("temp id ");
        Serial.print(cti);
        Serial.print(" val ");
        Serial.print(temp);
        Serial.print("\t");
        if (cti % 4 == 3)
            Serial.println();
        cti++;
    }
    Serial.println();

    int temp_index = 0;
    for (auto bt : data.board_temperatures)
    {
        Serial.print("board temp id ");
        Serial.print(temp_index);
        Serial.print(" val ");
        Serial.print("");
        Serial.print(bt);
        Serial.print("\t");
        if (temp_index % 4 == 3)
            Serial.println();
        temp_index++;
    }
    Serial.print("Number of Global Faults: ");
    Serial.println(ACUDataInstance::instance().max_consecutive_invalid_packet_count);
    Serial.println("Number of Consecutive Faults Per Chip: ");
    for (size_t c = 0; c < ACUConstants::NUM_CHIPS; c++) {
        Serial.print("CHIP ");
        Serial.print(c);
        Serial.print(": ");
        // Serial.print(ACUFaultDataInstance::instance().consecutive_fault_count_per_chip[c]);
        // Serial.print(" ");
        Serial.print(data.valid_read_packets[c].valid_read_cells_1_to_3);
        Serial.print(" ");
        Serial.print(data.valid_read_packets[c].valid_read_cells_4_to_6);
        Serial.print(" ");
        Serial.print(data.valid_read_packets[c].valid_read_cells_7_to_9);
        Serial.print(" ");
        Serial.print(data.valid_read_packets[c].valid_read_cells_10_to_12);
        Serial.print(" ");
        Serial.print(data.valid_read_packets[c].valid_read_gpios_1_to_3);
        Serial.print(" ");
        Serial.print(data.valid_read_packets[c].valid_read_gpios_4_to_6);
        Serial.print("\t");
    }
    Serial.println();
    Serial.println();
}

bool debug_print(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
{
    if (ACUDataInstance::instance().acu_ok)
    {
        Serial.print("BMS is OK\n");
    }
    else
    {
        Serial.print("BMS is NOT OK\n");
    }

    Serial.printf("IMD OK: %d\n", WatchdogInstance::instance().read_imd_ok());
    Serial.printf("SHDN OUT: %d\n", WatchdogInstance::instance().read_shdn_out());

    Serial.print("TS OUT Filtered: ");
    Serial.println(WatchdogInstance::instance().read_ts_out_filtered(), 4);
    Serial.print("PACK OUT Filtered: ");
    Serial.println(WatchdogInstance::instance().read_pack_out_filtered(), 4);

    Serial.println();

    Serial.print("Pack Voltage: ");
    Serial.println(ACUDataInstance::instance().pack_voltage, 4);

    Serial.print("Minimum Cell Voltage: ");
    Serial.println(ACUDataInstance::instance().min_cell_voltage, 4);

    Serial.print("Maximum Cell Voltage: ");
    Serial.println(ACUDataInstance::instance().max_cell_voltage, 4);

    Serial.print("Maximum Board Temp: ");
    Serial.println(ACUDataInstance::instance().max_board_temp, 4);

    Serial.print("Maximum Cell Temp: ");
    Serial.println(ACUDataInstance::instance().max_cell_temp, 4);

    Serial.printf("Cell Balance Statuses: %d\n", ACUDataInstance::instance().cell_balancing_statuses);

    Serial.print("ACU State: ");
    Serial.println(static_cast<int>(ACUStateMachineInstance::instance().get_state()));

    Serial.println();

    return true;
}