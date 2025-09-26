#include "ACU_InterfaceTasks.h"

using chip_type = LTC6811_Type_e;
const auto start_time = std::chrono::high_resolution_clock::now();
const size_t num_total_bms_packets = ACUConstants::NUM_CHIPS * sizeof(BMSFaultCountData_s);

void initialize_all_interfaces()
{
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV2); // 16MHz (Arduino Clock Frequency) / 2 = 8MHz -> SPI Clock
    Serial.begin(ACUConstants::SERIAL_BAUDRATE);
    analogReadResolution(ACUConstants::ANALOG_READ_RESOLUTION);

    /* Watchdog Interface */
    WatchdogInstance::create();
    WatchdogInstance::instance().init(sys_time::hal_millis()); 

    /* ACU Data Struct */
    ACUDataInstance::create();
    ACUDataInstance::instance().bms_ok = true;
    ACUAllDataInstance::create();
    ACUAllDataInstance::instance().fw_version_info.fw_version_hash = convert_version_to_char_arr(device_status_t::firmware_version);
    ACUAllDataInstance::instance().fw_version_info.project_on_main_or_master = device_status_t::project_on_main_or_master;
    ACUAllDataInstance::instance().fw_version_info.project_is_dirty = device_status_t::project_is_dirty;

    /* BMS Driver */
    BMSDriverInstance<ACUConstants::NUM_CHIPS, ACUConstants::NUM_CHIP_SELECTS, chip_type::LTC6811_1>::create(ACUConstants::CS, ACUConstants::CS_PER_CHIP, ACUConstants::ADDR);
    BMSDriverInstance<ACUConstants::NUM_CHIPS, ACUConstants::NUM_CHIP_SELECTS, chip_type::LTC6811_1>::instance().init();
    /* Get Initial Pack Voltage for SoC and SoH Approximations */
    auto data = BMSDriverInstance<ACUConstants::NUM_CHIPS, ACUConstants::NUM_CHIP_SELECTS, chip_type::LTC6811_1>::instance().read_data();
    ACUDataInstance::instance().pack_voltage = data.total_voltage;

    /* Ethernet Interface */
    ACUEthernetInterfaceInstance::create();
    ACUEthernetInterfaceInstance::instance().init_ethernet_device();

    /* CCU Interface */
    CCUInterfaceInstance::create(sys_time::hal_millis(), 1000);

    /* VCR Interface */
    VCRInterfaceInstance::create(sys_time::hal_millis());

    /* EM Interface */
    EMInterfaceInstance::create(sys_time::hal_millis());

    /* CAN Interfaces Construct */
    CANInterfacesInstance::create(CCUInterfaceInstance::instance(), EMInterfaceInstance::instance());
}

HT_TASK::TaskResponse run_kick_watchdog(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
{
    WatchdogInstance::instance().update_watchdog_state(sys_time::hal_millis());
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse sample_bms_data(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
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
    ACUDataInstance::instance().min_cell_temp = data.min_cell_temp;
    ACUDataInstance::instance().cell_temps = data.cell_temperatures;
    ACUDataInstance::instance().max_board_temp = data.max_board_temp;

    /* Store into ACUCoreDataInstance */
    ACUAllDataInstance::instance().cell_voltages = data.voltages;
    ACUAllDataInstance::instance().imd_voltages = data.imd_voltages;
    ACUAllDataInstance::instance().cell_temps = data.cell_temperatures;
    ACUAllDataInstance::instance().imd_cell_temperatures = data.imd_cell_temperatures;
    ACUAllDataInstance::instance().board_temps = data.board_temperatures;
    ACUAllDataInstance::instance().imd_board_temperatures = data.imd_board_temperatures;
    ACUAllDataInstance::instance().core_data.avg_cell_voltage = ACUDataInstance::instance().avg_cell_voltage;
    ACUAllDataInstance::instance().core_data.max_cell_voltage = ACUDataInstance::instance().max_cell_voltage;
    ACUAllDataInstance::instance().core_data.min_cell_voltage = ACUDataInstance::instance().min_cell_voltage;
    ACUAllDataInstance::instance().core_data.pack_voltage = ACUDataInstance::instance().pack_voltage;
    ACUAllDataInstance::instance().core_data.max_cell_temp = ACUDataInstance::instance().max_cell_temp;
    ACUAllDataInstance::instance().core_data.min_cell_temp = ACUDataInstance::instance().min_cell_temp;
    ACUAllDataInstance::instance().core_data.max_board_temp = ACUDataInstance::instance().max_board_temp;

    ACUAllDataInstance::instance().max_cell_voltage_id = data.max_cell_voltage_id;
    ACUAllDataInstance::instance().min_cell_voltage_id = data.min_cell_voltage_id;
    ACUAllDataInstance::instance().max_cell_temp_id = data.max_cell_temperature_cell_id;

    std::array<size_t, ACUConstants::NUM_CHIPS> chip_max_invalid_cmd_counts = {};
    std::array<size_t, sizeof(BMSFaultCountData_s)> temp = {};
    size_t num_valid_packets = 0;
    
    for (size_t chip = 0; chip < data.valid_read_packets.size(); chip++)
    {
        ACUFaultDataInstance::instance().chip_invalid_cmd_counts[chip].invalid_cell_1_to_3_count = (!data.valid_read_packets[chip].valid_read_cells_1_to_3) ? ACUFaultDataInstance::instance().chip_invalid_cmd_counts[chip].invalid_cell_1_to_3_count+1 : 0;
        ACUFaultDataInstance::instance().chip_invalid_cmd_counts[chip].invalid_cell_4_to_6_count = (!data.valid_read_packets[chip].valid_read_cells_4_to_6) ? ACUFaultDataInstance::instance().chip_invalid_cmd_counts[chip].invalid_cell_4_to_6_count+1 : 0;
        ACUFaultDataInstance::instance().chip_invalid_cmd_counts[chip].invalid_cell_7_to_9_count = (!data.valid_read_packets[chip].valid_read_cells_7_to_9) ? ACUFaultDataInstance::instance().chip_invalid_cmd_counts[chip].invalid_cell_7_to_9_count+1 : 0;
        ACUFaultDataInstance::instance().chip_invalid_cmd_counts[chip].invalid_cell_10_to_12_count = (!data.valid_read_packets[chip].valid_read_cells_10_to_12) ? ACUFaultDataInstance::instance().chip_invalid_cmd_counts[chip].invalid_cell_10_to_12_count+1 : 0;
        ACUFaultDataInstance::instance().chip_invalid_cmd_counts[chip].invalid_gpio_1_to_3_count = (!data.valid_read_packets[chip].valid_read_gpios_1_to_3) ? ACUFaultDataInstance::instance().chip_invalid_cmd_counts[chip].invalid_gpio_1_to_3_count+1 : 0;
        ACUFaultDataInstance::instance().chip_invalid_cmd_counts[chip].invalid_gpio_4_to_6_count = (!data.valid_read_packets[chip].valid_read_gpios_4_to_6) ? ACUFaultDataInstance::instance().chip_invalid_cmd_counts[chip].invalid_gpio_4_to_6_count+1 : 0;
        num_valid_packets += static_cast<size_t>(data.valid_read_packets[chip].valid_read_cells_1_to_3 + data.valid_read_packets[chip].valid_read_cells_4_to_6 + data.valid_read_packets[chip].valid_read_cells_7_to_9 + 
                              data.valid_read_packets[chip].valid_read_cells_10_to_12 + data.valid_read_packets[chip].valid_read_gpios_1_to_3 + data.valid_read_packets[chip].valid_read_gpios_4_to_6);

        temp = {ACUFaultDataInstance::instance().chip_invalid_cmd_counts[chip].invalid_cell_1_to_3_count,
            ACUFaultDataInstance::instance().chip_invalid_cmd_counts[chip].invalid_cell_4_to_6_count,
            ACUFaultDataInstance::instance().chip_invalid_cmd_counts[chip].invalid_cell_7_to_9_count,
            ACUFaultDataInstance::instance().chip_invalid_cmd_counts[chip].invalid_cell_10_to_12_count,
            ACUFaultDataInstance::instance().chip_invalid_cmd_counts[chip].invalid_gpio_1_to_3_count,
            ACUFaultDataInstance::instance().chip_invalid_cmd_counts[chip].invalid_gpio_4_to_6_count};
        chip_max_invalid_cmd_counts[chip] = *etl::max_element(temp.begin(), temp.end());      
        ACUFaultDataInstance::instance().consecutive_invalid_packet_counts[chip] = chip_max_invalid_cmd_counts[chip];
    }
    ACUAllDataInstance::instance().valid_packet_rate = static_cast<float>(static_cast<float>(num_valid_packets) / num_total_bms_packets);
    ACUDataInstance::instance().max_consecutive_invalid_packet_count = *etl::max_element(chip_max_invalid_cmd_counts.begin(), chip_max_invalid_cmd_counts.end());
    ACUAllDataInstance::instance().max_consecutive_invalid_packet_count = ACUDataInstance::instance().max_consecutive_invalid_packet_count;
    ACUAllDataInstance::instance().consecutive_invalid_packet_counts = ACUFaultDataInstance::instance().consecutive_invalid_packet_counts;
    //print_bms_data(data);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse write_cell_balancing_config(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
{
    BMSDriverInstance<ACUConstants::NUM_CHIPS, ACUConstants::NUM_CHIP_SELECTS, chip_type::LTC6811_1>::instance().write_configuration(dcto_write, ACUDataInstance::instance().cell_balancing_statuses);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse handle_send_ACU_core_ethernet_data(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
{
    ACUEthernetInterfaceInstance::instance().handle_send_ethernet_acu_core_data(ACUEthernetInterfaceInstance::instance().make_acu_core_data_msg(ACUAllDataInstance::instance().core_data));
    
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse handle_send_ACU_all_ethernet_data(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
{
    ACUAllDataInstance::instance().measured_bspd_current = WatchdogInstance::instance().read_bspd_current();

    ACUEthernetInterfaceInstance::instance().handle_send_ethernet_acu_all_data(ACUEthernetInterfaceInstance::instance().make_acu_all_data_msg(ACUAllDataInstance::instance()));
    ACUAllDataInstance::instance().core_data.measured_glv = 0;
    ACUAllDataInstance::instance().core_data.measured_pack_out_voltage = 0;
    ACUAllDataInstance::instance().core_data.measured_ts_out_voltage = 0;

    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse handle_send_all_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    CCUInterfaceInstance::instance().set_system_latch_state(sys_time::hal_millis(), WatchdogInstance::instance().read_shdn_out());
    ACUCANInterfaceImpl::send_all_CAN_msgs(ACUCANInterfaceImpl::ccu_can_tx_buffer, &ACUCANInterfaceImpl::CCU_CAN);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueue_ACU_ok_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo) {
    if (ACUStateMachineInstance::instance().get_state() != ACUState_e::FAULTED) {
        ACUDataInstance::instance().veh_bms_fault_latched = ACUDataInstance::instance().veh_imd_fault_latched = false;
    } 
    if (!WatchdogInstance::instance().read_imd_ok(sys_time::hal_millis())) {
        ACUDataInstance::instance().veh_imd_fault_latched = true;
    }
    if (!ACUDataInstance::instance().bms_ok) {
        ACUDataInstance::instance().veh_bms_fault_latched = true;
    }
    VCRInterfaceInstance::instance().set_monitoring_data(!ACUDataInstance::instance().veh_imd_fault_latched, !ACUDataInstance::instance().veh_bms_fault_latched);
    VCRInterfaceInstance::instance().handle_enqueue_acu_ok_CAN_message();
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueue_ACU_core_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo) {
    CCUInterfaceInstance::instance().set_ACU_data<ACUConstants::NUM_CELLS, ACUConstants::NUM_CELL_TEMPS, ACUConstants::NUM_CHIPS>(ACUAllDataInstance::instance());
    CCUInterfaceInstance::instance().handle_enqueue_acu_status_CAN_message();
    CCUInterfaceInstance::instance().handle_enqueue_acu_core_voltages_CAN_message();
    return HT_TASK::TaskResponse::YIELD;
}


HT_TASK::TaskResponse enqueue_ACU_all_voltages_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo) {
    if (CCUInterfaceInstance::instance().is_connected_to_CCU()) {
        CCUInterfaceInstance::instance().handle_enqueue_acu_voltages_CAN_message();
    }
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueue_ACU_all_temps_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo) {
    if (CCUInterfaceInstance::instance().is_connected_to_CCU()) {
        CCUInterfaceInstance::instance().handle_enqueue_acu_temps_CAN_message();
    }
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse sample_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo) {
    etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)> main_can_recv = etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)>::create<ACUCANInterfaceImpl::acu_CAN_recv>();
    process_ring_buffer(ACUCANInterfaceImpl::ccu_can_rx_buffer, CANInterfacesInstance::instance(), sys_time::hal_millis(), main_can_recv); 
    process_ring_buffer(ACUCANInterfaceImpl::em_can_rx_buffer, CANInterfacesInstance::instance(), sys_time::hal_millis(), main_can_recv); 
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse idle_sample_interfaces(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo) {
    /* Find the maximums of GLV, Pack out, and TS Out within every ethernet send period */
    if (WatchdogInstance::instance().read_global_lv_value() > ACUAllDataInstance::instance().core_data.measured_glv) { ACUAllDataInstance::instance().core_data.measured_glv = WatchdogInstance::instance().read_global_lv_value(); }
    if (WatchdogInstance::instance().read_pack_out_filtered() > ACUAllDataInstance::instance().core_data.measured_pack_out_voltage) { ACUAllDataInstance::instance().core_data.measured_pack_out_voltage = WatchdogInstance::instance().read_pack_out_filtered(); }
    if (WatchdogInstance::instance().read_ts_out_filtered() > ACUAllDataInstance::instance().core_data.measured_ts_out_voltage) { ACUAllDataInstance::instance().core_data.measured_ts_out_voltage = WatchdogInstance::instance().read_ts_out_filtered(); }
    return HT_TASK::TaskResponse::YIELD;
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

    Serial.print("Maximum Voltage: ");
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
        Serial.print(ACUFaultDataInstance::instance().chip_invalid_cmd_counts[c].invalid_cell_1_to_3_count);
        Serial.print(" ");
        Serial.print(ACUFaultDataInstance::instance().chip_invalid_cmd_counts[c].invalid_cell_4_to_6_count);
        Serial.print(" ");
        Serial.print(ACUFaultDataInstance::instance().chip_invalid_cmd_counts[c].invalid_cell_7_to_9_count);
        Serial.print(" ");
        Serial.print(ACUFaultDataInstance::instance().chip_invalid_cmd_counts[c].invalid_cell_10_to_12_count);
        Serial.print(" ");
        Serial.print(ACUFaultDataInstance::instance().chip_invalid_cmd_counts[c].invalid_gpio_1_to_3_count);
        Serial.print(" ");
        Serial.print(ACUFaultDataInstance::instance().chip_invalid_cmd_counts[c].invalid_gpio_4_to_6_count);
        Serial.print("\t");
    }
    Serial.println();
    Serial.println();
}

HT_TASK::TaskResponse debug_print(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
{
    if (ACUDataInstance::instance().bms_ok)
    {
        Serial.print("BMS is OK\n");
    }
    else
    {
        Serial.print("BMS is NOT OK\n");
    }

    Serial.printf("IMD OK: %d\n", WatchdogInstance::instance().read_imd_ok(sys_time::hal_millis()));
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

    Serial.print("CCU Charging Requested? : ");
    Serial.println(CCUInterfaceInstance::instance().get_latest_data(sys_time::hal_millis()).charging_requested);
    Serial.print("State of Charge: ");
    Serial.print(ACUDataInstance::instance().SoC * 100, 3);
    Serial.println("%");
    Serial.print("Measured GLV: ");
    Serial.print(ACUAllDataInstance::instance().core_data.measured_glv, 3);
    Serial.println("V");
    Serial.println();

    Serial.print("Number of Global Faults: ");
    Serial.println(ACUDataInstance::instance().max_consecutive_invalid_packet_count);
    Serial.println("Number of Consecutive Faults Per Chip: ");
    for (size_t c = 0; c < ACUConstants::NUM_CHIPS; c++) {
        Serial.print("CHIP ");
        Serial.print(c);
        Serial.print(": ");
        Serial.print(ACUFaultDataInstance::instance().consecutive_invalid_packet_counts[c]);
        Serial.print("\t");
        Serial.print(ACUFaultDataInstance::instance().chip_invalid_cmd_counts[c].invalid_cell_1_to_3_count);
        Serial.print(" ");
        Serial.print(ACUFaultDataInstance::instance().chip_invalid_cmd_counts[c].invalid_cell_4_to_6_count);
        Serial.print(" ");
        Serial.print(ACUFaultDataInstance::instance().chip_invalid_cmd_counts[c].invalid_cell_7_to_9_count);
        Serial.print(" ");
        Serial.print(ACUFaultDataInstance::instance().chip_invalid_cmd_counts[c].invalid_cell_10_to_12_count);
        Serial.print(" ");
        Serial.print(ACUFaultDataInstance::instance().chip_invalid_cmd_counts[c].invalid_gpio_1_to_3_count);
        Serial.print(" ");
        Serial.print(ACUFaultDataInstance::instance().chip_invalid_cmd_counts[c].invalid_gpio_4_to_6_count);
        Serial.print(" ");
    }
    Serial.println();

    return HT_TASK::TaskResponse::YIELD;
}