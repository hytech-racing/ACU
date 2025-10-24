#include "ACU_InterfaceTasks.h"

const auto start_time = std::chrono::high_resolution_clock::now();

// Helper: assemble ACUAllDataType_s from BMS driver data and watchdog getWatchDogData
static ACUAllDataType_s make_acu_all_data()
{
    ACUAllDataType_s out{};

    auto bms = BMSDriver_t::instance().get_bms_data();
    auto fault_data = BMSFaultDataManager_t::instance().get_fault_data();
    // Copy per-cell data
    out.cell_voltages = bms.voltages;
    out.cell_temps = bms.cell_temperatures;
    out.board_temps = bms.board_temperatures;

    // Core data from BMS
    out.core_data.avg_cell_voltage = bms.avg_cell_voltage;
    out.core_data.max_cell_voltage = bms.max_cell_voltage;
    out.core_data.min_cell_voltage = bms.min_cell_voltage;
    out.core_data.pack_voltage = bms.total_voltage;
    out.core_data.max_cell_temp = bms.max_cell_temp;
    out.core_data.min_cell_temp = bms.min_cell_temp;
    out.core_data.max_board_temp = bms.max_board_temp;

    // IDs
    out.max_cell_voltage_id = bms.max_cell_voltage_id;
    out.min_cell_voltage_id = bms.min_cell_voltage_id;
    out.max_cell_temp_id = bms.max_cell_temperature_cell_id;

    // Faults and packet stats
    out.max_consecutive_invalid_packet_count = fault_data.max_consecutive_invalid_packet_count;
    out.consecutive_invalid_packet_counts = fault_data.consecutive_invalid_packet_counts;
    out.valid_packet_rate = fault_data.valid_packet_rate;

    auto watchdog = WatchdogMetricsInstance::instance().get_watchdog_metrics();
    // Watchdog-derived fields
    out.measured_bspd_current = ADCInterfaceInstance::instance().read_bspd_current();
    out.core_data.max_measured_glv = watchdog.max_measured_glv;
    out.core_data.max_measured_pack_out_voltage = watchdog.max_measured_pack_out_voltage;
    out.core_data.max_measured_ts_out_voltage = watchdog.max_measured_ts_out_voltage;
    out.core_data.min_measured_glv = watchdog.min_measured_glv;
    out.core_data.min_measured_pack_out_voltage = watchdog.min_measured_pack_out_voltage;
    out.core_data.min_measured_ts_out_voltage = watchdog.min_measured_ts_out_voltage;
    out.core_data.min_shdn_out_voltage = watchdog.min_shdn_out_voltage; 
    // SoC/SoH placeholders (leave unchanged here)
    out.SoC = ACUController_t::instance().get_status().SoC;

    return out;
}

void initialize_all_interfaces()
{
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV8); // 16MHz (Arduino Clock Frequency) / 8 = 2MHz -> SPI Clock
    Serial.begin(ACUInterfaces::SERIAL_BAUDRATE);
    analogReadResolution(ACUInterfaces::ANALOG_READ_RESOLUTION);

    /* Watchdog Interface */
    WatchdogInstance::create(WatchdogPinout_s {ACUInterfaces::TEENSY_OK_PIN,
                                    ACUInterfaces::WD_KICK_PIN,
                                    ACUInterfaces::N_LATCH_EN_PIN});
    WatchdogInstance::instance().init();

    /* Fault Latch Manager */
    FaultLatchManagerInstance::create();
    FaultLatchManagerInstance::instance().set_shdn_out_latched(true); // Start shdn out latch cleared

    /* BMS Driver */
    BMSDriver_t::create(ACUConstants::CS, ACUConstants::CS_PER_CHIP, ACUConstants::ADDR);
    BMSDriver_t::instance().init();
    /* Get Initial Pack Voltage for SoC and SoH Approximations */
    auto data = BMSDriver_t::instance().read_data();

    BMSFaultDataManager_t::create();
    BMSFaultDataManager_t::instance().update_from_valid_packets(data.valid_read_packets);
    /* Ethernet Interface */
    ACUEthernetInterfaceInstance::create();
    ACUEthernetInterfaceInstance::instance().init_ethernet_device();

    /* CCU Interface */
    CCUInterfaceInstance::create(millis());

    /* VCR Interface */
    VCRInterfaceInstance::create(millis());

    /* EM Interface */
    EMInterfaceInstance::create(millis());

    /* ADC Interface */
    ADCInterfaceInstance::create(ADCPinout_s {ACUInterfaces::IMD_OK_PIN,
                                ACUInterfaces::PRECHARGE_PIN,
                                ACUInterfaces::SHDN_OUT_PIN,
                                ACUInterfaces::TS_OUT_FILTERED_PIN,
                                ACUInterfaces::PACK_OUT_FILTERED_PIN,
                                ACUInterfaces::BSPD_CURRENT_PIN,
                                ACUInterfaces::SCALED_24V_PIN},
                                ADCConversions_s {ACUInterfaces::SHUTDOWN_CONV_FACTOR,
                                ACUInterfaces::PRECHARGE_CONV_FACTOR,
                                ACUInterfaces::PACK_AND_TS_OUT_CONV_FACTOR,
                                ACUInterfaces::SHDN_OUT_CONV_FACTOR,
                                ACUInterfaces::BSPD_CURRENT_CONV_FACTOR,
                                ACUInterfaces::GLV_CONV_FACTOR},
                                ACUInterfaces::BIT_RESOLUTION);
    ADCInterfaceInstance::instance().init(millis());

    /* CAN Interfaces Construct */
    CANInterfacesInstance::create(CCUInterfaceInstance::instance(), EMInterfaceInstance::instance());
}

HT_TASK::TaskResponse run_kick_watchdog(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
{
    WatchdogInstance::instance().update_watchdog_state(millis());
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse sample_bms_data(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
{
    auto data = BMSDriver_t::instance().read_data();
    BMSFaultDataManager_t::instance().update_from_valid_packets(data.valid_read_packets);
    /* Store into ACUCoreDataInstance */
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse write_cell_balancing_config(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
{
    BMSDriver_t::instance().write_configuration(dcto_write, ACUController_t::instance().get_status().cell_balancing_statuses);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse handle_send_ACU_core_ethernet_data(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
{ 
    auto data = make_acu_all_data();
    ACUEthernetInterfaceInstance::instance().handle_send_ethernet_acu_core_data(ACUEthernetInterfaceInstance::instance().make_acu_core_data_msg(data.core_data));
    
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse handle_send_ACU_all_ethernet_data(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
{
    // build a one-shot ACUAllData from current BMS + Watchdog getWatchDogData
    auto send_data = make_acu_all_data();

    ACUEthernetInterfaceInstance::instance().handle_send_ethernet_acu_all_data(ACUEthernetInterfaceInstance::instance().make_acu_all_data_msg(send_data));

    // reset local extrema after sending a report period
    WatchdogMetricsInstance::instance().reset_metrics();

    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse handle_send_all_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    CCUInterfaceInstance::instance().set_system_latch_state(millis(), ADCInterfaceInstance::instance().read_shdn_out());
    ACUCANInterfaceImpl::send_all_CAN_msgs(ACUCANInterfaceImpl::ccu_can_tx_buffer, &ACUCANInterfaceImpl::CCU_CAN);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueue_ACU_ok_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo) {
    FaultLatchManagerInstance::instance().clear_if_not_faulted(ACUStateMachineInstance::instance().get_state() == ACUState_e::FAULTED);
    FaultLatchManagerInstance::instance().update_imd_and_bms_latches(ACUController_t::instance().get_status().bms_ok, ADCInterfaceInstance::instance().read_imd_ok(millis()));

    //TODO: Where should I get veh_shdn_out_latched from?
    VCRInterfaceInstance::instance().set_monitoring_data(!FaultLatchManagerInstance::instance().get_latches().imd_fault_latched, !FaultLatchManagerInstance::instance().get_latches().bms_fault_latched, FaultLatchManagerInstance::instance().get_latches().shdn_out_latched);
    VCRInterfaceInstance::instance().handle_enqueue_acu_ok_CAN_message();
    
    //TODO: Put this in fault manager
    // Reset shdn out latch state
    // ACUDataInstance::instance().veh_shdn_out_latched = true;

    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueue_ACU_core_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo) {
    auto data = make_acu_all_data();
    CCUInterfaceInstance::instance().set_ACU_data<ACUConstants::NUM_CELLS, ACUConstants::NUM_CELL_TEMPS, ACUConstants::NUM_CHIPS>(data);
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
    etl::delegate<void(CANInterfaces_s &, const CAN_message_t &, unsigned long)> main_can_recv = etl::delegate<void(CANInterfaces_s &, const CAN_message_t &, unsigned long)>::create<ACUCANInterfaceImpl::acu_CAN_recv>();
    process_ring_buffer(ACUCANInterfaceImpl::ccu_can_rx_buffer, CANInterfacesInstance::instance(), millis(), main_can_recv); 
    process_ring_buffer(ACUCANInterfaceImpl::em_can_rx_buffer, CANInterfacesInstance::instance(), millis(), main_can_recv); 
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse idle_sample_interfaces(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo) {
    WatchdogMetricsInstance::instance().update_metrics(
        ADCInterfaceInstance::instance().read_global_lv_value(),
        ADCInterfaceInstance::instance().read_pack_out_filtered(),
        ADCInterfaceInstance::instance().read_ts_out_filtered(),
        ADCInterfaceInstance::instance().read_shdn_voltage(),
        millis());
    FaultLatchManagerInstance::instance().update_shdn_out_latch(WatchdogMetricsInstance::instance().is_shdn_out_voltage_invalid(millis()));    
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
    Serial.println(BMSFaultDataManager_t::instance().get_fault_data().max_consecutive_invalid_packet_count);
    Serial.println("Number of Consecutive Faults Per Chip: ");
    for (size_t c = 0; c < ACUConstants::NUM_CHIPS; c++) {
        Serial.print("CHIP ");
        Serial.print(c);
        Serial.print(": ");
        // Serial.print(ACUFaultDataInstance::instance().consecutive_fault_count_per_chip[c]);
        // Serial.print(" ");
        Serial.print(data.chip_invalid_cmd_counts[c].invalid_cell_1_to_3_count);
        Serial.print(" ");
        Serial.print(data.chip_invalid_cmd_counts[c].invalid_cell_4_to_6_count);
        Serial.print(" ");
        Serial.print(data.chip_invalid_cmd_counts[c].invalid_cell_7_to_9_count);
        Serial.print(" ");
        Serial.print(data.chip_invalid_cmd_counts[c].invalid_cell_10_to_12_count);
        Serial.print(" ");
        Serial.print(data.chip_invalid_cmd_counts[c].invalid_gpio_1_to_3_count);
        Serial.print(" ");
        Serial.print(data.chip_invalid_cmd_counts[c].invalid_gpio_4_to_6_count);
        Serial.print("\t");
    }
    Serial.println();
    Serial.println();
}

HT_TASK::TaskResponse debug_print(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
{
    if (ACUController_t::instance().get_status().bms_ok)
    {
        Serial.print("BMS is OK\n");
    }
    else
    {
        Serial.print("BMS is NOT OK\n");
    }

    Serial.printf("IMD OK: %d\n", ADCInterfaceInstance::instance().read_imd_ok(millis()));

    Serial.printf("SHDN VOLTAGE: %d\t", ADCInterfaceInstance::instance().read_shdn_voltage());
    Serial.printf("SHDN OUT: %d\n", ADCInterfaceInstance::instance().read_shdn_out());

    Serial.printf("PRECHARGE VOLTAGE: %d\t", ADCInterfaceInstance::instance().read_precharge_voltage());
    Serial.printf("PRECHARGE OUT: %d\n", ADCInterfaceInstance::instance().read_precharge_out());

    Serial.print("TS OUT Filtered: ");
    Serial.println(ADCInterfaceInstance::instance().read_ts_out_filtered(), 4);
    Serial.print("PACK OUT Filtered: ");
    Serial.println(ADCInterfaceInstance::instance().read_pack_out_filtered(), 4);

    Serial.println();

    Serial.print("Pack Voltage: ");
    Serial.println(BMSDriver_t::instance().get_bms_data().total_voltage, 4);

    Serial.print("Minimum Cell Voltage: ");
    Serial.println(BMSDriver_t::instance().get_bms_data().min_cell_voltage, 4);

    Serial.print("Maximum Cell Voltage: ");
    Serial.println(BMSDriver_t::instance().get_bms_data().max_cell_voltage, 4);

    Serial.print("Maximum Board Temp: ");
    Serial.println(BMSDriver_t::instance().get_bms_data().max_board_temp, 4);

    Serial.print("Maximum Cell Temp: ");
    Serial.println(BMSDriver_t::instance().get_bms_data().max_cell_temp, 4);

    Serial.printf("Cell Balance Statuses: %d\n", ACUController_t::instance().get_status().cell_balancing_statuses);

    Serial.print("ACU State: ");
    Serial.println(static_cast<int>(ACUStateMachineInstance::instance().get_state()));

    Serial.print("CCU Charging Requested? : ");
    Serial.println(CCUInterfaceInstance::instance().get_latest_data(millis()).charging_requested);
    Serial.print("State of Charge: ");
    Serial.print(ACUController_t::instance().get_status().SoC * 100, 3);
    Serial.println("%");
    Serial.print("Measured GLV: ");
    Serial.println("V");
    Serial.println();

    Serial.print("Number of Global Faults: ");
    Serial.println(BMSFaultDataManager_t::instance().get_fault_data().max_consecutive_invalid_packet_count);
    // Serial.println("Number of Consecutive Faults Per Chip: ");
    // for (size_t c = 0; c < ACUConstants::NUM_CHIPS; c++) {
    //     Serial.print("CHIP ");
    //     Serial.print(c);
    //     Serial.print(": ");
    //     Serial.print(ACUFaultDataInstance::instance().consecutive_invalid_packet_counts[c]);
    //     Serial.print("\t");
    //     Serial.print(ACUFaultDataInstance::instance().chip_invalid_cmd_counts[c].invalid_cell_1_to_3_count);
    //     Serial.print(" ");
    //     Serial.print(ACUFaultDataInstance::instance().chip_invalid_cmd_counts[c].invalid_cell_4_to_6_count);
    //     Serial.print(" ");
    //     Serial.print(ACUFaultDataInstance::instance().chip_invalid_cmd_counts[c].invalid_cell_7_to_9_count);
    //     Serial.print(" ");
    //     Serial.print(ACUFaultDataInstance::instance().chip_invalid_cmd_counts[c].invalid_cell_10_to_12_count);
    //     Serial.print(" ");
    //     Serial.print(ACUFaultDataInstance::instance().chip_invalid_cmd_counts[c].invalid_gpio_1_to_3_count);
    //     Serial.print(" ");
    //     Serial.print(ACUFaultDataInstance::instance().chip_invalid_cmd_counts[c].invalid_gpio_4_to_6_count);
    //     Serial.print(" ");
    // }
    // Serial.println();

    return HT_TASK::TaskResponse::YIELD;
}