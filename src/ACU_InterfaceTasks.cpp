#include "ACU_InterfaceTasks.h"

const auto start_time = std::chrono::high_resolution_clock::now();

// Helper: assemble ACUAllDataType_s from BMS driver data and watchdog getWatchDogData
static ACUAllDataType_s make_acu_all_data()
{
    ACUAllDataType_s out{};

    auto bms = BMSDriverInstance_t::instance().get_bms_data();
    auto fault_data = BMSFaultDataManagerInstance_t::instance().get_fault_data();
    // Copy per-cell data
    std::copy(bms.voltages.begin(), bms.voltages.end(), out.cell_voltages.begin());
    std::copy(bms.cell_temperatures.begin(), bms.cell_temperatures.end(), out.cell_temps.begin());
    std::copy(bms.board_temperatures.begin(), bms.board_temperatures.end(), out.board_temps.begin());

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
    std::copy(fault_data.consecutive_invalid_packet_counts.begin(), fault_data.consecutive_invalid_packet_counts.end(), out.consecutive_invalid_packet_counts.begin());
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
    out.core_data.hv_plus_out_voltage = ADCInterfaceInstance::instance().read_hv_plus_out_ok_voltage();
    out.core_data.main_ok_voltage = ADCInterfaceInstance::instance().read_main_ok_voltage();
    out.core_data.precharge_ok_voltage = ADCInterfaceInstance::instance().read_precharge_voltage();
    out.core_data.main_under_threshold_voltage = ADCInterfaceInstance::instance().read_main_under_threshold_voltage();
    out.core_data.precharge_under_threshold_voltage = ADCInterfaceInstance::instance().read_precharge_under_threshold_voltage();
    out.core_data.tractive_system_current = ADCInterfaceInstance::instance().read_shunt_current();
    out.core_data.acu_sm_state = ACUStateMachineInstance::instance().get_state();

    // SoC/SoH placeholders (leave unchanged here)
    auto ACUStatus = ACUControllerInstance::instance().get_status();

    out.SoC = ACUStatus.SoC;
    out.SoH = ACUStatus.SoH;
    out.SoE_percentage = ACUStatus.SoE_percentage;
    out.lifetime_ah_throughput = ACUStatus.lifetime_ah_throughput;
    out.V1 = ACUStatus.V1;
    out.remaining_pack_wh = ACUStatus.remaining_pack_wh;

    out.core_data.high_side_contactor_welded = ACUStatus.high_side_contactor_welded;
    out.core_data.low_side_contactor_welded = ACUStatus.low_side_contactor_welded;

    return out;
}

void initialize_all_interfaces()
{
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV16); // 16MHz (Arduino Clock Frequency) / 16 = 1MHz -> SPI Clock

    SPI1.begin();
    SPI1.setClockDivider(SPI_CLOCK_DIV16); // 16MHz (Arduino Clock Frequency) / 16 = 1MHz -> SPI Clock
    SPI1.setMOSI(ACUInterfaces::SPI1_MOSI_PIN); // set up pins because it's not the default SPI1 MISO
    SPI1.setSCK(ACUInterfaces::SPI1_SCK_PIN);
    SPI1.setMISO(ACUInterfaces::SPI1_MISO_PIN);

    Serial.begin(ACUInterfaces::SERIAL_BAUDRATE);
    analogReadResolution(ACUInterfaces::ANALOG_READ_RESOLUTION);
    /* Watchdog Interface */
    WatchdogInstance::create(WatchdogPinout_s {
                                ACUInterfaces::TEENSY_OK_PIN,
                                ACUInterfaces::WD_KICK_PIN,
                                ACUInterfaces::N_FAULTED_STATE_PIN,
                                ACUInterfaces::SW_NOT_OK_PIN
                            }
    );
    WatchdogInstance::instance().init();

    /* ADC Interface */
    ADCInterfaceInstance::create(ADCPinout_s {
                                    ACUInterfaces::IMD_OK_PIN,
                                    ACUInterfaces::PRECHARGE_PIN,
                                    ACUInterfaces::SHDN_OUT_PIN,
                                    ACUInterfaces::HV_PLUS_OUT_OK_PIN,
                                    ACUInterfaces::MAIN_OK_PIN,
                                    ACUInterfaces::MAIN_UNDER_THRESH_PIN,
                                    ACUInterfaces::PRECHARGE_THRESH_PIN,
                                    ACUInterfaces::TS_OUT_FILTERED_PIN,
                                    ACUInterfaces::PACK_OUT_FILTERED_PIN,
                                    ACUInterfaces::BSPD_CURRENT_PIN,
                                    ACUInterfaces::SCALED_24V_PIN,
                                    ACUInterfaces::ADC0_CS_PIN,
                                    ACUInterfaces::ADC0_MOSI_PIN,
                                    ACUInterfaces::ADC0_MISO_PIN,
                                    ACUInterfaces::ADC0_CLK_PIN,
                                    ACUInterfaces::ADC0_NOT_SHDN_PIN
                                },
                                ADCChannels_s {
                                    ACUInterfaces::ISO_PACK_N_CHANNEL,
                                    ACUInterfaces::ISO_PACK_P_CHANNEL,
                                    ACUInterfaces::PACK_VOLTAGE_SENSE_CHANNEL,
                                    ACUInterfaces::SHUNT_CURRENT_OUT_CHANNEL,
                                    ACUInterfaces::SHUNT_CURRENT_P_CHANNEL,
                                    ACUInterfaces::SHUNT_CURRENT_N_CHANNEL,
                                    ACUInterfaces::TS_OUT_FILTERED_CHANNEL,
                                    ACUInterfaces::PACK_OUT_FILTERED_CHANNEL
                                },
                                ADCConversions_s {
                                    ACUInterfaces::SHUTDOWN_CONV_FACTOR,
                                    ACUInterfaces::PRECHARGE_CONV_FACTOR,
                                    ACUInterfaces::PACK_AND_TS_OUT_CONV_FACTOR,
                                    ACUInterfaces::SHDN_OUT_CONV_FACTOR,
                                    ACUInterfaces::BSPD_CURRENT_CONV_FACTOR,
                                    ACUInterfaces::GLV_CONV_FACTOR,
                                    ACUInterfaces::STD_5V_3V3_CONVERSION_FACTOR
                                },
                                ADCScales_s {
                                    ACUInterfaces::ISO_PACK_N_SCALE,
                                    ACUInterfaces::ISO_PACK_P_SCALE,
                                    ACUInterfaces::PACK_VOLTAGE_SENSE_SCALE,
                                    ACUInterfaces::SHUNT_CURRENT_OUT_SCALE,
                                    ACUInterfaces::SHUNT_CURRENT_P_SCALE,
                                    ACUInterfaces::SHUNT_CURRENT_N_SCALE,
                                    ACUInterfaces::TS_OUT_FILTERED_SCALE,
                                    ACUInterfaces::PACK_OUT_FILTERED_SCALE
                                },
                                ADCOffsets_s {
                                    ACUInterfaces::ISO_PACK_N_OFFSET,
                                    ACUInterfaces::ISO_PACK_P_OFFSET,
                                    ACUInterfaces::PACK_VOLTAGE_SENSE_OFFSET,
                                    ACUInterfaces::SHUNT_CURRENT_OUT_OFFSET,
                                    ACUInterfaces::SHUNT_CURRENT_P_OFFSET,
                                    ACUInterfaces::SHUNT_CURRENT_N_OFFSET,
                                    ACUInterfaces::TS_OUT_FILTERED_OFFSET,
                                    ACUInterfaces::PACK_OUT_FILTERED_OFFSET
                                },
                                MAX114XChannels_s {
                                    CHANNEL_TYPE_e::NOT_USED,
                                    CHANNEL_TYPE_e::SINGLE,
                                    CHANNEL_TYPE_e::NOT_USED,
                                    CHANNEL_TYPE_e::NOT_USED
                                },
                                ACUInterfaces::ADC0_SPEED,
                                ACUInterfaces::BIT_RESOLUTION
    );
    ADCInterfaceInstance::instance().init(sys_time::hal_millis());

    /* Fault Latch Manager */
    FaultLatchManagerInstance::create();
    FaultLatchManagerInstance::instance().set_shdn_out_latched(true); // Start shdn out latch cleared

    /* BMS Driver */
    BMSDriverInstance_t::create(ACUConstants::CS, ACUConstants::CS_PER_CHIP, ACUConstants::ADDR);
    BMSDriverInstance_t::instance().init();
    /* Get Initial Pack Voltage for SoC and SoH Approximations */
    BMSDriverInstance_t::instance().read_data();

    BMSFaultDataManagerInstance_t::create();

    /* Ethernet Interface */
    ACUEthernetInterfaceInstance::create();
    ACUEthernetInterfaceInstance::instance().init_ethernet_device();

    /* CCU Interface */
    CCUInterfaceInstance::create(sys_time::hal_millis());

    /* VCR Interface */
    VCRInterfaceInstance::create(sys_time::hal_millis());

    /* EM Interface */
    EMInterfaceInstance::create(sys_time::hal_millis());

    /* Datalogger */
    // DataLoggingInterfaceInstance::create();
    // DataLoggingInterfaceInstance::instance().init();

    /* SoH Persistence Interface (lifetime Ah throughput in EEPROM) */
    SoHPersistenceInterfaceInstance::create();
    SoHPersistenceInterfaceInstance::instance().init();

    /* Only run for INITIALIZING THE AH THROUGHPUT FROM EXTERNAL ANALYSIS */
    // double historical_ah_throughput = 963.2;
    // SoHPersistenceInterfaceInstance::instance().save(historical_ah_throughput, sys_time::hal_millis(), true);

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
    auto start = sys_time::hal_micros();
    // Serial.print("PREVIOUS SPI STATE: "); Serial.println(BMSDriverInstance_t::instance().get_spi_state_name());
    BMSDriverInstance_t::instance().read_data();
    auto data = BMSDriverInstance_t::instance().get_bms_data();
    BMSFaultDataManagerInstance_t::instance().update_from_valid_packets(data.valid_read_packets,
                                                                        BMSDriverInstance_t::instance().get_current_read_group(),
                                                                        data.cs_index);

    // Serial.print("CURRENT READ GROUP: "); Serial.println(BMSDriverInstance_t::instance().get_current_read_group_name());
    // Serial.print("CURRENT SPI STATE:  "); Serial.println(BMSDriverInstance_t::instance().get_spi_state_name());
    // print_bms_data(data);

    // Serial.println();
    // auto end = sys_time::hal_micros();
    // auto diff = end - start;
    // Serial.println(diff);

    return HT_TASK::TaskResponse::YIELD;
}

std::array<bool, ACUConstants::NUM_CELLS> check_and_get_balancing_status()
{
    std::array<bool, ACUConstants::NUM_CELLS> cell_balancing_statuses = {false};
    if(ACUControllerInstance::instance().get_status().balancing_enabled)
    {
        ACUControllerInstance::instance().calculate_cell_balance_statuses(cell_balancing_statuses.data(), BMSDriverInstance_t::instance().get_bms_data().voltages.data(), ACUConstants::NUM_CELLS, BMSDriverInstance_t::instance().get_bms_data().min_cell_voltage);
    }
    return cell_balancing_statuses;
}

HT_TASK::TaskResponse write_cell_balancing_config(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
{
    BMSDriverInstance_t::instance().request_write_configuration(check_and_get_balancing_status());
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse sample_adc(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    ADCInterfaceInstance::instance().tick();
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
    WatchdogMetricsInstance::instance().reset_metrics(
        ADCInterfaceInstance::instance().read_global_lv_value(),
        ADCInterfaceInstance::instance().read_pack_out_filtered(),
        ADCInterfaceInstance::instance().read_ts_out_filtered(),
        ADCInterfaceInstance::instance().read_shdn_voltage()
    );

    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse handle_send_all_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    CCUInterfaceInstance::instance().set_system_latch_state(sys_time::hal_millis(), ADCInterfaceInstance::instance().read_shdn_out());
    ACUCANInterfaceImpl::send_all_CAN_msgs(ACUCANInterfaceInstance::instance().ccu_can_tx_buffer, &ACUCANInterfaceInstance::instance().CCU_CAN);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueue_ACU_ok_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo) {
    FaultLatchManagerInstance::instance().clear_if_not_faulted(ACUStateMachineInstance::instance().get_state() == ACUState_e::FAULTED);
    FaultLatchManagerInstance::instance().update_imd_and_bms_latches(ADCInterfaceInstance::instance().read_imd_ok(sys_time::hal_millis()), ACUControllerInstance::instance().get_status().bms_ok);

    //TODO: Where should I get veh_shdn_out_latched from?
    VCRInterfaceInstance::instance().set_monitoring_data(!FaultLatchManagerInstance::instance().get_latches().imd_fault_latched, !FaultLatchManagerInstance::instance().get_latches().bms_fault_latched, FaultLatchManagerInstance::instance().get_latches().shdn_out_latched);
    VCRInterfaceInstance::instance().handle_enqueue_acu_ok_CAN_message();

    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueue_EM_measurement_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    EM_MEASUREMENT_t msg = {};
    msg.em_current_ro = HYTECH_em_current_ro_toS(ADCInterfaceInstance::instance().read_shunt_current());
    msg.em_voltage_ro = HYTECH_em_voltage_ro_toS(ADCInterfaceInstance::instance().read_pack_voltage_sense());
    CAN_util::enqueue_msg(&msg, &Pack_EM_MEASUREMENT_hytech, ACUCANInterfaceInstance::instance().ccu_can_tx_buffer);

    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueue_ACU_core_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo) {
    auto data = make_acu_all_data();
    CCUInterfaceInstance::instance().set_ACU_data<ACUConstants::NUM_CELLS, ACUConstants::NUM_CELL_TEMPS, ACUConstants::NUM_CHIPS>(data);
    CCUInterfaceInstance::instance().handle_enqueue_acu_status_CAN_message();
    CCUInterfaceInstance::instance().handle_enqueue_acu_core_voltages_CAN_message();
    CCUInterfaceInstance::instance().handle_enqueue_acu_SoC_CAN_message();
    CCUInterfaceInstance::instance().handle_enqueue_acu_SoH_CAN_message();
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
    process_ring_buffer(ACUCANInterfaceInstance::instance().ccu_can_rx_buffer, CANInterfacesInstance::instance(), sys_time::hal_millis(), ACUCANInterfaceInstance::instance().can_recv_switch, CANInterfaceType_e::CCU);
    process_ring_buffer(ACUCANInterfaceInstance::instance().em_can_rx_buffer, CANInterfacesInstance::instance(), sys_time::hal_millis(), ACUCANInterfaceInstance::instance().can_recv_switch, CANInterfaceType_e::EM);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse idle_sample_interfaces(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo) {
    WatchdogMetricsInstance::instance().update_metrics(
        ADCInterfaceInstance::instance().read_global_lv_value(),
        ADCInterfaceInstance::instance().read_pack_out_filtered(),
        ADCInterfaceInstance::instance().read_ts_out_filtered(),
        ADCInterfaceInstance::instance().read_shdn_voltage(),
        sys_time::hal_millis());
    FaultLatchManagerInstance::instance().update_shdn_out_latch(WatchdogMetricsInstance::instance().is_shdn_out_voltage_invalid(sys_time::hal_millis()));
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse init_soh_persistence(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    // One-time setup (runs once at task registration, after both init phases complete): restore the
    // persisted lifetime Ah throughput into the controller so SoH is valid before the first eval tick.
    // The interface pushes restored data into the system; the system never depends on the interface.
    ACUControllerInstance::instance().restore_lifetime_throughput(
        SoHPersistenceInterfaceInstance::instance().get_lifetime_ah_throughput());
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse persist_soh_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    // Throttled persist of lifetime Ah throughput; the interface guards EEPROM endurance internally
    // and pulls the value from the controller's published status (interface-reads-system data flow).
    SoHPersistenceInterfaceInstance::instance().save(
        ACUControllerInstance::instance().get_status().lifetime_ah_throughput,
        sys_time::hal_millis());
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
    for (auto chip_voltages : data.voltages)
    {
        Serial.print("Cell ");
        Serial.print (chip_index); Serial.print(" ");
        if (chip_voltages)
        {
            Serial.print((chip_voltages), 4);
            Serial.print("V  ");
        }
        else
        {
            Serial.print("The voltage at "); Serial.print(chip_index); Serial.println(" is not valid. ");
        }
        chip_index++;
        if ((chip_index - 1) % ACUConstants::NUM_CHIPS == 0)
        {
            Serial.println();
        }
        // Serial.println();
    }
    Serial.println();

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

    // chip_index = 0;
    // Serial.println("Balancing status : ");
    // for(bool status : check_and_get_balancing_status()) {
    //     if (status)
    //     {
    //         Serial.print("Chip "); Serial.print(chip_index); Serial.print(" DISC\t");
    //     }
    //     chip_index++;
    // }
    // Serial.println();

    Serial.print("Number of Global Faults: ");
    auto faults = BMSFaultDataManagerInstance_t::instance().get_fault_data();
    Serial.println(faults.max_consecutive_invalid_packet_count);

    Serial.print("Valid Packet Rate: "); Serial.println(faults.valid_packet_rate);

    Serial.println("FAULTS DURING THIS BMS SAMPLE");
    for (size_t c = 0; c < ACUConstants::NUM_CHIPS; c++)
    {
        ValidPacketData_s v = data.valid_read_packets[c];
        Serial.print("CHIP #"); Serial.print(c); Serial.print(":\t");
        Serial.print(v.valid_read_cells_1_to_3); Serial.print(" ");
        Serial.print(v.valid_read_cells_4_to_6); Serial.print(" ");
        Serial.print(v.valid_read_cells_7_to_9); Serial.print(" ");
        Serial.print(v.valid_read_cells_10_to_12); Serial.print(" ");
        Serial.print(v.valid_read_gpios_1_to_3); Serial.print(" ");
        Serial.print(v.valid_read_gpios_4_to_6); Serial.print("\t");
        Serial.print(faults.chip_invalid_cmd_counts[c].invalid_cell_1_to_3_count); Serial.print(" ");
        Serial.print(faults.chip_invalid_cmd_counts[c].invalid_cell_4_to_6_count); Serial.print(" ");
        Serial.print(faults.chip_invalid_cmd_counts[c].invalid_cell_7_to_9_count); Serial.print(" ");
        Serial.print(faults.chip_invalid_cmd_counts[c].invalid_cell_10_to_12_count); Serial.print(" ");
        Serial.print(faults.chip_invalid_cmd_counts[c].invalid_gpio_1_to_3_count); Serial.print(" ");
        Serial.print(faults.chip_invalid_cmd_counts[c].invalid_gpio_4_to_6_count); Serial.println();
    }

    Serial.println();
}

::HT_TASK::TaskResponse run_data_logging(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    DataLoggingInterfaceInstance::instance().log_data();
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse debug_print(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
{
    // if (ACUControllerInstance::instance().get_status().bms_ok)
    // {
    //     Serial.print("BMS is OK\n");
    // }
    // else
    // {
    //     Serial.print("BMS is NOT OK\n");
    // }

    // Serial.printf("IMD OK: %d\n", ADCInterfaceInstance::instance().read_imd_ok(sys_time::hal_millis()));

    // Serial.print("SHDN VOLTAGE: "); Serial.print(ADCInterfaceInstance::instance().read_shdn_voltage());
    // Serial.printf("\tSHDN OUT: %d\n", ADCInterfaceInstance::instance().read_shdn_out());

    // Serial.printf("PRECHARGE VOLTAGE: %d\t", ADCInterfaceInstance::instance().read_precharge_voltage());
    // Serial.printf("PRECHARGE OUT: %d\n", ADCInterfaceInstance::instance().read_precharge_out());

    // Serial.print("TS OUT Filtered: ");
    // Serial.println(ADCInterfaceInstance::instance().read_ts_out_filtered(), 4);
    // Serial.print("PACK OUT Filtered: ");
    // Serial.println(ADCInterfaceInstance::instance().read_pack_out_filtered(), 4);

    // Serial.print("HV PLUS OUT OK VOLTAGE: ");
    // Serial.println(ADCInterfaceInstance::instance().read_hv_plus_out_ok_voltage(), 4);

    // Serial.print("MAIN OK VOLTAGE: ");
    // Serial.println(ADCInterfaceInstance::instance().read_main_ok_voltage(), 4);

    // Serial.print("MAIN UNDER THRESHOLD VOLTAGE: ");
    // Serial.println(ADCInterfaceInstance::instance().read_main_under_threshold_voltage(), 4);

    // Serial.print("PRECHARGE UNDER THRESHOLD VOLTAGE: ");
    // Serial.println(ADCInterfaceInstance::instance().read_precharge_under_threshold_voltage(), 4);

    // Serial.println();

    // Serial.print("Pack Voltage: ");
    // Serial.println(BMSDriverInstance_t::instance().get_bms_data().total_voltage, 4);

    // Serial.print("Minimum Cell Voltage: ");
    // Serial.println(BMSDriverInstance_t::instance().get_bms_data().min_cell_voltage, 4);

    // Serial.print("Maximum Cell Voltage: ");
    // Serial.println(BMSDriverInstance_t::instance().get_bms_data().max_cell_voltage, 4);

    // Serial.print("Maximum Board Temp: ");
    // Serial.println(BMSDriverInstance_t::instance().get_bms_data().max_board_temp, 4);

    // Serial.print("Maximum Cell Temp: ");
    // Serial.println(BMSDriverInstance_t::instance().get_bms_data().max_cell_temp, 4);

    // Serial.print("ACU State: ");
    // Serial.println(static_cast<int>(ACUStateMachineInstance::instance().get_state()));

    // Serial.print("CCU Charging Requested? : ");
    // Serial.println(CCUInterfaceInstance::instance().get_latest_data(sys_time::hal_millis()).charging_requested);
    // Serial.print("State of Charge: ");
    // Serial.print(ACUControllerInstance::instance().get_status().SoC * 100, 3);
    // Serial.println("%");
    // Serial.print("Measured GLV: "); Serial.print(ADCInterfaceInstance::instance().read_global_lv_value());
    // Serial.println("V");
    // Serial.println();

    // Serial.print("Is charging enabled: "); Serial.print(ACUControllerInstance::instance().get_status().balancing_enabled ? "YES" : "NO"); Serial.println(" Balancing status : ");
    // for(bool status : check_and_get_balancing_status()) {
    //     Serial.print(status);
    //     Serial.print(" ");
    // }
    // Serial.println();

    // Serial.print("Number of Global Faults: ");
    // auto faults = BMSFaultDataManagerInstance_t::instance().get_fault_data();
    // Serial.println(faults.max_consecutive_invalid_packet_count);
    // Serial.print("Valid Packet Rate: "); Serial.println(faults.valid_packet_rate);
    // Serial.println("Number of Consecutive Faults Per Chip: ");
    // for (size_t c = 0; c < ACUConstants::NUM_CHIPS; c++) {
    //    Serial.print("CHIP ");
    //     Serial.print(c);
    //     Serial.print(": ");
    //     Serial.print(faults.consecutive_invalid_packet_counts[c]);
    //     Serial.print(" ");

    //     Serial.print(faults.chip_invalid_cmd_counts[c].invalid_cell_1_to_3_count);
    //     Serial.print(" ");
    //     Serial.print(faults.chip_invalid_cmd_counts[c].invalid_cell_4_to_6_count);
    //     Serial.print(" ");
    //     Serial.print(faults.chip_invalid_cmd_counts[c].invalid_cell_7_to_9_count);
    //     Serial.print(" ");
    //     Serial.print(faults.chip_invalid_cmd_counts[c].invalid_cell_10_to_12_count);
    //     Serial.print(" ");
    //     Serial.print(faults.chip_invalid_cmd_counts[c].invalid_gpio_1_to_3_count);
    //     Serial.print(" ");
    //     Serial.print(faults.chip_invalid_cmd_counts[c].invalid_gpio_4_to_6_count);
    //     Serial.print("\t");
    //     Serial.print(" ");
    // }
    // Serial.println();

    // Serial.println("\nMAX114X Output:");
    // Serial.print(" CH 0&1: ");
    // Serial.print(ADCInterfaceInstance::instance().read_iso_pack());
    // Serial.print(" CH 2: ");
    // Serial.print(ADCInterfaceInstance::instance().read_pack_voltage_sense());
    // Serial.print(" CH 3:");
    // Serial.print(ADCInterfaceInstance::instance().read_shunt_current());
    // Serial.print(" CH 4&5: ");
    // Serial.print(ADCInterfaceInstance::instance().read_differential_shunt_current());
    // Serial.print(" CH 6: ");
    // Serial.print(ADCInterfaceInstance::instance().read_ts_out_filtered());
    // Serial.print(" CH 7: ");
    // Serial.print(ADCInterfaceInstance::instance().read_pack_out_filtered());
    // Serial.println();

    // Serial.println("\nSoC SoH SoE Info:");
    // Serial.print("SoH (0-1):   ");
    // Serial.println(ACUControllerInstance::instance().get_status().SoH, 3);
    // Serial.print("Lifetime Ah: ");
    // Serial.println(ACUControllerInstance::instance().get_status().lifetime_ah_throughput, 2);
    // Serial.print("SoC:         ");
    // Serial.print(ACUControllerInstance::instance().get_status().SoC * 100.0f, 1); Serial.println("%");
    // Serial.print("SoE:         ");
    // Serial.print(ACUControllerInstance::instance().get_status().SoE_percentage, 1); Serial.println("%");
    // Serial.print("Remaining:   ");
    // Serial.print(ACUControllerInstance::instance().get_status().remaining_pack_wh, 1); Serial.println(" Wh");

    // Print CSV header once
    // static bool header_printed = false;
    // if (!header_printed) {
    //     Serial.println("timestamp_ms|em_current_A|min_cell_v|soc_pct|lifetime_ah|soh|soe_pct|remaining_wh");
    //     header_printed = true;
    // }

    // auto status  = ACUControllerInstance::instance().get_status();
    // auto em_data = EMInterfaceInstance::instance().get_latest_data(sys_time::hal_millis());

    // Serial.print(sys_time::hal_millis());                                               Serial.print("|");
    // Serial.print(em_data.em_current, 4);                                                Serial.print("|");
    // Serial.print(BMSDriverInstance_t::instance().get_bms_data().min_cell_voltage, 4);   Serial.print("|");
    // Serial.print(status.SoC * 100.0f, 2);                                               Serial.print("|");
    // Serial.print((float)status.lifetime_ah_throughput, 2);                              Serial.print("|");
    // Serial.print(status.SoH, 4);                                                        Serial.print("|");
    // Serial.print(status.SoE_percentage, 2);                                             Serial.print("|");
    // Serial.println(status.remaining_pack_wh, 1);

    return HT_TASK::TaskResponse::YIELD;
}
