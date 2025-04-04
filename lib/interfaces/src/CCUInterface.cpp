#include "CCUInterface.h"

#include "ACUCANInterface.h"

void CCUInterface::receive_CCU_status_message(const CAN_message_t& msg, unsigned long curr_millis) {
    CCU_STATUS_t ccu_msg;
    Unpack_CCU_STATUS_hytech(&ccu_msg, &msg.buf[0], msg.len);
    if (ccu_msg.charger_enabled == false || (_curr_data.charging_requested && ccu_msg.charger_enabled)) {
        _curr_data.last_time_charging_requested = curr_millis;
    } 
    // Serial.printf("Charging Requested: %d\n", _curr_data.charging_requested);
}

void CCUInterface::handle_enqueue_acu_status_CAN_message() {
    BMS_STATUS_t msg = {};
    if (_curr_data.charging_requested == true) {
        msg.state = 2;
    } else {
        return;
    }
    // Serial.println("ACU STATUS CAN msg enqueued");
    CAN_util::enqueue_msg(&msg, &Pack_BMS_STATUS_hytech, ACUCANInterfaceImpl::ccu_can_tx_buffer);
}  

void CCUInterface::handle_enqueue_acu_core_voltages_CAN_message() {
    BMS_VOLTAGES_t msg = {};
    msg.high_voltage_ro = HYTECH_high_voltage_ro_toS(_acu_core_data.max_cell_voltage);
    msg.low_voltage_ro = HYTECH_low_voltage_ro_toS(_acu_core_data.min_cell_voltage); 
    msg.total_voltage_ro = HYTECH_total_voltage_ro_toS(_acu_core_data.pack_voltage);
    msg.average_voltage_ro = HYTECH_average_voltage_ro_toS(_acu_core_data.avg_cell_voltage);
    CAN_util::enqueue_msg(&msg, &Pack_BMS_VOLTAGES_hytech, ACUCANInterfaceImpl::ccu_can_tx_buffer);
}

void CCUInterface::handle_enqueue_acu_voltages_A_CAN_message() {
    BMS_DETAILED_VOLTAGES_t detailed_msg = {};
    size_t cell = 0;
    for (size_t chip = 0; chip < 6; chip++) {
        size_t group_count = (chip % 2 == 0) ? 4 : 3;
        for (size_t group = 0; group < group_count; group++) {
            detailed_msg.group_id = static_cast<uint8_t>(group);
            detailed_msg.ic_id = static_cast<uint8_t>(chip);
            //Serial.printf("Chip %d Group: %d \t", chip, group);
            //Serial.print("Chip "); Serial.print(chip); Serial.print(" Group "); Serial.print(group); Serial.print("\t");
            detailed_msg.voltage_0_ro = HYTECH_voltage_0_ro_toS(_acu_all_data.cell_voltages[cell]);
            detailed_msg.voltage_1_ro = HYTECH_voltage_1_ro_toS(_acu_all_data.cell_voltages[cell + 1]);
            detailed_msg.voltage_2_ro = HYTECH_voltage_2_ro_toS(_acu_all_data.cell_voltages[cell + 2]);
            cell += 3;
            CAN_util::enqueue_msg(&detailed_msg, &Pack_BMS_DETAILED_VOLTAGES_hytech, ACUCANInterfaceImpl::ccu_can_tx_buffer);
        }
        //Serial.println();
    }
} 

void CCUInterface::handle_enqueue_acu_voltages_B_CAN_message() {
    BMS_DETAILED_VOLTAGES_t detailed_msg = {};
    size_t cell = 63;
    for (size_t chip = 6; chip < 12; chip++) {
        size_t group_count = (chip % 2 == 0) ? 4 : 3;
        for (size_t group = 0; group < group_count; group++) {
            detailed_msg.group_id = static_cast<uint8_t>(group);
            detailed_msg.ic_id = static_cast<uint8_t>(chip);
            //Serial.printf("Chip %d Group: %d \t", chip, group);
            //Serial.print("Chip "); Serial.print(chip); Serial.print(" Group "); Serial.print(group); Serial.print("\t");
            detailed_msg.voltage_0_ro = HYTECH_voltage_0_ro_toS(_acu_all_data.cell_voltages[cell]);
            detailed_msg.voltage_1_ro = HYTECH_voltage_1_ro_toS(_acu_all_data.cell_voltages[cell + 1]);
            detailed_msg.voltage_2_ro = HYTECH_voltage_2_ro_toS(_acu_all_data.cell_voltages[cell + 2]);
            cell += 3;
            CAN_util::enqueue_msg(&detailed_msg, &Pack_BMS_DETAILED_VOLTAGES_hytech, ACUCANInterfaceImpl::ccu_can_tx_buffer);
        }
        Serial.println();
    }
} 

CCUCANInterfaceData_s CCUInterface::get_latest_data(unsigned long curr_millis) {
    _curr_data.charging_requested = (curr_millis - _curr_data.last_time_charging_requested) < _min_charging_enable_threshold;
    return _curr_data;
}

