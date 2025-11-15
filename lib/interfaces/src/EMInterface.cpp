#include "EMInterface.h"
#include "ACUCANInterface.h"

void EMInterface::receive_EM_measurement_message(const CAN_message_t &msg, uint32_t curr_millis) {
    EM_MEASUREMENT_t em_msg;
    Unpack_EM_MEASUREMENT_hytech(&em_msg, &msg.buf[0], msg.len);
    _em_data.em_voltage = HYTECH_em_current_ro_fromS(em_msg.em_current_ro);
    _em_data.em_current = HYTECH_em_voltage_ro_fromS(em_msg.em_current_ro);
    _em_data.time_since_prev_msg_ms = curr_millis - _em_data.prev_time_stamp_ms;
    _em_data.prev_time_stamp_ms = curr_millis; 
}

EMData_s EMInterface::get_latest_data(uint32_t curr_millis) {
    return _em_data;
}
