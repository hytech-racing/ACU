#pragma once

/* Library Includes */
#include <string.h>
#include <stdint.h>
#ifdef HT_DEBUG_EN
    #include "Arduino.h"
#endif

enum class CELL_DISCHARGE_e {
    CELL_1 = 1<<0,
    CELL_2 = 1<<1,
    CELL_3 = 1<<2,
    CELL_4 = 1<<3,
    CELL_5 = 1<<4,
    CELL_6 = 1<<5,
    CELL_7 = 1<<6,
    CELL_8 = 1<<7,
    CELL_9 = 1<<8,
    CELL_10 = 1<<9,
    CELL_11 = 1<<10,
    CELL_12 = 1<<11
};

enum class GPIO_CONFIG_e {
    GPIO1_SET = 1<<0,
    GPIO2_SET = 1<<1,
    GPIO3_SET = 1<<2,
    GPIO4_SET = 1<<3,
    GPIO5_SET = 1<<4,
};

enum class S_CONTROL_e {
    S_DRIVE_HIGH = 0,
    S_1_PULSE,
    S_2_PULSES,
    S_3_PULSES,
    S_4_PULSES,
    S_5_PULSES,
    S_6_PULSES,
    S_7_PULSES,
    S_DRIVE_LOW
};

class Register_Group {
public:
    /* -------------------- Config Setup / Constructors -------------------- */

    Register_Group() = default;
    Register_Group(uint8_t gpio_, bool refon_, bool adcpot_, uint16_t undervoltage_, uint16_t overvoltage_, uint16_t discharge_, uint8_t dcto_) {
        config_byte[0] = (gpio_ << 3) | (static_cast<int>(refon_) << 2) | static_cast<int>(adcpot_);
        config_byte[1] = (undervoltage_ & 0x0FF);
        config_byte[2] = ((overvoltage_ & 0x00F) << 4) | ((undervoltage_ & 0xF00) >> 8);
        config_byte[3] = ((overvoltage_ & 0xFF0) >> 4);
        config_byte[4] = ((discharge_ & 0x0FF));
        config_byte[5] = ((dcto_ & 0x0F) << 4) | ((discharge_ & 0xF00) >> 8);
    };
    Register_Group(uint8_t* byte_arr) {
        for (int i = 0; i < 6; i++) {
            config_byte[i] = byte_arr[i];
        }
    };



    /* -------------------- SETTERS -------------------- */

    // 12 cells
    void set_register_group_A_cell(uint16_t c1, uint16_t c2, uint16_t c3) {
        cell_voltage_A[0] = c1;
        cell_voltage_A[1] = c2;
        cell_voltage_A[2] = c3;
    };

    void set_register_group_cell(uint8_t* cell_byte_arr_A, uint8_t* cell_byte_arr_B, uint8_t* cell_byte_arr_C, uint8_t* cell_byte_arr_D) {
        cell_voltage_A[0] = (cell_byte_arr_A[1] << 8) + cell_byte_arr_A[0];
        cell_voltage_A[1] = (cell_byte_arr_A[3] << 8) + cell_byte_arr_A[2];
        cell_voltage_A[2] = (cell_byte_arr_A[5] << 8) + cell_byte_arr_A[4];
        cell_voltage_B[0] = (cell_byte_arr_B[1] << 8) + cell_byte_arr_B[0];
        cell_voltage_B[1] = (cell_byte_arr_B[3] << 8) + cell_byte_arr_B[2];
        cell_voltage_B[2] = (cell_byte_arr_B[5] << 8) + cell_byte_arr_B[4];
        cell_voltage_C[0] = (cell_byte_arr_C[1] << 8) + cell_byte_arr_C[0];
        cell_voltage_C[1] = (cell_byte_arr_C[3] << 8) + cell_byte_arr_C[2];
        cell_voltage_C[2] = (cell_byte_arr_C[5] << 8) + cell_byte_arr_C[4];
        cell_voltage_D[0] = (cell_byte_arr_D[1] << 8) + cell_byte_arr_D[0];
        cell_voltage_D[1] = (cell_byte_arr_D[3] << 8) + cell_byte_arr_D[2];
        cell_voltage_D[2] = (cell_byte_arr_D[5] << 8) + cell_byte_arr_D[4];
    };
    
    // 6 GPIOs
    void set_register_group_gpio(uint16_t g1, uint16_t g2, uint16_t g3, uint16_t g4, uint16_t g5, uint16_t g6) {
        gpio_voltage_A[0] = g1;
        gpio_voltage_A[1] = g2;
        gpio_voltage_A[2] = g3;
        gpio_voltage_B[0] = g4;
        gpio_voltage_B[1] = g5;
        gpio_voltage_B[2] = g6;
    };

    void set_register_group_gpio(uint8_t* byte_arr_A, uint8_t* byte_arr_B) {
        gpio_voltage_A[0] = (byte_arr_A[1] << 8) + byte_arr_A[0];
        gpio_voltage_A[1] = (byte_arr_A[3] << 8) + byte_arr_A[2];
        gpio_voltage_A[2] = (byte_arr_A[5] << 8) + byte_arr_A[4];
        gpio_voltage_B[0] = (byte_arr_B[1] << 8) + byte_arr_B[0];
        gpio_voltage_B[1] = (byte_arr_B[3] << 8) + byte_arr_B[2];
        gpio_voltage_B[2] = (byte_arr_B[5] << 8) + byte_arr_B[4];
    };
    
    // 12 PWMS
    void set_pwm(uint8_t pwm_all) {
        for (int i = 0; i < 6; i++) {
            pwm_byte[i] = pwm_all << 4 | pwm_all;
        }
    };

    void set_pwm(uint8_t pwm1_,
                    uint8_t pwm2_,
                    uint8_t pwm3_,
                    uint8_t pwm4_,
                    uint8_t pwm5_,
                    uint8_t pwm6_,
                    uint8_t pwm7_,
                    uint8_t pwm8_,
                    uint8_t pwm9_,
                    uint8_t pwm10_,
                    uint8_t pwm11_,
                    uint8_t pwm12_) {
        pwm_byte[0] = (pwm2_ << 4) + pwm1_;
        pwm_byte[1] = (pwm4_ << 4) + pwm3_;
        pwm_byte[2] = (pwm6_ << 4) + pwm5_;
        pwm_byte[3] = (pwm8_ << 4) + pwm7_;
        pwm_byte[4] = (pwm10_ << 4) + pwm9_;
        pwm_byte[5] = (pwm12_ << 4) + pwm11_;
    };

    void set_pwm(uint8_t* byte_arr) {
        for (int i = 0; i < 6; i++) {
            pwm_byte[i] = byte_arr[i];
        }
    };

    // 9 COMM 
    void set_comm(uint8_t d0, 
                    uint8_t d1, 
                    uint8_t d2, 
                    uint8_t icom0, 
                    uint8_t icom1, 
                    uint8_t icom2, 
                    uint8_t fcom0, 
                    uint8_t fcom1,
                    uint8_t fcom2) {
        comm_byte[0] = (icom0 << 4) + ((d0 & 0xF0) >> 4);
        comm_byte[1] = ((d0 & 0x0F) << 4) + fcom0;
        comm_byte[2] = (icom1 << 4) + ((d1 & 0xF0) >> 4);
        comm_byte[3] = ((d1 & 0x0F) << 4) + fcom1;
        comm_byte[4] = (icom2 << 4) + ((d2 & 0xF0) >> 4);
        comm_byte[5] = ((d2 & 0x0F) << 4) + fcom2;
    };
    
    void set_comm(uint8_t* byte_arr) {
        for(int i = 0; i < 6; i++) {
            comm_byte[i] = byte_arr[i];
        }
    };

    // 12 S CONTROL
    void set_s_control(S_CONTROL_e pin1_,
                    S_CONTROL_e pin2_, 
                    S_CONTROL_e pin3_,
                    S_CONTROL_e pin4_,
                    S_CONTROL_e pin5_,
                    S_CONTROL_e pin6_,
                    S_CONTROL_e pin7_,
                    S_CONTROL_e pin8_,
                    S_CONTROL_e pin9_,
                    S_CONTROL_e pin10_,
                    S_CONTROL_e pin11_,
                    S_CONTROL_e pin12_) {
        s_control_byte[0] = ((int)pin2_ << 4) + (int)pin1_;
        s_control_byte[1] = ((int)pin4_ << 4) + (int)pin3_;
        s_control_byte[2] = ((int)pin6_ << 4) + (int)pin5_;
        s_control_byte[3] = ((int)pin8_ << 4) + (int)pin7_;
        s_control_byte[4] = ((int)pin10_ << 4) + (int)pin9_;
        s_control_byte[5] = ((int)pin12_ << 4) + (int)pin11_;
    }

    void set_s_control(uint8_t* byte_arr) {
        for (int i = 0; i < 6; i++) {
            s_control_byte[i] = byte_arr[i];
        }
    };

    // STATUSES
    void set_status(uint16_t vd, uint16_t sum_of_cells, uint16_t internal_temp, uint16_t Va, uint8_t byte2_, uint8_t byte3_, uint8_t byte4_, uint8_t byte5_) {
        digital_supply_voltage = vd;
        sum_of_all_cells = sum_of_cells;
        internal_die_temp = internal_temp;
        analog_supply_voltage = Va;
        status_bytes[2] = byte2_;
        status_bytes[3] = byte3_;
        status_bytes[4] = byte4_;
        status_bytes[5] = byte5_;
    };

    void set_status(uint8_t *digital_byte_arr, uint8_t *analog_byte_arr) {
        digital_supply_voltage = (digital_byte_arr[1] << 8) + digital_byte_arr[0];
        for (int i = 2; i < 6; i++) {
            status_bytes[i] = digital_byte_arr[i];
        }
        sum_of_all_cells = (analog_byte_arr[1] << 8) + analog_byte_arr[0];
        internal_die_temp = (analog_byte_arr[3] << 8) + analog_byte_arr[2];
        analog_supply_voltage = (analog_byte_arr[5] << 8) + analog_byte_arr[4];
    };

    /* -------------------- GETTERS -------------------- */

    // 12 Cells
    /*
    uint16_t get_cell1_voltage() { return cell_voltage_A[0]; };
    uint16_t get_cell2_voltage() { return cell_voltage_A[1]; };
    uint16_t get_cell3_voltage() { return cell_voltage_A[2]; };
    uint16_t get_cell4_voltage() { return cell_voltage_B[0]; };
    uint16_t get_cell5_voltage() { return cell_voltage_B[1]; };
    uint16_t get_cell6_voltage() { return cell_voltage_B[2]; };
    uint16_t get_cell7_voltage() { return cell_voltage_C[0]; };
    uint16_t get_cell8_voltage() { return cell_voltage_C[1]; };
    uint16_t get_cell9_voltage() { return cell_voltage_C[2]; };
    uint16_t get_cell10_voltage() { return cell_voltage_D[0]; };
    uint16_t get_cell11_voltage() { return cell_voltage_D[1]; };
    uint16_t get_cell12_voltage() { return cell_voltage_D[2]; };
    */
    // 6 GPIOs
    /*
    uint16_t get_gpio1_voltage() { return gpio_voltage_A[0]; };
    uint16_t get_gpio2_voltage() { return gpio_voltage_A[1]; };
    uint16_t get_gpio3_voltage() { return gpio_voltage_A[2]; };
    uint16_t get_gpio4_voltage() { return gpio_voltage_B[0]; };
    uint16_t get_gpio5_voltage() { return gpio_voltage_B[1]; };
    uint16_t get_gpio6_voltage() { return gpio_voltage_B[2]; };
    //uint16_t get_gpio_voltage(int num) { return gpio_voltage[num - 1]; }
    */
    // Configs
    uint8_t get_gpio() { return (config_byte[0] & 0b11111000) >> 3; };
    bool get_refon() { return (config_byte[0] & 0b100) > 0; };
    bool get_dten() { return (config_byte[0] & 0b10) > 0; };
    bool get_adcpot() { return (config_byte[0] & 0b1) > 0; };
    uint16_t get_undervoltage() { return ((config_byte[2] & 0x0F) << 8) | (config_byte[1]); };
    uint16_t get_overvoltage() { return ((config_byte[2] & 0xF0) >> 4) | (config_byte[3] << 4); };
    uint16_t get_discharge() { return ((config_byte[5] & 0x0F) << 8) | config_byte[4]; };
    uint8_t get_dcto() { return (config_byte[5] & 0xF0) >> 4; };

    // 12 PWM
    /*
    uint8_t get_pwm1() { return pwm_byte[0] & 0x0F; };
    uint8_t get_pwm2() { return (pwm_byte[0] & 0xF0) >> 4; };
    uint8_t get_pwm3() { return pwm_byte[1] & 0x0F; };
    uint8_t get_pwm4() { return (pwm_byte[1] & 0xF0) >> 4; };
    uint8_t get_pwm5() { return pwm_byte[2] & 0x0F; };
    uint8_t get_pwm6() { return (pwm_byte[2] & 0xF0) >> 4; };
    uint8_t get_pwm7() { return pwm_byte[3] & 0x0F; };
    uint8_t get_pwm8() { return (pwm_byte[3] & 0xF0) >> 4; };
    uint8_t get_pwm9() { return pwm_byte[4] & 0x0F; };
    uint8_t get_pwm10() { return (pwm_byte[4] & 0xF0) >> 4; };
    uint8_t get_pwm11() { return pwm_byte[5] & 0x0F; };
    uint8_t get_pwm12() { return (pwm_byte[5] & 0xF0) >> 4; };
    */
    uint8_t get_pwm(int num) {
        return (num % 2 != 0) ? (pwm_byte[num / 2] & 0x0F) : (pwm_byte[num / 2] & 0xF0 >> 4);
    };

    // 12 COMM
    uint8_t get_comm_byte0() { return ((comm_byte[1] & 0xF0) >> 4) + ((comm_byte[0] & 0x0F) << 4); };
    uint8_t get_comm_byte1() { return ((comm_byte[3] & 0xF0) >> 4) + ((comm_byte[2] & 0x0F) << 4); };
    uint8_t get_comm_byte2() { return ((comm_byte[5] & 0xF0) >> 4) + ((comm_byte[4] & 0x0F) << 4); };
    uint8_t initial_comm_control0() { return (comm_byte[0] & 0xF0) >> 4; };
    uint8_t initial_comm_control1() { return (comm_byte[2] & 0xF0) >> 4; };
    uint8_t initial_comm_control2() { return (comm_byte[4] & 0xF0) >> 4; };
    uint8_t final_comm_control0() { return comm_byte[1] & 0x0F; };
    uint8_t final_comm_control1() { return comm_byte[3] & 0x0F; };
    uint8_t final_comm_control2() { return comm_byte[5] & 0x0F; };

    // 12 S Control
    /*
    S_CONTROL_e get_s_control_pin1() { return (S_CONTROL_e)(s_control_byte[0] & 0x0F); };
    S_CONTROL_e get_s_control_pin2() { return (S_CONTROL_e)((s_control_byte[0] & 0xF0) >> 4); };
    S_CONTROL_e get_s_control_pin3() { return (S_CONTROL_e)(s_control_byte[1] & 0x0F); };
    S_CONTROL_e get_s_control_pin4() { return (S_CONTROL_e)((s_control_byte[1] & 0xF0) >> 4); };
    S_CONTROL_e get_s_control_pin5() { return (S_CONTROL_e)(s_control_byte[2] & 0x0F); };
    S_CONTROL_e get_s_control_pin6() { return (S_CONTROL_e)((s_control_byte[2] & 0xF0) >> 4); };
    S_CONTROL_e get_s_control_pin7() { return (S_CONTROL_e)(s_control_byte[3] & 0x0F); };
    S_CONTROL_e get_s_control_pin8() { return (S_CONTROL_e)((s_control_byte[3] & 0xF0) >> 4); };
    S_CONTROL_e get_s_control_pin9() { return (S_CONTROL_e)(s_control_byte[4] & 0x0F); };
    S_CONTROL_e get_s_control_pin10() { return (S_CONTROL_e)((s_control_byte[4] & 0xF0) >> 4); };
    S_CONTROL_e get_s_control_pin11() { return (S_CONTROL_e)(s_control_byte[5] & 0x0F); };
    S_CONTROL_e get_s_control_pin12() { return (S_CONTROL_e)((s_control_byte[5] & 0xF0) >> 4); };
    */
    S_CONTROL_e get_s_control_pin(int num) {
        return (num % 2 != 0) ? (S_CONTROL_e)(pwm_byte[num / 2] & 0x0F) : (S_CONTROL_e)(pwm_byte[num / 2] & 0xF0 >> 4);
    }

    // STATUS
    uint16_t get_sum_of_all_cells() { return sum_of_all_cells; };
    
    uint16_t get_internal_die_temp() { return internal_die_temp; };
    
    uint16_t get_analog_supply_voltage() { return analog_supply_voltage; };
    
    //make a better way of doing this that makes more sense please
    uint16_t get_cell_overvoltage_flags() { 
        return ((status_bytes[2] & 0b10) >> 1) + 
            ((status_bytes[2] & 0b1000) >> 2) + 
            ((status_bytes[2] & 0b100000) >> 3) + 
            ((status_bytes[2] & 0b10000000) >> 4) +
            ((status_bytes[3] & 0b10) << 3) + 
            ((status_bytes[3] & 0b1000) << 2) + 
            ((status_bytes[3] & 0b100000) << 1) + 
            (status_bytes[3] & 0b10000000) +
            ((status_bytes[3] & 0b10) << 7) + 
            ((status_bytes[3] & 0b1000) << 6) + 
            ((status_bytes[3] & 0b100000) << 5) + 
            ((status_bytes[3] & 0b10000000) << 4);
    };
    
    uint16_t get_cell_undervoltage_flags() { 
        return (status_bytes[2] & 0b1) + 
            ((status_bytes[2] & 0b100) >> 1) + 
            ((status_bytes[2] & 0b10000) >> 2) + 
            ((status_bytes[2] & 0b1000000) >> 3) +
            ((status_bytes[3] & 0b1) << 4) + 
            ((status_bytes[3] & 0b100) << 3) + 
            ((status_bytes[3] & 0b10000) << 2) + 
            ((status_bytes[3] & 0b1000000) << 1) +
            ((status_bytes[4] & 0b1) << 8) + 
            ((status_bytes[4] & 0b100) << 7) + 
            ((status_bytes[4] & 0b10000) << 6) + 
            ((status_bytes[4] & 0b1000000) << 5); 
    };

    uint16_t get_digital_supply_voltage() { return digital_supply_voltage; };
    uint8_t get_revision_code() { return (status_bytes[5] & 0xF0) >> 4; };
    bool get_mux_self_test_result() { return (status_bytes[5] & 0b10) > 0; };
    bool get_thermal_shutdown_status() { return (status_bytes[5] & 0b1) > 0; };

    /* -------------------- Primary Getter functions -------------------- */
    uint8_t* config_buf(){
        return reinterpret_cast<uint8_t*>(this->config_byte);
    }
    uint8_t* cell_voltage_buf_A(){
        return reinterpret_cast<uint8_t*>(this->cell_voltage_A);
    }
    uint8_t* cell_voltage_buf_B(){
        return reinterpret_cast<uint8_t*>(this->cell_voltage_B);
    }
    uint8_t* cell_voltage_buf_C(){
        return reinterpret_cast<uint8_t*>(this->cell_voltage_C);
    }
    uint8_t* cell_voltage_buf_D(){
        return reinterpret_cast<uint8_t*>(this->cell_voltage_D);
    }
    uint8_t* gpio_voltage_A_buf(){
        return reinterpret_cast<uint8_t*>(this->gpio_voltage_A);
    }
    uint8_t* gpio_voltage_B_buf(){
        return reinterpret_cast<uint8_t*>(this->gpio_voltage_B);
    }
    uint8_t* pwm_buf(){
        return reinterpret_cast<uint8_t*>(this->pwm_byte);
    }
    uint8_t* comm_buf(){
        return reinterpret_cast<uint8_t*>(this->comm_byte);
    }
    uint8_t* s_control_buf(){
        return reinterpret_cast<uint8_t*>(this->s_control_byte);
    }

private:
    // PRIMARIES : CELL & GPIO Voltages
    uint16_t cell_voltage_A[3];
    uint16_t cell_voltage_B[3];
    uint16_t cell_voltage_C[3];
    uint16_t cell_voltage_D[3];

    uint16_t gpio_voltage_A[3];
    uint16_t gpio_voltage_B[3];

    // SECONDARIES : CONFIG, PWM, COMM, SCONTROL
    uint8_t config_byte[6];

    uint8_t pwm_byte[6];

    uint8_t comm_byte[6];

    uint8_t s_control_byte[6];

    // STATUSES
    uint16_t sum_of_all_cells;
    uint16_t internal_die_temp;
    uint16_t analog_supply_voltage;
    
    uint16_t digital_supply_voltage; // Are the first 2 bytes
    uint8_t status_bytes[4];         // 4 subsequent bytes
};