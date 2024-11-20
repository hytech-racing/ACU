#pragma once
/* Library Includes */
#include <string.h>
#include <stdint.h>
#include <SPI.h>
#include <Arduino.h>
#include "Register_Group.h"
#include "option_enums.h"

//#ifdef HT_DEBUG_EN
//#endif

// Command Codes for 
enum class CMD_CODES_e : uint16_t {
    // WRITES
    WRITE_CONFIG = 0x1,
    WRITE_S_CONTROL = 0x14,
    WRITE_PWM = 0x20,
    WRITE_COMM = 0x721,
    // READS
    READ_CONFIG = 0x2,
    READ_CELL_VOLTAGE_GROUP_A = 0x4,
    READ_CELL_VOLTAGE_GROUP_B = 0x6,
    READ_CELL_VOLTAGE_GROUP_C = 0x8,
    READ_CELL_VOLTAGE_GROUP_D = 0xA,
    READ_GPIO_VOLTAGE_GROUP_A = 0xC,
    READ_GPIO_VOLTAGE_GROUP_B = 0xE,
    READ_STATUS_GROUP_A = 0x10,
    READ_STATUS_GROUP_B = 0x12,  
    READ_S_CONTROL = 0x16,
    READ_PWM = 0x22,
    READ_COMM = 0x722,
    // STARTS
    START_S_CONTROL = 0x19,
    START_CV_ADC_CONVERSION = 0x260,
    START_GPIO_ADC_CONVERSION = 0x460,
    START_CV_GPIO_ADC_CONVERSION = 0x46F,
    START_CV_SC_CONVERSION = 0x467,
    START_COMM = 0x723,
    // CLEARS
    CLEAR_S_CONTROL = 0x18,
    CLEAR_GPIOS = 0x712,
    CLEAR_STATUS = 0x713,
    // POLL ADC STATUS, DIAGNOSE MUX
    POLL_ADC_STATUS = 0x714,
    DIAGNOSE_MUX_POLL_STATUS = 0x715
}; 

uint16_t pec15Table_pointer[256];

class LTC6811_2_HT09 {
public:
    //Variables for the PEC functions
    static const uint16_t CRC15_POLY = 0x4599;
    Register_Group register_group;

    LTC6811_2_HT09() = default;
    LTC6811_2_HT09(int addr_) :
            address(addr_),
            pec_error(false),
            adc_mode(0x3),
            discharge_permitted(0x0) { };
    void init();

    //write register commands
    void write_config(Register_Group reg_group);
    void write_S_control();
    void write_pwm();
    void write_comm();

    Register_Group get_register_group() { return register_group; }
    Register_Group read_from_cmd_code(CMD_CODES_e cmd_code, int length);
    //read register commands
    Register_Group read_config();

    /**
     * Read/Setter Functions
    */
    void read_cell_voltages();
    void read_gpio_voltages();
    // 12 Cells
    uint8_t* read_cell_voltage_a();
    uint8_t* read_cell_voltage_b();
    uint8_t* read_cell_voltage_c();
    uint8_t* read_cell_voltage_d();

    uint8_t* read_gpio_a();
    uint8_t* read_gpio_b();

    uint8_t* read_status_a();
    uint8_t* read_status_b();

    void read_S_control();
    void read_pwm();

    void read_comm();


    // start -action- commands
    void start_S_control();
    void start_cv_adc_conversion_poll_status(CELL_SELECT cell_select);
    // void adow(ADC_MODE adc_mode, OPEN_WIRE_CURRENT_PUP pup, DISCHARGE discharge, CELL_SELECT cell_select)
    // void cvst(ADC_MODE adc_mode, SELF_TEST_MODE self_test);
    // void adol(ADC_MODE adc_mode, DISCHARGE discharge);
    void start_gpio_adc_conversion_poll_status(GPIO_SELECT gpio_select);
    // void adaxd(ADC_MODE adc_mode, GPIO_SELECT gpio_select);
    // void axst(ADC_MODE adc_mode, SELF_TEST_MODE self_test);
    // void adstat(ADC_MODE adc_mode, STATUS_GROUP_SELECT status_group);
    // void adstatd(ADC_MODE adc_mode, STATUS_GROUP_SELECT status_group);
    // void statst(ADC_MODE adc_mode, SELF_TEST_MODE self_test);
    void start_cv_gpio_adc_conversion_poll_status();
    void start_cv_SC_conversion_poll_status();


    // Clear commands
    void clear_S_control();
    void clear_gpios();
    void clear_status();

    // Miscellaneous commands
    void poll_adc_status();
    void diagnose_mux_poll_status();
    void start_comm();
    void wakeup();

    // handlers and helper functions
    void spi_write_reg(uint8_t *cmd, uint8_t *cmd_pec, uint8_t *data, uint8_t *data_pec);
    void spi_read_reg(uint8_t *cmd, uint8_t* cmd_pec, uint8_t *data_in);
    void spi_cmd(uint8_t *cmd, uint8_t* cmd_pec);
    void spi_set_chip_select(uint8_t cs);
    void write_register_group(uint16_t cmd_code, const uint8_t *buffer);
    void read_register_group(uint16_t cmd_code, uint8_t *data);
    void non_register_cmd(uint16_t cmd_code);
    uint8_t get_cmd_address();
    void init_PEC15_Table();
    void generate_pec(uint8_t *data, uint8_t *pec, int num_bytes);
    void set_pec_error(bool flag);
    bool get_pec_error();
    void set_adc_mode(ADC_MODE mode);
    const double get_adc_delay_ms();
    void set_discharge_permit(DISCHARGE permit);

    // adc timer
    bool check(uint8_t state);

private:
    uint8_t address;
    bool pec_error;
    uint8_t  adc_mode;
    uint8_t discharge_permitted;
    uint8_t chip_select = 10;
    uint8_t adc_state = 0; // 0: wait to begin voltage conversions; 1: adcs converting voltage values; 2: wait to begin gpio conversions; 3: adcs converting GPIO values
    elapsedMillis adc_timer = 0; // timer that determines wait time for ADCs to finish their conversions
};
