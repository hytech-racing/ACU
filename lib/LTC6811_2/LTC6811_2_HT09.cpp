/* Library Includes */
#include "LTC6811_2_HT09.h"
#include <string.h>
#include <stdio.h>

// debug mode
#define DEBUG false
// SPI alternate pin definitions (not needed unless using Teensy 3.2 alternate SPI pins)
#define MOSI 7 // pin 11 by default
#define MISO 8 
// pin 12 by default
#define SCLK 14 // pin 13 by default



/* -------------------- Serial Peripheral Interface (SPI) -------------------- */

/* Note; SPI transfer is a simultaneous send/receive; the spi_write_reg function disregards the received data, and
 * the spi_read_reg function sends dummy values in order to read in values from the slave device. */
// SPI write
void LTC6811_2_HT09::spi_write_reg(uint8_t *cmd, uint8_t *cmd_pec, uint8_t *data, uint8_t *data_pec) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(this->chip_select, LOW);
    delayMicroseconds(1);
    SPI.transfer(cmd[0]);
    SPI.transfer(cmd[1]);
    SPI.transfer(cmd_pec[0]);
    SPI.transfer(cmd_pec[1]);
    for (int i = 0; i < 6; i++) {
        SPI.transfer(data[i]);
    }
    SPI.transfer(data_pec[0]);
    SPI.transfer(data_pec[1]);
    digitalWrite(this->chip_select, HIGH);
    delayMicroseconds(1);
    SPI.endTransaction();
#if DEBUG
    //Serial.println("SPI write complete.");
#endif
}

// SPI read; IF CODE DOES NOT WORK, THIS IS A GOOD PLACE TO START DEBUGGING
void LTC6811_2_HT09::spi_read_reg(uint8_t *cmd, uint8_t* cmd_pec, uint8_t *data_in) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(this->chip_select, LOW);
    delayMicroseconds(1);
    SPI.transfer(cmd[0]);
    SPI.transfer(cmd[1]);
    SPI.transfer(cmd_pec[0]);
    SPI.transfer(cmd_pec[1]);
    // read in data and PEC; bytes 0 to 5 data bytes; bytes 6 to 7 PEC bytes
    for (int i = 0; i < 8; i++) {
        data_in[i] = SPI.transfer(0); // transfer dummy value over SPI in order to read bytes into data
#if DEBUG
        Serial.print("SPI in byte: "); Serial.println(data_in[i], BIN);
#endif
    }
    digitalWrite(this->chip_select, HIGH);
    delayMicroseconds(1);
    SPI.endTransaction();
}

// SPI command
void LTC6811_2_HT09::spi_cmd(uint8_t *cmd, uint8_t* cmd_pec) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(this->chip_select, LOW);
    delayMicroseconds(1);
    SPI.transfer(cmd[0]);
    SPI.transfer(cmd[1]);
    SPI.transfer(cmd_pec[0]);
    SPI.transfer(cmd_pec[1]);
    digitalWrite(this->chip_select, HIGH);
    delayMicroseconds(1);
    SPI.endTransaction();
}

// Set SPI chip select
void LTC6811_2_HT09::spi_set_chip_select(uint8_t cs) {
    this->chip_select = cs;
}

// returns the address of the specific LTC6811-2 chip to send command to
uint8_t LTC6811_2_HT09::get_cmd_address() {
    return 0x80 | (this->address << 3);
}



/* -------------------- Packet Error Code (PEC) -------------------- */

/* Packet Error Code (PEC) generated using algorithm specified on datasheet page 53-54
 * PEC array structure:
 * pec[0] = bits 14 downto 7 of PEC
 * pec[1] = bits 6 downto 0 of PEC; LSB of pec[1] is a padded zero as per datasheet
 */
 void LTC6811_2_HT09::init_PEC15_Table() {
     for (int i = 0; i < 256; i++) {
         uint16_t remainder = i << 7;
         for (int bit = 8; bit > 0; --bit) {
             if(remainder & 0x4000) {
                 remainder = ((remainder << 1));
                 remainder = (remainder ^ CRC15_POLY);
             } else {
                 remainder = ((remainder << 1));
             }
         }
         pec15Table_pointer[i] = remainder & 0xFFFF;
     }
 }

//PEC lookup function
void LTC6811_2_HT09::generate_pec(uint8_t *data, uint8_t *pec, int num_bytes) {
    uint16_t remainder;
    uint16_t addr;
    remainder = 16; //PEC seed
    for (int i = 0; i < num_bytes; i++) {
        addr = ((remainder >> 7) ^ data[i]) & 0xff; //calculate PEC table address
        remainder = (remainder << 8 ) ^ (uint16_t) pec15Table_pointer[addr];
    }
    remainder = remainder * 2; //The CRC15 has a 0 in the LSB so the final value must be multiplied by 2
    pec[0] = (uint8_t) ((remainder >> 8) & 0xFF);
    pec[1] = (uint8_t) (remainder & 0xFF);
}
// setter for PEC error flag
void LTC6811_2_HT09::set_pec_error(bool flag) {
    pec_error = flag;
}
// getter for PEC error flag
bool LTC6811_2_HT09::get_pec_error() {
    return pec_error;
}
// Set the ADC mode per datasheet page 22; NOTE: ADCOPT is updated every time rdcfga is called
void LTC6811_2_HT09::set_adc_mode(ADC_MODE mode) {
    // NOTE: FAST, NORMAL, and FILTERED not well defined for ADCOPT; see datasheet page 61
    adc_mode = static_cast<uint8_t>(mode);
}
/* Return the delay needed for ADC operations to complete. NOTE: delays for the longer of the two possible times to
 * accommodate either ADCOPT = 0 or ADCOPT = 1 */
const double LTC6811_2_HT09::get_adc_delay_ms() {
    switch(adc_mode) {
        case 0:
            return 13;
        case 1:
            return 1.3;
        case 2:
            return 3.1;
        case 3:
            return 203;
    }
    return 0;
}
// Set the discharge permission during cell measurement, see datasheet page 73
void LTC6811_2_HT09::set_discharge_permit(DISCHARGE permit) {
    discharge_permitted = static_cast<uint8_t>(permit);
}

/* cmd array structure as related to datasheet:
    * cmd[0] = CMD0;
    * cmd[1] = CMD1;
*/
// Write register commands
// General write register group handler
void LTC6811_2_HT09::write_register_group(uint16_t cmd_code, const uint8_t *buffer) {
    uint8_t cmd[2] = {(uint8_t) (get_cmd_address() | cmd_code >> 8), (uint8_t) cmd_code};

#if DEBUG
    uint16_t dec_cmd_code = cmd[0] << 8 | cmd[1];
    Serial.print("Raw Command Code: ");
    Serial.println(cmd_code, BIN);
    Serial.print("Addressed Command Code: ");
    Serial.println(dec_cmd_code, BIN);
#endif
    uint8_t cmd_pec[2];
    uint8_t data[6];
    uint8_t data_pec[2];
    // generate PEC from command bytes
    generate_pec(cmd, cmd_pec, 2);
    for (int i = 0; i < 6; i++) {
        data[i] = buffer[i];
    }
    // generate PEC from data bytes
    generate_pec(data, data_pec, 6);
#if DEBUG
    uint16_t dec_pec = cmd_pec[0] << 8 | cmd_pec[1];
    Serial.print("Command PEC: ");
    Serial.println(dec_pec, BIN);
    Serial.println("Prepare to spi_write_reg");
#endif
    // write out via SPI 
    spi_write_reg(cmd, cmd_pec, data, data_pec);
}
// Write Configuration Register Group A
void LTC6811_2_HT09::write_config(Register_Group reg_group) {
    this->register_group = reg_group;
    write_register_group((uint16_t) CMD_CODES_e::WRITE_CONFIG, register_group.config_buf());
}
// Write S Control Register Group
void LTC6811_2_HT09::write_S_control() {
    write_register_group((uint16_t) CMD_CODES_e::WRITE_S_CONTROL, register_group.s_control_buf());
}
// Write PWM Register group
void LTC6811_2_HT09::write_pwm() {
    write_register_group((uint16_t) CMD_CODES_e::WRITE_PWM, register_group.pwm_buf());
}
// Write COMM Register Group
void LTC6811_2_HT09::write_comm() {
    write_register_group((uint16_t) CMD_CODES_e::WRITE_COMM, register_group.comm_buf());
}


// Read register commands
// general read register group handler
void LTC6811_2_HT09::read_register_group(uint16_t cmd_code, uint8_t *data) {
    // bytes 0 to 5 data bytes; bytes 6 to 7 PEC bytes
    uint8_t data_in[8];
    uint8_t cmd[2] = {(uint8_t) (get_cmd_address() | cmd_code >> 8), (uint8_t) cmd_code};
    uint8_t cmd_pec[2];
    uint8_t data_pec[2];
    // Generate PEC from command bytes
    generate_pec(cmd, cmd_pec, 2);
    // read in via SPI
    spi_read_reg(cmd, cmd_pec, data_in);
    // generate PEC from read-in data bytes
    generate_pec(data_in, data_pec, 6);

    for (int i = 0; i < 6; i++) {
            data[i] = data_in[i];
    }
    
    // Check if the PEC locally generated on the data that is read in matches the PEC that is read in
    pec_error = (data_pec[0] == data_in[6]) || (data_pec[1] == data_in[7]);
    
    // After confirming matching PECs, add the data that was read in to the array that was passed into the function
        
#if DEBUG
        Serial.println("PEC OK");
#endif
        // set flag indicating there is a PEC error
#if DEBUG
        Serial.println("PEC Error");
        Serial.print("LTC PEC: "); Serial.print(data_in[6], BIN); Serial.println(data_in[7], BIN);
        Serial.print("Calculated PEC: "); Serial.print(data_pec[0], BIN); Serial.println(data_pec[1], BIN);
#endif

}

Register_Group LTC6811_2_HT09::read_from_cmd_code(CMD_CODES_e cmd_code, int length) {
    uint8_t buffer[length];
    read_register_group((uint16_t) cmd_code, buffer);
    return {buffer};
}

// Read Configuration Register Group A
Register_Group LTC6811_2_HT09::read_config() {
    uint8_t buffer[6];
    // If PECs do not match, return empty buffer
    read_register_group((uint16_t) CMD_CODES_e::READ_CONFIG, buffer);
    return {buffer};
}

void LTC6811_2_HT09::read_cell_voltages() {
    register_group.set_register_group_cell(read_cell_voltage_a(), read_cell_voltage_b(), read_cell_voltage_c(), read_cell_voltage_d());
}

void LTC6811_2_HT09::read_gpio_voltages() {
    register_group.set_register_group_gpio(read_gpio_a(), read_gpio_b());
}

// Read Cell Voltage Register Group A
uint8_t* LTC6811_2_HT09::read_cell_voltage_a() {
    uint8_t buffer[6];
    read_register_group((uint16_t) CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_A, buffer);
    return buffer;
}
// Read Cell Voltage Register Group B
uint8_t*  LTC6811_2_HT09::read_cell_voltage_b() {
    uint8_t buffer[6];
    read_register_group((uint16_t) CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_B, buffer);
    return buffer;
}
// Read Cell Voltage Register Group C
uint8_t*  LTC6811_2_HT09::read_cell_voltage_c() {
    uint8_t buffer[6];
    read_register_group((uint16_t) CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_C, buffer);
    return buffer;
}
// Read Cell Voltage Register Group D
uint8_t* LTC6811_2_HT09::read_cell_voltage_d() {
    uint8_t buffer[6];
    read_register_group((uint16_t) CMD_CODES_e::READ_CELL_VOLTAGE_GROUP_D, buffer);
    return buffer;
}
// Read Auxiliary Register Group A
uint8_t* LTC6811_2_HT09::read_gpio_a() {
    uint8_t buffer[6];
    read_register_group((uint16_t) CMD_CODES_e::READ_GPIO_VOLTAGE_GROUP_A, buffer);
    return buffer;
}
// Read Auxiliary Register Group B
uint8_t* LTC6811_2_HT09::read_gpio_b() {
    uint8_t buffer[6];
    read_register_group((uint16_t) CMD_CODES_e::READ_GPIO_VOLTAGE_GROUP_B, buffer);
    return buffer;
}
// Read Status Register Group A
uint8_t* LTC6811_2_HT09::read_status_a() {
    uint8_t buffer[6];
    read_register_group((uint16_t) CMD_CODES_e::READ_STATUS_GROUP_A, buffer);
    return buffer;
}
// Read Status Register Group B
uint8_t* LTC6811_2_HT09::read_status_b() {
    uint8_t buffer[6];
    read_register_group((uint16_t) CMD_CODES_e::READ_STATUS_GROUP_B, buffer);
    return buffer;
}
// Read S Control Register Group
void LTC6811_2_HT09::read_S_control() {
    uint8_t buffer[6];
    read_register_group((uint16_t) CMD_CODES_e::READ_S_CONTROL, buffer);
    register_group.set_s_control(buffer);
}
// Read PWM Register Group
void LTC6811_2_HT09::read_pwm() {
    uint8_t buffer[6];
    read_register_group((uint16_t) CMD_CODES_e::READ_PWM, buffer);
    register_group.set_pwm(buffer);
}
// Read COMM Register Group
void LTC6811_2_HT09::read_comm() {
    uint8_t buffer[6];
    read_register_group((uint16_t) CMD_CODES_e::READ_COMM, buffer);
    register_group.set_comm(buffer);
}

// General non-register command handler
void LTC6811_2_HT09::non_register_cmd(uint16_t cmd_code) {
    uint8_t cmd[2] = {(uint8_t) (get_cmd_address() | (uint16_t) cmd_code >> 8), (uint8_t) cmd_code};
    uint8_t cmd_pec[2];
    // generate PEC from command bytes
    generate_pec(cmd, cmd_pec, 2);
    // write out via SPI
    spi_cmd(cmd, cmd_pec);
}



/* -------------------- START ACTION COMMANDS (S CONTROL) -------------------- */

// Start -action- commands
// Start S Control Pulsing and Poll Status
void LTC6811_2_HT09::start_S_control() {
    non_register_cmd((uint16_t) CMD_CODES_e::START_S_CONTROL);
}
// Start Cell Voltage ADC Conversion and Poll Status
void LTC6811_2_HT09::start_cv_adc_conversion_poll_status(CELL_SELECT cell_select) {
    uint16_t adc_cmd = (uint16_t) CMD_CODES_e::START_CV_ADC_CONVERSION | (adc_mode << 7) | (discharge_permitted << 4) | static_cast<uint8_t>(cell_select);
    non_register_cmd(adc_cmd);
}
// Start GPIOs ADC Conversion and Poll Status
void LTC6811_2_HT09::start_gpio_adc_conversion_poll_status(GPIO_SELECT gpio_select) {
    uint16_t adc_cmd = (uint16_t) CMD_CODES_e::START_GPIO_ADC_CONVERSION | (adc_mode << 7) | static_cast<uint8_t>(gpio_select);
    non_register_cmd(adc_cmd);
}
// Start Combined Cell Voltage and GPIO1, GPIO2 Conversion and Poll Status
void LTC6811_2_HT09::start_cv_gpio_adc_conversion_poll_status() {
    uint16_t adc_cmd = (uint16_t) CMD_CODES_e::START_CV_GPIO_ADC_CONVERSION | (adc_mode << 7) | (discharge_permitted << 4);
    non_register_cmd(adc_cmd);
}
// Start Combined Cell Voltage and SC Conversion and Poll Status
void LTC6811_2_HT09::start_cv_SC_conversion_poll_status() {
    uint16_t adc_cmd = (uint16_t) CMD_CODES_e::START_CV_SC_CONVERSION | (adc_mode << 7) | (discharge_permitted << 4);
    non_register_cmd(adc_cmd);
}



/* -------------------- NON REGISTER COMMANDS -------------------- */

// Clear register commands
// Clear Cell Voltage Register Groups
void LTC6811_2_HT09::clear_S_control() {
    non_register_cmd((uint16_t) CMD_CODES_e::CLEAR_S_CONTROL);
}
// Clear Auxiliary Register Group
void LTC6811_2_HT09::clear_gpios() {
    non_register_cmd((uint16_t) CMD_CODES_e::CLEAR_GPIOS);
}
// Clear Status Register Groups
void LTC6811_2_HT09::clear_status() {
    non_register_cmd((uint16_t) CMD_CODES_e::CLEAR_STATUS);
}
// Poll ADC Conversion Status
void LTC6811_2_HT09::poll_adc_status() {
    // NOTE: in parallel isoSPI mode, this command is not necessarily needed; see datasheet page 55
    non_register_cmd((uint16_t) CMD_CODES_e::POLL_ADC_STATUS);
}
// Diagnose MUX and Poll Status: sets MUXFAIL bit to 1 in Status Register Group B if any channel decoder fails; see datasheet pg. 32
void LTC6811_2_HT09::diagnose_mux_poll_status() {
    non_register_cmd((uint16_t) CMD_CODES_e::DIAGNOSE_MUX_POLL_STATUS);
}
// Start I2C/SPI Communication to a slave device when the LTC6811-2 acts as the master
void LTC6811_2_HT09::start_comm() {
    non_register_cmd((uint16_t) CMD_CODES_e::START_COMM);
}



/* -------------------- WAKEUP FUNCTIONS -------------------- */

/* Wakeup functions
 * The LTC6811 wakes up after receiving a differential pulse on the isoSPI port.
 * This can be achieved by toggling the chip select signal; see LTC6820 datasheet page 11.
 */
// Wakeup LTC6811 from core SLEEP state and/ or isoSPI IDLE state to ready for ADC measurements or isoSPI comms
void LTC6811_2_HT09::wakeup() {
    digitalWrite(this->chip_select, LOW);
    delayMicroseconds(1);
    SPI.transfer(0);
    digitalWrite(this->chip_select, HIGH);
    delayMicroseconds(400); //t_wake is 400 microseconds; wait that long to ensure device has turned on.
}

// Checks the timer and returns whether voltage/gpio is ready to be read
bool LTC6811_2_HT09::check(uint8_t state) {
    if (adc_state == state) {
        if (adc_state % 2 == 0 || adc_timer > get_adc_delay_ms()) {
            adc_state = (adc_state + 1) % 4;
            adc_timer = 0;
            return true;
        }
    }
    return false;
}