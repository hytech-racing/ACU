#ifndef __ACU_HT09_H__
#define __ACU_HT09_H__

/* System Includes */
#include <string>
#include <Metro.h>
#include <Arduino.h>
#include <HyTech_CAN.h>
#include "FlexCAN_T4.h"
#include "hytech.h"
#include "LTC6811_2_HT09.h"

// CONSTANT DEFINITIONS: define important values, such as IC count and cells per IC
#define TOTAL_IC 12                  // Number of LTC6811-2 ICs that are used in the accumulator
#define EVEN_IC_CELLS 12             // Number of cells monitored by ICs with even addresses
#define ODD_IC_CELLS 9               // Number of cells monitored by ICS with odd addresses
#define CHIP_SELECT_GROUP_ONE 9      // Chip select for first LTC6820 corresponding to first group of cells
#define CHIP_SELECT_GROUP_TWO 10     // Chip select for second LTC6820 corresponding to second group of cells
#define THERMISTORS_PER_IC 4         // Number of cell temperature monitoring thermistors connected to each IC
#define MAX_SUCCESSIVE_FAULTS 50     // Number of successive faults permitted before AMS fault is broadcast over CAN
#define MIN_VOLTAGE 30000            // Minimum allowable single cell voltage in units of 100μV
#define MAX_VOLTAGE 42000            // Maxiumum allowable single cell voltage in units of 100μV
#define MAX_TOTAL_VOLTAGE 5330000    // Maximum allowable pack total voltage in units of 100μV
#define MAX_THERMISTOR_VOLTAGE 26225 // Maximum allowable pack temperature corresponding to 60C in units 100μV
#define MAX_PACK_CHARGE 48600        // Maximum charge on cells
#define BALANCE_ON true
#define BALANCE_COOL 6000       // Sets balancing duty cycle as 33.3%
#define BALANCE_STANDARD 4000   // Sets balancing duty cycle as 50%
#define BALANCE_HOT 3000        // Sets balancing duty cycle as 66%
#define BALANCE_CONTINUOUS 2000 // Sets balancing duty cycle as 100%
#define BALANCE_MODE 1          // Mode 0 is normal balance, mode 1 is progressive balance

// ACU Shunt measurements pin definitions
#define CURR_SHUNT A2 // Include pin def in name for all
#define PACK_FILTERED A3
#define TS_OUT_FILTERED A4

Metro charging_timer = Metro(5000); // Timer to check if charger is still talking to ACU
Metro CAN_timer = Metro(2);         // Timer that spaces apart writes for CAN messages so as to not saturate CAN bus
Metro print_timer = Metro(500);
Metro balance_timer(BALANCE_STANDARD);
Metro timer_CAN_em_forward(100);
Metro timer_shunt(100);

bool next_pulse = true;    // AMS ok pulse
uint16_t min_voltage = 65535;
uint16_t max_voltage = 0;
bool overtemp_fault_state = false;    // enter fault state if 20 successive faults occur
bool uv_fault_state = false;          // enter fault state if 20 successive faults occur
bool ov_fault_state = false;          // enter fault state if 20 successive faults occur
bool pack_ov_fault_state = false;     // enter fault state if 20 successive faults occur

// CAN OBJECT AND VARIABLE DECLARATIONS
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> ENERGY_METER_CAN;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> TELEM_CAN;
CAN_message_t msg;

struct IC_Cell_Location
{
    int icIndex;
    int cellIndex;
    std::string toString() const;
};

class ACU_HT09
{
public:
    /**
     * Transfers CAN message to em_measurement and em_status struct 
     * @post writes CAN message to energy meter CAN line
    */
    inline void forward_CAN_em()
    {
        if (timer_CAN_em_forward.check())
        {
            em_measurement.write(msg.buf);
            msg.id = ID_EM_MEASUREMENT;
            msg.len = sizeof(em_measurement);
            TELEM_CAN.write(msg);

            em_status.write(msg.buf);
            msg.id = ID_EM_STATUS;
            msg.len = sizeof(em_status);
            TELEM_CAN.write(msg);
        }
    }
 
    /**
     * Check whether all LTC6811-2's are at the same state and ready to be read
     * @pre ICs are instantiated in setup() with a starting state of 0
     * @return true if all 12 ICs have same state as input, else false
    */
    bool check_ics(int state)
    {
        for (LTC6811_2_HT09 ic_unit : ic)
        {
            if (!ic_unit.check(state))
            {
                return false;
            }
        }
        return true;
    }



    /* -------------------- Fault Checks -------------------- */

    /**
     * Checks if there are any voltage faults and updates instance variables
     * @pre read_voltages() is called otherwise there is no new data to update with
     * @post updates all of the tracker under/over voltage variables and sends info to bms_status struct
    */
    void voltage_fault_check()
    {
        // detect any uv fault conditions, set appropriate error flags, and print relevant message to console
        uv_fault_counter = (min_voltage < MIN_VOLTAGE) ? uv_fault_counter++ : 0;
        uv_fault_state = uv_fault_counter > MAX_SUCCESSIVE_FAULTS;
        bms_status.set_undervoltage(uv_fault_state);

        // detect any ov fault conditions, set appropriate error flags, and print relevant message to console
        ov_fault_counter = (max_voltage > MAX_VOLTAGE) ? ov_fault_counter++ : 0;
        ov_fault_state = ov_fault_counter > MAX_SUCCESSIVE_FAULTS;
        bms_status.set_overvoltage(ov_fault_state);

        // detect any pack ov fault conditions, set appropriate error flags, and print relevant message to console
        pack_ov_fault_counter = (total_voltage > MAX_TOTAL_VOLTAGE) ? pack_ov_fault_counter++ : 0;
        pack_ov_fault_state = pack_ov_fault_counter > MAX_SUCCESSIVE_FAULTS;
        bms_status.set_total_voltage_high(pack_ov_fault_state);
    }

    /**
     * Checks if there are any temperature/humidity faults and updates instance variables
     * @pre read_gpios() is called otherwise there is no new data to update with
     * @post updates all of the tracker over temp variables and sends info to bms_status struct
    */
    void temp_fault_check()
    {
        overtemp_fault_counter = (max_thermistor_voltage > MAX_THERMISTOR_VOLTAGE) ? overtemp_fault_counter++ : 0;
        overtemp_fault_state = overtemp_fault_counter > MAX_SUCCESSIVE_FAULTS;
        bms_status.set_discharge_overtemp(overtemp_fault_state);
    }

    /**
     * Checks if there are any active faults via tracker variables & calls debugging print statements if true
     * @return true if min/max voltage is outside valid range or if fault tracker variables are true
     * Calls has_active_voltage_fault(); otherwise false
    */
    static bool has_active_fault();

    /**
     * Checks if there are any active voltage/temperature tracker booleans are true
     * @return boolean; true if any of overtemp, undervoltage, overvoltage, or pack overvoltage fault states are true
     * else returns false
    */
    static bool has_active_voltage_fault() { return overtemp_fault_state || uv_fault_state || ov_fault_state || pack_ov_fault_state; }



    /* -------------------- Set / Read / Print Voltage and GPIOs -------------------- */

    /**
     * @param voltageObj reference CAN struct
     * @param minV IC's current min voltage
     * @param maxV IC's current max voltage
     * @param totalV IC's current total voltage
     * @param count number of applicable cells 
     * @post update voltageObj's min, max, total and average
    */
    void setVoltageData(BMS_voltages &voltageObj, uint16_t minV, int maxV, int totalV, int count);

    /**
     * @param tempObj reference to CAN struct
     * @param temps reference to 2D array of gpio voltages
     * @param minLoc location of lowest
     * @param maxLoc location of highest
     * @param totalTemp total temperature
     * @param count number of applicable temp/humidity sensors
     * @post update tempObj's min/max location, total, and average temps
    */
    void setTemperatureData(auto &tempObj, const auto &temps, const IC_Cell_Location &minLoc, const IC_Cell_Location &maxLoc, uint32_t totalTemp, int count);
    
    /**
     * Reads Cell Voltage registers A/B/C/D from LTC6811-2 IC, which contains data on cell voltages 
     * @pre ICs are in adc_state 0 or 1, which represent waiting to begin or currently reading Voltages
     * @post if in adc_state 0, all of the ICs are configured with a Register Group; 
     * if 1, all the ICs are read and the max/min value/locations are updated
     * NOTE: voltages are read in with units of 100μV
    */
    void read_voltages();

    /**
     * Reads GPIO registers A/B from LTC6811-2 IC, which contains data on temperature and humidity 
     * @pre ICs are in adc_state 2 or 3, which represent waiting to begin or currently reading GPIOs
     * @post if in adc_state 2, all of the ICs are configured with a Register Group; 
     * if 3, all the ICs are read and the max/min value/locations are updated
    */
    void read_gpio();

    /**
     * read_XXX() helper method to update the maximum voltage, humidity, or temperature location
     * @param arr_val is the value we are reading at [ic_index][cell_index]
     * @param max_val is the current stored maximum
     * @param location is the specific temp/voltage/humidity location var we want to keep track of
     * @param ic_index is the IC we are looking at
     * @param cell_index is the cell we are looking at
     * @pre arr_val has been given proper value by accessing IC data from SPI through Command Code
     * @post no return but if arr_val exceeds the known max, then update the location and max_val
     */
    void update_maximum_location(uint16_t arr_val, uint16_t &max_val, IC_Cell_Location &location, int ic_index, int cell_index)
    {
        if (arr_val > max_val)
        {
            max_val = arr_val;
            location.icIndex = ic_index;
            location.cellIndex = cell_index;
        }
    }

    /**
     * read_XXX() helper method to update the minimum voltage, humidity, or temperature location
     * @param arr_val is the value we are reading at [ic_index][cell_index]
     * @param min_val is the current stored minimum
     * @param location is the specific temp/voltage/humidity location var we want to keep track of
     * @param ic_index is the IC we are looking at
     * @param cell_index is the cell we are looking at
     * @pre arr_val has been given proper value by accessing IC data from SPI through Command Code
     * @post no return but if arr_val is lower than the known minimum, then update the location and min_val
     */
    void update_minimum_location(uint16_t arr_val, uint16_t &min_val, IC_Cell_Location &location, int ic_index, int cell_index)
    {
        if (arr_val < min_val)
        {
            min_val = arr_val;
            location.icIndex = ic_index;
            location.cellIndex = cell_index;
        }
    }

    /**
     * Prints the cell voltages and voltage information
     * @pre the read_voltages() function is called so that the max/min value/locations are known
     * @post Overvoltage, undervoltage, faults, totals, averages are printed
    */
    void print_voltages()
    {
        Serial.println("------------------------------------------------------------------------------------------------------------------------------------------------------------");
        if (min_voltage < MIN_VOLTAGE)
        {
            Serial.print("UNDERVOLTAGE FAULT: ");
            Serial.print("IC #: ");
            Serial.print(min_voltage_location.icIndex);
            Serial.print("\tCell #: ");
            Serial.print(min_voltage_location.cellIndex);
            Serial.print("\tFault Voltage: ");
            Serial.print(min_voltage / 10000.0, 4);
            Serial.print("\tConsecutive fault #: ");
            Serial.println(uv_fault_counter);
        }
        if (max_voltage > MAX_VOLTAGE)
        {
            Serial.print("OVERVOLTAGE FAULT: ");
            Serial.print("IC #: ");
            Serial.print(max_voltage_location.icIndex);
            Serial.print("\tCell #: ");
            Serial.println(max_voltage_location.cellIndex);
            Serial.print("\tFault Voltage: ");
            Serial.print(max_voltage / 10000.0, 4);
            Serial.print("\tConsecutive fault #: ");
            Serial.println(ov_fault_counter);
        }
        if (total_voltage > MAX_TOTAL_VOLTAGE)
        {
            Serial.print("PACK OVERVOLTAGE:");
            Serial.print("\tConsecutive fault #: ");
            Serial.println(pack_ov_fault_counter);
        }

        Serial.print("Total pack voltage: ");
        Serial.print(total_voltage / 10000.0, 4);
        Serial.print("V\t");
        Serial.print("Max voltage differential: ");
        Serial.print(max_voltage / 10000.0 - min_voltage / 10000.0, 4);
        Serial.println("V");
        Serial.print("AMS status: ");
        Serial.println(bms_status.get_state() == BMS_STATE_DISCHARGING ? "Discharging" : "Charging");

        Serial.println("------------------------------------------------------------------------------------------------------------------------------------------------------------");
        Serial.print("Max Voltage: ");
        Serial.print(cell_voltages[max_voltage_location.icIndex][max_voltage_location.cellIndex] / 10000.0, 4);
        Serial.print("V \t ");
        Serial.print("Min Voltage: ");
        Serial.print(cell_voltages[min_voltage_location.icIndex][min_voltage_location.cellIndex] / 10000.0, 4);
        Serial.print("V \t");
        Serial.print("Avg Voltage: ");
        Serial.print(total_voltage / 1260000.0, 4);
        Serial.println("V \t");
        Serial.println("------------------------------------------------------------------------------------------------------------------------------------------------------------");
        Serial.println("Raw Cell Voltages\t\t\t\t\t\t\t\t\t\t\t\t\tBalancing Status");
        Serial.print("\tC0\tC1\tC2\tC3\tC4\tC5\tC6\tC7\tC8\tC9\tC10\tC11\t\t");
        Serial.println(currently_balancing ? "\tC0\tC1\tC2\tC3\tC4\tC5\tC6\tC7\tC8\tC9\tC10\tC11" : "");

        for (int ic = 0; ic < TOTAL_IC; ic++)
        {
            Serial.print("IC");
            Serial.print(ic);
            Serial.print("\t");
            for (int cell = 0; cell < EVEN_IC_CELLS; cell++)
            {
                Serial.print(cell_voltages[ic][cell] / 10000.0, 4);
                Serial.print("V\t");
            }
            if (currently_balancing)
            {
                Serial.print("\t\t");
                for (int cell = 0; cell < EVEN_IC_CELLS; cell++)
                {
                    Serial.print(cell_balance_status[ic][cell]);
                    Serial.print("\t");
                }
            }
            Serial.println();
        }
    }
    
    /**
     * Prints the temperature and humidity sensor data from GPIOs
     * @pre the read_gpios() function is called so that the max/min value/locations are known
     * @post Max thermistor/humidity, board temps, faults, totals, averages are printed
    */
    void print_gpios()
    {
        Serial.println("------------------------------------------------------------------------------------------------------------------------------------------------------------");
        if (max_thermistor_voltage > MAX_THERMISTOR_VOLTAGE)
        {
            Serial.print("OVERTEMP FAULT: ");
            Serial.print("\tConsecutive fault #: ");
            Serial.println(overtemp_fault_counter);
        }
        Serial.print("Max Board Temp: ");
        Serial.print(gpio_temps[max_board_temp_location.icIndex][max_board_temp_location.cellIndex], 3);
        Serial.print("C \t ");
        Serial.print("Min Board Temp: ");
        Serial.print(gpio_temps[min_board_temp_location.icIndex][min_board_temp_location.cellIndex], 3);
        Serial.print("C \t");
        Serial.print("Avg Board Temp: ");
        Serial.print(total_board_temps / 6, 3);
        Serial.println("C \t");
        Serial.print("Max Thermistor Temp: ");
        Serial.print(gpio_temps[max_thermistor_location.icIndex][max_thermistor_location.cellIndex], 3);
        Serial.print("C \t");
        Serial.print("Min Thermistor Temp: ");
        Serial.print(gpio_temps[min_thermistor_location.icIndex][min_thermistor_location.cellIndex], 3);
        Serial.print("C \t");
        Serial.print("Avg Thermistor Temp: ");
        Serial.print(total_thermistor_temps / 48, 3);
        Serial.println("C \t");
        Serial.print("Max Humidity: ");
        Serial.print(gpio_temps[max_humidity_location.icIndex][max_humidity_location.cellIndex], 3);
        Serial.println("% \t ");
        Serial.println("------------------------------------------------------------------------------------------------------------------------------------------------------------");
        Serial.println("Raw Segment Temperatures");
        Serial.println("                  \tT0\tT1\tT2\tT3");
        for (int ic = 0; ic < TOTAL_IC; ic++)
        {
            Serial.print("Cell Temperatures");
            Serial.print(ic);
            Serial.print("\t");
            for (int cell = 0; cell < 4; cell++)
            {
                Serial.print(gpio_temps[ic][cell], 3);
                Serial.print("C\t");
            }
            if ((ic % 2))
            {
                Serial.print("PCB Temps: ");
                Serial.print(gpio_temps[ic][4], 3);
                Serial.print("C\t");
            }
            else
            {
                Serial.print("PCB Humidity: ");
                Serial.print(gpio_temps[ic][4], 3);
                Serial.print("%\t");
            }
            Serial.print("\t");
            Serial.println();
        }
        Serial.println("------------------------------------------------------------------------------------------------------------------------------------------------------------");
    }



    /* -------------------- CAN & Updating-------------------- */

    /**
     * Primary CAN Writer Function
     * Sets voltage/temperature data onto BMS structs
     * @pre max/min voltage/temperature/humidity data has been gathered
     * @post Writes CAN messages for general information & detailed CAN messages
    */
    void write_CAN_messages();

    /**
     * Parses through the Charging Control Unit (CCU) CAN status to see if we are charging
     * @post if CCU CAN message enters TELEM_CAN bus, switch says BMS_STATE_DICHARGING to BMS_STATE_CHARGING
     * parse incoming CAN messages for CCU status message and changes the state of the BMS in software
    */
    void parse_CAN_CCU_status();

    /**
     * Forwarding CAN message function
     * @post if else statement is redundant, but writes EM message to TELEM_CAM
    */
    static void parse_energy_meter_can_message(const CAN_message_t &RX_msg);

    /**
     * write_CAN_messages() helper method, used for every applicable struct (BMS_XXX, ACU_SHUNT, etc.)
     * @param timer milliseonds
     * @param timer_threshold milliseconds
     * @param msg_id CAN ID for certain BMS CAN struct
     * @param msg_len buffer bit size
     * @param write_func writer function for specific BMS CAN struct
     * @post if timer exceeds threshold, write 
    */
    void write_CAN_message_helper(elapsedMillis &timer, uint32_t timer_threshold, uint32_t msg_id,
                                  size_t msg_len, std::function<void(uint8_t *)> write_func);

    /**
     * Helper method for writing detailed CAN
     * @param timer milliseconds
     * @param threshold milliseconds
     * @param action Calls either write_CAN_detailed_voltages() or write_CAN_detailed_temps() 
     * @post If the timer exceeds the threshold, we execute the action and reset the timer
    */
    void write_CAN_detailed_helper(elapsedMillis &timer, uint32_t threshold, std::function<void()> action);

    /**
     * Extender method for write_CAN_messages(); sends specific data of every can_voltage group 
     * and every cell in that group
     * @pre Data from IC has been read; called read_voltages()
     * @post Every time we call this method, can_voltage_group loops through 0 -> 3 -> 6 -> 9
     * And can_voltage_ic will iterate through 0 -> 11 as we iterate through every group of 0,3,6,9
     *
     * NOTE: writes the CAN message for one ic group at a time (6 bytes)
     */
    void write_CAN_detailed_voltages();

    /**
     * Extender method for write_CAN_messages(); sends specific data of every can_gpio group 
     * and every cell in that group
     * @pre Data from IC has been read; called read_gpios()
     * @post Every time this method is called, can_gpio_group = 0 -> 3 -> 0 -> 3 -> 0 -> 3 and so forth
     * For IC from 0 -> 11, gpio_group will be iterated from 0 -> 3 and then back again
     * 
     * NOTE: writes the CAN message for one ic group at a time (6 bytes)
     */
    void write_CAN_detailed_temps();

    /**
     * Watchdog Circuit Protocol on Pin 5 
     * @post Pulses unless there is an active voltage/temperature fault, calls has_active_voltage_fault()
    */
    static void ams_ok_pulse();

    /**
     * Calls the primary read and send functions
     * @pre CAN status and events() have been called & STATE of charging is updated for bms_status
     * @post read voltage/gpio, send shunt measurements, and update balancing 
    */
    void update_ACU_status();

    /**
     * Setter for ACU_SHUNT_MEASUREMENT_t's instance variables
     * @post raw values for shunt current, pack filtered, and ts_out_filtered should be assigned to container
    */
    void send_acu_shunt_measurements();

    /* -------------------- Balancing Cells -------------------- */

    // Cell Balancing function. NOTE: Must call read_voltages() in order to obtain balancing voltage;
    /**
     * Cell Balancing function
     * @pre MUST call read_voltages() or there are no usable voltage data to balance
     * @post Rewrites Register_Group configuration bytes, which are spent through SPI to the IC
     * In the Register_Group, this is the 5th config_byte, where the LSB to the 12th/9th bit represent 
     * whether a cell is balanced or not. If the 10th bit is 0, the IC knows to discharge it
    */
    void balance_cells();

    /* -------------------- Setup & Loop -------------------- */
   
    /**
     * Arduino setup() function
     * @post Enable CAN buses, start watchdog pulse, initialize IC PEC table, set chip select for SPI
    */
    void setup();
    
    /**
     * Arduino primary loop() function, continuously run
     * @post READS + PRINTS voltages/gpios, balances cells given the received data
    */
    void loop();

private:
    /* -------------------- VARIABLE DECLARATIONS -------------------- */

    uint16_t vuv = 1874;                    // 3V           // Minimum voltage value following datasheet formula: Comparison Voltage = (VUV + 1) • 16 • 100μV
    uint16_t vov = 2625;                    // 4.2V         // Maximum voltage value following datasheet formula: Comparison Voltage = VOV • 16 • 100μV
    uint16_t cell_voltages[TOTAL_IC][12];   // 2D Array to hold cell voltages being read in; voltages are read in with the base unit as 100μV
    bool cell_balance_status[TOTAL_IC][12]; // 2D array where true indicates cell is balancing
    bool currently_balancing = false;
    uint32_t total_voltage;                       // the total voltage of the pack
    float avg_cell_voltage = total_voltage / 126; // avg voltage on one pack cell

    uint16_t gpio_voltages[TOTAL_IC][6]; // 2D Array to hold GPIO voltages being read in; voltages are read in with the base unit as 100μV
    float gpio_temps[TOTAL_IC][6];       // 2D Array to hold GPIO temperatures being read in; temperatures are read in with the base unit as K

    uint16_t max_humidity = 0;
    uint16_t max_thermistor_voltage = 0;
    uint16_t min_thermistor_voltage = 65535;
    uint16_t max_board_temp_voltage = 0;
    uint16_t min_board_temp_voltage = 65535;
    float total_board_temps = 0;
    // underestimate state of charge
    // pack voltage divided by 126
    float total_thermistor_temps = 0;

    IntervalTimer pulse_timer; // AMS ok pulse timer
    
    uint8_t can_voltage_ic;    // counter for the current IC data to send for detailed voltage CAN message
    uint8_t can_voltage_group; // counter for current group data to send for detailed voltage CAN message
    uint8_t can_gpio_ic;       // counter for the current IC data to send for detailed voltage CAN message
    uint8_t can_gpio_group;    // counter for current group data to send for detailed voltage CAN message

    // CONSECUTIVE FAULT COUNTERS: counts successive faults; resets to zero if normal reading breaks fault chain
    unsigned long uv_fault_counter;       // undervoltage fault counter
    unsigned long ov_fault_counter;       // overvoltage fault counter
    unsigned long pack_ov_fault_counter;  // total voltage overvoltage fault counter
    unsigned long overtemp_fault_counter; // total overtemperature fault counter
    
    IC_Cell_Location min_voltage_location;    // [0]: IC#; [1]: Cell#
    IC_Cell_Location max_voltage_location;    // [0]: IC#; [1]: Cell#
    IC_Cell_Location max_board_temp_location; // [0]: IC#; [1]: Cell#
    IC_Cell_Location min_board_temp_location; // [0]: IC#; [1]: Cell#
    IC_Cell_Location max_thermistor_location; // [0]: IC#; [1]: Cell#
    IC_Cell_Location max_humidity_location;   // [0]: IC#; [1]: Cell#
    IC_Cell_Location min_thermistor_location; // [0]: IC#; [1]: Cell#

    elapsedMillis can_bms_status_timer = 0;
    elapsedMillis can_bms_detailed_voltages_timer = 2;
    elapsedMillis can_bms_detailed_temps_timer = 4;
    elapsedMillis can_bms_voltages_timer = 6;
    elapsedMillis can_bms_temps_timer = 8;
    elapsedMillis can_bms_onboard_temps_timer = 10;
    elapsedMicros CC_integrator_timer = 0; // Timer used to provide estimation of pack charge from shunt current

    // LTC6811_2 OBJECT DECLARATIONS
    LTC6811_2_HT09 ic[TOTAL_IC];

    // OUTBOUND CAN MESSAGES
    EM_measurement em_measurement;
    EM_status em_status;

    // BMS CAN MESSAGE AND STATE MACHINE OBJECT DECLARATIONS
    BMS_status bms_status;                               // Message class that contains flags for AMS errors as well as a variable encoding the current state of the AMS (charging vs. discharging)
    BMS_voltages bms_voltages;                           // Message class containing general voltage information
    BMS_temperatures bms_temperatures;                   // Message class containing general temperature information
    BMS_onboard_temperatures bms_onboard_temperatures;   // Message class containing general AMS temperature information
    BMS_detailed_voltages bms_detailed_voltages;         // Message class containing detailed voltage information
    BMS_detailed_temperatures bms_detailed_temperatures; // message class containing detailed temperature information
    ACU_shunt_measurements acu_shunt_measurements;
    CCU_status ccu_status;
};

#endif