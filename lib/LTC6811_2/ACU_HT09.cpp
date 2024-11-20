/* System Includes */
#include <Arduino.h>
#include <SPI.h>
#include <FlexCAN_T4.h>
#include <HyTech_CAN.h>
#include <Metro.h>
#include <LTC6811_2_HT09.h>
#include <string>
#include "ACU_HT09.h"

std::string IC_Cell_Location::toString() const
{
  return "IC #" + std::to_string(icIndex) + ", Cell #" + std::to_string(cellIndex);
}

void ACU_HT09::update_ACU_status()
{
  read_voltages();
  read_gpio();
  send_acu_shunt_measurements();
  currently_balancing = bms_status.get_state() == BMS_STATE_CHARGING && BALANCE_ON && ccu_status.get_charger_enabled() && gpio_temps[max_board_temp_location.icIndex][max_board_temp_location.cellIndex] <= 80;
}

bool ACU_HT09::has_active_fault()
{
  if (min_voltage < 30000 || min_voltage > 42000)
  {
    Serial.print("BALANCE HALT: BALANCE VOLTAGE SET AS ");
    Serial.print(min_voltage / 10000.0, 4);
    Serial.println(", OUTSIDE OF SAFE BOUNDS.");
    return true;
  }
  if (has_active_voltage_fault())
  {
    Serial.print("BALANCE HALT: CHECK PACK FAULTS");
    return true;
  }
  return false;
}



/*-------------------- Reading Voltages & GPIOs -------------------- */

void ACU_HT09::read_voltages()
{
  if (check_ics(0))
  {
    Register_Group register_group = Register_Group((uint8_t)0x1F, false, false, vuv, vov, (uint16_t)0x0, (uint8_t)0x1);
    for (int ic_Index = 0; ic_Index < TOTAL_IC; ic_Index++)
    {
      ic[ic_Index].wakeup();
      ic[ic_Index].write_config(register_group);
      ic[ic_Index].start_cv_adc_conversion_poll_status(static_cast<CELL_SELECT>(0));
    }
  }
  if (check_ics(1))
  {
    total_voltage = 0;
    max_voltage = 0;
    min_voltage = 65535;
    for (int ic_Index = 0; ic_Index < TOTAL_IC; ic_Index++)
    {
      ic[ic_Index].wakeup();
      
      Register_Group reg_group = ic[ic_Index].get_register_group();
      int cell_count = (ic_Index % 2 == 0) ? 12 : 9; // Even indexed ICs have 12 cells, odd have 9
      for (int cell_Index = 0; cell_Index < cell_count; cell_Index++)
      {
        uint8_t *cell_voltage_buffer;
        switch (cell_Index / 3) // There are 4 groups of CELL VOLTAGES, each with 3 cells
        {
        case 0: cell_voltage_buffer = reg_group.cell_voltage_buf_A(); break;
        case 1: cell_voltage_buffer = reg_group.cell_voltage_buf_B(); break;
        case 2: cell_voltage_buffer = reg_group.cell_voltage_buf_C(); break;
        case 3: cell_voltage_buffer = reg_group.cell_voltage_buf_D(); break; // We will never get here for odd indexed ICs
        }

        cell_voltages[ic_Index][cell_Index] = cell_voltage_buffer[2 * (cell_Index % 3) + 1] << 8 | cell_voltage_buffer[2 * (cell_Index % 3)];
        total_voltage += cell_voltages[ic_Index][cell_Index];
        if (cell_voltages[ic_Index][cell_Index] < min_voltage)
        {
          min_voltage = cell_voltages[ic_Index][cell_Index];
          min_voltage_location.icIndex = ic_Index;
          min_voltage_location.cellIndex = cell_Index;
        }
        if (cell_voltages[ic_Index][cell_Index] > max_voltage)
        {
          max_voltage = cell_voltages[ic_Index][cell_Index];
          max_voltage_location.icIndex = ic_Index;
          max_voltage_location.cellIndex = cell_Index;
        }
      }
    }
    voltage_fault_check();
  }
}

void ACU_HT09::read_gpio()
{
  if (check_ics(2))
  {
    Register_Group configuration = Register_Group((uint8_t)0x1F, false, false, vuv, vov, (uint16_t)0x0, (uint8_t)0x1); // base configuration for the configuration register group
    for (int ic_Index = 0; ic_Index < TOTAL_IC; ic_Index++)
    {
      ic[ic_Index].wakeup();
      ic[ic_Index].write_config(configuration);
      ic[ic_Index].start_gpio_adc_conversion_poll_status(static_cast<GPIO_SELECT>(0));
    }
  }
  if (check_ics(3))
  {
    max_humidity = 0;
    max_thermistor_voltage = 0;
    min_thermistor_voltage = 65535;
    max_board_temp_voltage = 0;
    min_board_temp_voltage = 65535;
    total_board_temps = 0;
    total_thermistor_temps = 0;
    for (int ic_Index = 0; ic_Index < TOTAL_IC; ic_Index++)
    {
      ic[ic_Index].wakeup();
      Register_Group register_group = ic[ic_Index].get_register_group();
      for (int cell_Index = 0; cell_Index < 6; cell_Index++)
      {
        uint8_t *gpio_voltage_buf;
        switch (cell_Index / 3)
        {
        case 0: gpio_voltage_buf = register_group.gpio_voltage_A_buf(); break;
        case 1: gpio_voltage_buf = register_group.gpio_voltage_B_buf(); break;
        } 
        gpio_voltages[ic_Index][cell_Index] = gpio_voltage_buf[2 * (cell_Index % 3) + 1] << 8 | gpio_voltage_buf[2 * (cell_Index % 3)];

        if ((ic_Index % 2) && cell_Index == 4)
        {
          gpio_temps[ic_Index][cell_Index] = -66.875 + 218.75 * (gpio_voltages[ic_Index][cell_Index] / 50000.0); // caculation for SHT31 temperature in C
          total_board_temps += gpio_temps[ic_Index][cell_Index];
          update_maximum_location(gpio_voltages[ic_Index][cell_Index], max_board_temp_voltage, max_board_temp_location, ic_Index, cell_Index);
          update_minimum_location(gpio_voltages[ic_Index][cell_Index], min_board_temp_voltage, min_board_temp_location, ic_Index, cell_Index);
        }
        else if (cell_Index == 4)
        {
          gpio_temps[ic_Index][cell_Index] = -12.5 + 125 * (gpio_voltages[ic_Index][cell_Index]) / 50000.0; // humidity calculation
          update_maximum_location(gpio_temps[ic_Index][cell_Index], max_humidity, max_humidity_location, ic_Index, cell_Index);
        }
        else if (cell_Index < 4)
        {
          float thermistor_resistance = (2740 / (gpio_voltages[ic_Index][cell_Index] / 50000.0)) - 2740;
          gpio_temps[ic_Index][cell_Index] = 1 / ((1 / 298.15) + (1 / 3984.0) * log(thermistor_resistance / 10000.0)) - 273.15; // calculation for thermistor temperature in C
          total_thermistor_temps += gpio_temps[ic_Index][cell_Index];
          update_maximum_location(gpio_voltages[ic_Index][cell_Index], max_thermistor_voltage, max_thermistor_location, ic_Index, cell_Index);
          update_minimum_location(gpio_voltages[ic_Index][cell_Index], min_thermistor_voltage, min_thermistor_location, ic_Index, cell_Index);
        }
      }
    }
    temp_fault_check();
  }
}

void ACU_HT09::send_acu_shunt_measurements()
{
  // integrate shunt current over time to count coulombs and provide state of charge

  // ----------- THIS CODE HAS BEEN COMMENTED SO THAT MCU WILL HANDLE THE INTEGRATION ---------- //
  // current_shunt_read = (analogRead(CURR_SHUNT) * 3.3) / 4095; //.68
  // shunt_voltage_input = (current_shunt_read * (9.22 / 5.1)) - 3.3 - 0.03;
  // shunt_current = (shunt_voltage_input / 0.005);
  // charge -= (CC_integrator_timer * shunt_current) / 1000000;
  // state_of_charge = charge / MAX_PACK_CHARGE;
  // CC_integrator_timer = 0; 
  // ----------- THIS CODE HAS BEEN COMMENTED SO THAT MCU WILL HANDLE THE INTEGRATION ---------- //

  acu_shunt_measurements.set_shunt_current(analogRead(CURR_SHUNT));
  acu_shunt_measurements.set_pack_filtered(analogRead(PACK_FILTERED));
  acu_shunt_measurements.set_ts_out_filtered(analogRead(TS_OUT_FILTERED));
}



/*-------------------- Cell Balancing Functions -------------------- */

void ACU_HT09::balance_cells()
{
  if (balance_timer.check())
  {
    balance_timer.reset();

    if (!has_active_voltage_fault())
    {
      Serial.print("Balancing voltage: ");
      Serial.println(min_voltage / 10000.0, 4);
      for (uint16_t ic_Index = 0; ic_Index < TOTAL_IC; ic_Index++)
      {
        uint16_t cell_balance_setting = 0x0;
        // determine which cells of the IC need balancing
        uint8_t cell_count = (ic_Index % 2) ? 9 : 12;

        for (uint16_t cell = 0; cell < cell_count; cell++)
        {
#ifdef BALANCE_MODE
          if (max_voltage - cell_voltages[ic_Index][cell] < 200 && cell_voltages[ic_Index][cell] - min_voltage > 200)
          { // balance if the cell voltage differential from the max voltage is .02V or less and if the cell voltage differential from the minimum voltage is 0.02V or greater (progressive)
            cell_balance_setting = (0b1 << cell) | cell_balance_setting;
            cell_balance_status[ic_Index][cell] = true;
          }
#else
          if (cell_voltages[i][cell] - min_voltage > 200)
          { // if the cell voltage differential from the minimum voltage is 0.02V or greater (normal)
            cell_balance_setting = (0b1 << cell) | cell_balance_setting;
            cell_balance_status[i][cell] = true;
          }
#endif
          else
          {
            cell_balance_status[ic_Index][cell] = false;
          }
        }

        Register_Group configuration = Register_Group((uint8_t)0x1F, false, false, vuv, vov, (uint16_t)cell_balance_setting, (uint8_t)0x0); // base configuration for the configuration register group
        ic[ic_Index].wakeup();
        ic[ic_Index].write_config(configuration);
      }
      delay(2000);
    }
  }
}



/*-------------------- Writing CAN Messages Functions -------------------- */

void ACU_HT09::parse_CAN_CCU_status()
{
  while (TELEM_CAN.read(msg))
  {
    if (msg.id == ID_CCU_STATUS)
    {
      ccu_status.load(msg.buf);
      charging_timer.reset();
      if (bms_status.get_state() == BMS_STATE_DISCHARGING)
      {
        bms_status.set_state(BMS_STATE_CHARGING);
      }
    }
  }
}

void ACU_HT09::write_CAN_detailed_voltages()
{
  if (can_voltage_group > 9)
  {
    can_voltage_ic++;
    can_voltage_group = 0;
  }
  if (can_voltage_ic > (TOTAL_IC - 1))
  {
    can_voltage_ic = 0;
    can_voltage_group = 0;
  }
  if (!(can_voltage_ic % 2 && can_voltage_group == 9))
  {
    bms_detailed_voltages.set_ic_id(can_voltage_ic);
    bms_detailed_voltages.set_group_id(can_voltage_group / 3);
    bms_detailed_voltages.set_voltage_0(cell_voltages[can_voltage_ic][can_voltage_group]);
    bms_detailed_voltages.set_voltage_1(cell_voltages[can_voltage_ic][can_voltage_group + 1]);
    bms_detailed_voltages.set_voltage_2(cell_voltages[can_voltage_ic][can_voltage_group + 2]);
    msg.id = ID_BMS_DETAILED_VOLTAGES;
    msg.len = sizeof(bms_detailed_voltages);
    bms_detailed_voltages.write(msg.buf);
    TELEM_CAN.write(msg);
  }
  can_voltage_group += 3;
}

void ACU_HT09::write_CAN_detailed_temps()
{
  if (can_gpio_group > 3)
  {
    can_gpio_ic++;
    can_gpio_group = 0;
  }
  if (can_gpio_ic > (TOTAL_IC - 1))
  {
    can_gpio_ic = 0;
  }
  bms_detailed_temperatures.set_ic_id(can_gpio_ic);
  bms_detailed_temperatures.set_group_id(can_gpio_group / 3);
  bms_detailed_temperatures.set_temperature_0((uint16_t)(gpio_temps[can_gpio_ic][can_gpio_group] * 100));
  bms_detailed_temperatures.set_temperature_1((uint16_t)(gpio_temps[can_gpio_ic][can_gpio_group + 1] * 100));
  bms_detailed_temperatures.set_temperature_2((uint16_t)(gpio_temps[can_gpio_ic][can_gpio_group + 2] * 100));
  msg.id = ID_BMS_DETAILED_TEMPERATURES;
  msg.len = sizeof(bms_detailed_temperatures);
  bms_detailed_temperatures.write(msg.buf);
  TELEM_CAN.write(msg);
  can_gpio_group += 3;
}

void ACU_HT09::write_CAN_message_helper(elapsedMillis &timer, uint32_t timer_threshold, uint32_t msg_id,
                                        size_t msg_len, std::function<void(uint8_t *)> write_func)
{
  if (timer > timer_threshold)
  {
    msg.id = msg_id;
    msg.len = msg_len;
    write_func(msg.buf);
    TELEM_CAN.write(msg);
    timer = 0;
  }
}

void ACU_HT09::write_CAN_detailed_helper(elapsedMillis &timer, uint32_t threshold, std::function<void()> action)
{
  if (timer > threshold)
  {
    action();
    timer = 0;
  }
}

void ACU_HT09::setVoltageData(BMS_voltages &voltageObj, uint16_t minV, int maxV, int totalV, int count)
{
  voltageObj.set_low(minV);
  voltageObj.set_high(maxV);
  voltageObj.set_average(totalV / count);
  voltageObj.set_total(totalV / 100);
}

void ACU_HT09::setTemperatureData(auto &tempObj, const auto &temps, const IC_Cell_Location &minLoc, const IC_Cell_Location &maxLoc, uint32_t totalTemp, int count)
{
  tempObj.set_low_temperature(temps[minLoc.icIndex][minLoc.cellIndex] * 100);
  tempObj.set_high_temperature(temps[maxLoc.icIndex][maxLoc.cellIndex] * 100);
  tempObj.set_average_temperature(totalTemp * 100 / count);
}

void ACU_HT09::write_CAN_messages()
{
  // Set voltage message values
  setVoltageData(bms_voltages, min_voltage, max_voltage, total_voltage, 126);

  // Set temperature message values for BMS temperatures
  setTemperatureData(bms_temperatures, gpio_temps, min_thermistor_location, max_thermistor_location, total_thermistor_temps, 48);

  // Set onboard temperature message values
  setTemperatureData(bms_onboard_temperatures, gpio_temps, min_board_temp_location, max_board_temp_location, total_board_temps, 6);

  // Write BMS_status message
  write_CAN_message_helper(can_bms_status_timer, 250, ID_BMS_STATUS, sizeof(bms_status), [&](uint8_t *buf)
                           { bms_status.write(buf); });

  // Write BMS_voltages message
  write_CAN_message_helper(can_bms_voltages_timer, 100, ID_BMS_VOLTAGES, sizeof(bms_voltages), [&](uint8_t *buf)
                           { bms_voltages.write(buf); });

  // Write BMS_temperatures message
  write_CAN_message_helper(can_bms_temps_timer, 300, ID_BMS_TEMPERATURES, sizeof(bms_temperatures), [&](uint8_t *buf)
                           { bms_temperatures.write(buf); });

  // Write BMS_onboard_temperatures message
  write_CAN_message_helper(can_bms_onboard_temps_timer, 200, ID_BMS_ONBOARD_TEMPERATURES, sizeof(bms_onboard_temperatures), [&](uint8_t *buf)
                           { bms_onboard_temperatures.write(buf); });

  if (timer_shunt.check())
  {
    msg.id = ID_ACU_SHUNT_MEASUREMENT;
    msg.len = sizeof(acu_shunt_measurements);
    acu_shunt_measurements.write(msg.buf);
    TELEM_CAN.write(msg);
  }

  // writing detailed voltages for one IC group
  write_CAN_detailed_helper(can_bms_detailed_voltages_timer, 100, [this]()
                            { this->write_CAN_detailed_voltages(); });

  write_CAN_detailed_helper(can_bms_detailed_temps_timer, 300, [this]()
                            { this->write_CAN_detailed_temps(); });
}



/*-------------------- Static Functions -------------------- */

void ACU_HT09::ams_ok_pulse()
{
  if (!has_active_fault())
  {
    next_pulse = !next_pulse;
  }
  digitalWrite(5, (next_pulse ? HIGH : LOW));
}

void ACU_HT09::parse_energy_meter_can_message(const CAN_message_t &RX_msg)
{
  CAN_message_t rx_msg = RX_msg;
  switch (rx_msg.id)
  {
  case ID_EM_MEASUREMENT:
    //      em_measurement.load(rx_msg.buf);
    TELEM_CAN.write(rx_msg);
    break;
  case ID_EM_STATUS:
    //      em_status.load(rx_msg.buf);
    TELEM_CAN.write(rx_msg);
    break;
  }
}



/*-------------------- Setup & Main Loop -------------------- */

void ACU_HT09::setup()
{
  // put your setup code here, to run once:
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);

  digitalWrite(6, HIGH); // write Teensy_OK pin high

  // chip select defines
  pinMode(CHIP_SELECT_GROUP_ONE, OUTPUT);
  pinMode(CHIP_SELECT_GROUP_TWO, OUTPUT);
  digitalWrite(CHIP_SELECT_GROUP_ONE, HIGH);
  digitalWrite(CHIP_SELECT_GROUP_TWO, HIGH);
  
  pulse_timer.begin(ams_ok_pulse, 50000); // timer to pulse pin 5 every 50 milliseconds
  Serial.begin(115200);
  SPI.begin();
  TELEM_CAN.begin();
  TELEM_CAN.setBaudRate(500000);
  //  TELEM_CAN.setBaudRate(1000000);    // Test CAN capacity 1,000,000 baud
  ENERGY_METER_CAN.begin();
  ENERGY_METER_CAN.setBaudRate(500000);
  ENERGY_METER_CAN.enableMBInterrupts();
  ENERGY_METER_CAN.onReceive(parse_energy_meter_can_message);
  for (int i = 0; i < 64; i++)
  {                                                                        // Fill all filter slots with Charger Control Unit message filter
    TELEM_CAN.setMBFilter(static_cast<FLEXCAN_MAILBOX>(i), ID_CCU_STATUS); // Set CAN mailbox filtering to only watch for charger controller status CAN messages
  }
  // initialize the PEC table
  //init_PEC15_Table();

  // add 12 (TOTAL_IC) instances of LTC6811_2 ato the object array, each addressed appropriately
  for (int ic_Index = 0; ic_Index < TOTAL_IC; ic_Index++)
  {
    ic[ic_Index] = LTC6811_2_HT09(ic_Index); // ic_Index is it's address
    // Assign chip select group based on index, I don't know exactly why these are the specific indexes per chip select, but I copied the prior version
    if ((ic_Index >= 0 && ic_Index <= 1) || (ic_Index >= 6 && ic_Index <= 9))
    {
      ic[ic_Index].spi_set_chip_select(CHIP_SELECT_GROUP_ONE);
    }
    else if ((ic_Index >= 2 && ic_Index <= 5) || (ic_Index >= 10 && ic_Index <= 11))
    {
      ic[ic_Index].spi_set_chip_select(CHIP_SELECT_GROUP_TWO);
    }
  }
  bms_status.set_state(BMS_STATE_DISCHARGING);
  parse_CAN_CCU_status();

  analogReadResolution(12);
}

void ACU_HT09::loop()
{
  // put your main code here, to run repeatedly:
  TELEM_CAN.events();
  ENERGY_METER_CAN.events();
  // Code to set BMS_status's CHARGING STATE ---- 
  // In this version, we are only balancing cells if state is CHARGING
  parse_CAN_CCU_status();
  if (charging_timer.check() && bms_status.get_state() == BMS_STATE_CHARGING)
  {
    bms_status.set_state(BMS_STATE_DISCHARGING);
  }
  update_ACU_status();
  write_CAN_messages();
  if (print_timer.check())
  {
    print_voltages();
    print_gpios();
    print_timer.reset();
  }
  if (currently_balancing)
  {
    balance_cells();
  }

  //  forward_CAN_em();
}