#include "FlexCAN_T4.h"
#include <Arduino.h>
#include "hytech.h"

#ifndef CAN_Enabled
// Un-comment line below if using CAN
#define CAN_Enabled
#endif

#ifndef ETHERNET_Enabled
// Un-comment line below if using Ethernet
//#define ETHERNET_Enabled
#endif

/**
 * Forwards the Charger Control Unit (CCU) data over onto the TELEM_CAN line
*/
void parse_CCU_status();

/**
 * Forwards the Energy Meter (EM) data onto the TELEM_CAN line
*/
void parse_EM_status();

/**
 * Writes the voltage data onto the TELEM_CAN line
 * Calls detailed voltages function
*/
void write_voltages_data();

/**
 * Writes the temperature data onto the TELEM_CAN line
 * Calls detailed temperatures function
*/
void write_temperatures_data();

/**
 * Writes the voltage data onto the TELEM_CAN line
*/
void write_detailed_voltages_data();

/**
 * Writes ALL of the temperature data onto the TELEM_CAN line
*/
void write_detailed_temperatures_data();

/**
 * Writes the shunt / current measurements onto the TELEM_CAN line
*/
void write_shunt_measurements();