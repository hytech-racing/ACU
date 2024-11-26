#ifndef __ACUCALCULATIONS_H__
#define __ACUCALCULATIONS_H__
/**
 * PREAMBLE: This file will just hold functions and manipulate the the general variable:
 * this includes the voltages, temperatures, fault states, fault counters, and CAN message structs
*/

/* System Includes */
#include "Configuration.h"

/**
 * @pre voltage data has been gathered
 * @return boolean, true if there exists at least 1 voltage fault
*/
bool check_voltage_faults();

/**
 * @pre temperature data has been gathered
 * @return boolean, true if there exists a temperature fault
*/
bool check_temperature_faults();

/**
 * @pre voltage data has been recorded
 * @post updates configuration bytes and sends configuration command 
*/
void balance_cells();

/**
 * 
*/
void columb_counting();

#endif