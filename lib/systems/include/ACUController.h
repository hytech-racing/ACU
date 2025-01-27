#ifndef __ACUCONTROLLER_H__
#define __ACUCONTROLLER_H__

#include "Configuration.h"
#include <array>
#include <stddef.h>
#include <stdio.h>
#include <cstdint>
#include "etl/optional.h"

using volt = float; 

/**
 * @pre data has been gathered
 * @return boolean, true if there exists any fault
*/
template<size_t num_chips>
bool check_faults(ACU_State_s<num_chips> acu_state);

/**
 * @pre voltage data has been gathered
 * @return boolean, true if there exists at least 1 voltage fault
*/
bool check_voltage_faults(size_t ov_counter, size_t uv_counter);

/**
 * @pre temperature data has been gathered
 * @return boolean, true if there exists a temperature fault
*/
bool check_temperature_faults(size_t ot_counter);

/**
 * @pre voltage data has been recorded
 * @post updates configuration bytes and sends configuration command 
*/
template<size_t num_chips>
void update_acu_state(ACU_State_s<num_chips> acu_state, std::array<std::array<etl::optional<volt>, 12>, num_chips> voltages, 
                     float min_voltage, float max_voltage);

/**
 * 
*/
void columb_counting();

#include "ACUController.tpp"
#endif