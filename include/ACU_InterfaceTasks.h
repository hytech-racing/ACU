#ifndef ACU_INTERFACETASKS
#define ACU_INTERFACETASKS

#include "ACU_Constants.h"
#include "ACU_Globals.h"

/* Interface Library Includes */
#include "BMSDriverGroup.h"

using chip_type = LTC6811_Type_e;

void initialize_interfaces();

void set_bit(uint16_t & value, uint8_t index, bool bitValue);

#endif 