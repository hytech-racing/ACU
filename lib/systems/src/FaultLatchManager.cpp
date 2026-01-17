#include "FaultLatchManager.h"

void FaultLatchManager::clear_if_not_faulted(bool is_faulted) {
  if (!is_faulted) {
    _latches.bms_fault_latched = false;
    _latches.imd_fault_latched = false;
  }
}

void FaultLatchManager::update_imd_and_bms_latches(bool imd_ok, bool bms_ok) {
  if (!imd_ok) _latches.imd_fault_latched = true;
  if (!bms_ok) _latches.bms_fault_latched = true;
  _latches.shdn_out_latched = true;
}

void FaultLatchManager::update_shdn_out_latch(bool shdn_out_invalid) {
  if (shdn_out_invalid) _latches.shdn_out_latched = false;
} 
