#include "FaultLatchManager.h"

void FaultLatchManager::clear_if_not_faulted(bool is_faulted) {
  if (!is_faulted) {
    _latches.bms_fault_latched = false;
    _latches.imd_fault_latched = false;
  }
}

void FaultLatchManager::update(bool imd_ok, bool bms_ok) {
  if (!imd_ok) _latches.imd_fault_latched = true;
  if (!bms_ok) _latches.bms_fault_latched = true;
}
