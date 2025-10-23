#include "FaultLatchManagerInterface.h"

void FaultLatchManager::clear_if_not_faulted(bool is_faulted) {
  if (!is_faulted) {
    latches_.bms_fault_latched = false;
    latches_.imd_fault_latched = false;
  }
}

void FaultLatchManager::update(bool imd_ok, bool bms_ok) {
  if (!imd_ok) latches_.imd_fault_latched = true;
  if (!bms_ok) latches_.bms_fault_latched = true;
}
