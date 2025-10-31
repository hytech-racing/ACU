#ifndef FAULT_LATCH_MANAGER_INTERFACE_H
#define FAULT_LATCH_MANAGER_INTERFACE_H


#include <etl/singleton.h>
#include <stdint.h>


struct FaultLatches {
  bool bms_fault_latched = false;
  bool imd_fault_latched = false;
  bool shdn_out_latched = true;
};

class FaultLatchManager {
public:
  // Clear latches whenever we are NOT in FAULTED (i.e., after recovery)
  void clear_if_not_faulted(bool is_faulted);

  // Update from live signals: latch on any not-OK
  void update_imd_and_bms_latches(bool imd_ok, bool bms_ok);

  void update_shdn_out_latch(bool shdn_out_invalid);

  void set_bms_fault_latched(bool latched) { _latches.bms_fault_latched = latched; }
  void set_imd_fault_latched(bool latched) { _latches.imd_fault_latched = latched; }
  void set_shdn_out_latched(bool latched) { _latches.shdn_out_latched = latched; }
  // Snapshot for publishing
  FaultLatches get_latches() const { return _latches; }

private:  
  FaultLatches _latches;
};

using FaultLatchManagerInstance = etl::singleton<FaultLatchManager>;

#endif // FAULT_LATCH_MANAGER_H
