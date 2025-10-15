#ifndef FAULT_LATCH_MANAGER_INTERFACE_H
#define FAULT_LATCH_MANAGER_INTERFACE_H


#include <etl/singleton.h>
#include <stdint.h>


struct FaultLatches {
  bool bms_fault_latched = false;
  bool imd_fault_latched = false;
};

class FaultLatchManagerInterface {
public:
  // Clear latches whenever we are NOT in FAULTED (i.e., after recovery)
  void clear_if_not_faulted(bool is_faulted);

  // Update from live signals: latch on any not-OK
  void update(bool imd_ok, bool bms_ok);

  void set_bms_fault_latched(bool latched) { latches_.bms_fault_latched = latched; }
  void set_imd_fault_latched(bool latched) { latches_.imd_fault_latched = latched; }
  // Snapshot for publishing
  FaultLatches get_latches() const { return latches_; }

private:  
  FaultLatches latches_;
};

using FaultLatchManagerInstance = etl::singleton<FaultLatchManagerInterface>;

#endif // FAULT_LATCH_MANAGER_H
