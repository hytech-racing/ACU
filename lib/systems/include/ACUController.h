#ifndef __ACUCONTROLLER_H__
#define __ACUCONTROLLER_H__

#include <array>
#include <stddef.h>
#include <stdio.h>
#include <cstdint>
#include "etl/optional.h"
#include "SharedFirmwareTypes.h"

namespace ACU_CONTROLLER_DEFAULT_PARAMS
{
    constexpr const float OV_THRESH = 4.2; // Volts
    constexpr const float UV_THRESH = 3.2; // Volts
    constexpr const float OT_THRESH = 65.0; // Celcius
};

template <size_t num_chips>
struct ACU_State_s
{
    size_t uv_counter;
    size_t ov_counter;
    size_t ot_counter;
    bool has_voltage_fault;
    bool charging_enabled;
    bool current_pulse;
    std::array<uint16_t, num_chips> cell_balance_statuses;
};

template<size_t num_chips>
class ACUController
{
public:

    using ACUData = ACU_State_s<num_chips>;

    /**
     * ACU Controller Constructor
     * @param ov_thresh_v overvoltage threshold value | units in volts 
     * @param uv_thresh_v undervoltage threshold value | units in volts
     * @param ot_thresh_c overtemp threshold value | units in celcius
    */
    ACUController(volt ov_thresh_v = ACU_CONTROLLER_DEFAULT_PARAMS::OV_THRESH, 
                       volt uv_thresh_v = ACU_CONTROLLER_DEFAULT_PARAMS::UV_THRESH, 
                       celcius ot_thresh_c = ACU_CONTROLLER_DEFAULT_PARAMS::OT_THRESH) : 
        _ov_thresh_v(ov_thresh_v),
        _uv_thresh_v(uv_thresh_v),
        _ot_thresh_c(ot_thresh_c)
        {};

    /**
     * @pre voltage data has been recorded
     * @post updates configuration bytes and sends configuration command
     */
    void update_acu_state(std::array<std::array<etl::optional<volt>, 12>, num_chips> voltages,
                          float min_voltage, float max_voltage);
private:
    /**
     * @pre data has been gathered
     * @return boolean, true if there exists any fault
     */
    bool _check_faults();

    /**
     * @pre voltage data has been gathered
     * @return boolean, true if there exists at least 1 voltage fault
     */
    bool _check_voltage_faults();

    /**
     * @pre temperature data has been gathered
     * @return boolean, true if there exists a temperature fault
     */
    bool _check_temperature_faults();

    /**
     * NOTE: TBD
    */
    void _columb_counting();

private: 
    /**
     * @brief ACU State Holder
     * Most importantly, holding the current cell balances, fault counters, and watchdog HIGH?LOW
     * state is packaged this way so that we can feed it directly into the message interface as a struct
    */
    ACUData _acu_state;

    // Overvoltage threshold in volts
    const volt _ov_thresh_v = 0;

    // Undervoltage threshold in volts
    const volt _uv_thresh_v = 0;

    // Overtemp threshold in celcius
    const celcius _ot_thresh_c = 0;
};

#include "ACUController.tpp"
#endif