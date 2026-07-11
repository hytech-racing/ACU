#ifndef SOH_PERSISTENCE_INTERFACE_H
#define SOH_PERSISTENCE_INTERFACE_H

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include <Arduino.h>
#include "SharedFirmwareTypes.h"
#include <EEPROM.h>


/**
 * @brief Persists the battery's lifetime Amp-hour throughput to the Teensy's EEPROM non volatile memory
 * so that State of Health survives power cycles.
 *
 * SoH itself isn't stored, rather it is recomputed from the throughput by ACUController
 */
class SoHPersistenceInterface
{
public:

    SoHPersistenceInterface() = default;

    /**
     * @brief Read the stored throughput from EEPROM
     */
    void init();

    /**
     * @return float lifetime Amp-hours read from EEPROM during init()
     */
    double get_lifetime_ah_throughput() const { return _lifetime_ah_throughput; }

    /**
     * @brief Throttled persist of the lifetime throughput.
     * @param lifetime_ah_throughput current accumulator value to persist
     * @param now current time in milliseconds
     * @param force bypass the throttle and write immediately
     * @return true if a physical EEPROM write occurred
     */
    bool save(double lifetime_ah_throughput, time_ms now, bool force = false);

private:

    // EEPROM byte memory map
    static constexpr int EEPROM_MAGIC_ADDR = 16;
    static constexpr int EEPROM_THROUGHPUT_ADDR = 20;
    static constexpr uint32_t MAGIC_NUMBER = 0x50484F53;

    static constexpr time_ms MIN_WRITE_INTERVAL_MS = 60000; // 1 minute
    static constexpr float MIN_DELTA_AH = 0.5f;

    double _lifetime_ah_throughput = 0.0f;
    double _last_saved_ah = 0.0f;
    time_ms _last_write_ms = 0;
    
};

using SoHPersistenceInterfaceInstance = etl::singleton<SoHPersistenceInterface>;

#endif /* SOH_PERSISTENCE_INTERFACE_H */
