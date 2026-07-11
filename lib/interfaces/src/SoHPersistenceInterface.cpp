#include "SoHPersistenceInterface.h"


void SoHPersistenceInterface::init()
{
    uint32_t magic_check = 0;
    EEPROM.get(EEPROM_MAGIC_ADDR, magic_check);

    if (magic_check != MAGIC_NUMBER)
    {
        // First boot / unprovisioned EEPROM: establish a clean baseline.
        _lifetime_ah_throughput = 0.0f;
        EEPROM.put(EEPROM_MAGIC_ADDR, MAGIC_NUMBER);
        EEPROM.put(EEPROM_THROUGHPUT_ADDR, _lifetime_ah_throughput);
    }
    else
    {
        EEPROM.get(EEPROM_THROUGHPUT_ADDR, _lifetime_ah_throughput);
        if (!(_lifetime_ah_throughput >= 0.0f))
        {
            _lifetime_ah_throughput = 0.0f;
        }
    }

    _last_saved_ah = _lifetime_ah_throughput;
    _last_write_ms = 0;
}

bool SoHPersistenceInterface::save(double lifetime_ah_throughput, time_ms now, bool force)
{
    _lifetime_ah_throughput = lifetime_ah_throughput;

    bool interval_elapsed = (now - _last_write_ms) >= MIN_WRITE_INTERVAL_MS;
    bool meaningful_change = (lifetime_ah_throughput - _last_saved_ah) >= MIN_DELTA_AH;

    if (!force && !(interval_elapsed && meaningful_change))
    {
        return false;
    }

    EEPROM.put(EEPROM_THROUGHPUT_ADDR, lifetime_ah_throughput);
    _last_saved_ah = lifetime_ah_throughput;
    _last_write_ms = now;
    return true;
}
