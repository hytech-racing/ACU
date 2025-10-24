#ifndef WATCHDOG_METRICS_H
#define WATCHDOG_METRICS_H
#include "SharedFirmwareTypes.h"
#include "etl/singleton.h"

class WatchdogMetrics {
    public: 
       struct WatchdogMetricsData {
            volt max_measured_glv;
            volt max_measured_pack_out_voltage;
            volt max_measured_ts_out_voltage;

            volt min_measured_glv;
            volt min_measured_pack_out_voltage;
            volt min_measured_ts_out_voltage;

            volt min_shdn_out_voltage;

            float last_valid_shdn_out_ms;
        };


        void update_metrics(volt measured_glv, volt mesaured_pack_out_voltage, volt measured_ts_out_voltage, volt shdn_out_voltage, unsigned long millis);

        void reset_metrics();

        bool is_shdn_out_voltage_invalid(unsigned long millis);

        WatchdogMetricsData get_watchdog_metrics();

    private:
        static constexpr float MAX_RESET_VALUE = -50000;
        static constexpr float MIN_RESET_VALUE = 50000;
        static constexpr float VALID_SHDN_OUT_MIN_VOLTAGE_THRESHOLD = 12.0F;
        static constexpr uint32_t MIN_ALLOWED_INVALID_SHDN_OUT_MS = 10; 
        WatchdogMetricsData _watchdog_metrics_data;
        
};


using WatchdogMetricsInstance = etl::singleton<WatchdogMetrics>;

#endif