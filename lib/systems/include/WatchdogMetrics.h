#ifndef WATCHDOG_METRICS_H
#define WATCHDOG_METRICS_H

#include <Arduino.h>
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
        };


        void update_metrics(volt measured_glv, volt mesaured_pack_out_voltage, volt measured_ts_out_voltage, volt shdn_out_voltage);

        void reset_metrics();

        WatchdogMetricsData get_watchdog_metrics();

    private:
        static const float MAX_RESET_VALUE = -50000;
        static const float MIN_RESET_VALUE = 50000;
        WatchdogMetricsData _watchdog_metrics_data;
};


using WatchdogMetricsInstance = etl::singleton<WatchdogMetrics>;

#endif