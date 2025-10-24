#include "WatchdogMetrics.h"

void WatchdogMetrics::update_metrics(volt measured_glv, volt measured_pack_out_voltage, volt measured_ts_out_voltage, volt shdn_out_voltage) {
    if (measured_glv > _watchdog_metrics_data.max_measured_glv) _watchdog_metrics_data.max_measured_glv = measured_glv;
    if (measured_pack_out_voltage > _watchdog_metrics_data.max_measured_pack_out_voltage) _watchdog_metrics_data.max_measured_pack_out_voltage = measured_pack_out_voltage;
    if (measured_ts_out_voltage > _watchdog_metrics_data.max_measured_ts_out_voltage) _watchdog_metrics_data.max_measured_ts_out_voltage = measured_ts_out_voltage;

    if (measured_glv < _watchdog_metrics_data.min_measured_glv) _watchdog_metrics_data.min_measured_glv = measured_glv;
    if (measured_pack_out_voltage < _watchdog_metrics_data.min_measured_pack_out_voltage) _watchdog_metrics_data.min_measured_pack_out_voltage = measured_pack_out_voltage;
    if (measured_ts_out_voltage < _watchdog_metrics_data.min_measured_ts_out_voltage) _watchdog_metrics_data.min_measured_ts_out_voltage = measured_ts_out_voltage;

    if (shdn_out_voltage < _watchdog_metrics_data.min_shdn_out_voltage) _watchdog_metrics_data.min_shdn_out_voltage = shdn_out_voltage;
}

void WatchdogMetrics::reset_metrics() {
    _watchdog_metrics_data.max_measured_glv = MAX_RESET_VALUE;
    _watchdog_metrics_data.max_measured_pack_out_voltage = MAX_RESET_VALUE;
    _watchdog_metrics_data.max_measured_ts_out_voltage = MAX_RESET_VALUE;

    _watchdog_metrics_data.min_measured_glv = MIN_RESET_VALUE;
    _watchdog_metrics_data.min_measured_pack_out_voltage = MIN_RESET_VALUE;
    _watchdog_metrics_data.min_measured_ts_out_voltage = MIN_RESET_VALUE;

    _watchdog_metrics_data.min_shdn_out_voltage = MIN_RESET_VALUE;
}

WatchdogMetrics::WatchdogMetricsData WatchdogMetrics::get_watchdog_metrics() {
    return _watchdog_metrics_data;
}
