#include "gtest/gtest.h"
#include <ArduinoFake.h>
#include "ADCInterface.h"

using namespace fakeit;

ADCPinout_s pinout = {ACUConstants::adc_default_params::pinout::IMD_OK_PIN,
                            ACUConstants::adc_default_params::pinout::PRECHARGE_PIN,
                            ACUConstants::adc_default_params::pinout::SHDN_OUT_PIN,
                            ACUConstants::adc_default_params::pinout::TS_OUT_FILTERED_PIN,
                            ACUConstants::adc_default_params::pinout::PACK_OUT_FILTERED_PIN,
                            ACUConstants::adc_default_params::pinout::BSPD_CURRENT_PIN,
                            ACUConstants::adc_default_params::pinout::SCALED_24V_PIN
                    };

ADCConversions_s conversions = {ACUConstants::adc_default_params::conversions::SHUTDOWN_CONV_FACTOR,
                                ACUConstants::adc_default_params::conversions::PRECHARGE_CONV_FACTOR,
                                ACUConstants::adc_default_params::conversions::PACK_AND_TS_OUT_CONV_FACTOR,
                                ACUConstants::adc_default_params::conversions::SHDN_OUT_CONV_FACTOR,
                                ACUConstants::adc_default_params::conversions::BSPD_CURRENT_CONV_FACTOR,
                                ACUConstants::adc_default_params::conversions::GLV_CONV_FACTOR
                                };

ADCThresholds_s thresholds = {ACUConstants::adc_default_params::thresholds::TEENSY41_MIN_DIGITAL_READ_VOLTAGE_THRESH,
                                ACUConstants::adc_default_params::thresholds::TEENSY41_MAX_DIGITAL_READ_VOLTAGE_THRESH,
                                ACUConstants::adc_default_params::thresholds::SHUTDOWN_VOLTAGE_DIGITAL_THRESHOLD
                            };

ADCConfigs_s configs = {ACUConstants::adc_default_params::configs::IMD_STARTUP_TIME,
                            ACUConstants::adc_default_params::configs::BIT_RESOLUTION,
                            ACUConstants::adc_default_params::configs::TEENSY41_MAX_INPUT_VOLTAGE
                        };

TEST (ADCInterfaceTesting, init) {
    When(Method(ArduinoFake(), pinMode)).AlwaysReturn();

    ADCInterfaceInstance::create(pinout, conversions, thresholds, configs);
    ADCInterfaceInstance::instance().init(0);

    ASSERT_EQ(ADCInterfaceInstance::instance().is_in_imd_startup_period(), true);

    Verify(Method(ArduinoFake(), pinMode).Using(pinout.teensy_imd_ok_pin, INPUT)).Once();
    Verify(Method(ArduinoFake(), pinMode).Using(pinout.teensy_precharge_pin, INPUT)).Once();
    Verify(Method(ArduinoFake(), pinMode).Using(pinout.teensy_shdn_out_pin, INPUT)).Once();
    Verify(Method(ArduinoFake(), pinMode).Using(pinout.teensy_ts_out_filtered_pin, INPUT)).Once();
    Verify(Method(ArduinoFake(), pinMode).Using(pinout.teensy_pack_out_filtered_pin, INPUT)).Once();
    Verify(Method(ArduinoFake(), pinMode).Using(pinout.teensy_bspd_current_pin, INPUT)).Once();
    Verify(Method(ArduinoFake(), pinMode).Using(pinout.teensy_scaled_24V_pin, INPUT)).Once();
}

TEST (ADCInterfaceTesting, read_imd_ok) {
    When(Method(ArduinoFake(), pinMode)).AlwaysReturn();
    When(Method(ArduinoFake(), analogRead)).AlwaysReturn(0);

    ADCInterfaceInstance::create(pinout, conversions, thresholds, configs);
    ADCInterfaceInstance::instance().init(0);

    ADCInterfaceInstance::instance().read_imd_ok(1000);
    ASSERT_EQ(ADCInterfaceInstance::instance().is_in_imd_startup_period(), true);
    Verify(Method(ArduinoFake(), analogRead)).Never();

    ADCInterfaceInstance::instance().read_imd_ok(3000);
    ASSERT_EQ(ADCInterfaceInstance::instance().is_in_imd_startup_period(), false);
    Verify(Method(ArduinoFake(), analogRead)).Never();

    ADCInterfaceInstance::instance().read_imd_ok(3000);
    Verify(Method(ArduinoFake(), analogRead).Using(pinout.teensy_imd_ok_pin)).Once();
}

TEST (ADCInterfaceTesting, read_functions) {
    When(Method(ArduinoFake(), pinMode)).AlwaysReturn();
    When(Method(ArduinoFake(), analogRead)).AlwaysReturn(0);

    ADCInterfaceInstance::create(pinout, conversions, thresholds, configs);
    ADCInterfaceInstance::instance().init(3000);
    ADCInterfaceInstance::instance().read_shdn_voltage();
    ADCInterfaceInstance::instance().read_shdn_out_voltage();
    ADCInterfaceInstance::instance().read_precharge_out();
    ADCInterfaceInstance::instance().read_precharge_voltage();
    ADCInterfaceInstance::instance().read_ts_out_filtered();
    ADCInterfaceInstance::instance().read_pack_out_filtered();
    ADCInterfaceInstance::instance().read_bspd_current();
    ADCInterfaceInstance::instance().read_global_lv_value();
   
    Verify(Method(ArduinoFake(), analogRead).Using(pinout.teensy_shdn_out_pin)).Exactly(2_Times);
    Verify(Method(ArduinoFake(), analogRead).Using(pinout.teensy_precharge_pin)).Exactly(2_Times);
    Verify(Method(ArduinoFake(), analogRead).Using(pinout.teensy_ts_out_filtered_pin)).Once();
    Verify(Method(ArduinoFake(), analogRead).Using(pinout.teensy_pack_out_filtered_pin)).Once();
    Verify(Method(ArduinoFake(), analogRead).Using(pinout.teensy_bspd_current_pin)).Once();
    Verify(Method(ArduinoFake(), analogRead).Using(pinout.teensy_scaled_24V_pin)).Once();
}