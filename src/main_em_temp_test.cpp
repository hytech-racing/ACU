#include "EMTempSensorInterface.h"
#include "DS2480BInterface.h"

const uint32_t baudrate = 115200;

void setup()
{
    Serial.begin(baudrate);
    while (!Serial) {}

    DS2480BInterfaceInstance::create();

    EMTempSensorInterfaceInstance::create(DS2480BInterfaceInstance::instance());
    EMTempSensorInterfaceInstance::instance().init();

    Serial.println("Setup complete");
}

void loop()
{
    uint32_t now = micros();

    EMTempSensorInterfaceInstance::instance().tick(now);

    static uint8_t last_sensor_index = 0;
    uint8_t curr_index = EMTempSensorInterfaceInstance::instance().get_current_sensor_index();

    // Only print when we've just completed a full cycle (index wraps to 0)
    if (last_sensor_index != 0 && curr_index == 0)
    {
        for (uint8_t i = 0; i < EMtemp_default_parameters::NUM_TEMP_SENSORS; i++)
        {
            celsius temp = EMTempSensorInterfaceInstance::instance().get_temperature(i);

            Serial.print("Sensor ");
            Serial.print(i);
            Serial.print(": ");

            if (isnan(temp))
                Serial.println("NO READ");
            else
            {
                Serial.print(temp, 2);
                Serial.println(" C");
            }
        }

        celsius max_temp = EMTempSensorInterfaceInstance::instance().get_max_temperature();
        Serial.print("Max: ");
        isnan(max_temp) ? Serial.println("NO READ") : (Serial.print(max_temp, 2), Serial.println(" C"));

        Serial.print("Overtemp: ");
        Serial.println(EMTempSensorInterfaceInstance::instance().is_overtemp() ? "YES" : "NO");
        Serial.println();
    }

    last_sensor_index = curr_index;
}