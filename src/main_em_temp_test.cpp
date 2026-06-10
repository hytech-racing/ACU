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

    Serial.println();
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
}