#include "EMTempSensorInterface.h"
#include "DS2480BInterface.h"

const uint32_t baudrate = 115200;

void setup()
{
    Serial.begin(baudrate);
    while (!Serial) {}  // wait for serial monitor to open

    // Create the DS2480B singleton first — EMTempSensor depends on it
    DS2480BInterfaceInstance::create();
    DS2480BInterfaceInstance::instance().init();

    // Create the temp sensor singleton, passing the DS2480B instance as the bus
    EMTempSensorInterfaceInstance::create(DS2480BInterfaceInstance::instance());
    EMTempSensorInterfaceInstance::instance().init();

    Serial.println("Setup complete");
}

void loop()
{
    uint32_t now = millis();

    EMTempSensorInterfaceInstance::instance().tick(now);

    // Print temperatures once per second
    static uint32_t last_print_ms = 0;

    if (now - last_print_ms >= 500)
    {
        last_print_ms = now;

        uint32_t print_start = micros();

        Serial.println("====================================");

        celsius temp = 0.0f;
        celsius max_temp = 0.0f;

        for (uint8_t i = 0; i < EMtemp_default_parameters::NUM_TEMP_SENSORS; i++)
        {
            temp = EMTempSensorInterfaceInstance::instance().get_temperature(i);

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

        max_temp = EMTempSensorInterfaceInstance::instance().get_max_temperature();
        Serial.print("Max: ");
        isnan(max_temp) ? Serial.println("NO READ") : (Serial.print(max_temp, 2), Serial.println(" C"));

        Serial.print("Overtemp: ");
        Serial.println(EMTempSensorInterfaceInstance::instance().is_overtemp() ? "YES" : "NO");

        uint32_t print_elapsed = micros() - print_start;

        Serial.print("Print overhead: ");
        Serial.print(print_elapsed);
        Serial.println(" us");

         Serial.println("");
    }
}