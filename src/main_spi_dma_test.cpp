#include <Arduino.h>
#include <SPI.h>
#include <EventResponder.h>

#include <array>

#include "ACU_Constants.h"

#include "SystemTimeInterface.h"
#include "BMSDriverGroup.h"

using namespace std;

const constexpr size_t num_bytes_command_and_pec = 4;
const constexpr size_t num_chips = 2;

const constexpr size_t buffer_size = num_chips * 8 + num_bytes_command_and_pec;

EventResponder spi_event;
array<uint8_t, buffer_size> tx_buf;
array<uint8_t, buffer_size> rx_buf;
volatile bool dma_busy;

unsigned long current_time = 0; 
elapsedMillis timer = 0;

void asyncEventResponder(EventResponderRef event_responder)
{
    dma_busy = false;

    digitalWrite(38, HIGH);
    delayMicroseconds(1);
    SPI1.endTransaction();

    Serial.println("RX DATA after Callback:");
    for (size_t i = 0; i < buffer_size; i++)
    {   
        Serial.print(rx_buf[i], HEX); Serial.print(" ");
    }
    Serial.println();
}

void setup()
{
    // Serial init
    Serial.begin(ACUInterfaces::SERIAL_BAUDRATE);

    // SPI1 init
    SPI1.begin();
    SPI1.setMOSI(ACUInterfaces::SPI1_MOSI_PIN);
    SPI1.setMISO(ACUInterfaces::SPI1_MISO_PIN);
    SPI1.setSCK(ACUInterfaces::SPI1_SCK_PIN);

    // CS init
    pinMode(ACUConstants::CS[1], OUTPUT);
    digitalWrite(ACUConstants::CS[1], HIGH);

    // EventResponder init
    spi_event.attachImmediate(&asyncEventResponder);

    // set static tx buf
    tx_buf = {0x00, 0x04, 0x07, 0xC2};
}

void loop()
{
    if (timer > 1)
    {
        Serial.print("TIMER AT: "); Serial.println(timer);
        timer = 0;
        if (!dma_busy)
        {
            auto start = sys_time::hal_micros();
            SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));

            digitalWrite(ACUCosntants::CS[1], LOW);
            delayMicroseconds(1);
            
            SPI1.transfer(tx_buf.data(), rx_buf.data(), buffer_size, spi_event);
            dma_busy = true;

            auto end = sys_time::hal_micros();
            auto diff = end - start;
            Serial.print("Send and Received Time: "); Serial.println(diff);
        }
    }
}