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
    SPI1.endTransaction();

    Serial.println("RX DATA after Callback");
    for (int i = 0; i < buffer_size; i++)
    {   
        Serial.println(rx_buf[i]);
    }
    
}

void setup()
{
    // Serial init
    Serial.begin(115200);

    // SPI1 init
    SPI1.begin();
    SPI1.setMOSI(26);
    SPI1.setMISO(39);
    SPI1.setSCK(27);

    // CS init
    pinMode(38, OUTPUT);
    digitalWrite(38, HIGH);

    // EventResponder init
    spi_event.attachImmediate(&asyncEventResponder);

    // set static tx buf
    tx_buf = {0x00, 0x04, 0x07, 0xC2};
}

void loop()
{
    if (timer > 5000 && !dma_busy)
    {
        timer = 0;

        auto start = sys_time::hal_millis();
        SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));

        digitalWrite(38, LOW);
        
        SPI1.transfer(tx_buf.data(), rx_buf.data(), buffer_size, spi_event);
        dma_busy = true;

        auto end = sys_time::hal_millis();
        auto diff = end - start;
        Serial.print("Send and Received Time: "); Serial.println(diff);
    }
}