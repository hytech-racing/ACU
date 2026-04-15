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
const array<uint8_t, 4> poll_tx_buf = {0x02, 0xE0, 0x38, 0x06};
const array<uint8_t, 4> poll_rx_buf = {0x00, 0x04, 0x07, 0xC2};
array<uint8_t, buffer_size> read_tx_buf;
array<uint8_t, buffer_size> read_rx_buf;
volatile bool dma_busy;

volatile SPIState_e spi_state = SPIState_e::IDLE;

elapsedMillis conversion_timer;

unsigned long current_time = 0; 
elapsedMillis timer = 0;

const int pulse_time = 250;
const int spi_baudrate = 1000000;

void async_event_responder(EventResponderRef event_responder)
{
    digitalWrite(ACUConstants::CS[1], HIGH);
    delayMicroseconds(1);
    SPI1.endTransaction();

    if (spi_state == SPIState_e::WAIT_POLL_ADC_COMPLETE) 
    {
        conversion_timer = 0;
        dma_busy = false;
        spi_state = SPIState_e::WAIT_CONVERSION;
    } 
    else if (spi_state == SPIState_e::WAIT_READ_COMPLETE)
    {
        Serial.println("RX DATA after Callback:");
        for (size_t i = 0; i < buffer_size; i++)
        {   
            Serial.print(read_rx_buf[i], HEX); Serial.print(" ");
        }
        Serial.println();
        dma_busy = false;
        spi_state = SPIState_e::IDLE;
    }
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
}

void loop()
{
    if (timer > 3)
    {
        Serial.print("TIMER AT: "); Serial.println(timer);
        timer = 0;
        if (!dma_busy)
        {
            auto send = [&](auto* tx, auto* rx, size_t len) {
                ltc_spi_interface::write_and_delay_low(ACUConstants::CS[1], pulse_time);
                ltc_spi_interface::write_and_delay_high(ACUConstants::CS[1], pulse_time);
                SPI1.beginTransaction(SPISettings(spi_baudrate, MSBFIRST, SPI_MODE3));
                digitalWrite(ACUConstants::CS[1], LOW);
                delayMicroseconds(1);
                auto start = sys_time::hal_micros();
                SPI1.transfer(tx, rx, len, spi_event);
                dma_busy = true;
                Serial.println(sys_time::hal_micros() - start);
            };

            if (spi_state == SPIState_e::IDLE)
            {
                Serial.print("(POLL) Send and Received Time: ");
                send(poll_tx_buf.data(), poll_rx_buf.data(), 4);
                spi_state = SPIState_e::WAIT_POLL_ADC_COMPLETE;
            }
            else if (spi_state == SPIState_e::WAIT_CONVERSION && conversion_timer > 3)
            {
                Serial.print("(READ) Send and Received Time: ");
                send(read_tx_buf.data(), read_rx_buf.data(), buffer_size);
                spi_state = SPIState_e::WAIT_READ_COMPLETE;
            }
        }
    }
}