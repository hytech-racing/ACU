#pragma once
#include <Arduino.h>
#include <SPI.h>
#include "MCPSPIInterface.h"

// --- CS helpers ---
void mcp_spi_interface::_write_and_delay_low(int cs, int us) {
  digitalWrite(cs, LOW);
  delayMicroseconds(us);
}

void mcp_spi_interface::_write_and_delay_high(int cs, int us) {
  digitalWrite(cs, HIGH);
  delayMicroseconds(us);
}

byte mcp_spi_interface::make_cmd(uint8_t channel, bool singleEnded) {
  const uint8_t read_type = singleEnded ? MCP_READING_SINGLE : MCP_READING_DIFF;
  const byte command = ((MCP_START_READING << 7) |                    // start bit
                        (read_type << 6) | // single or differential
                        ((channel & 0x07) << 3));   // channel number
  return command;
}

// --- Read  Once ---
uint16_t mcp_spi_interface::read_channel(int cs, byte cmd) {
  SPI.beginTransaction(SPISettings(MCP_SPI_HZ, MSBFIRST, MCP_SPI_MODE));
  _write_and_delay_low(cs, 0);
  byte command, b0, b1, b2;

  b0 = SPI.transfer(command);
  b1 = SPI.transfer(0x00);
  b2 = SPI.transfer(0x00);

  _write_and_delay_high(cs, 1);
  SPI.endTransaction();
  uint16_t read_data = (b0 & 0x01) << 11 | (b1 & 0xFF) << 3 | (b2 & 0xE0) >> 5;
  return static_cast<uint16_t>(read_data & 0x0FFF);
}

volt mcp_spi_interface::read_channel_voltage(int cs, byte cmd) {
  return static_cast<float>(read_channel(cs, cmd)) * REFERENCE_VOLTAGE / RESOLUTION;

}
