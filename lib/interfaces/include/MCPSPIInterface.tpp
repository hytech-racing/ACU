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

std::array<uint8_t,3>
mcp_spi_interface::make_cmd(uint8_t channel, bool singleEnded) {
  const uint8_t b0 = singleEnded ? MCP_START_READING_SINGLE : MCP_START_READING_DIFF;
  const uint8_t b1 = static_cast<uint8_t>((channel & 0x07) << 5);
  const uint8_t b2 = 0x00; //placeholder
  return { b0, b1, b2 };
}

// --- Read  Once ---
uint16_t mcp_spi_interface::read_channel(int cs, const std::array<uint8_t,3>& cmd) {
  SPI.beginTransaction(SPISettings(MCP_SPI_HZ, MSBFIRST, MCP_SPI_MODE));
  _write_and_delay_low(cs, 1);

  
  SPI.transfer(cmd[0]);          // starts read
  const uint8_t hi = SPI.transfer(cmd[1]); // sets which channel and reads high byte
  const uint8_t lo = SPI.transfer(cmd[2]); // reads low byte

  _write_and_delay_high(cs, 1);
  SPI.endTransaction();

  return static_cast<uint16_t>(((hi & 0x0F) << 8) | lo); //combines hi and low bits
}

volt mcp_spi_interface::read_channel_voltage(int cs, const std::array<uint8_t,3>& cmd) {
  return static_cast<float>(read_channel(cs, cmd)) * REFERENCE_VOLTAGE / RESOLUTION;

}
