#ifndef MCPSPIINTERFACE_H
#define MCPSPIINTERFACE_H

/* Interface Includes (keep it light and LTC-like) */
#include <SPI.h>
#include <array>
#include <stdint.h>

namespace mcp_spi_interface {

    // Fixed SPI settings (simple & consistent)
    static constexpr uint32_t MCP_SPI_HZ   = 1'000'000;   // 1 MHz
    static constexpr uint8_t  MCP_SPI_MODE = SPI_MODE0;   // MCP3208 supports Mode 0 or 1
    static constexpr uint8_t  MCP_START_READING_SINGLE = 0x03;
    static constexpr uint8_t  MCP_START_READING_DIFF   = 0x02;

    // CS helpers (matching the LTC style)
    void _write_and_delay_low (int cs, int delay_microSeconds);
    void _write_and_delay_high(int cs, int delay_microSeconds);

    /**
     * Build the 3-byte MCP3208 command.
     * Packing (MSB-first on the wire):
     *   byte0: 0000 001S  (Start=1 at bit1; SGL=1 or 0 at bit0)
     *   byte1: D2 D1 D0 0 0000  (bit4 = Null=0; lower 4 bits 0)
     *   byte2: 0000 0000  (dummy clocks)
     *
     * @param channel       0..7 (channel for single-ended, pair code for differential)
     * @param singleEnded   true = SGL/DIFF=1 (single-ended), false = 0 (differential)
     * @return              {byte0, byte1, byte2}
     */
    std::array<uint8_t, 3> make_cmd(uint8_t channel, bool singleEnded);

    /**
     * Perform one MCP3208 read using the provided command.
     * Sends 3 bytes, returns assembled 12-bit raw result (0..4095).
     *
     * @param cs    chip-select pin
     * @param cmd   3-byte command (from make_cmd)
     * @return      12-bit conversion code (0..4095)
     */
    uint16_t read_channel(int cs, const std::array<uint8_t, 3>& cmd);

}

#include <MCPSPIInterface.tpp>
#endif
