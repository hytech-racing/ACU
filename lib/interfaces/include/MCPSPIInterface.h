#ifndef MCPSPIINTERFACE_H
#define MCPSPIINTERFACE_H

#include <SPI.h>
#include <array>
#include <stdint.h>

enum class CHANNEL_CODES_e : uint8_t
{
    CH0 = 0,
    CH1 = 1,
    CH2 = 2,
    CH3 = 3,
    CH4 = 4,
    CH5 = 5,
    CH6 = 6,
    CH7 = 7,
};


namespace mcp_spi_interface {

    // Fixed SPI settings
    static constexpr uint32_t MCP_SPI_HZ   = 1'000'000;   // 1 MHz
    static constexpr uint8_t  MCP_SPI_MODE = SPI_MODE0;   // MCP3208 supports Mode 0 or 1
    static constexpr uint8_t  MCP_START_READING = 0x01;
    static constexpr uint8_t  MCP_READING_SINGLE = 0x01;
    static constexpr uint8_t  MCP_READING_DIFF   = 0x00;
    static constexpr uint16_t RESOLUTION = 4095;
    static constexpr float REFERENCE_VOLTAGE = 3.3;

    // CS helpers (matching the LTC style)
    void _write_and_delay_low (int cs, int delay_microSeconds);
    void _write_and_delay_high(int cs, int delay_microSeconds);

    /**
     * Build the 3-byte MCP3208 command.
     * @param channel       0..7 (channel code)
     * @param singleEnded   true = SGL/DIFF=1 (single-ended), false = 0 (differential)
     * @return              {byte0, byte1, byte2}
     */
    byte make_cmd(uint8_t channel, bool singleEnded);

    /**
     * Perform one MCP3208 read using the provided command.
     * Sends 3 bytes, returns assembled 12-bit raw result
     *
     * @param cs    chip-select pin
     * @param cmd   3-byte command (from make_cmd)
     * @return      12-bit conversion code (0..4095)
     */
    uint16_t read_channel(int cs, byte cmd);

    /**
     * Perform one MCP3208 read using the provided command.
     * Sends 3 bytes, returns voltage
     *
     * @param cs    chip-select pin
     * @param cmd   3-byte command (from make_cmd)
     * @return      volt
     */
    volt read_channel_voltage(int cs, byte cmd);

}

#include <MCPSPIInterface.tpp>
#endif
