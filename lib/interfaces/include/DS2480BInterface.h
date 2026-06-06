#ifndef DS2480BInterface_H
#define DS2480BInterface_H

#include "Arduino.h"
#include "etl/singleton.h"


namespace ds2480b_default_parameters
{
    constexpr const uint8_t BREAK_CMD = 0x00;

    constexpr const uint8_t RESET_CMD = 0xC5;
    constexpr const uint8_t SET_DATA_MODE = 0xE1;
    constexpr const uint8_t SET_CMD_MODE = 0xE3;
    constexpr const uint8_t PULSE_TERMINATION = 0xF1;

    // DS2480B_Detect config bytes
    constexpr const uint8_t TIMING_BYTE = 0xC1;
    constexpr const uint8_t SET_PDSRC = 0x17;
    constexpr const uint8_t SET_W1LD = 0x45;
    constexpr const uint8_t SET_DSO_W0RT = 0x5B;
    constexpr const uint8_t READ_RBR = 0x0F;
    constexpr const uint8_t SEND_1WIRE_BIT = 0x91;

    // Expected detect responses
    constexpr const uint8_t PDSRC_RESP = 0x16;
    constexpr const uint8_t W1LD_RESP = 0x44;
    constexpr const uint8_t DSO_W0RT_RESP = 0x5A;
    constexpr const uint8_t RBR_RESP = 0x00;
    constexpr const uint8_t BIT_RESP = 0x93;

    // RESET command code (bits 1:0 of response)
    constexpr const uint8_t RESET_SHORTED = 0x00;
    constexpr const uint8_t RESET_PRESENCE = 0x01;
    constexpr const uint8_t RESET_ALARM = 0x02;
    constexpr const uint8_t RESET_NO_PRESENCE = 0x03;
};

struct DS2480BCommands_s
{
    const uint8_t break_cmd;
    const uint8_t reset_cmd;
    const uint8_t set_data_mode;
    const uint8_t set_cmd_mode;
    const uint8_t pulse_termination;
    const uint8_t timing_byte;
    const uint8_t detect_sequence[5];
    const uint8_t detect_response[5];
};

struct DS2480BInterfaceParams_s
{
    DS2480BCommands_s commands;
    uint16_t baud_rate;
};

enum class DS2480B_Mode
{
    DATA_MODE,
    COMMAND_MODE,
    NUM_MODES
};

class DS2480BInterface
{
public:
    DS2480BInterface(uint16_t baud_rate = 9600,
                 DS2480BCommands_s commands =
                 {
                    .break_cmd = ds2480b_default_parameters::BREAK_CMD,
                    .reset_cmd = ds2480b_default_parameters::RESET_CMD,
                    .set_data_mode = ds2480b_default_parameters::SET_DATA_MODE,
                    .set_cmd_mode = ds2480b_default_parameters::SET_CMD_MODE,
                    .pulse_termination = ds2480b_default_parameters::PULSE_TERMINATION,
                    .timing_byte = ds2480b_default_parameters::TIMING_BYTE,
                    .detect_sequence = {
                                            ds2480b_default_parameters::SET_PDSRC,
                                            ds2480b_default_parameters::SET_W1LD,
                                            ds2480b_default_parameters::SET_DSO_W0RT,
                                            ds2480b_default_parameters::READ_RBR,
                                            ds2480b_default_parameters::SEND_1WIRE_BIT
                                        },
                    .detect_response  = {
                                            ds2480b_default_parameters::PDSRC_RESP,
                                            ds2480b_default_parameters::W1LD_RESP,
                                            ds2480b_default_parameters::DSO_W0RT_RESP,
                                            ds2480b_default_parameters::RBR_RESP,
                                            ds2480b_default_parameters::BIT_RESP
                                        }
                 }
        ) : _ds2480b_params
            {
                commands,
                baud_rate
            },
            _curr_mode(DS2480B_Mode::COMMAND_MODE)
    {}

    void init();

    /**
     *  @brief Resets and configures the DS2480B with "flex mode" settings.
     *  @return true if chip responds correctly.
     */
    bool OWDetect();

    /**
     * @brief Sends 1-Wire reset and checks for presence pulse.
     * @return true if a device presence pulse was detected, false otherwise
     */
    bool OWReset();

    /**
     *  @brief Sends a single byte in Data Mode.
     *  @return -1 if the bus is hung
     */
    int OWWriteByte(uint8_t data);

    /**
     *  @brief Read is done by writing 0xFF and sampling the response.
     *  @return -1 if the bus is hung
     */
    int OWReadByte();


private:
    const DS2480BInterfaceParams_s _ds2480b_params;
    DS2480B_Mode _curr_mode;

    /**
     *
     */
    void _flushRXBuffer();

    /**
     *
     */
    void _ensureCommandMode();

    /**
     *
     */
    void _ensureDataMode();
};


using DS2480BInterfaceInstance = etl::singleton<DS2480BInterface>;

#endif