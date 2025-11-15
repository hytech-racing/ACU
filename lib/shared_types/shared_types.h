#ifndef SHAREDTYPES_H
#define SHAREDTYPES_H

#include "SharedFirmwareTypes.h"

struct BMSCoreData_s {
    volt min_cell_voltage;
    volt max_cell_voltage;
    volt pack_voltage;
    celsius max_cell_temp; 
    celsius min_cell_temp; 
    celsius max_board_temp;
};

/**
 * CurrentReadGroup_e - State machine for incremental BMS register group reading
 *
 * This enum defines the 6-state read cycle used to distribute voltage and GPIO readings
 * across multiple read_data() calls. Instead of reading all registers at once (which takes
 * significant time), each call reads ONE register group, cycling through all 6 groups.
 *
 * STATE MACHINE CYCLE:
 *
 *   Call 1 → CURRENT_GROUP_A     → Read cells 0-2   (CV Group A)
 *   Call 2 → CURRENT_GROUP_B     → Read cells 3-5   (CV Group B)
 *   Call 3 → CURRENT_GROUP_C     → Read cells 6-8   (CV Group C)
 *   Call 4 → CURRENT_GROUP_D     → Read cells 9-11  (CV Group D)  [9-cell chips: skip]
 *   Call 5 → CURRENT_GROUP_AUX_A → Read GPIO 0-2    (Aux Group A: thermistors 0-2)
 *   Call 6 → CURRENT_GROUP_AUX_B → Read GPIO 3-5    (Aux Group B: thermistors 3-4, board temp)
 *   [GOTO Call 1]
 *
 * HARDWARE MAPPING:
 * - Each LTC6811 chip has 12 cell voltage registers divided into 4 groups (A-D)
 * - Each group contains 3 consecutive cell readings (6 bytes = 3 × 16-bit values)
 * - GPIO registers are divided into 2 groups (AUX_A, AUX_B)
 * - GPIO 0-3: Cell thermistors (temperature sensors)
 * - GPIO 4: Board temperature sensor (MCP9701)
 * - GPIO 5: Not used (padding in AUX_B)
 *
 * TIMING & FREQUENCY:
 * - Original (main): Read all 6 groups at 50 Hz (20ms period)
 * - Optimized (this branch): Read 1 group per call at 300 Hz (3ms period)
 * - Effective sampling rate per cell: 300 Hz / 6 groups = 50 Hz (same as before)
 *
 * CHIP ARCHITECTURE NOTES:
 * - System has 12 ICs total in pairs: 6 × (12-cell + 9-cell) = 126 cells total
 * - Even-indexed chips (0, 2, 4, 6, 8, 10): 12 cells each
 * - Odd-indexed chips  (1, 3, 5, 7, 9, 11): 9 cells each
 * - For 9-cell chips, GROUP_D read is skipped (no cells 10-12)
 *
 * VALIDATION:
 * Each group read has its own validity flag in ValidPacketData_s:
 *   - CURRENT_GROUP_A     → valid_read_cells_1_to_3
 *   - CURRENT_GROUP_B     → valid_read_cells_4_to_6
 *   - CURRENT_GROUP_C     → valid_read_cells_7_to_9
 *   - CURRENT_GROUP_D     → valid_read_cells_10_to_12
 *   - CURRENT_GROUP_AUX_A → valid_read_gpios_1_to_3
 *   - CURRENT_GROUP_AUX_B → valid_read_gpios_4_to_6
 */

enum ReadGroup_e {
    CV_GROUP_A = 0, CV_GROUP_B, CV_GROUP_C, CV_GROUP_D,
    AUX_GROUP_A,   AUX_GROUP_B, NUM_GROUPS
};


#endif