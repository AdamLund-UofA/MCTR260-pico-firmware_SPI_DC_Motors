
#include "core/command_packet.h"

// DC Motor 1
#define IN_MOT_1P_BIT  6
#define IN_MOT_1N_BIT  7

// DC Motor 2
#define IN_MOT_2P_BIT  8
#define IN_MOT_2N_BIT  9

/**
 * @brief Apply aux channel inputs to DC Motors 1 and 2.
 *
 * Reads the configured aux slider channels and sets H-bridge direction
 * pins on PICO. Motor behavior:
 *   - Aux > 0:  Forward 
 *   - Aux < 0:  Reverse 
 *   - |Aux| <= 15: Off (both pins LOW, motor brakes to stop)
 *
 * @param cmd Parsed control command from BLE.
 *
 */

void profile_DC_over_SPI_apply(const control_command_t *cmd);

void profile_DC_SPI_motors_stop();
