#ifndef __USER_H__
#define __USER_H__

#include <stdio.h>			
#include "type.h"
#include "error.h"

#define User_MAX_PKT_SIZE               255
#define User_RETRY_MAX_LIMIT            500
#define User_USE_SHORT_CMD              0
#define IT913X_ADDRESS                 (0x38>>1)

/** Define I2C master speed, the default value 0x07 means 366KHz (1000000000 / (24.4 * 16 * User_I2C_SPEED)). */
#define User_I2C_SPEED              	0x16//0x07

/** Define I2C address of secondary chip when Diversity mode or PIP mode is active. */
#define User_I2C_ADDRESS           		0x38
#define User_Chip2_I2C_ADDRESS      	0x38//0x3A


#define	User_SBUS Bus_I2C

/** Define USB frame size */
#define User_USB20_MAX_PACKET_SIZE      512
#define User_USB20_FRAME_SIZE           (188 * 348)
#define User_USB20_FRAME_SIZE_DW        (User_USB20_FRAME_SIZE / 4)
#define User_USB11_MAX_PACKET_SIZE      64
#define User_USB11_FRAME_SIZE           (188 * 21)
#define User_USB11_FRAME_SIZE_DW        (User_USB11_FRAME_SIZE / 4)

/**
 * Delay Function
 */
uint32_t User_delay (
      uint32_t           dwMs
);


/**
 * Enter critical section
 */
uint32_t User_enterCriticalSection (
);


/**
 * Leave critical section
 */
uint32_t User_leaveCriticalSection (
);


/**
 * Config MPEG2 interface
 */
uint32_t User_mpegConfig (
);


/**
 * Write data via "Control Bus"
 * I2C mode : uc2WireAddr mean demodulator chip address, the default value is 0x38
 * USB mode : uc2WireAddr is useless, don't have to send this data
 */
uint32_t User_busTx (
      uint32_t           bufferLength,
      uint8_t*           buffer
);


/**
 * Read data via "Control Bus"
 * I2C mode : uc2WireAddr mean demodulator chip address, the default value is 0x38
 * USB mode : uc2WireAddr is useless, don't have to send this data
 */
uint32_t User_busRx (
      uint32_t           bufferLength,
     uint8_t*           buffer
);


#endif
