#ifndef __USER_H__
#define __USER_H__

#include "orion_cal.h"

#define IN
#define OUT
#define INOUT

#define Bus_I2C             						1
#define Bus_USB            							2
#define Bus_9035U2I         						3  /** I2C bus for Ganymede USB */
#define IT9510User_MAX_PKT_SIZE               		255

#define IT9510User_RETRY_MAX_LIMIT            		/*1000*/ 1 //???


/** Define I2C master speed, the default value 0x07 means 366KHz (1000000000 / (24.4 * 16 * IT9510User_I2C_SPEED)). */
#define IT9510User_IIC_SPEED                  		0x07

/** Define I2C address of secondary chip when Diversity mode or PIP mode is active. */
#define IT9510User_IIC_ADDRESS                		0x38
#define IT9510User_SlaveIIC_ADDRESS          		0x3A
#define IT9510User_DEVICETYPE			      		0
#define IT951X_ADDRESS                              (0x38>>1)

/** Define USB frame size */

#define IT9510User_USB20_MAX_PACKET_SIZE_EP4      	256
#define IT9510User_USB20_MAX_PACKET_SIZE_EP5      	512
#define IT9510User_USB20_FRAME_SIZE_EP4           	(188 * 1)
#define IT9510User_USB20_FRAME_SIZE_EP5           	(188 * 348)
#define IT9510User_USB20_FRAME_SIZE_DW        		(IT9510User_USB20_FRAME_SIZE / 4)
#define IT9510User_USB11_MAX_PACKET_SIZE      		64
#define IT9510User_USB11_FRAME_SIZE           		(188 * 21)
#define IT9510User_USB11_FRAME_SIZE_DW        		(IT9510User_USB11_FRAME_SIZE / 4)
#define IT9510User_MAXFRAMESIZE						63 /*10*/ // 10 shall fail i2c xfer...

#define IT9507Cmd_buildCommand(command, processor)  (command + (uint16_t) (processor << 12))
#define IT9517Cmd_buildCommand(command, processor)  (command + (uint16_t) (processor << 12))
/**
 * Memory copy Function
 */
uint32_t IT9510User_memoryCopy (
		IT9510INFO*    modulator,
		void*           dest,
		void*           src,
		uint32_t           count
		);


/**
 * Delay Function
 */
uint32_t IT9510User_delay (
		uint32_t           dwMs
		);

/**
 * printf Function
 */
uint32_t IT9510User_printf (const char* format,...);

/**
 * Enter critical section
 */
uint32_t IT9510User_enterCriticalSection (void);


/**
 * Leave critical section
 */
uint32_t IT9510User_leaveCriticalSection (void);


/**
 * Config MPEG2 interface
 */
uint32_t IT9510User_mpegConfig (
		IT9510INFO*    modulator
		);


/**
 * Write data via "Control Bus"
 * I2C mode : uc2WireAddr mean modulator chip address, the default value is 0x38
 * USB mode : uc2WireAddr is useless, don't have to send this data
 */
uint32_t IT9510User_busTx (
		IT9510INFO*    modulator,
		uint32_t           bufferLength,
		uint8_t*           buffer
		);

uint32_t IT9510User_busTx2 (
		IT9510INFO*    modulator,
		uint32_t           bufferLength,
		uint8_t*           buffer
		);
/**
 * Read data via "Control Bus"
 * I2C mode : uc2WireAddr mean modulator chip address, the default value is 0x38
 * USB mode : uc2WireAddr is useless, don't have to send this data
 */
uint32_t IT9510User_busRx (
		IT9510INFO*    modulator,
		uint32_t           bufferLength,
		uint8_t*           buffer
		);
uint32_t IT9510User_busRx2 (
		IT9510INFO*    modulator,
		uint32_t           bufferLength,
		uint8_t*           buffer
		);

uint32_t IT9510User_setBus (
		IT9510INFO*	modulator
		);

uint32_t IT9510User_Initialization  (
		IT9510INFO*    modulator
		);

uint32_t IT9510User_Finalize  (
		IT9510INFO*    modulator
		);

uint32_t IT9510User_acquireChannel (
		IT9510INFO*    modulator,
		uint16_t          bandwidth,
		uint32_t         frequency
		);

uint32_t IT9510User_acquireChannelDual(
		IT9510INFO*    modulator,
		uint16_t          bandwidth,
		uint32_t         frequency
		);

uint32_t IT9510User_setTxModeEnable (
		IT9510INFO*            modulator,
		uint8_t                    enable
		);

uint32_t IT9510User_LoadDCCalibrationTable (
		IT9510INFO*            modulator
		);

uint32_t IT9510User_rfPowerOn (
		IT9510INFO*            modulator,
		Bool                   isPowerOn
		);

#endif

