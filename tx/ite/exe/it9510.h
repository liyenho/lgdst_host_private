#ifndef __IT9510_H__
#define __IT9510_H__

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "type.h"
#include "user.h"
#include "error.h"
#include "register.h"
#include "variable.h"
#include "version.h"
#include "firmware.h"
#include "IQ_fixed_table.h"
#include "tuner.h"
#include "pcr_restamp.h"

/**
 * Write one byte (8 bits) to a specific register in modulator.
 *
 * @param modulator the handle of modulator.
 * @param processor The processor of specified register. Because each chip
 *        has two processor so user have to specify the processor. The
 *        possible values are Processor_LINK and Processor_OFDM.
 * @param registerAddress the address of the register to be written.
 * @param value the value to be written.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     Eagle eagle;
 *
 *     // Set the value of register 0xA000 in modulator to 0.
 *     error = IT9510_writeRegister ((IT9510INFO*) &eagle, 0, Processor_LINK, 0xA000, 0);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 * </pre>
 */
uint32_t IT9510_writeRegister (
		IT9510INFO*    modulator,
		Processor       processor,
		uint32_t           registerAddress,
		uint8_t            value
		);


/**
 * Write a sequence of bytes to the contiguous registers in modulator.
 * The maximum burst size is restricted by the capacity of bus. If bus
 * could transfer N bytes in one cycle, then the maximum value of
 * bufferLength would be N - 5.
 *
 * @param modulator the handle of modulator.
 * @param processor The processor of specified register. Because each chip
 *        has two processor so user have to specify the processor. The
 *        possible values are Processor_LINK and Processor_OFDM.
 * @param registerAddress the start address of the registers to be written.
 * @param bufferLength the number of registers to be written.
 * @param buffer a byte array which is used to store values to be written.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     uint8_t buffer[3] = { 0x00, 0x01, 0x02 };
 *     Eagle eagle;
 *
 *     // Set the value of register 0xA000 in modulator to 0.
 *     // Set the value of register 0xA001 in modulator to 1.
 *     // Set the value of register 0xA002 in modulator to 2.
 *     error = IT9510_writeRegisters ((IT9510INFO*) &eagle, Processor_LINK, 0xA000, 3, buffer);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 * </pre>
 */
uint32_t IT9510_writeRegisters (
		IT9510INFO*    modulator,
		Processor       processor,
		uint32_t           registerAddress,
		uint8_t            bufferLength,
		uint8_t*           buffer
		);

uint32_t IT9510_writeRegisters2 (
		IT9510INFO*    modulator,
		Processor       processor,
		uint32_t           registerAddress,
		uint8_t            bufferLength,
		uint8_t*           buffer
		);

/**
 * Write a sequence of bytes to the contiguous registers in slave device
 * through specified interface (1, 2, 3).
 * The maximum burst size is restricted by the capacity of bus. If bus
 * could transfer N bytes in one cycle, then the maximum value of
 * bufferLength would be N - 6 (one more byte to specify tuner address).
 *
 * @param modulator the handle of modulator.
 * @param interfaceIndex the index of interface. The possible values are
 *        1~3.
 * @param slaveAddress the I2c address of slave device.
 * @param bufferLength the number of registers to be read.
 * @param buffer a byte array which is used to store values to be read.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t IT9510_writeGenericRegisters (
		IT9510INFO*    modulator,
		uint8_t            slaveAddress,
		uint8_t            bufferLength,
		uint8_t*           buffer
		);


/**
 * Write a sequence of bytes to the contiguous cells in the EEPROM.
 * The maximum burst size is restricted by the capacity of bus. If bus
 * could transfer N bytes in one cycle, then the maximum value of
 * bufferLength would be N - 5 (firmware will detect EEPROM address).
 *
 * @param modulator the handle of modulator.
 * @param registerAddress the start address of the cells to be written.
 * @param registerAddressLength the valid bytes of registerAddress.
 * @param bufferLength the number of cells to be written.
 * @param buffer a byte array which is used to store values to be written.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     uint8_t buffer[3] = { 0x00, 0x01, 0x02 };
 *     Eagle eagle;
 *
 *     // Set the value of cell 0x0000 in EEPROM to 0.
 *     // Set the value of cell 0x0001 in EEPROM to 1.
 *     // Set the value of cell 0x0002 in EEPROM to 2.
 *     error = IT9510_writeEepromValues ((IT9510INFO*) &eagle, 0, 0x0000, 3, buffer);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 * </pre>
 */
uint32_t IT9510_writeEepromValues (
		IT9510INFO*    modulator,
		uint16_t            registerAddress,
		uint8_t            bufferLength,
		uint8_t*           buffer
		);


/**
 * Modify bits in the specific register.
 *
 * @param modulator the handle of modulator.
 * @param processor The processor of specified register. Because each chip
 *        has two processor so user have to specify the processor. The
 *        possible values are Processor_LINK and Processor_OFDM.
 * @param registerAddress the address of the register to be written.
 * @param position the start position of bits to be modified (0 means the
 *        LSB of the specifyed register).
 * @param length the length of bits.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     Eagle eagle;
 *
 *     // Modify the LSB of register 0xA000 in modulator to 0.
 *     error = IT9510_writeRegisterBits ((IT9510INFO*) &eagle, 0, Processor_LINK, 0xA000, 0, 1, 0);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 * </pre>
 */
uint32_t IT9510_writeRegisterBits (
		IT9510INFO*    modulator,
		Processor       processor,
		uint32_t           registerAddress,
		uint8_t            position,
		uint8_t            length,
		uint8_t            value
		);


/**
 * Read one byte (8 bits) from a specific register in modulator.
 *
 * @param modulator the handle of modulator.
 * @param processor The processor of specified register. Because each chip
 *        has two processor so user have to specify the processor. The
 *        possible values are Processor_LINK and Processor_OFDM.
 * @param registerAddress the address of the register to be read.
 * @param value the pointer used to store the value read from modulator
 *        register.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     uint8_t value;
 *     Eagle eagle;
 *
 *     // Get the value of register 0xA000 in modulator.
 *     error = IT9510_readRegister ((IT9510INFO*) &eagle, 0, Processor_LINK, 0xA000, &value);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 *     printf ("The value of 0xA000 is %2x", value);
 * </pre>
 */
uint32_t IT9510_readRegister (
		IT9510INFO*    modulator,
		Processor       processor,
		uint32_t           registerAddress,
		uint8_t*           value
		);


/**
 * Read a sequence of bytes from the contiguous registers in modulator.
 * The maximum burst size is restricted by the capacity of bus. If bus
 * could transfer N bytes in one cycle, then the maximum value of
 * bufferLength would be N - 5.
 *
 * @param modulator the handle of modulator.
 * @param processor The processor of specified register. Because each chip
 *        has two processor so user have to specify the processor. The
 *        possible values are Processor_LINK and Processor_OFDM.
 * @param registerAddress the address of the register to be read.
 * @param bufferLength the number of registers to be read.
 * @param buffer a byte array which is used to store values to be read.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     uint8_t buffer[3];
 *     Eagle eagle;
 *
 *     // Get the value of register 0xA000, 0xA001, 0xA002 in modulator.
 *     error = IT9510_readRegisters ((IT9510INFO*) &eagle, 0, Processor_LINK, 0xA000, 3, buffer);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 *     printf ("The value of 0xA000 is %2x", buffer[0]);
 *     printf ("The value of 0xA001 is %2x", buffer[1]);
 *     printf ("The value of 0xA002 is %2x", buffer[2]);
 * </pre>
 */
uint32_t IT9510_readRegisters (
		IT9510INFO*    modulator,
		Processor       processor,
		uint32_t           registerAddress,
		uint8_t            bufferLength,
		uint8_t*           buffer
		);
uint32_t IT9510_readRegisters2 (
		IT9510INFO*    modulator,
		Processor       processor,
		uint32_t           registerAddress,
		uint8_t            bufferLength,
		uint8_t*           buffer
		);

/**
 * Read a sequence of bytes from the contiguous registers in slave device
 * through specified interface (1, 2, 3).
 * The maximum burst size is restricted by the capacity of bus. If bus
 * could transfer N bytes in one cycle, then the maximum value of
 * bufferLength would be N - 6 (one more byte to specify tuner address).
 *
 * @param modulator the handle of modulator.
 * @param slaveAddress the I2c address of slave device.
 * @param bufferLength the number of registers to be read.
 * @param buffer a byte array which is used to store values to be read.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t IT9510_readGenericRegisters (
		IT9510INFO*    modulator,
		uint8_t            slaveAddress,
		uint8_t            bufferLength,
		uint8_t*           buffer
		);


/**
 * Read a sequence of bytes from the contiguous cells in the EEPROM.
 * The maximum burst size is restricted by the capacity of bus. If bus
 * could transfer N bytes in one cycle, then the maximum value of
 * bufferLength would be N - 5 (firmware will detect EEPROM address).
 *
 * @param modulator the handle of modulator.
 * @param registerAddress the start address of the cells to be read.
 * @param bufferLength the number of cells to be read.
 * @param buffer a byte array which is used to store values to be read.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     uint8_t buffer[3];
 *     Eagle eagle;
 *
 *     // Get the value of cell 0x0000, 0x0001, 0x0002 in EEPROM.
 *     error = IT9510_readEepromValues ((IT9510INFO*) &eagle, 0, 0x0000, 3, buffer);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 *     printf ("The value of 0x0000 is %2x", buffer[0]);
 *     printf ("The value of 0x0001 is %2x", buffer[1]);
 *     printf ("The value of 0x0002 is %2x", buffer[2]);
 * </pre>
 */
uint32_t IT9510_readEepromValues (
		IT9510INFO*    modulator,
		uint16_t            registerAddress,
		uint8_t            bufferLength,
		uint8_t*           buffer
		);


/**
 * Read bits of the specified register.
 *
 * @param modulator the handle of modulator.
 * @param processor The processor of specified register. Because each chip
 *        has two processor so user have to specify the processor. The
 *        possible values are Processor_LINK and Processor_OFDM.
 * @param registerAddress the address of the register to be read.
 * @param position the start position of bits to be read (0 means the
 *        LSB of the specifyed register).
 * @param length the length of bits.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     uint8_t value;
 *     Eagle eagle;
 *
 *     // Read the LSB of register 0xA000 in modulator.
 *     error = IT9510_readRegisterBits ((IT9510INFO*) &eagle, 0, Processor_LINK, 0xA000, 0, 1, &value);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 *     printf ("The value of LSB of 0xA000 is %2x", value);
 * </pre>
 */
uint32_t IT9510_readRegisterBits (
		IT9510INFO*    modulator,
		Processor       processor,
		uint32_t           registerAddress,
		uint8_t            position,
		uint8_t            length,
		uint8_t*           value
		);


/**
 * Get the version of firmware.
 *
 * @param modulator the handle of modulator.
 * @param version the version of firmware.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     uint32_t version;
 *     Eagle eagle;
 *
 *     // Get the version of Link layer firmware.
 *     error = IT9510_getFirmwareVersion ((IT9510INFO*) &eagle, Processor_LINK, &version);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("The version of firmware is : %X", version);
 * </pre>
 */
uint32_t IT9510_getFirmwareVersion (
		IT9510INFO*    modulator,
		Processor       processor,
		uint32_t*          version
		);



/**
 * Load the IR table for USB device.
 *
 * @param modulator the handle of modulator.
 * @param tableLength The length of IR table.
 * @param table The content of IR table.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 * </pre>
 */
uint32_t IT9510_loadIrTable (
		IT9510INFO*    modulator,
		uint16_t            tableLength,
		uint8_t*           table
		);


/**
 * First, download firmware from host to modulator. Actually, firmware is
 * put in firmware.h as a part of source code. Therefore, in order to
 * update firmware the host have to re-compile the source code.
 * Second, setting all parameters which will be need at the beginning.
 *
 * @param modulator the handle of modulator.
 * @param streamType The format of TS interface type.
 * @param busId The id of bus type.1:Bus_I2C 2:Bus_USB
 * @param i2cAddr The address of i2c bus.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     Modulator eagle;
 *
 *     // Initialize demodulators.
 *     // SAW Filter  : 8MHz
 *     // Stream Type : IP Datagram.
 *     error = IT9510_TXinitialize ((IT9510INFO*) &eagle, SERIAL_TS_INPUT, Bus_I2C, 0x38);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 * </pre>
 */
uint32_t IT9510_initialize (
		IT9510INFO*    modulator,
		TsInterface   streamType,
		uint8_t            busId,
		uint8_t            i2cAddr
		);


/**
 * Power off the demodulators.
 *
 * @param modulator the handle of modulator.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     Modulator eagle;
 *
 *     // Finalize demodulators.
 *     error = IT9510_finalize ((IT9510INFO*) &eagle);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 * </pre>
 */
uint32_t IT9510_finalize (
		IT9510INFO*    modulator
		);


/**
 * Reset modulator.
 *
 * @param modulator the handle of modulator.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     Modulator eagle;
 *
 *     // Reset modulator.
 *     error = IT9510_reset ((IT9510INFO*) &eagle);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 * </pre>
 */
uint32_t IT9510_reset (
		IT9510INFO*    modulator
		);



/**
 * Set Tx channel modulation related information.
 *
 * @param modulator the handle of modulator.
 * @param channelModulation The modulation of channel.
 * @return Error_NO_ERROR: successful, other non-zero error code otherwise.
 * @example <pre>
 * </pre>
 */
uint32_t IT9510_setTXChannelModulation (
		IT9510INFO*            modulator,
		ChannelModulation*      channelModulation
		);


uint32_t IT9510_setISDBTChannelModulation (
		IT9510INFO*            modulator,
		ISDBTModulation      isdbtModulation
		) ;
/**
 * Set TX mode enable(output data).
 *
 * @param modulator the handle of modulator.
 * @param enable The flag of enable(1:on / 0:off).
 * @return Error_NO_ERROR: successful, other non-zero error code otherwise.
 */
uint32_t IT9510_setTxModeEnable (
		IT9510INFO*            modulator,
		uint8_t                    enable
		);

/**
 * Set output frequency.
 *
 * @param modulator the handle of modulator.
 * @param frequency the channel frequency in KHz.
 * @return Error_NO_ERROR: successful, other non-zero error code otherwise.
 */
uint32_t IT9510_setFrequency (
		IT9510INFO*    modulator,
		uint32_t           frequency
		);

/**
 * Specify the bandwidth of channel and tune the channel to the specific
 * frequency. Afterwards, host could use output parameter dvbH to determine
 * if there is a DVB-H signal.
 * In DVB-T mode, after calling this function the output parameter dvbH
 * should return False and host could use output parameter "locked" to check
 * if the channel has correct TS output.
 * In DVB-H mode, after calling this function the output parameter dvbH should
 * return True and host could start get platform thereafter.
 *
 * @param modulator the handle of modulator.
 * @param bandwidth The channel bandwidth.
 *        DVB-T: 5000, 6000, 7000, and 8000 (KHz).
 * @param frequency the channel frequency in KHz.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     Eagle eagle;
 *
 *     error = IT9510_acquireTxChannel ((IT9510INFO*) &eagle, 8000, 666000);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");*
 *    
 * </pre>
 */

uint32_t IT9510_acquireTxChannel (
		IT9510INFO*      modulator,
		uint16_t            bandwidth,
		uint32_t           frequency
		);

/**
 * Specify the bandwidth of channel and tune the channel to the specific
 * frequency. Afterwards, host could use output parameter dvbH to determine
 * if there is a DVB-H signal.
 * In DVB-T mode, after calling this function the output parameter dvbH
 * should return False and host could use output parameter "locked" to check
 * if the channel has correct TS output.
 * In DVB-H mode, after calling this function the output parameter dvbH should
 * return True and host could start get platform thereafter.
 *
 * @param modulator the handle of modulator.
 * @param bandwidth The channel bandwidth.
 *        DVB-T: 5000, 6000, 7000, and 8000 (KHz).
 * @param frequency1 the channel frequency in KHz(for Eagle2, less than 950 MHz).
 * @param frequency2 the channel frequency in KHz(for RF2072, for 2.4G) .
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 */

uint32_t IT9510_acquireTxChannelDual(
		IT9510INFO*      modulator,
		uint16_t            bandwidth,
		uint32_t           frequency1,
		uint32_t			frequency2
		);

/**
 * reset PSB buffer
 *
 * @param modulator the handle of modulator.
 * @return Error_NO_ERROR: successful, other non-zero error code otherwise.
 */
uint32_t IT9510_resetPSBBuffer (
		IT9510INFO*    modulator
		);

/**
 * Set the output stream type of chip. Because the device could output in
 * many stream type, therefore host have to choose one type before receive
 * data.
 *
 * Note: After host know all the available channels, and want to change to
 *       specific channel, host have to choose output mode before receive
 *       data. Please refer the example of IT9510_setTsInterface.
 *
 * @param modulator the handle of modulator.
 * @param streamType the possible values are
 *        DVB-T:    PARALLEL_TS_INPUT
 *                  SERIAL_TS_INPUT
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     Eagle eagle;
 *
 *     error = IT9510_setTsInterface ((IT9510INFO*) &eagle, SERIAL_TS_INPUT)
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 * </pre>
 */
uint32_t IT9510_setTsInterface (
		IT9510INFO*    modulator,
		TsInterface   streamType
		);

/**
 *
 * @param modulator the handle of modulator.
 * @param code the value of IR raw code, the size should be 4 or 6,
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 * </pre>
 */
uint32_t IT9510_getIrCode (
		IT9510INFO*    modulator,
		uint32_t*          code
		);


/**
 * Return to boot code
 *
 * @param modulator the handle of modulator.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 * </pre>
 */
uint32_t IT9510_TXreboot (
		IT9510INFO*    modulator
		);



/**
 *
 * @param modulator the handle of modulator.
 * @param contorl 1: Power up, 0: Power down;
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 * </pre>
 */
uint32_t IT9510_controlPowerSaving (
		IT9510INFO*    modulator,
		uint8_t            control
		);



/**
 * Control PID fileter
 *
 * @param modulator the handle of modulator.
 * @param control 0: tha same PID pass 1: difference PID pass.
 * @param enable 0: FID filter Disable, 1: FID filter Enable.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 * </pre>
 */
uint32_t IT9510_controlPidFilter (
		IT9510INFO*    modulator,
		uint8_t            control,
		uint8_t            enable
		);


/**
 * Reset PID filter.
 *
 * @param modulator the handle of modulator.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     Eagle eagle;
 *
 *     error = IT9510_resetPidFilter ((IT9510INFO*) &eagle, 0);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 * </pre>
 */
uint32_t IT9510_resetPidFilter (
		IT9510INFO*    modulator
		);


/**
 * Add PID to PID filter.
 *
 * @param modulator the handle of modulator.
 * @param index the index of PID filter.
 * @param pid the PID that will be add to PID filter.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     Pid pid;
 *     Eagle eagle;
 *
 *     pid.value = 0x0000;
 *
 *     // Add PID to PID filter.
 *     error = IT9510_addPidToFilter ((IT9510INFO*) &eagle, 0, 1, pid);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 * </pre>
 */
uint32_t IT9510_addPidToFilter (
		IT9510INFO*    modulator,
		uint8_t            index,
		Pid             pid
		);


uint32_t IT9510_addPidToISDBTPidFilter (
		IT9510INFO*    modulator,
		uint8_t            index,
		Pid             pid,
		TransportLayer  layer
		);

/**
 * Access & send HW PSI table.
 *
 * @param modulator the handle of modulator.
 * @param pbuffer the data that will be write to HW PSI table.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t IT9510_sendHwPSITable (
		IT9510INFO*    modulator,
		uint8_t*            pbuffer
		);

/**
 * Access FW PSI table.
 *
 * @param modulator the handle of modulator.
 * @param the index of FW PSI table. The possible values are
 *        1~5.
 * @param pbuffer the data that will be write to HW PSI table.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t IT9510_accessFwPSITable (
		IT9510INFO*    modulator,
		uint8_t		  psiTableIndex,
		uint8_t*         pbuffer
		);

/**
 * set FW PSI table output timer.
 *
 * @param modulator the handle of modulator.
 * @param the index of FW PSI table. The possible values are
 *        1~5.
 * @param timer the timer unit is ms.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t IT9510_setFwPSITableTimer (
		IT9510INFO*    modulator,
		uint8_t		  psiTableIndex,
		uint16_t          timer_ms
		); 

/**
 * set Slave IIC Address.
 *
 * @param modulator the handle of modulator.
 * @param SlaveAddress Slave device IIC Addr.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t IT9510_setSlaveIICAddress (
		IT9510INFO*    modulator,
		uint8_t          SlaveAddress
		);

/**
 * run IT9517 Calibration.
 *
 * @param modulator the handle of modulator.
 * @param bandwidth channel bandwidth
 * @param frequency channel frequency
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t IT9510_runTxCalibration (
		IT9510INFO*    modulator,
		uint16_t            bandwidth,
		uint32_t           frequency
		);

/**
 * set IT9517 IQ value.
 *
 * @param modulator the handle of modulator.
 * @param bandwidth channel bandwidth
 * @param frequency channel frequency
 * @
 */
uint32_t IT9510_setIQValue(
		IT9510INFO*    modulator,
		int dAmp,
		int dPhi
		);

/**
 * adjust Output Gain.
 *
 * @param modulator the handle of modulator.
 * @param gain adjust output gain value.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t IT9510_adjustOutputGain (
		IT9510INFO*    modulator,
		int			  *gain	   
		); 

/**
 * get Output Gain.
 *
 * @param modulator the handle of modulator.
 * @param gain : output gain value.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t IT9510_getOutputGain (
		IT9510INFO*    modulator,
		int			  *gain	   
		);


/**
 * Get the Min/Max Gain range of the specified frequency and bandwidth to IT9500 TX.
 * @param modulator the handle of modulator.
 * @param frequency: Specify the frequency, in KHz.
 * @param bandwidth: Specify the bandwidth, in KHz.
 * @param pMaxGain:  The Maximum Gain/Attenuation can be set, in dB
 * @param pMinGain:  The Minumum Gain/Attenuation can be set, in dB 
 * @return:          ERROR_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t IT9510_getGainRange (
		IT9510INFO*    modulator,
		uint32_t           frequency,
		uint16_t            bandwidth,    
		int*			maxGain,
		int*			minGain
		);

/**
 * suspend Mode.
 *
 * @param modulator the handle of modulator.
 * @param enable 1:suspend mode/0: resume.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t IT9510_suspendMode (
		IT9510INFO*    modulator,
		uint8_t          enable
		);


/**
 * SET TPS settings
 *
 * @param modulator the handle of modulator.
 * @param TPS: Transmission Parameter Signalling 
 * @return: ERROR_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t IT9510_setTPS (
		IT9510INFO*    modulator,
		TPS           tps,
		Bool		  actualInfo
		);

/**
 * GET current TPS settings
 *
 * @param modulator the handle of modulator.
 * @param pTPS: Transmission Parameter Signalling 
 * @return: ERROR_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t IT9510_getTPS (
		IT9510INFO*    modulator,
		pTPS           pTps
		);

/**
 * set IQ fixed table point
 *
 * @param modulator the handle of modulator.
 * @param IQ_table_ptr: IQ fixed table point
 * @return: ERROR_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t IT9510_setIQtable (
		IT9510INFO*    modulator,
		CalibrationInfo calibrationInfo
		);

uint32_t IT9510_setDCtable (
		IT9510INFO*    modulator,
		DCInfo dcInfo
		);

/**
 * set dc Value
 *
 * @param modulator the handle of modulator.
 * @param dc_i & dc_q:dc_i & dc_q value
 * @return: ERROR_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t IT9510_setDCCalibrationValue (
		IT9510INFO*	modulator,
		int			dc_i,
		int			dc_q
		);

/**
 * get dc Value
 *
 * @param modulator the handle of modulator.
 * @param dc_i & dc_q: return dc_i & dc_q value
 * @return: ERROR_NO_ERROR: successful, non-zero error code otherwise.
 * 
 */
uint32_t IT9510_getDCCalibrationValue (
		IT9510INFO*	modulator,
		int*		dc_i,
		int*		dc_q
		);

uint32_t IT9510_setTMCCInfo (
		IT9510INFO*    modulator,
		TMCCINFO      TmccInfo,
		Bool		  actualInfo
		);
/*
   uint32_t IT9510_setTMCCInfo2 (
   IN  IT9510INFO*    modulator,
   IN  TMCCINFO      TmccInfo
   );*/

uint32_t IT9510_getTMCCInfo (
		IT9510INFO*    modulator,
		pTMCCINFO     pTmccInfo
		);


uint32_t IT9510_getTSinputBitRate (
		IT9510INFO*    modulator,
		uint16_t*     BitRate_Kbps
		);



uint32_t IT9510_setPcrMode (
		IT9510INFO*		modulator,
		PcrMode			mode
		);

uint32_t IT9510_setPcrModeEnable (
		IT9510INFO*		modulator,
		uint8_t			enable
		);

uint32_t IT9510_setNullPacketMode (
		IT9510INFO*		modulator,
		NullPacketMode	mode
		);
/**
 * check Ts Buffer Overflow
 *
 * @param modulator the handle of modulator.
 * @param overflow: return 1:overflow
 * @return: ERROR_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t IT9510_isTsBufferOverflow (
		IT9510INFO*	modulator,
		Bool		*overflow	
		);

/**
 * get chip type
 *
 * @param modulator the handle of modulator.
 * @param chipType: 9503 or 9507 
 * @return: ERROR_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t IT9510_getChipType (
		IT9510INFO*	modulator,
		uint16_t		*chipType	
		);

/**
 * set OFS Value
 *
 * @param modulator the handle of modulator.
 * @param ofs_i & ofs_q:ofs_i & ofs_q value
 * @return: ERROR_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t IT9510_setOFSCalibrationValue (
		IT9510INFO*	modulator,
		uint8_t		ofs_i,
		uint8_t		ofs_q
		);

/**
 * get OFS Value
 *
 * @param modulator the handle of modulator.
 * @param ofs_i & ofs_q: return ofs_i & ofs_q value
 * @return: ERROR_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t IT9510_getOFSCalibrationValue (
		IT9510INFO*	modulator,
		uint8_t*		ofs_i,
		uint8_t*		ofs_q
		);

/**
 * enable AES function
 *
 * @param modulator the handle of modulator.
 * @param enable: 1: on  0:off 
 * @return: ERROR_NO_ERROR: successful, non-zero error code otherwise.
 */

uint32_t IT9510_aesEncryptionEnable (
		IT9510INFO*	modulator,
		Bool		enable

		);

/**
 * set AES key
 *
 * @param modulator the handle of modulator.
 * @param key: 128 bits AES key
 * @return: ERROR_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t IT9510_setAesEncryptionKey (
		IT9510INFO*	modulator,
		uint8_t*		key

		);


/**
 * set Encryption Start Address
 *
 * @param modulator the handle of modulator.
 * @param addr[5:0]: 188 packet Encryption start address (0~43)
 * @return: ERROR_NO_ERROR: successful, non-zero error code otherwise.
 */

uint32_t IT9510_setEncryptionStartAddress (
		IT9510INFO*	modulator,
		uint8_t		addr

		);

/**
 * set Sine Tone output
 *
 * @param modulator the handle of modulator.
 * @param on_off: 0: off 1:on
 * @return: ERROR_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t IT9510_setSineTone (
		IT9510INFO*	modulator, 
		Bool		on_off

		);

/**
 * enable Tps Encryption function
 *
 * @param modulator the handle of modulator.
 * @param key: Tps Encryption key
 * @return: ERROR_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t IT9510_enableTpsEncryption (
		IT9510INFO*	modulator,
		uint32_t		key

		);

/**
 * disable Tps Encryption function
 *
 * @param modulator the handle of modulator.
 * @return: ERROR_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t IT9510_disableTpsEncryption (
		IT9510INFO*	modulator
		);


/**
 * write IT9560's EEPROM data
 *
 * @param modulator the handle of modulator.
 * @param slaveAddress: EEPROM Chip I2C Address(e.g :0xA8)
 * @param startAddress: register start address
 * @param buffer: data source
 * @param key: Tps Encryption key
 * @return: ERROR_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t IT9510_writeIT9560EEPROM(
		IT9510INFO*	modulator,
		uint8_t        slaveAddress,
		uint16_t        startAddress,
		unsigned char* buffer, 
		int writeSize
		);

/**
 * read IT9560's EEPROM data
 *
 * @param modulator the handle of modulator.
 * @param slaveAddress: EEPROM Chip I2C Address(e.g :0xA8)
 * @param startAddress: register start address
 * @param buffer: read data buffer
 * @param key: Tps Encryption key
 * @return: ERROR_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t IT9510_readIT9560EEPROM(
		IT9510INFO*	modulator,
		uint8_t        slaveAddress,
		uint16_t        startAddress,
		unsigned char* buffer, 
		int readSize
		);

/**
 * get IT9560 EEPROM Fw Version
 *
 * @param modulator the handle of modulator.
 * @param slaveAddress: EEPROM Chip I2C Address(e.g :0xA8)
 * @param version: IT9560 FW version in EEPROM(8 uint8_ts) 
 * @return: ERROR_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t IT9510_getIT9560FwVersion(
		IT9510INFO*	modulator,
		uint8_t        slaveAddress,
		uint8_t *version
		);





uint32_t IT9510_setPcrInfo(
		IT9510INFO*	modulator,
		PCRCALINFO	pcrCalInfo
		);

#endif
