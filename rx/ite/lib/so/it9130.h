#ifndef __DEMODULATOR_H__
#define __DEMODULATOR_H__


#include "type.h"
#include "user.h"
#include "error.h"
#include "register.h"
#include "variable.h"
#include "cmd.h"
#include "standard.h"
#include "version.h"

/**
 * Write one uint8_t (8 bits) to a specific register in demodulator.
 *
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7.
 * @param processor The processor of specified register. Because each chip
 *        has two processor so user have to specify the processor. The
 *        possible values are Processor_LINK and Processor_OFDM.
 * @param registerAddress the address of the register to be written.
 * @param value the value to be written.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     IT9130 it9130;
 *
 *     // Set the value of register 0xA000 in demodulator to 0.
 *     error = Demodulator_writeRegister ((Demodulator*) &it9130, 0, Processor_LINK, 0xA000, 0);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 * </pre>
 */
uint32_t Demodulator_writeRegister (
		Demodulator*    demodulator,
      uint8_t            chip,
      Processor       processor,
      uint32_t           registerAddress,
      uint8_t            value
);


/**
 * Write a sequence of uint8_ts to the contiguous registers in demodulator.
 * The maximum burst size is restricted by the capacity of bus. If bus
 * could transfer N uint8_ts in one cycle, then the maximum value of
 * bufferLength would be N - 5.
 *
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7.
 * @param processor The processor of specified register. Because each chip
 *        has two processor so user have to specify the processor. The
 *        possible values are Processor_LINK and Processor_OFDM.
 * @param registerAddress the start address of the registers to be written.
 * @param bufferLength the number of registers to be written.
 * @param buffer a uint8_t array which is used to store values to be written.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     uint8_t buffer[3] = { 0x00, 0x01, 0x02 };
 *     IT9130 it9130;
 *
 *     // Set the value of register 0xA000 in demodulator to 0.
 *     // Set the value of register 0xA001 in demodulator to 1.
 *     // Set the value of register 0xA002 in demodulator to 2.
 *     error = Demodulator_writeRegisters ((Demodulator*) &it9130, 0, Processor_LINK, 0xA000, 3, buffer);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 * </pre>
 */
uint32_t Demodulator_writeRegisters (
		Demodulator*    demodulator,
      uint8_t            chip,
      Processor       processor,
      uint32_t           registerAddress,
      uint8_t            bufferLength,
      uint8_t*           buffer
);

/**
 * Modify bits in the specific register.
 *
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7.
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
 *     IT9130 it9130;
 *
 *     // Modify the LSB of register 0xA000 in demodulator to 0.
 *     error = Demodulator_writeRegisterBits ((Demodulator*) &it9130, 0, Processor_LINK, 0xA000, 0, 1, 0);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 * </pre>
 */
uint32_t Demodulator_writeRegisterBits (
		Demodulator*    demodulator,
      uint8_t            chip,
      Processor       processor,
      uint32_t           registerAddress,
      uint8_t            position,
      uint8_t            length,
      uint8_t            value
);


/**
 * Read one uint8_t (8 bits) from a specific register in demodulator.
 *
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7.
 * @param processor The processor of specified register. Because each chip
 *        has two processor so user have to specify the processor. The
 *        possible values are Processor_LINK and Processor_OFDM.
 * @param registerAddress the address of the register to be read.
 * @param value the pointer used to store the value read from demodulator
 *        register.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     uint8_t value;
 *     IT9130 it9130;
 *
 *     // Get the value of register 0xA000 in demodulator.
 *     error = Demodulator_readRegister ((Demodulator*) &it9130, 0, Processor_LINK, 0xA000, &value);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 *     printf ("The value of 0xA000 is %2x", value);
 * </pre>
 */
uint32_t Demodulator_readRegister (
		Demodulator*    demodulator,
      uint8_t            chip,
      Processor       processor,
      uint32_t           registerAddress,
      uint8_t*           value
);


/**
 * Read a sequence of uint8_ts from the contiguous registers in demodulator.
 * The maximum burst size is restricted by the capacity of bus. If bus
 * could transfer N uint8_ts in one cycle, then the maximum value of
 * bufferLength would be N - 5.
 *
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7.
 * @param processor The processor of specified register. Because each chip
 *        has two processor so user have to specify the processor. The
 *        possible values are Processor_LINK and Processor_OFDM.
 * @param registerAddress the address of the register to be read.
 * @param bufferLength the number of registers to be read.
 * @param buffer a uint8_t array which is used to store values to be read.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     uint8_t buffer[3];
 *     IT9130 it9130;
 *
 *     // Get the value of register 0xA000, 0xA001, 0xA002 in demodulator.
 *     error = Demodulator_readRegisters ((Demodulator*) &it9130, 0, Processor_LINK, 0xA000, 3, buffer);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 *     printf ("The value of 0xA000 is %2x", buffer[0]);
 *     printf ("The value of 0xA001 is %2x", buffer[1]);
 *     printf ("The value of 0xA002 is %2x", buffer[2]);
 * </pre>
 */
uint32_t Demodulator_readRegisters (
		Demodulator*    demodulator,
      uint8_t            chip,
      Processor       processor,
      uint32_t           registerAddress,
      uint8_t            bufferLength,
      uint8_t*           buffer
);


/**
 * Read bits of the specified register.
 *
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7.
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
 *     IT9130 it9130;
 *
 *     // Read the LSB of register 0xA000 in demodulator.
 *     error = Demodulator_readRegisterBits ((Demodulator*) &it9130, 0, Processor_LINK, 0xA000, 0, 1, &value);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 *     printf ("The value of LSB of 0xA000 is %2x", value);
 * </pre>
 */
uint32_t Demodulator_readRegisterBits (
		Demodulator*    demodulator,
      uint8_t            chip,
      Processor       processor,
      uint32_t           registerAddress,
      uint8_t            position,
      uint8_t            length,
      uint8_t*           value
);

/**
 * Get the version of firmware.
 *
 * @param demodulator the handle of demodulator.
 * @param version the version of firmware.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     uint32_t version;
 *     IT9130 it9130;
 *
 *     // Get the version of Link layer firmware.
 *     error = Demodulator_getFirmwareVersion ((Demodulator*) &it9130, Processor_LINK, &version);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("The version of firmware is : %X", version);
 * </pre>
 */
uint32_t Demodulator_getFirmwareVersion (
		Demodulator*    demodulator,
      Processor       processor,
      uint32_t*          version
);


/**
 * Get post VitBer
 *
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7.
 * @param postErrorCount error count after viterbi
 * @param postBitCount total count after viterbi
 * @param abortCount error count after reed-soloman
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 * </pre>
 */
uint32_t Demodulator_getPostVitBer (
		Demodulator*    demodulator,
      uint8_t            chip,
     uint32_t*          postErrorCount,  /** 24 bits */
     uint32_t*          postBitCount,    /** 16 bits */
     uint16_t*           abortCount
);

/**
 * Get RF AGC gain.
 *
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7.
 * @param rfAgc the value of RF AGC.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     uint8_t rfAgc;
 *     IT9130 it9130;
 *
 *     // Set I2C as the control bus.
 *     error = Demodulator_getRfAgcGain ((Demodulator*) &it9130, 0, rfAgc);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 * </pre>
 */
uint32_t Demodulator_getRfAgcGain (
		Demodulator*    demodulator,
      uint8_t            chip,
     uint8_t*           rfAgc
);


/**
 * Get IF AGC.
 *
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7.
 * @param ifAgc the value of IF AGC.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     uint8_t ifAgc;
 *     IT9130 it9130;
 *
 *     // Set I2C as the control bus.
 *     error = Demodulator_getIfAgcGain ((Demodulator*) &it9130, 0, ifAgc);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 * </pre>
 */
uint32_t Demodulator_getIfAgcGain (
		Demodulator*    demodulator,
      uint8_t            chip,
     uint8_t*           ifAgc
);


/**
 * Get siganl quality.
 *
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7. NOTE: When the architecture is set to Architecture_DCA
 *        this parameter is regard as don't care.
 * @param quality The value of signal quality.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 * </pre>
 */
uint32_t Demodulator_getSignalQuality (
		Demodulator*    demodulator,
      uint8_t            chip,
     uint8_t*           quality
);

/**
 * Get siganl quality Indication.
 *
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7. NOTE: When the architecture is set to Architecture_DCA
 *        this parameter is regard as don't care.
 * @param quality The value of signal quality.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 * </pre>
 */
uint32_t Demodulator_getSignalQualityIndication (
		Demodulator*    demodulator,
      uint8_t            chip,
     uint8_t*           quality
);

/**
 * Get signal strength
 *
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7.
 * @param strength The value of signal strength that calculations of "score mapping" from the signal strength (dBm) to the "0-100" scoring.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 * </pre>
 */
uint32_t Demodulator_getSignalStrengthIndication (
		Demodulator*    demodulator,
      uint8_t            chip,
      uint8_t*           strength
);

/**
 * Get signal strength
 *
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7.
 * @param strength The value of signal strength.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 * </pre>
 */
uint32_t Demodulator_getSignalStrength (
		Demodulator*    demodulator,
     uint8_t            chip,
     uint8_t*           strength
);


/**
 * Get signal strength in dbm
 *
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7.
 * @param rfpullUpVolt_X10 the pullup voltag of RF multiply 10.
 * @param ifpullUpVolt_X10 the pullup voltag of IF multiply 10.
 * @param strengthDbm The value of signal strength in DBm.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 * </pre>
 */
uint32_t Demodulator_getSignalStrengthDbm (
		Demodulator*    demodulator,
     uint8_t            chip,
     Long*           strengthDbm           /** DBm                                   */
);



/**
 * First, download firmware from host to demodulator. Actually, firmware is
 * put in firmware.h as a part of source code. Therefore, in order to
 * update firmware the host have to re-compile the source code.
 * Second, setting all parameters which will be need at the beginning.
 *
 * @param demodulator the handle of demodulator.
 * @param chipNumber The total number of demodulators.
 * @param sawBandwidth SAW filter bandwidth in KHz. The possible values
 *        are 6000, 7000, and 8000 (KHz).
 * @param streamType The format of output stream.
 * @param architecture the architecture of demodulator.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     IT9130 it9130;
 *
 *     // Initialize demodulators.
 *     // SAW Filter  : 8MHz
 *     // Stream Type : IP Datagram.
 *     error = Demodulator_initialize ((Demodulator*) &it9130, 1, 8, StreamType_IP_DATAGRAM, Architecture_DCA);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 * </pre>
 */
uint32_t Demodulator_initialize (
		Demodulator*    demodulator,
      uint8_t            chipNumber,
      uint16_t            sawBandwidth,
      StreamType      streamType,
      Architecture    architecture
);


/**
 * Power off the demodulators.
 *
 * @param demodulator the handle of demodulator.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     IT9130 it9130;
 *
 *     // Finalize demodulators.
 *     error = Demodulator_finalize ((Demodulator*) &it9130);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 * </pre>
 */
uint32_t Demodulator_finalize (
		Demodulator*    demodulator
);

/**
 *
 * @param demodulator the handle of demodulator.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 * </pre>
 */
uint32_t Demodulator_isTpsLocked (
		Demodulator*    demodulator,
      uint8_t            chip,
     Booll*           locked
);


/**
 *
 * @param demodulator the handle of demodulator.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 * </pre>
 */
uint32_t Demodulator_isMpeg2Locked (
		Demodulator*    demodulator,
      uint8_t            chip,
     Booll*           locked
);


/**
 *
 * @param demodulator the handle of demodulator.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @param chip The index of demodulator. The possible values are
 *        0~7. NOTE: When the architecture is set to Architecture_DCA
 *        this parameter is regard as don't care.
 * @param locked the result of frequency tuning. True if there is
 *        demodulator can lock signal, False otherwise.
 * @see Demodulator_acquireChannel
 */
uint32_t Demodulator_isLocked (
		Demodulator*    demodulator,
      uint8_t            chip,
      Booll*           locked
);


/**
 * Reset demodulator.
 *
 * @param demodulator the handle of demodulator.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     IT9130 it9130;
 *
 *     // Reset demodulator.
 *     error = Demodulator_reset ((Demodulator*) &it9130);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 * </pre>
 */
uint32_t Demodulator_reset (
		Demodulator*    demodulator
);


/**
 * Get channel modulation related information.
 *
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7.
 * @param channelModulation The modulation of channel.
 * @return Error_NO_ERROR: successful, other non-zero error code otherwise.
 * @example <pre>
 * </pre>
 */
uint32_t Demodulator_getChannelModulation (
		Demodulator*    demodulator,
      uint8_t                    chip,
     ChannelModulation*      channelModulation
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
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7. NOTE: When the architecture is set to Architecture_DCA
 *        this parameter is regard as don't care.
 * @param bandwidth The channel bandwidth.
 *        DVB-T: 5000, 6000, 7000, and 8000 (KHz).
 *        DVB-H: 5000, 6000, 7000, and 8000 (KHz).
 *        T-DMB: 5000, 6000, 7000, and 8000 (KHz).
 *        FM: 100, and 200 (KHz).
 * @param frequency the channel frequency in KHz.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     Bool locked;
 *     IT9130 it9130;
 *
 *     error = Demodulator_acquireChannel ((Demodulator*) &it9130, 0, 8000, 666000);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 *
 *     error = Demodulator_isLocked ((Demodulator*) &it9130, 0, &locked);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 *
 *     if (locked == True) {
 *         // In DVB-T mode.
 *         // Start to process TS
 *         // Because DVB-T could be multiplex with DVB-H
 *     }
 * </pre>
 */
uint32_t Demodulator_acquireChannel (
		Demodulator*    demodulator,
      uint8_t            chip,
      uint16_t            bandwidth,
      uint32_t           frequency
);


/**
 * Set the output stream type of chip. Because the device could output in
 * many stream type, therefore host have to choose one type before receive
 * data.
 *
 * Note: After host know all the available channels, and want to change to
 *       specific channel, host have to choose output mode before receive
 *       data. Please refer the example of Demodulator_setStreamType.
 *
 * @param demodulator the handle of demodulator.
 * @param streamType the possible values are
 *        DVB-H:    StreamType_DVBH_DATAGRAM
 *                  StreamType_DVBH_DATABURST
 *        DVB-T:    StreamType_DVBT_DATAGRAM
 *                  StreamType_DVBT_PARALLEL
 *                  StreamType_DVBT_SERIAL
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     IT9130 it9130;
 *
 *     error = Demodulator_setStreamType ((Demodulator*) &it9130, StreamType_DVBT_PARALLEL)
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 * </pre>
 */
uint32_t Demodulator_setStreamType (
		Demodulator*    demodulator,
      StreamType      streamType
);


/**
 * Set the architecture of chip. When two of our device are using, they could
 * be operated in Diversity Combine Architecture (DCA) or (PIP). Therefore,
 * host could decide which mode to be operated.
 *
 * @param demodulator the handle of demodulator.
 * @param architecture the possible values are
 *        Architecture_DCA
 *        Architecture_PIP
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     IT9130 it9130;
 *
 *     // Set architecture.
 *     error = Demodulator_setArchitecture ((Demodulator*) &it9130, Architecture_DCA)
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 * </pre>
 */
uint32_t Demodulator_setArchitecture (
		Demodulator*    demodulator,
      Architecture    architecture
);


/**
 * Set the counting range for Pre-Viterbi and Post-Viterbi.
 *
 * @param demodulator the handle of demodulator.
 * @param frameCount the number of super frame for Pre-Viterbi.
 * @param packetUnit the number of packet unit for Post-Viterbi.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     IT9130 it9130;
 *
 *     // Set Viterbi range.
 *     error = Demodulator_setViterbiRange ((Demodulator*) &it9130, 0, 1, 10000);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 * </pre>
 */
uint32_t Demodulator_setViterbiRange (
		Demodulator*    demodulator,
      uint8_t            chip,
      uint8_t            superFrameCount,
      uint16_t            packetUnit
);



/**
 * Get the counting range for Pre-Viterbi and Post-Viterbi.
 *
 * @param demodulator the handle of demodulator.
 * @param frameCount the number of super frame for Pre-Viterbi.
 * @param packetUnit the number of packet unit for Post-Viterbi.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     uint8_t superFrameCount;
 *     uint16_t packetUnit;
 *     IT9130 it9130;
 *
 *     // Set Viterbi range.
 *     error = Demodulator_getViterbiRange ((Demodulator*) &it9130, 0, &superFrameCount, &packetUnit);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 * </pre>
 */
uint32_t Demodulator_getViterbiRange (
		Demodulator*    demodulator,
      uint8_t            chip,
      uint8_t*           superFrameCount,
      uint16_t*           packetUnit
);


/**
 * Get the statistic values of demodulator, it includes Pre-Viterbi BER,
 * Post-Viterbi BER, Abort Count, Signal Presented Flag, Signal Locked Flag,
 * Signal Quality, Signal Strength, Delta-T for DVB-H time slicing.
 *
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7.
 * @param statistic the structure that store all statistic values.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     Statistic statistic;
 *     double preBer;
 *     double postBer;
 *     IT9130 it9130;
 *
 *     // Set statistic range.
 *     error = Demodulator_getStatistic ((Demodulator*) &it9130, 0, &statistic);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 *     preBer = (double) statistic.preVitErrorCount / (double) statistic.preVitBitCount;
 *     printf ("Pre-Viterbi BER = %f\n", preBer);
 *     postBer = (double) statistic.postVitErrorCount / (double) statistic.postVitBitCount;
 *     printf ("Post-Viterbi BER = %f\n", postBer);
 *     printf ("Abort Count = %d\n", statistic.abortCount);
 *     if (statistic.signalPresented == True)
 *         printf ("Signal Presented = True\n");
 *     else
 *         printf ("Signal Presented = False\n");
 *     if (statistic.signalLocked == True)
 *         printf ("Signal Locked = True\n");
 *     else
 *         printf ("Signal Locked = False\n");
 *     printf ("Signal Quality = %d\n", statistic.signalQuality);
 *     printf ("Signal Strength = %d\n", statistic.signalStrength);
 * </pre>
 */
uint32_t Demodulator_getStatistic (
		Demodulator*    demodulator,
      uint8_t            chip,
     Statistic*      statistic
);

/**
 * Return to boot code
 *
 * @param demodulator the handle of demodulator.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 * </pre>
 */
uint32_t Demodulator_reboot (
		Demodulator*    demodulator
);


/**
 *
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7.
 * @param contorl 1: Power up, 0: Power down;
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 * </pre>
 */
uint32_t Demodulator_controlPowerSaving (
		Demodulator*    demodulator,
      uint8_t            chip,
      uint8_t            control
);

uint32_t Demodulator_controlTunerLeakage (
      Demodulator*    demodulator,
      uint8_t            chip,
      uint8_t            control
);

/**
 *
 * @param demodulator the handle of demodulator.
 * @param contorl 1: Power up, 0: Power down;
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 * </pre>
 */
uint32_t Demodulator_controlTunerPowerSaving (
		Demodulator*    demodulator,
      uint8_t            chip,
	  uint8_t            control
);



/**
 * Control PID fileter
 *
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7.
 * @param contorl 0: Disable, 1: Enable.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 * </pre>
 */
uint32_t Demodulator_controlPidFilter (
		Demodulator*    demodulator,
      uint8_t            chip,
      uint8_t            control
);


/**
 * Reset PID filter.
 *
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     IT9130 it9130;
 *
 *     error = Demodulator_resetPidFilter ((Demodulator*) &it9130, 0);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 * </pre>
 */
uint32_t Demodulator_resetPidFilter (
		Demodulator*    demodulator,
      uint8_t            chip
);


/**
 * Add PID to PID filter.
 *
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7.
 * @param index the index of PID filter.
 * @param pid the PID that will be add to PID filter.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     Pid pid;
 *     IT9130 it9130;
 *
 *     pid.value = 0x0000;
 *
 *     // Add PID to PID filter.
 *     error = Demodulator_addPidToFilter ((Demodulator*) &it9130, 0, 1, pid);
 *     if (error)
 *         printf ("Error Code = %X", error);
 *     else
 *         printf ("Success");
 * </pre>
 */
uint32_t Demodulator_addPidToFilter (
		Demodulator*    demodulator,
      uint8_t            chip,
      uint8_t            index,
      Pid             pid
);


/**
 * get SNR .
 *
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7.
 * @param SNR (db).
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t Demodulator_getSNR (
		Demodulator*    demodulator,
	  uint8_t            chip,
     uint8_t*           snr
) ;


/**
 *
 * @param demodulator the handle of demodulator.
 * @param multiplier ADC frequency multiplier;
 * @return Error_NO_ERROR: successful, other non-zero error code otherwise.
 * @example <pre>
 * </pre>
 */
uint32_t Demodulator_setMultiplier (
		Demodulator*    demodulator,
	  Multiplier		multiplier
);

/**
 * Set StreamPriority
 *
 * @param demodulator the handle of demodulator.
 * @param  StreamPriority
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 */

uint32_t Demodulator_setStreamPriority (
		Demodulator*    demodulator,
	  uint8_t          chip,
	  Priority      priority
);

#endif