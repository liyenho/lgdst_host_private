#ifndef __STANDARD_H__
#define __STANDARD_H__


#include "type.h"
#include "user.h"
#include "error.h"
#include "register.h"
#include "variable.h"
#include "version.h"

/**
 * Send the command to device.
 *
 * @param demodulator the handle of demodulator.
 * @param command the command to be send.
 * @param chip The index of demodulator. The possible values are
 *        0~7.
 * @param processor The processor of specified register. Because each chip
 *        has two processor so user have to specify the processor. The
 *        possible values are Processor_LINK and Processor_OFDM.
 * @param writeBufferLength the number of registers to be write.
 * @param writeBuffer a uint8_t array which is used to store values to be write.
 * @param readBufferLength the number of registers to be read.
 * @param readBuffer a uint8_t array which is used to store values to be read.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t Standard_sendCommand (
		Demodulator*    demodulator,
     uint16_t            command,
      uint8_t            chip,
      Processor       processor,
      uint32_t           writeBufferLength,
      uint8_t*           writeBuffer,
      uint32_t           readBufferLength,
     uint8_t*           readBuffer
);


/**
 * Program the bandwidth related parameters to demodulator.
 *
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7. NOTE: When the architecture is set to Architecture_DCA
 *        this parameter is regard as don't care.
 * @param bandwidth DVB channel bandwidth in MHz. The possible values
 *        are 5, 6, 7, and 8 (MHz).
 * @param adcFrequency The value of desire internal ADC frequency (Hz).
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t Standard_selectBandwidth (
		Demodulator*    demodulator,
      uint8_t            chip,
      uint16_t            bandwidth,          /** KHz                 */
      uint32_t           adcFrequency        /** Hz, ex: 20480000    */
);


/**
 * Set frequency.
 *
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7.
 * @param frequency The desired frequency.
 * @return Error_NO_ERROR: successful, other non-zero error code otherwise.
 */
uint32_t Standard_setFrequency (
		Demodulator*    demodulator,
      uint8_t            chip,
      uint32_t           frequency
);


/**
 * Load firmware to device
 *
 * @param demodulator the handle of demodulator.
 * @firmwareCodes pointer to fw binary.
 * @firmwareSegments pointer to fw segments.
 * @firmwarePartitions pointer to fw partition.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t Standard_loadFirmware (
		Demodulator*    demodulator,
      uint8_t*           firmwareCodes,
      Segment*        firmwareSegments,
      uint16_t*           firmwarePartitions
);


/**
 * Load initial script to device
 *
 * @param demodulator the handle of demodulator.
 * @streamType current stream type (useless for IT9130).
 * @scriptSets pointer to scriptSets.
 * @scripts pointer to fw scripts.
 * @tunerScriptSets pointer to tunerScriptSets.
 * @tunerScripts pointer to tunerScripts.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t Standard_loadScript (
		Demodulator*    demodulator,
      StreamType      streamType,
      uint16_t*           scriptSets,
      ValueSet*       scripts,
      uint16_t*           tunerScriptSets,
      ValueSet*       tunerScripts
);


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
 */
uint32_t Standard_writeRegister (
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
 */
uint32_t Standard_writeRegisters (
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
 */
uint32_t Standard_writeRegisterBits (
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
 */
uint32_t Standard_readRegister (
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
 */
uint32_t Standard_readRegisters (
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
 */
uint32_t Standard_readRegisterBits (
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
 */
uint32_t Standard_getFirmwareVersion (
		Demodulator*    demodulator,
      Processor       processor,
     uint32_t*          version
);


/**
 * Set the counting range for Post-Viterbi and Post-Viterbi.
 *
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7. NOTE: When the architecture is set to Architecture_DCA
 *        this parameter is regard as don't care.
 * @param postErrorCount the number of super frame for Pre-Viterbi.
 * @param postBitCount the number of packet unit for Post-Viterbi.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t Standard_getPostVitBer (
		Demodulator*    demodulator,
      uint8_t            chip,
     uint32_t*          postErrorCount, /** 24 bits */
     uint32_t*          postBitCount,   /** 16 bits */
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
 */
uint32_t Standard_getRfAgcGain (
		Demodulator*    demodulator,
      uint8_t            chip,
     uint8_t*           rfAgc
);


/**
 * Get IF AGC gain.
 *
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7.
 * @param ifAgc the value of IF AGC.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t Standard_getIfAgcGain (
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
 */
uint32_t Standard_getSignalQuality (
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
 */

uint32_t Standard_getSignalQualityIndication (
		Demodulator*    demodulator,
      uint8_t            chip,
     uint8_t*           quality
);

/**
 * Get siganl strength Indication.
 *
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7. NOTE: When the architecture is set to Architecture_DCA
 *        this parameter is regard as don't care.
 * @param strength The value of signal strength that calculations of "score mapping" from the signal strength (dBm) to the "0-100" scoring.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t Standard_getSignalStrengthIndication (
		Demodulator*    demodulator,
    uint8_t            chip,
     uint8_t*           strength
);
/**
 * Get siganl strength.
 *
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7. NOTE: When the architecture is set to Architecture_DCA
 *        this parameter is regard as don't care.
 * @param strength The value of signal strength.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t Standard_getSignalStrength (
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
 */
uint32_t Standard_getSignalStrengthDbm (
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
 * @param sawBandwidth SAW filter bandwidth in MHz. The possible values
 *        are 6000, 7000, and 8000 (KHz).
 * @param streamType The format of output stream.
 * @param architecture the architecture of demodulator.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t Standard_initialize (
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
 */
uint32_t Standard_finalize (
		Demodulator*    demodulator
);

/**
 *
 * @param demodulator the handle of demodulator.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 * </pre>
 */
uint32_t Standard_isTpsLocked (
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
uint32_t Standard_isMpeg2Locked (
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
 * @example <pre>
 * </pre>
 */
uint32_t Standard_isLocked (
		Demodulator*    demodulator,
      uint8_t            chip,
     Booll*           locked
);

/**
 * Reset demodulator.
 *
 * @param demodulator the handle of demodulator.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t Standard_reset (
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
 */
uint32_t Standard_getChannelModulation (
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
 */
uint32_t Standard_acquireChannel (
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
 * Note: Please refer to the example of Standard_acquireChannel when host want
 *       to detect the available channels.
 * Note: After host know all the available channels, and want to change to
 *       specific channel, host have to choose output mode before receive
 *       data. Please refer the example of Standard_setStreamType.
 *
 * @param demodulator the handle of demodulator.
 * @param streamType the possible values are
 *        DVB-H:    StreamType_DVBH_DATAGRAM
 *                  StreamType_DVBH_DATABURST
 *        DVB-T:    StreamType_DVBT_DATAGRAM
 *                  StreamType_DVBT_PARALLEL
 *                  StreamType_DVBT_SERIAL
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t Standard_setStreamType (
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
 */
uint32_t Standard_setArchitecture (
		Demodulator*    demodulator,
      Architecture    architecture
);


/**
 * Set the counting range for Pre-Viterbi and Post-Viterbi.
 *
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7. NOTE: When the architecture is set to Architecture_DCA
 *        this parameter is regard as don't care.
 * @param frameCount the number of super frame for Pre-Viterbi.
 * @param packetUnit the number of packet unit for Post-Viterbi.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t Standard_setViterbiRange (
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
 */
uint32_t Standard_getViterbiRange (
		Demodulator*    demodulator,
      uint8_t            chip,
      uint8_t*           frameCount,
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
 */
uint32_t Standard_getStatistic (
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
uint32_t Standard_reboot (
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
uint32_t Standard_controlPowerSaving (
		Demodulator*    demodulator,
      uint8_t            chip,
      uint8_t            control
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
uint32_t Standard_controlTunerLeakage (
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
uint32_t Standard_controlTunerPowerSaving (
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
uint32_t Standard_controlPidFilter (
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
 */
uint32_t Standard_resetPidFilter (
		Demodulator*    demodulator,
      uint8_t            chip
);


/**
 * Add PID to PID filter.
 *
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7.
 * @param pid the PID that will be add to PID filter.
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t Standard_addPidToFilter (
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
uint32_t Standard_getSNR (
		Demodulator*    demodulator,
	  uint8_t            chip,
      uint8_t*           snr
);
/**
 * get SNR data .
 *
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7.
 * @param SNR data(hex).
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t Standard_getSNRValue(
		Demodulator*    demodulator,
	    uint8_t            chip,
       uint32_t*        snr_value
);

/**
 *
 * @param demodulator the handle of demodulator.
 * @param multiplier ADC frequency multiplier;
 * @return Error_NO_ERROR: successful, other non-zero error code otherwise.
 * @example <pre>
 * </pre>
 */
uint32_t Standard_setMultiplier (
		Demodulator*    demodulator,
      Multiplier      multiplier
) ;

/**
 * Set StreamPriority
 *
 * @param demodulator the handle of demodulator.
 * @param chip The index of demodulator. The possible values are
 *        0~7.
 * @param  StreamPriority
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 */
uint32_t Standard_setStreamPriority (
		Demodulator*    demodulator,
	  uint8_t          chip,
	  Priority      priority
);

uint32_t Standard_initDetectTable (
		Demodulator*    demodulator,
	  uint8_t            chip
);

uint32_t Standard_initDecryptTable (
		Demodulator*    demodulator,
	  uint8_t            chip
);

/**
* set DCA Latch Time for DCA or DCAEx moce.
*
* @param demodulator the handle of demodulator.

* @param lowerValue lowerValue for chip0.
* @param upperValue upperValue for chip1.
* @return Error_NO_ERROR: successful, non-zero error code otherwise.

*/
uint32_t Standard_setDcaLatchTime(
		Demodulator*    demodulator,
	  uint8_t            masterValue,
	  uint8_t            slaveValue
);

uint32_t Standard_acquireChannelEx(
		Demodulator*    demodulator,
		uint16_t			bandwidth,
	    uint32_t           masterFreq,
	   uint32_t           slaveFreq
);

#endif