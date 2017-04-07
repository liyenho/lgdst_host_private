#include "it9130.h"


uint32_t Demodulator_writeRegister (
		Demodulator*    demodulator,
      uint8_t            chip,
      Processor       processor,
      uint32_t           registerAddress,
      uint8_t            value
) {
    return (Standard_writeRegister (demodulator, chip, processor, registerAddress, value));
}


uint32_t Demodulator_writeRegisters (
		Demodulator*    demodulator,
      uint8_t            chip,
      Processor       processor,
      uint32_t           registerAddress,
      uint8_t            bufferLength,
      uint8_t*           buffer
) {
    return (Standard_writeRegisters (demodulator, chip, processor, registerAddress, bufferLength, buffer));
}

uint32_t Demodulator_writeRegisterBits (
		Demodulator*    demodulator,
      uint8_t            chip,
      Processor       processor,
      uint32_t           registerAddress,
      uint8_t            position,
      uint8_t            length,
      uint8_t            value
)
{
    return (Standard_writeRegisterBits (demodulator, chip, processor, registerAddress, position, length, value));
}


uint32_t Demodulator_readRegister (
		Demodulator*    demodulator,
      uint8_t            chip,
      Processor       processor,
      uint32_t           registerAddress,
      uint8_t*           value
) {
    return (Standard_readRegister (demodulator, chip, processor, registerAddress, value));
}


uint32_t Demodulator_readRegisters (
		Demodulator*    demodulator,
      uint8_t            chip,
      Processor       processor,
      uint32_t           registerAddress,
      uint8_t            bufferLength,
      uint8_t*           buffer
) {
    return (Standard_readRegisters (demodulator, chip, processor, registerAddress, bufferLength, buffer));
}

uint32_t Demodulator_readRegisterBits (
		Demodulator*    demodulator,
      uint8_t            chip,
      Processor       processor,
      uint32_t           registerAddress,
      uint8_t            position,
      uint8_t            length,
      uint8_t*           value
) {
    return (Standard_readRegisterBits (demodulator, chip, processor, registerAddress, position, length, value));
}

uint32_t Demodulator_getFirmwareVersion (
		Demodulator*    demodulator,
      Processor       processor,
      uint32_t*          version
) {
    return (Standard_getFirmwareVersion (demodulator, processor, version));
}


uint32_t Demodulator_getPostVitBer (
		Demodulator*    demodulator,
     uint8_t            chip,
     uint32_t*          postErrorCount,  /** 24 bits */
     uint32_t*          postBitCount,    /** 16 bits */
     uint16_t*           abortCount
){
	return (Standard_getPostVitBer(demodulator, chip, postErrorCount, postBitCount, abortCount));
}
uint32_t Demodulator_getRfAgcGain (
		Demodulator*    demodulator,
      uint8_t            chip,
      uint8_t*           rfAgc
) {
    return (Standard_getRfAgcGain (demodulator, chip, rfAgc));
}

uint32_t Demodulator_getIfAgcGain (
		Demodulator*    demodulator,
      uint8_t            chip,
      uint8_t*           ifAgc
) {
    return (Standard_getIfAgcGain (demodulator, chip, ifAgc));
}


uint32_t Demodulator_getSignalQuality (
		Demodulator*    demodulator,
      uint8_t            chip,
      uint8_t*           quality
) {
    return (Standard_getSignalQuality (demodulator, chip, quality));
}

uint32_t Demodulator_getSignalQualityIndication (
		Demodulator*    demodulator,
      uint8_t            chip,
      uint8_t*           quality
) {
    return (Standard_getSignalQualityIndication (demodulator, chip, quality));
}

uint32_t Demodulator_getSignalStrengthIndication (
		Demodulator*    demodulator,
      uint8_t            chip,
      uint8_t*           strength
) {
    return (Standard_getSignalStrengthIndication (demodulator, chip, strength));
}

uint32_t Demodulator_getSignalStrength (
		Demodulator*    demodulator,
      uint8_t            chip,
      uint8_t*           strength
) {
    return (Standard_getSignalStrength (demodulator, chip, strength));
}


uint32_t Demodulator_getSignalStrengthDbm (
		Demodulator*    demodulator,
      uint8_t            chip,
      Long*           strengthDbm           /** DBm                                */
) {
    return (Standard_getSignalStrengthDbm (demodulator, chip, strengthDbm));
}

uint32_t Demodulator_initialize (
		Demodulator*    demodulator,
      uint8_t            chipNumber,
      uint16_t            sawBandwidth,
      StreamType      streamType,
      Architecture    architecture
) {
    return (Standard_initialize (demodulator, chipNumber, sawBandwidth, streamType, architecture));
}

uint32_t Demodulator_finalize (
		Demodulator*    demodulator
) {
    return (Standard_finalize (demodulator));
}

uint32_t Demodulator_isTpsLocked (
		Demodulator*    demodulator,
      uint8_t            chip,
      Booll*           locked
) {
    return (Standard_isTpsLocked (demodulator, chip, locked));
}

uint32_t Demodulator_isMpeg2Locked (
		Demodulator*    demodulator,
      uint8_t            chip,
      Booll*           locked
) {
    return (Standard_isMpeg2Locked (demodulator, chip, locked));
}

uint32_t Demodulator_isLocked (
		Demodulator*    demodulator,
      uint8_t            chip,
      Booll*           locked
)
{
    return (Standard_isLocked (demodulator, chip, locked));
}

uint32_t Demodulator_reset (
		Demodulator*    demodulator
) {
    return (Standard_reset (demodulator));
}

uint32_t Demodulator_getChannelModulation (
		Demodulator*    demodulator,
      uint8_t                    chip,
      ChannelModulation*      channelModulation
) {
    return (Standard_getChannelModulation (demodulator, chip, channelModulation));
}

uint32_t Demodulator_acquireChannel (
		Demodulator*    demodulator,
      uint8_t            chip,
      uint16_t            bandwidth,
      uint32_t           frequency
) {
    return (Standard_acquireChannel (demodulator, chip, bandwidth, frequency));
}

uint32_t Demodulator_setStreamType (
		Demodulator*    demodulator,
      StreamType      streamType
) {
    return (Standard_setStreamType (demodulator, streamType));
}

uint32_t Demodulator_setArchitecture (
		Demodulator*    demodulator,
      Architecture    architecture
) {
    return (Standard_setArchitecture (demodulator, architecture));
}

uint32_t Demodulator_setViterbiRange (
		Demodulator*    demodulator,
      uint8_t            chip,
      uint8_t            superFrameCount,
      uint16_t            packetUnit
) {
    return (Standard_setViterbiRange (demodulator, chip, superFrameCount, packetUnit));
}

uint32_t Demodulator_getViterbiRange (
		Demodulator*    demodulator,
      uint8_t            chip,
      uint8_t*           superFrameCount,
      uint16_t*           packetUnit
) {
    return (Standard_getViterbiRange (demodulator, chip, superFrameCount, packetUnit));
}

uint32_t Demodulator_getStatistic (
		Demodulator*    demodulator,
      uint8_t            chip,
     Statistic*      statistic
) {
    return (Standard_getStatistic (demodulator, chip, statistic));
}

uint32_t Demodulator_reboot (
		Demodulator*    demodulator
)  {
    return (Standard_reboot (demodulator));
}

uint32_t Demodulator_controlPowerSaving (
		Demodulator*    demodulator,
      uint8_t            chip,
      uint8_t            control
) {
    return (Standard_controlPowerSaving (demodulator, chip, control));
}

uint32_t Demodulator_controlTunerLeakage (
		Demodulator*    demodulator,
      uint8_t            chip,
      uint8_t            control
) {
    return (Standard_controlTunerLeakage (demodulator, chip, control));
}

uint32_t Demodulator_controlTunerPowerSaving (
		Demodulator*    demodulator,
      uint8_t            chip,
	  uint8_t            control
) {
    return (Standard_controlTunerPowerSaving (demodulator, chip, control));
}

uint32_t Demodulator_controlPidFilter (
		Demodulator*    demodulator,
      uint8_t            chip,
      uint8_t            control
) {
    return (Standard_controlPidFilter (demodulator, chip, control));
}

uint32_t Demodulator_resetPidFilter (
		Demodulator*    demodulator,
    IN  uint8_t            chip
) {
    return (Standard_resetPidFilter (demodulator, chip));
}

uint32_t Demodulator_addPidToFilter (
		Demodulator*    demodulator,
      uint8_t            chip,
      uint8_t            index,
      Pid             pid
) {
    return (Standard_addPidToFilter (demodulator, chip, index, pid));
}

uint32_t Demodulator_getSNR (
		Demodulator*    demodulator,
	  uint8_t            chip,
     uint8_t*           snr
) {
    return (Standard_getSNR (demodulator, chip, snr));

}

uint32_t Demodulator_setMultiplier (
		Demodulator*    demodulator,
	  Multiplier		multiplier
) {
	return (Standard_setMultiplier (demodulator, multiplier));
}

uint32_t Demodulator_setStreamPriority (
		Demodulator*    demodulator,
	  uint8_t            chip,
	  Priority        priority
) {
    return (Standard_setStreamPriority (demodulator, chip, priority));

}


