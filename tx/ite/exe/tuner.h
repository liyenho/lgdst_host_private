#ifndef __IT9510_TUNER_H__
#define __IT9510_TUNER_H__


uint32_t IT9510Tuner_setIQCalibration(
		IT9510INFO*    modulator,
		uint32_t         frequency	
		);

uint32_t IT9510Tuner_setIQCalibrationEx(
		IT9510INFO*    modulator,
		int dAmp,
		int dPhi
		); 

uint32_t IT9510Tuner_calIQCalibrationValue(
		IT9510INFO*    modulator,
		uint32_t         frequency,
		uint8_t*		  val
		);


#endif