/**
 * @(#)Afatech_OMEGA.cpp
 *
 * ==========================================================
 * Version: 2.0
 * Date:    2009.06.15
 * ==========================================================
 *
 * ==========================================================
 * History:
 *
 * Date         Author      Description
 * ----------------------------------------------------------
 *
 * 2009.06.15   M.-C. Ho    new tuner
 * ==========================================================
 *
 * Copyright 2009 Afatech, Inc. All rights reserved.
 *
 */


#include <stdio.h>
#include "type.h"
#include "error.h"
#include "user.h"
#include "register.h"
#include "standard.h"
#include "omega.h"
#include "Firmware_Afa_Omega_Script.h"
#include "Firmware_Afa_Omega_LNA_Config_1_Script.h"
#include "Firmware_Afa_Omega_LNA_Config_2_Script.h"
#include "Firmware_Afa_Omega_Script_V2.h"
#include "Firmware_Afa_Omega_LNA_Config_1_Script_V2.h"
#include "Firmware_Afa_Omega_LNA_Config_2_Script_V2.h"
#include "Firmware_Afa_Omega_LNA_Config_3_Script_V2.h"
#include "Firmware_Afa_Omega_LNA_Config_4_Script_V2I.h"
#include "Firmware_Afa_Omega_LNA_Config_5_Script_V2I.h"
#include "Firmware_Afa_Omega_LNA_Config_3_Script_V2W.h"



uint32_t OMEGA_open (
Demodulator*    demodulator,
	  uint8_t			chip
) {
	uint32_t error = Error_NO_ERROR;
  puts("calling omega_init");
	error = omega_init(demodulator, chip);
  puts("omega_init called");
	return (error);
}

uint32_t OMEGA_close (
	  uint8_t			chip
) {
	return (Error_NO_ERROR);
}

uint32_t OMEGA_set (
Demodulator*    demodulator,
	  uint8_t			chip,
	  uint16_t			bandwidth,
	  uint32_t			frequency
) {
	uint32_t error = Error_NO_ERROR;
  puts("omega tuner set frequency");
	error = omega_setfreq(demodulator, chip, (unsigned int)bandwidth, (unsigned int)frequency);
  puts("done, omega tuner set frequency");
	return (error);
}


TunerDescription tunerDescription= {
    OMEGA_open,
    OMEGA_close,
    OMEGA_set,
    NULL,
    NULL,
    OMEGA_ADDRESS,			/** tuner i2c address */
    2,						/** length of tuner register address */
    0,						/** tuner if */
    False,					/** spectrum inverse */
    0x38,					/** tuner id */
};

uint32_t OMEGA_supportLNA (
Demodulator*    demodulator,
      uint8_t            supporttype
 ) {
     uint32_t error = Error_INVALID_DEV_TYPE;

     uint8_t chip_version = 0;
	 uint32_t chip_Type;
	 uint8_t var[2];

	error = Standard_readRegister(demodulator, 0, Processor_LINK, chip_version_7_0, &chip_version);
	if (error) goto exit;
	error = Standard_readRegisters(demodulator, 0, Processor_LINK, chip_version_7_0+1, 2, var);
	if (error) goto exit;
	chip_Type = var[1]<<8 | var[0];
   printf("chip_Type=%x,chip_version=%x\n",chip_Type,chip_version);

	if(chip_Type==0x9135 && chip_version == 2){
		switch (supporttype){
			case OMEGA_NORMAL:
				tunerDescription.tunerScript = V2_OMEGA_scripts;
				tunerDescription.tunerScriptSets = V2_OMEGA_scriptSets;
                tunerDescription.tunerId = 0x60;
				error = Error_NO_ERROR;
				break;
			case OMEGA_LNA_Config_1:
				tunerDescription.tunerScript = V2_OMEGA_LNA_Config_1_scripts;
				tunerDescription.tunerScriptSets = V2_OMEGA_LNA_Config_1_scriptSets;
				tunerDescription.tunerId = 0x61;
				error = Error_NO_ERROR;
				break;
			case OMEGA_LNA_Config_2:
				tunerDescription.tunerScript = V2_OMEGA_LNA_Config_2_scripts;
				tunerDescription.tunerScriptSets = V2_OMEGA_LNA_Config_2_scriptSets;
				tunerDescription.tunerId = 0x62;
				error = Error_NO_ERROR;
				break;
			case OMEGA_LNA_Config_3:
				tunerDescription.tunerScript = V2W_OMEGA_LNA_Config_3_scripts;
				tunerDescription.tunerScriptSets = V2W_OMEGA_LNA_Config_3_scriptSets;
				tunerDescription.tunerId = 0x63;
				error = Error_NO_ERROR;
				break;
			case OMEGA_LNA_Config_4:
				tunerDescription.tunerScript = V2I_OMEGA_LNA_Config_4_scripts;
				tunerDescription.tunerScriptSets = V2I_OMEGA_LNA_Config_4_scriptSets;
				tunerDescription.tunerId = 0x64;
				error = Error_NO_ERROR;
				break;
			case OMEGA_LNA_Config_5:
				tunerDescription.tunerScript = V2I_OMEGA_LNA_Config_5_scripts;
				tunerDescription.tunerScriptSets = V2I_OMEGA_LNA_Config_5_scriptSets;
				tunerDescription.tunerId = 0x65;
				error = Error_NO_ERROR;
				break;
			default:
				tunerDescription.tunerScript = V2_OMEGA_scripts;
				tunerDescription.tunerScriptSets = V2_OMEGA_scriptSets;
                tunerDescription.tunerId = 0x60;
				error = Error_NO_ERROR;
		        break;
	   }
	}else{
		switch (supporttype){
			case OMEGA_NORMAL:
				tunerDescription.tunerScript = OMEGA_scripts;
				tunerDescription.tunerScriptSets = OMEGA_scriptSets;
				tunerDescription.tunerId = 0x38;
				error = Error_NO_ERROR;
				break;
			case OMEGA_LNA_Config_1:
				tunerDescription.tunerScript = OMEGA_LNA_Config_1_scripts;
				tunerDescription.tunerScriptSets = OMEGA_LNA_Config_1_scriptSets;
				tunerDescription.tunerId = 0x51;
				error = Error_NO_ERROR;
				break;
			case OMEGA_LNA_Config_2:
				tunerDescription.tunerScript = OMEGA_LNA_Config_2_scripts;
				tunerDescription.tunerScriptSets = OMEGA_LNA_Config_2_scriptSets;
				tunerDescription.tunerId = 0x52;
				error = Error_NO_ERROR;
				break;
			default:
				tunerDescription.tunerScript = OMEGA_scripts;
				tunerDescription.tunerScriptSets = OMEGA_scriptSets;
				tunerDescription.tunerId = 0x38;
				error = Error_NO_ERROR;
				break;

		 }
	}
exit:

	return error;
}
