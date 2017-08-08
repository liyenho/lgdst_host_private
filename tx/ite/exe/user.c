#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h> // for mutex declr
#include <libusb.h>
#include "usb_tx.h"
#include "user.h"

extern struct libusb_device_handle *devh;
extern pthread_mutex_t mux;

static DCtable dc_table[7];
static DCtable ofs_table[7];

uint32_t IT9510User_i2cSwitchToADRF6755 (  IT9510INFO*    modulator)
{
	return IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh2_o, 1);
}


uint32_t IT9510User_i2cSwitchToOrion (  IT9510INFO*    modulator)
{
	IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh7_o, 0);
	IT9510User_delay(20);

	IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh8_o, 1);
	IT9510User_delay(20);

	return ModulatorError_NO_ERROR;
}

uint32_t IT9510User_i2cSwitchToOrion2(  IT9510INFO*    modulator)
{
	IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh8_o, 0);
	IT9510User_delay(20);

	IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh7_o, 1);
	IT9510User_delay(20);

	ModulatorError_NO_ERROR;
}

uint32_t IT9510User_getDeviceType (
	  IT9510INFO*    modulator,
	  uint8_t*		  deviceType
){
	uint32_t error = ModulatorError_NO_ERROR;
	uint8_t temp;

	error = IT9510_readRegister (modulator, Processor_LINK, 0x4979, &temp);//has eeprom ??
	if((temp == 1) && (error == ModulatorError_NO_ERROR)){
		error = IT9510_readRegister (modulator, Processor_LINK, 0x49D5, &temp);
		if(error == ModulatorError_NO_ERROR){
			*deviceType = temp;
		}else if(temp == 0){ // No eeprom
			*deviceType = IT9510User_DEVICETYPE;
		}
	}
	else
	{
		*deviceType = 1;
	}

	return(error);
}

uint32_t IT9510User_memoryCopy (
      IT9510INFO*    modulator,
      void*           dest,
      void*           src,
      uint32_t           count
) {
    /*
     *  ToDo:  Add code here
     *
     *  //Pseudo code
     *  memcpy(dest, src, (size_t)count);
     *  return (0);
     */
	memcpy(dest, src, (size_t)count);
    return (ModulatorError_NO_ERROR);
}

uint32_t IT9510User_delay (
      uint32_t           dwMs
) {
    /*
     *  ToDo:  Add code here
     *
     *  //Pseudo code
     *  delay(dwMs);
     *  return (0);
     */
	int kilo = (stream_on)? 1000 : 1000;
	if(dwMs > 0)
		usleep(dwMs*kilo); // cheating with a quarter of actual delay requested, liyenho
    return (ModulatorError_NO_ERROR);
}

uint32_t IT9510User_printf (const char* format,...){

	perror(format);
	return (ModulatorError_NO_ERROR);
}

uint32_t IT9510User_enterCriticalSection (
	void
) {
    /*
     *  ToDo:  Add code here
     *
     *  //Pseudo code
     *  return (0);
     */
    return (ModulatorError_NO_ERROR);
}

uint32_t IT9510User_leaveCriticalSection (
	void
) {
    /*
     *  ToDo:  Add code here
     *
     *  //Pseudo code
     *  return (0);
     */
    return (ModulatorError_NO_ERROR);
}

uint32_t IT9510User_mpegConfig (
      IT9510INFO*    modulator
) {
    /*
     *  ToDo:  Add code here
     *
     */
    return (ModulatorError_NO_ERROR);
}


uint32_t IT9510User_busTx (
      IT9510INFO*    modulator,
      uint32_t           bufferLength,
      uint8_t*           buffer
) {

	 int32_t 		i, r, msg[80]; // access buffer
	 dev_access *acs = (dev_access*)msg;
	 uint32_t     error = ModulatorError_NO_ERROR;
	int got_nack= 0, not_ready = 0, mis_align=0;
	float wait = (stream_on)? 0.02 : 0.01;

	 for (i = 0; i < IT9510User_MAXFRAMESIZE; i++)
	 	{
			 acs->access = IT951X_WRITE;
			 acs->dcnt = bufferLength;
			 acs->addr = IT951X_ADDRESS;
			 memcpy(acs->data, buffer, bufferLength);

		 	pthread_mutex_lock(&mux);
			r = libusb_control_transfer(devh,CTRL_OUT, USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0);
			pthread_mutex_unlock(&mux);
			short_sleep (wait); // NACK_RETRIES = 16

         if (r != (sizeof(*acs)+(acs->dcnt-1)))
         	continue ;

			while(1) {
		 		pthread_mutex_lock(&mux);
			    acs->addr = IT951X_ADDRESS;
			    acs->access = IT951X_WRITE;
				  if (libusb_control_transfer(devh,CTRL_IN, USB_RQ,USB_HOST_MSG_RX_VAL,USB_HOST_MSG_IDX,
							    (unsigned char*)acs, sizeof(*acs)-1/*byte*/, 0)) {
						pthread_mutex_unlock(&mux);
						break;
					}
				pthread_mutex_unlock(&mux);
				short_sleep(0.0005);
			}
			 if (0>(int8_t)acs->access )
				 switch((int8_t)acs->access) {
					 case -128:
					 		not_ready += 1;
					 		printf("WARNING: User_busTx: write not ready cnt=%d\n",
					 			not_ready);
					 		short_sleep (0.1); // extra wait to get atmel ready
					 		break;
					 case -127:
					 		got_nack += 1;
					 		printf("WARNING: User_busTx: write nack cnt=%d\n",
					 			got_nack);
					 		break;
					 default :
					 		mis_align += 1;
					 		printf("WARNING: User_busTx: write misaligned cnt=%d\n",
					 			mis_align);
					 		error = Error_I2C_WRITE_FAILED;
					 		goto exit;
				 }
			 else  // transfer is ok
			 	break;
			short_sleep (0.01);
		}
	 exit:
    return (IT9510User_MAXFRAMESIZE==i)?Error_I2C_WRITE_FAILED:(error);
}
uint32_t IT9510User_busTx2 (
      IT9510INFO*    modulator,
      uint32_t           bufferLength,
      uint8_t*           buffer
) {
#if 1
		return IT9510User_busTx (
		      modulator,
		      bufferLength,
		      buffer	);
#else
	 int32_t 		i, r, msg[80]; // access buffer
	 dev_access *acs = (dev_access*)msg;
	 uint32_t     error = ModulatorError_NO_ERROR;
	 acs->access = IT951X_WRITE;
	 acs->dcnt = bufferLength;
	 acs->addr = IT951X_ADDRESS;
	 memcpy(acs->data, buffer, bufferLength);
	 for (i = 0; i < IT9510User_MAXFRAMESIZE; i++)
	 	{
		 	pthread_mutex_lock(&mux);
			r = libusb_control_transfer(devh,CTRL_OUT, USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0);
			pthread_mutex_unlock(&mux);
			if (r == (sizeof(*acs)+(acs->dcnt-1))) goto exit;
			else error = -r; // error # defined in libusb, liyenho
			short_sleep (0.1);
			}
	 exit:

    return (error);
#endif
}


uint32_t IT9510User_busRx (
      IT9510INFO*    modulator,
      uint32_t           bufferLength,
      uint8_t*           buffer
) {
     int32_t	  i, r, msg[80]; // access buffer
     dev_access *acs = (dev_access*)msg;
	 uint32_t     error = ModulatorError_NO_ERROR;
	int got_nack =0, not_ready = 0, mis_align=0;
	float wait=(stream_on)?0.02:0.01;

	 for (i = 0; i < IT9510User_MAXFRAMESIZE; i++)
	 {
		 acs->access = IT951X_READ;
		 acs->dcnt = bufferLength;
		 acs->addr = IT951X_ADDRESS;

		 	pthread_mutex_lock(&mux);
			r =	libusb_control_transfer(devh,CTRL_OUT, USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,	(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0);
			pthread_mutex_unlock(&mux);
			short_sleep (wait); // NACK_RETRIES = 16

			if (r != (sizeof(*acs)+(acs->dcnt-1)))
				continue ;

			while(1) {
		 		pthread_mutex_lock(&mux);
				if (libusb_control_transfer(devh,CTRL_IN, USB_RQ,USB_HOST_MSG_RX_VAL,USB_HOST_MSG_IDX,
								(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0)){
					pthread_mutex_unlock(&mux);
					break;
				}
				pthread_mutex_unlock(&mux);
				short_sleep(0.0005);
			}
			 if (0>(int8_t)acs->access )
				 switch((int8_t)acs->access) {
					 case -128:
					 		not_ready += 1;
					 		printf("WARNING: Cmd_busRx: read not ready cnt=%d\n",
					 			not_ready);
			 				short_sleep (0.1); // extra wait to get atmel ready
					 		break;
					 case -127:
					 		got_nack += 1;
					 		printf("WARNING: Cmd_busRx: read nack cnt=%d\n",
					 			got_nack);
					 		break;
					 default :
					 		mis_align += 1;
					 		printf("WARNING: Cmd_busRx: read misaligned cnt=%d\n",
					 			mis_align);
					 		error = Error_I2C_READ_FAILED;
					 		goto exit;
				 }
			 else  // transfer is ok
				break ;
			short_sleep (wait);
	  }
	 memcpy(buffer, acs->data, bufferLength);
exit:
	 return (IT9510User_MAXFRAMESIZE==i)?Error_I2C_READ_FAILED:(error);
}
uint32_t IT9510User_busRx2 (
      IT9510INFO*    modulator,
      uint32_t           bufferLength,
      uint8_t*           buffer
) {
#if 1
		return IT9510User_busRx (
		      modulator,
		      bufferLength,
		      buffer	);
#else
     int32_t	  i, r, msg[80]; // access buffer
     dev_access *acs = (dev_access*)msg;
	 uint32_t     error = ModulatorError_NO_ERROR;
	 acs->access = IT951X_READ;
	 acs->dcnt = bufferLength;
	 acs->addr = IT951X_ADDRESS;
	 for (i = 0; i < IT9510User_MAXFRAMESIZE; i++)
	 {
		 	pthread_mutex_lock(&mux);
			r =	libusb_control_transfer(devh,CTRL_OUT, USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,	(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0);
			pthread_mutex_unlock(&mux);
			short_sleep(0.1); 	// validate echo after 0.1 sec
			if (r != (sizeof(*acs)+(acs->dcnt-1)))
				continue ;
			while(1) {
				pthread_mutex_lock(&mux);
				if (libusb_control_transfer(devh,CTRL_IN, USB_RQ,USB_HOST_MSG_RX_VAL,USB_HOST_MSG_IDX,	(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0)){
					pthread_mutex_unlock(&mux);
					break;
				}
				pthread_mutex_unlock(&mux);
				short_sleep(0.0005);
			}
			break ;
	  }
	 memcpy(buffer, acs->data, bufferLength);
	 return (error);
#endif
}


uint32_t IT9510User_setBus (
      IT9510INFO*    modulator
) {
	/*
     *  ToDo:  Add code here
     *
     *  // If no error happened return 0, else return error code.
     *  return (0);
     */
    uint32_t error = ModulatorError_NO_ERROR;

    return(error);
}

uint32_t IT9510User_Initialization  (
      IT9510INFO*    modulator
) {
	/*
     *  ToDo:  Add code here
     *
     *  // If no error happened return 0, else return error code.
     *  return (0);
     */
	uint32_t error = 0;
	//uint8_t deviceType = 0;

	modulator->isExtLo = False;
/*
	error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh8_en, 1);	//U/V filter control pin
	if (error) goto exit;
	error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh8_on, 1);
	if (error) goto exit;
*/
	if((modulator->tsInterfaceType == SERIAL_TS_INPUT) || (modulator->tsInterfaceType == PARALLEL_TS_INPUT)){
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh1_o, 1); //RX(IT9133) rest
		if (error) goto exit;

		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh1_en, 1);
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh1_on, 1);
		if (error) goto exit;
		IT9510User_delay(5);

		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh5_o, 0); //new U/V filter control pin
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh5_en, 1);
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh5_on, 1);
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh3_en, 1); //new U/V filter control pin
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh3_on, 1);
		if (error) goto exit;

		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh4_en, 1);
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh4_on, 1);
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh6_en, 1);
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh6_on, 1);
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh7_en, 1);
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh7_on, 1);
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh8_en, 1);
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh8_on, 1);
		if (error) goto exit;

		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh8_o, 0);
		if (error) goto exit;

		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh7_o, 1);
		if (error) goto exit;
	}

	error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh2_en, 1);
	if (error) goto exit;
	error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh2_on, 1);
	if (error) goto exit;


//#ifdef _ORION_CAL_H
	//----- init orion
	//IT9510User_getDeviceType(modulator, &deviceType);
	//if(deviceType == 0xD9 || deviceType == 0x99){
/*		IT9510User_i2cSwitchToOrion(modulator);
		if(modulator->isExtLo == True)
			error = orion_cal_init(modulator, 20000);
		else
			error = orion_cal_init(modulator, 12000);

		if (error) goto exit;
*/
	//}
//#endif
	error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh2_o, 0); //RF out power down
	if (error) goto exit;
	IT9510User_LoadDCCalibrationTable(modulator);
exit:

    return (error);

 }


uint32_t IT9510User_Finalize  (
      IT9510INFO*    modulator
) {
	/*
     *  ToDo:  Add code here
     *
     *  // If no error happened return 0, else return error code.
     *  return (0);
     */
#if 0
	uint32_t error = 0;
	if(modulator->busId == Bus_USB)
		error = Usb2_exitDriver((Modulator*)modulator);
	else if(modulator->busId == Bus_9035U2I)
		error = Af9035u2i_exitDriver((Modulator*)modulator);
	else

		error = ModulatorError_INVALID_BUS_TYPE;
#endif
    return (ModulatorError_NO_ERROR);

 }


uint32_t IT9510User_acquireChannel (
	  IT9510INFO*    modulator,
	  uint16_t          bandwidth,
	  uint32_t         frequency
){

	/*
     *  ToDo:  Add code here
     *
     *  // If no error happened return 0, else return error code.
     *  return (0);
     */
	uint32_t error = 0;

	//control U/V filter by gpioh8
/*
	if(frequency < 250000){ // <=250000 v-filter gpio set to Lo
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh8_o, 0);
		if (error) goto exit;

	}else{
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh8_o, 1);
		if (error) goto exit;
	}

	//control new 4CH U/V filter by gpioh3 & 5
	if(frequency <= 150000){ //
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh3_o, 1);
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh5_o, 1);
		if (error) goto exit;
	}else if((frequency > 150000)&&(frequency <= 250000)){
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh3_o, 1);
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh5_o, 0);
		if (error) goto exit;
	}else if((frequency > 250000)&&(frequency <= 450000)){
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh3_o, 0);
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh5_o, 1);
		if (error) goto exit;
	}else if(frequency > 450000){
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh3_o, 0);
		if (error) goto exit;
		error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh5_o, 0);
		if (error) goto exit;
	}
*/

	return (error);
}


uint32_t IT9510User_acquireChannelDual(
	  IT9510INFO*    modulator,
	  uint16_t          bandwidth,
	  uint32_t         frequency
	){

	/*
	*  ToDo:  Add code here
	*
	*  // If no error happened return 0, else return error code.
	*  return (0);
	*/
	uint32_t error = 0;
exit:

	return (error);

}

uint32_t IT9510User_setTxModeEnable (
	  IT9510INFO*            modulator,
	  uint8_t                    enable
) {
	/*
     *  ToDo:  Add code here
     *
     *  // If no error happened return 0, else return error code.
     *  return (0);
     */
	uint32_t error = ModulatorError_NO_ERROR;
	uint32_t         frequency = 0;
	if(enable){

//#ifdef _ORION_CAL_H
		if(modulator->isExtLo == True)
			frequency =  (modulator->frequency/1000)*1000; // fixed Accuracy issue
		else
			frequency =  modulator->frequency;
		//----- orion cal dc

			IT9510User_i2cSwitchToOrion(modulator);
		//	IT9510User_delay(20);

			printf("IT9510 orion initialize.\n");
			if(modulator->isExtLo == True)
				error = orion_cal_init(modulator, 20000);
			else
				error = orion_cal_init(modulator, 12000);
			if (error) goto exit;

		// cal dc with orion iqik module, Michael, 20140502

			//IT9510User_delay(200);
			IT9510User_delay(10);
			printf("IT9510 orion_calDC.\n");
			if(modulator->isExtLo == True)
				error = orion_calDC(modulator, frequency, False);
			else
				error = orion_calDC(modulator, frequency, True);
			if (error) goto exit;

			IT9510User_i2cSwitchToOrion2(modulator);

//#endif

	}else{

	}
	error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh2_o, enable);
	if (error) goto exit;
exit :
	return (error);
}

uint32_t IT9510User_LoadDCCalibrationTable (
	  IT9510INFO*            modulator
) {
    uint32_t error = ModulatorError_NO_ERROR;
	DCInfo dcInfo;
	uint8_t DCvalue[16];
	uint8_t OFS_I_value[7];
	uint8_t OFS_Q_value[7];
	uint8_t eeprom = 0;
	uint8_t DCcalibration = 0;
	uint8_t index = 0;
//	uint8_t i =0;

	//-------------set DC Calibration table
	error = IT9510_readRegister (modulator, Processor_LINK, 0x4979, &eeprom);//has eeprom ??
	if (error) goto exit;
	error = IT9510_readRegister (modulator, Processor_LINK, 0x49D6, &DCcalibration);//has DCcalibration ??
	if (error) goto exit;

	if((eeprom ==1) && ((DCcalibration & 0x80) == 0x80)){
		puts("found eeprom and chip supported DC calibration as well");  // liyenho
		error = IT9510_readRegisters (modulator, Processor_LINK, 0x49D6, 12, DCvalue);//DC calibration value
		if (error) goto exit;

		error = IT9510_readRegisters (modulator, Processor_LINK, 0x49CC, 5, OFS_I_value);//OFS_I calibration value
		if (error) goto exit;

		error = IT9510_readRegisters (modulator, Processor_LINK, 0x49E2, 5, OFS_Q_value);//OFS_Q calibration value
		if (error) goto exit;

		error = IT9510_readRegister (modulator, Processor_LINK, 0x49C9, &DCvalue[12]);//DC_6_i calibration value
		if (error) goto exit;
		error = IT9510_readRegister (modulator, Processor_LINK, 0x49D1, &DCvalue[13]);//DC_6_q calibration value
		if (error) goto exit;
		error = IT9510_readRegister (modulator, Processor_LINK, 0x49CB, &DCvalue[14]);//DC_7_i calibration value
		if (error) goto exit;
		error = IT9510_readRegister (modulator, Processor_LINK, 0x49E7, &DCvalue[15]);//DC_7_q calibration value
		if (error) goto exit;

		error = IT9510_readRegister (modulator, Processor_LINK, 0x49AB, &OFS_I_value[5]);//OFS_6_i calibration value
		if (error) goto exit;
		error = IT9510_readRegister (modulator, Processor_LINK, 0x49E8, &OFS_Q_value[5]);//OFS_6_q calibration value
		if (error) goto exit;
		error = IT9510_readRegister (modulator, Processor_LINK, 0x49C3, &OFS_I_value[6]);//OFS_7_i calibration value
		if (error) goto exit;
		error = IT9510_readRegister (modulator, Processor_LINK, 0x49E9, &OFS_Q_value[6]);//OFS_7_q calibration value
		if (error) goto exit;
		puts("set up I/Q-DC calibration table");  // liyenho
		for(index = 1; index<8; index++){

			if(index == 1){
				dc_table[index-1].startFrequency = 200000;
				ofs_table[index-1].startFrequency = 200000;
			} else if(index == 2){
				dc_table[index-1].startFrequency = 325000;
				ofs_table[index-1].startFrequency = 325000;
			} else if(index == 3){
				dc_table[index-1].startFrequency = 500000;
				ofs_table[index-1].startFrequency = 500000;
			} else if(index == 4){
				dc_table[index-1].startFrequency = 700000;
				ofs_table[index-1].startFrequency = 700000;
			} else if(index == 5){
				dc_table[index-1].startFrequency = 875000;
				ofs_table[index-1].startFrequency = 875000;
			} else if(index == 6){
				dc_table[index-1].startFrequency = 1250000;
				ofs_table[index-1].startFrequency = 1250000;
			} else if(index == 7){
				dc_table[index-1].startFrequency = 2400000;
				ofs_table[index-1].startFrequency = 2400000;
			}

			dc_table[index-1].i = DCvalue[(index*2)];
			if(((DCvalue[0] >> (index-1) ) &0x01) == 0)
				dc_table[index-1].i = dc_table[index-1].i * -1;

			dc_table[index-1].q = DCvalue[1 + (index*2)];
			if(((DCvalue[1] >> (index-1) ) &0x01) == 0)
				dc_table[index-1].q = dc_table[index-1].q * -1;


			ofs_table[index-1].i = OFS_I_value[index-1];
			ofs_table[index-1].q = OFS_Q_value[index-1];
		}

		dcInfo.ptrDCtable = dc_table;
		dcInfo.ptrOFStable = ofs_table;
		dcInfo.tableGroups = 7;

		error = IT9510_setDCtable(modulator, dcInfo);
	}
exit:
	return error;
}

uint32_t IT9510User_rfPowerOn (
	  IT9510INFO*            modulator,
	  Bool                   isPowerOn
){
	uint32_t error = ModulatorError_NO_ERROR;

	error = IT9510_writeRegister (modulator, Processor_LINK, p_IT9510_reg_top_gpioh2_o, (uint8_t)isPowerOn);

	return error;

}

