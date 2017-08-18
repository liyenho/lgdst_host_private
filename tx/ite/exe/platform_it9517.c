#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h> // for thread-safe design
#include "platform_it9517.h"

static pthread_mutex_t mux_thr; // protect at 1st tier (user level)
uint16_t I2c_IO_PORT = 0x38;
IQtable IQ_tableEx[65536];
IT9510INFO eagle;


uint32_t it9517_loadIQ_calibration_table (const char*file_name)
{

	uint32_t error = ModulatorError_NO_ERROR;
	FILE *pfile = NULL;
	char buf[16]={0};
	char inputFile[255]={0};
	int dAmp = 0;
	int dPhi = 0;
	int row;
	uint16_t groups;
	uint32_t freq = 0;
	uint32_t version = 0;
	CalibrationInfo calibrationInfo;


	pfile = fopen(file_name,"rb");
	if(pfile != NULL){
		fread(buf,16,1,pfile);
		version = buf[10]<<16 | buf[11]<<8 | buf[12];	//table version ex:0x0112
		groups = buf[14]<<8 | buf[15];	//frequency groups ex:(950000-50000)/10000 + 1 = 91
		for(row = 0; row < groups; row++){
			fread(&freq,4,1,pfile);
			fread(&dAmp,1,2,pfile);
			fread(&dPhi,1,2,pfile);

			IQ_tableEx[row].frequency = freq;
			IQ_tableEx[row].dAmp = (short)dAmp;
			IQ_tableEx[row].dPhi = (short)dPhi;

		}
		calibrationInfo.ptrIQtableEx = IQ_tableEx;
		calibrationInfo.tableGroups = groups;
		calibrationInfo.tableVersion = version;
		pthread_mutex_lock(&mux_thr);
		error = IT9510_setIQtable(&eagle, calibrationInfo);
		pthread_mutex_unlock(&mux_thr);
		if(error)
			printf("ModulatorError_NULL_PTR \n");
		else
			printf("Load IQ Calibration Table successful\n");

		fclose(pfile);
		pfile = NULL;
	}else{
		error = ModulatorError_OPEN_FILE_FAIL;
	}


	return error;
}

//id_bus=Bus_I2C
//stream_type= SERIAL_TS_INPUT
uint32_t it9517_initialize (uint8_t id_bus,TsInterface stream_type)
{
	pthread_mutex_init(&mux_thr, NULL);
	uint32_t error = ModulatorError_NO_ERROR;
	uint8_t var[2];
	uint8_t chip_version = 0;
	uint32_t chip_Type;
	uint8_t Bus_id = Bus_USB;
	TsInterface tsin_streamType = PARALLEL_TS_INPUT;

	Bus_id  =id_bus;
	tsin_streamType = stream_type;
	pthread_mutex_lock(&mux_thr);
	error = IT9510_initialize (&eagle, tsin_streamType, Bus_id, IT9510User_IIC_ADDRESS);
	pthread_mutex_unlock(&mux_thr);
	if (error) goto exit;
	pthread_mutex_lock(&mux_thr);
	error = IT9510_readRegister(&eagle, Processor_LINK, 0x1222, &chip_version);
	pthread_mutex_unlock(&mux_thr);
	if (error) goto exit;
	pthread_mutex_lock(&mux_thr);
	error = IT9510_readRegisters(&eagle, Processor_LINK, 0x1222+1, 2, var);
	pthread_mutex_unlock(&mux_thr);
	if (error) goto exit;

	chip_Type = var[1]<<8 | var[0];
	//printf("chip_Type = %x chip_version=%x\n",chip_Type,chip_version);
	printf("IT9510 Initialize successful.\n");
	pthread_mutex_lock(&mux_thr);
	IT9510User_LoadDCCalibrationTable(&eagle);
	pthread_mutex_unlock(&mux_thr);
exit:
	if (error)  printf("error=%x,%d\n",error,__LINE__);
	return error;
}

uint32_t it9517_monitor_version (void)
{
	uint32_t error = ModulatorError_NO_ERROR;
	uint32_t version = 0;

	printf("API Version = %04x.%08x.%02x\n", IT9510_Version_NUMBER, IT9510_Version_DATE, IT9510_Version_BUILD);
	pthread_mutex_lock(&mux_thr);
	error = IT9510_getFirmwareVersion (&eagle, Processor_LINK, &version);
	pthread_mutex_unlock(&mux_thr);
	if (error) {
		printf("IT9510 getFirmwareVersion(LINK) failed! Error = 0x%08x\n", error);

	} else {
		printf("IT9510 LINK FW Version = 0x%08x\n", version);
	}
	pthread_mutex_lock(&mux_thr);
	error = IT9510_getFirmwareVersion (&eagle, Processor_OFDM, &version);
	pthread_mutex_unlock(&mux_thr);
	if (error) {
		printf("IT9510 getFirmwareVersion(OFDM) failed! Error = 0x%08x\n", error);

	} else {
		printf("IT9510 OFDM FW Version = 0x%08x\n", version);
	}

	return error;
}

uint32_t it9517_suspend(uint8_t enable)
{
	uint32_t error = ModulatorError_NO_ERROR;
	pthread_mutex_lock(&mux_thr);
	error = IT9510_suspendMode(&eagle,enable);
	pthread_mutex_unlock(&mux_thr);
	if (error)
		printf("IT9510 suspend failed! error = 0x%08x\n", error);
	else
		printf("IT9510 suspend successful.\n");

	return error;
}

uint32_t it9517_reset(void)
{
	uint32_t error = ModulatorError_NO_ERROR;
	pthread_mutex_lock(&mux_thr);
	error = IT9510_reset(&eagle);
	pthread_mutex_unlock(&mux_thr);
	if (error)
		printf("IT9510 reset failed! error = 0x%08x\n", error);
	else
		printf("IT9510 reset successful.\n");

	return error;
}

uint32_t it9517_reboot(void)
{
	uint32_t error = ModulatorError_NO_ERROR;
	pthread_mutex_lock(&mux_thr);
	error = IT9510_TXreboot(&eagle);
	pthread_mutex_unlock(&mux_thr);
	if (error)
		printf("IT9510 TXreboot Failed! Error = 0x%08x\n", error);
	else
		printf("IT9510 TXreboot Back-To-Boot code successful.\n");

	return error;
}

uint32_t it9517_set_channel_modulation(ChannelModulation channel_modulation,NullPacketMode	mode)
{
	uint32_t error = ModulatorError_NO_ERROR;
	ChannelModulation      channelModulation;

	//printf ("constellation type? 0:QPSK 1:16QAM 2:64QAM\n");

	channelModulation.constellation=channel_modulation.constellation;

	//printf ("coding rate? 0:1/2, 1:2/3, 2:3/4, 3:5/6, 4:7/8\n");

	channelModulation.highCodeRate=channel_modulation.highCodeRate;
	//printf ("guard interval? 0:1/32, 1:1/16, 2:1/8, 3:1/4\n");

	channelModulation.interval=channel_modulation.interval;

	//printf ("transmission Mode? 0:2k, 1:8k\n");

	channelModulation.transmissionMode=channel_modulation.transmissionMode;

	//printf ("Null packet Mode? 1:SequentialMode, 2:NormalMode\n");

	eagle.nullPacketMode = mode;
	pthread_mutex_lock(&mux_thr);
	error=IT9510_setNullPacketMode(&eagle,eagle.nullPacketMode);
	pthread_mutex_unlock(&mux_thr);
	if (error){
		printf("IT9510 set NullPacketMode failed.\n");
		goto exit;
	}
	else{
		printf("IT9510 set NullPacketMode successful.\n");
	}
	pthread_mutex_lock(&mux_thr);
	error = IT9510_setTXChannelModulation(&eagle, &channelModulation);
	pthread_mutex_unlock(&mux_thr);
	if (error){
		printf("IT9510 setChannelModulation failed.\n");
		goto exit;
	}
	else{
		printf("IT9510 set Channel Modulation successful.\n");
	}
exit:
	if (error)  printf("error=%x,%d\n",error,__LINE__);

	return error;

}

//frequency=frequency-1583000
uint32_t it9517_set_frequency(uint32_t frequency)
{
	uint32_t error = ModulatorError_NO_ERROR;
	uint32_t freq=frequency;

	pthread_mutex_lock(&mux_thr);
	error = IT9510_setFrequency (&eagle,freq);
	pthread_mutex_unlock(&mux_thr);
	if (error) {
		printf("IT9510 setFrequency failed.\n");

		goto exit;
	}
	else{
		printf("IT9510 setFrequency successful.\n");
	}
exit:
	if (error)  printf("error=%x,%d\n",error,__LINE__);

	return error;

}

uint32_t it9517_set_ts_interface(TsInterface   streamType)
{
	uint32_t error = ModulatorError_NO_ERROR;
	pthread_mutex_lock(&mux_thr);
	error = IT9510_setTsInterface (&eagle,streamType);
	pthread_mutex_unlock(&mux_thr);
	if (error) {
		printf("IT9510 setTsInterface failed.\n");

		goto exit;
	}
	else{
		printf("IT9510 setTsInterface successful.\n");
	}
exit:
	if (error)  printf("error=%x,%d\n",error,__LINE__);

	return error;

}
//1: power up
//0: power down
uint32_t it9517_control_power_saving(uint8_t control)
{
	uint32_t error = ModulatorError_NO_ERROR;
	pthread_mutex_lock(&mux_thr);
	error = IT9510_controlPowerSaving (&eagle,control);
	pthread_mutex_unlock(&mux_thr);
	if (error) {
		printf("IT9510 controlPowerSaving failed.\n");
		goto exit;
	}
	else{
		printf("IT9510 controlPowerSaving successful.\n");
	}
exit:
	if (error)  printf("error=%x,%d\n",error,__LINE__);

	return error;

}

//frequency=frequency-1583000
uint32_t it9517_acquire_channel(uint32_t frequency,uint16_t bandwidth)
{
	uint32_t error = ModulatorError_NO_ERROR;
	uint32_t freq=frequency;
	uint16_t bw=bandwidth;
	pthread_mutex_lock(&mux_thr);
	error = IT9510_acquireTxChannel (&eagle,bw,freq);
	pthread_mutex_unlock(&mux_thr);
	if (error) {
		printf("IT9510 acquireTxChannel failed.\n");

		goto exit;
	}
	else{
		printf("IT9510 acquireTxChannel successful.\n");
	}
exit:
	if (error)  printf("error=%x,%d\n",error,__LINE__);

	return error;

}

//1:enable 0:disable
uint32_t it9517_enable_transmission_mode(uint8_t enable)
{
	uint32_t error = ModulatorError_NO_ERROR;
	pthread_mutex_lock(&mux_thr);
	error = IT9510_setTxModeEnable(&eagle, enable);
	pthread_mutex_unlock(&mux_thr);
	if (error){
		printf("IT9510 setTxModeEnable failed.\n");
		goto exit;
	}else{
		if(enable)
			printf("IT9510 setTxModeEnable enable successful.\n");
		else
			printf("IT9510 setTxModeEnable disable successful.\n");
	}
exit:
	if (error)  printf("error=%x,%d\n",error,__LINE__);

	return error;

}

uint32_t it9517_get_output_gain(void)
{
	uint32_t error = ModulatorError_NO_ERROR;

	int gain;
	pthread_mutex_lock(&mux_thr);
	error = IT9510_getOutputGain(&eagle,&gain);
	pthread_mutex_unlock(&mux_thr);
	if(error){
		printf("IT9510 getOutputGain failed.\n");
		goto exit;
	}
	else{
		printf("IT9510 getOutputGain successful. gain = %d\n",gain);
	}
exit:
	if (error)  printf("error=%x,%d\n",error,__LINE__);

	return error;

}

uint32_t it9517_get_output_gain_range(uint32_t frequency,uint16_t bandwidth)
{
	uint32_t error = ModulatorError_NO_ERROR;

	int min_gain,max_gain;
	pthread_mutex_lock(&mux_thr);
	error = IT9510_getGainRange(&eagle,frequency,bandwidth,&max_gain,&min_gain);
	pthread_mutex_unlock(&mux_thr);
	if(error){
		printf("IT9510 getGainRange failed.\n");
		goto exit;
	}
	else{
		printf("IT9510 getGainRange successful. max_gain = %d,min_gain=%d\n",max_gain,min_gain);
	}
exit:
	if (error)  printf("error=%x,%d\n",error,__LINE__);

	return error;

}

uint32_t it9517_adjust_output_gain(int gain)
{
	uint32_t error = ModulatorError_NO_ERROR;
	pthread_mutex_lock(&mux_thr);
	error = IT9510_adjustOutputGain(&eagle,&gain);
	pthread_mutex_unlock(&mux_thr);
	if(error){
		printf("IT9510 AdjustOutputGain failed.\n");
		goto exit;
	}
	else{
		printf("IT9510 AdjustOutputGain successful. gain = %d\n",gain);
	}
exit:
	if (error)  printf("error=%x,%d\n",error,__LINE__);

	return error;

}

//0:pass the specified PIDs<Pass mode>
//1:filter the specified PIDs<Block mode>
//0:disable PidFilter
//1:enable PidFilter
uint32_t it9517_control_pidfilter(uint8_t control,uint8_t enable)
{
	uint32_t error = ModulatorError_NO_ERROR;
	pthread_mutex_lock(&mux_thr);
	error = IT9510_controlPidFilter(&eagle, control,enable);
	pthread_mutex_unlock(&mux_thr);
	if (error) {
		printf("IT9510 controlPidFilter failed.\n");
		goto exit;
	}
	else{
		if(enable)
			printf("IT9510 enable PidFilter successful.\n");
		else
			printf("IT9510 disable PidFilter successful.\n");
	}
exit:
	if (error)  printf("error=%x,%d\n",error,__LINE__);

	return error;

}

uint32_t it9517_reset_pidfilter(void)
{
	uint32_t error = ModulatorError_NO_ERROR;
	pthread_mutex_lock(&mux_thr);
	error = IT9510_resetPidFilter(&eagle);
	pthread_mutex_unlock(&mux_thr);
	if (error) {
		printf("IT9510 resetPidFilter failed.\n");
		goto exit;
	}
	else{
		printf("IT9510 resetPidFilter successful.\n");
	}
exit:
	if (error)  printf("error=%x,%d\n",error,__LINE__);

	return error;
}


uint32_t it9517_add_pidfilter(uint8_t index,uint32_t value)
{
	uint32_t error = ModulatorError_NO_ERROR;

	Pid pid;
	pid.value = value;
	pthread_mutex_lock(&mux_thr);
	error = IT9510_addPidToFilter(&eagle, index, pid);
	pthread_mutex_unlock(&mux_thr);
	if (error){
		printf("IT9510 controlPidFilter failed.\n");
		goto exit;
	}
	else{
		printf("IT9510 add pid successful.\n");
	}
exit:
	if (error)  printf("error=%x,%d\n",error,__LINE__);

	return error;
}

uint32_t it9517_enable_tps_encryption(uint32_t key)
{
	uint32_t error = ModulatorError_NO_ERROR;
	pthread_mutex_lock(&mux_thr);
	error = IT9510_enableTpsEncryption(&eagle, key);
	pthread_mutex_unlock(&mux_thr);
	if (error){
		printf("IT9510 enableTpsEncryption failed.\n");
		goto exit;
	}
	else{
		printf("IT9510 enableTpsEncryption successful.\n");
	}
exit:
	if (error)  printf("error=%x,%d\n",error,__LINE__);

	return error;
}

uint32_t it9517_disable_tps_encryption(void)
{
	uint32_t error = ModulatorError_NO_ERROR;
	pthread_mutex_lock(&mux_thr);
	error = IT9510_disableTpsEncryption(&eagle);
	pthread_mutex_unlock(&mux_thr);
	if (error){
		printf("IT9510 disableTpsEncryption failed.\n");
		goto exit;
	}
	else{
		printf("IT9510 disableTpsEncryption successful.\n");
	}
exit:
	if (error)  printf("error=%x,%d\n",error,__LINE__);

	return error;
}


//begin : 0-43
//enable=1 :on
//enable=0 :off
uint32_t it9517_aes_encryption(uint8_t *buf,uint8_t begin,Bool enable)
{
	uint32_t error = ModulatorError_NO_ERROR;
	uint8_t AES_Key[16];
	uint8_t temp;
	int i;
	for(i=0; i<16; i++){
		//printf ("input AES key[%d] (total 16 byte)\n",i);
		//scanf_s ("%08X", &w_temp);
		AES_Key[i] = (uint8_t)buf;
	}
	pthread_mutex_lock(&mux_thr);
	error = IT9510_setAesEncryptionKey(&eagle, AES_Key);
	pthread_mutex_unlock(&mux_thr);
	if(error){
		printf("IT9510 setAesEncryptionKey failed.\n");
		goto exit;
	}else{
		printf("IT9510 setAesEncryptionKey successful.\n");
		for(i=0; i<16; i++){
			pthread_mutex_lock(&mux_thr);
			IT9510_readRegister(&eagle, Processor_OFDM, p_IT9510_reg_aes_key0_7_0 + i, &temp);
			printf("Key[%d] = %x \n",i,temp);
			pthread_mutex_unlock(&mux_thr); // don't place this before printf()
		}

	}
	pthread_mutex_lock(&mux_thr);
	error = IT9510_setEncryptionStartAddress(&eagle, begin);
	pthread_mutex_unlock(&mux_thr);
	if(error){
		printf("IT9510 setEncryptionStartAddress failed.\n");
		goto exit;
	}
	else{
		printf("IT9510 setEncryptionStartAddress successful addr = %d.\n", begin);
	}
	pthread_mutex_lock(&mux_thr);
	error = IT9510_aesEncryptionEnable(&eagle, enable);
	pthread_mutex_unlock(&mux_thr);
	if(error){
		printf("IT9510 aesEncryptionEnable failed.\n");
		goto exit;
	}else{
		if(enable)
			printf("IT9510 aesEncryptionEnable on.\n");
		else
			printf("IT9510 aesEncryptionEnable off.\n");
	}
exit:
	if (error)  printf("error=%x,%d\n",error,__LINE__);

	return error;


}

uint32_t it9517_pcr_restamp(PcrMode pcr_mode,uint8_t enable)
{
	uint32_t error = ModulatorError_NO_ERROR;
	pthread_mutex_lock(&mux_thr);
	error = IT9510_setPcrMode (&eagle, pcr_mode); // set to PCR re-stamping mode 1
	pthread_mutex_unlock(&mux_thr);
	if(error){
		printf("IT9510 setPcrMode failed.\n");
		goto exit;
	}else{
		printf("IT9510 setPcrMode successful.\n");
	}
	pthread_mutex_lock(&mux_thr);
	error = IT9510_setPcrModeEnable (&eagle, enable);
	pthread_mutex_unlock(&mux_thr);
	if(error){
		printf("IT9510 setPcrMode enable failed.\n");
		goto exit;
	}else{
		printf("IT9510 setPcrMode enable successful.\n");
	}

exit:
	if (error)  printf("error=%x,%d\n",error,__LINE__);

	return error;

}


uint32_t it9517_read_eeprom()
{
	uint32_t error = ModulatorError_NO_ERROR;
	uint8_t buffer[3];
	pthread_mutex_lock(&mux_thr);
	error = IT9510_readEepromValues (&eagle, 0, 0x0000,  buffer);
	pthread_mutex_unlock(&mux_thr);
	if(error){
		printf("IT9510 readEepromValues failed.\n");
		goto exit;
	}else{
		printf("IT9510_readEepromValues successful.\n");
		printf ("The value of 0x0000 is %2x", buffer[0]);
		printf ("The value of 0x0001 is %2x", buffer[1]);
		printf ("The value of 0x0002 is %2x", buffer[2]);
	}
exit:
	if (error)  printf("error=%x,%d\n",error,__LINE__);

	return error;
}


uint32_t it9517_write_eeprom()
{

	uint32_t error = ModulatorError_NO_ERROR;
	uint8_t buffer[3] = { 0x00, 0x01, 0x02 };

	// Set the value of cell 0x0000 in EEPROM to 0.
	// Set the value of cell 0x0001 in EEPROM to 1.
	// Set the value of cell 0x0002 in EEPROM to 2.
	pthread_mutex_lock(&mux_thr);
	error = IT9510_writeEepromValues (&eagle, 0x0000, 3, buffer);
	pthread_mutex_unlock(&mux_thr);
	if(error){
		printf("IT9510 writeEepromValues failed.\n");
		goto exit;
	}else{
		printf("IT9510 writeEepromValues successful.\n");
	}
exit:
	if (error)  printf("error=%x,%d\n",error,__LINE__);

	return error;

}


uint32_t it9517_check_tsbuffer_overflow(void)
{
	uint32_t error = ModulatorError_NO_ERROR;
	Bool overflow;
	pthread_mutex_lock(&mux_thr);
	error = IT9510_isTsBufferOverflow (&eagle,&overflow);
	pthread_mutex_unlock(&mux_thr);
	if(error){
		printf("IT9510 check tsbuffer overflow failed.\n");
		goto exit;
	}else{
		printf("IT9510 check tsbuffer overflow successful.\n");
		if(overflow)
			printf("it9517 tsbuffer is overflow!.\n");
		else
			printf("it9517 tsbuffer is ok.\n");
	}
exit:
	if (error)  printf("error=%x,%d\n",error,__LINE__);

	return error;

}


uint32_t it9517_finalize(void)
{
	uint32_t error = ModulatorError_NO_ERROR;
	pthread_mutex_lock(&mux_thr);
	error = IT9510_finalize (&eagle);
	pthread_mutex_unlock(&mux_thr);
	if(error){
		printf("IT9510_finalize failed.\n");
		goto exit;
	}else{
		printf("IT9510_finalize successful.\n");
	}
exit:
	if (error)  printf("error=%x,%d\n",error,__LINE__);
	pthread_mutex_destroy(&mux_thr);
	return error;

}

#include <libusb.h>
#include "usb_tx.h"
#include "timer.h"

extern struct libusb_device_handle *devh;
extern volatile bool ready_wait_for_mloop;
extern pthread_mutex_t mux;
extern volatile int do_exit ;
extern unsigned char radio_tpacket[RADIO_USR_TX_LEN],
												radio_rpacket[RADIO_USR_RX_LEN+RADIO_INFO_LEN];

uint32_t it9517_video_channel_select() {
	uint32_t i, error=ModulatorError_NO_ERROR;
	short vch ;
	long vif ;
	puts("...... begin to wait for selected video channels ......");
retry_vch_rec:
	i = 0;
	while ((1 == do_exit) &&
				(RADIO_USR_RX_LEN != i)) {
		short_sleep(0.1);
		while(1 == do_exit) {
			pthread_mutex_lock(&mux);
			if (libusb_control_transfer(devh,
					CTRL_IN,
					USB_RQ,
					RADIO_COMM_VAL,
					RADIO_DATA_RX_IDX,
					radio_rpacket,
					sizeof(radio_rpacket),
					0)){
				pthread_mutex_unlock(&mux);
				break;
			}
			pthread_mutex_unlock(&mux);
			short_sleep(0.0005);
		}
		for(i=0;i<RADIO_USR_RX_LEN;i++) { // RADIO_USR_RX_LEN==RADIO_USR_TX_LEN! liyenho
			if ( RADIO_USR_RX_LEN/2 != i &&
				RADIO_USR_TX_LEN-i-1 != radio_rpacket[i])
				break; // signature of vid ch sel packet
		}
	}
	if (1!=do_exit ) {
		ready_wait_for_mloop = True;
		return error;
	}
	vch = radio_rpacket[RADIO_USR_RX_LEN/2];
	if (vch >= NUM_OF_VID_CH) {
		short_sleep(1);
		goto retry_vch_rec;  // bad video ch idx
	}
#if 0  // disable acknowledge process, liyenho
	// send video channel ack to Rx side
	for(i=0;i<RADIO_USR_TX_LEN;i++) {
		radio_tpacket[i]= \
			RADIO_USR_TX_LEN-i-1; // signature of vid ch sel packet
	}
	radio_tpacket [RADIO_USR_TX_LEN/2] = True;
	i = 0;
	do {
		pthread_mutex_lock(&mux);
			libusb_control_transfer(devh,
			 													CTRL_OUT,
			 													USB_RQ,
																RADIO_COMM_VAL,
																RADIO_DATA_TX_IDX,
																radio_tpacket,
																RADIO_USR_TX_LEN,
																0);
		pthread_mutex_unlock(&mux);
		usleep(CTRL_SEND_POLLPERIOD);
	} while (100 > i++) ; // flood Rx with vch ack msg
#endif
	vif = (long)vch *VID_CH_TTL+VID_IF_CH_BASE;
	printf("...... video channel IF = %d @ %d selected......\n", vif, vch);
	error=it9517_acquire_channel(vif,VID_CH_BW);
	if(error) return error;
	error=it9517_adjust_output_gain(0/*-30*/);
	if(error) return error;

	error=it9517_enable_transmission_mode(1);
	return error;
}


