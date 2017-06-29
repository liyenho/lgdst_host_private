#include "platform_it9137.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h> // for thread-safe design

static pthread_mutex_t mux_thr; // protect at 1st tier (user level)
IT9130 it9130;

uint32_t it9137_init(void)
{
	pthread_mutex_init(&mux_thr, NULL);
	uint8_t chip_number=1;
	uint16_t saw_bandwidth=6000;
	StreamType stream_type =StreamType_DVBT_SERIAL;
	Architecture architecture = Architecture_DCA; //not invid in our application!
	uint32_t error=Error_NO_ERROR;
	error=OMEGA_supportLNA(&it9130, 0x03);
	if(error) return error;
	pthread_mutex_lock(&mux_thr);
	error = Demodulator_initialize(&it9130, chip_number,  saw_bandwidth, stream_type,  architecture);
	pthread_mutex_unlock(&mux_thr);
	if(error){
		printf("IT9133 initialize failed, error = 0x%x.\n", error);
	}

	return error;

}

uint32_t it9137_deinit(void)
{
	uint32_t error=Error_NO_ERROR;
	pthread_mutex_lock(&mux_thr);
	error=Demodulator_finalize(&it9130);
	pthread_mutex_unlock(&mux_thr);
	if(error){
		printf("IT9133 deinit failed, error = 0x%x.\n", error);
	}
	pthread_mutex_destroy(&mux_thr);
	return error;
}


//no use
uint32_t it9137_reset(void)
{
	uint32_t error=Error_NO_ERROR;
	pthread_mutex_lock(&mux_thr);
	error=Demodulator_reset(&it9130);
	pthread_mutex_unlock(&mux_thr);
	if(!error){

		printf("IT9133  reset ok.\n");
	}
	return error;
}


//no use
uint32_t it9137_reboot(void)
{
	uint32_t error=Error_NO_ERROR;
	pthread_mutex_lock(&mux_thr);
	error=Demodulator_reboot(&it9130);
	pthread_mutex_unlock(&mux_thr);
	if(!error){

		printf("IT9133 reboot ok.\n");
	}
	return error;
}

uint32_t it9137_get_firmwareversion(void)
{
	uint32_t error=Error_NO_ERROR;
	uint32_t link_firmwareversion;
	uint32_t ofdm_firmwareversion;
	pthread_mutex_lock(&mux_thr);
	error=Demodulator_getFirmwareVersion(&it9130, Processor_LINK, &link_firmwareversion);
	pthread_mutex_unlock(&mux_thr);
	if(error){
		printf("IT9133 getFirmwareVersion failed, error = 0x%x.\n", error);
	}else{
		printf("link_firmwareversion=0x%x\n",link_firmwareversion);
	}
	pthread_mutex_lock(&mux_thr);
	error=Demodulator_getFirmwareVersion(&it9130, Processor_OFDM, &ofdm_firmwareversion);
	pthread_mutex_unlock(&mux_thr);
	if(error){
		printf("IT9133 getFirmwareVersion failed, error = 0x%x.\n", error);
	}else{
		printf("ofdm_firmwareversion=0x%x.\n",ofdm_firmwareversion);
	}

	return error;

}

uint32_t it9137_acquire_channel(uint8_t chip,uint32_t frequency,uint16_t bandwidth)
{

	uint32_t error=Error_NO_ERROR;
	Booll locked;
	pthread_mutex_lock(&mux_thr);
	error= Demodulator_acquireChannel(&it9130, chip,  bandwidth, frequency);
	pthread_mutex_unlock(&mux_thr);
	if(error){
		printf("IT9133 acquireChannel failed, error = 0x%x.\n", error);
	}else{
		printf("frequency=%d,bandwidth=%d\n",frequency,bandwidth);
	}
	pthread_mutex_lock(&mux_thr);
	error=Demodulator_isLocked(&it9130, chip, &locked);
	pthread_mutex_unlock(&mux_thr);
	printf("locked=%d\n",locked);
	if(error){
		printf("IT9133 isLocked failed, error = 0x%x.\n", error);
	}else{
		printf(" IT9133 channel can be access.\n");
	}

	return error;

}

uint32_t it9137_scan_channel(uint8_t chip,uint32_t start_frequency,uint32_t end_frequency, uint16_t bandwidth)
{
	uint32_t error=Error_NO_ERROR;

	uint32_t  frequency;
	long strengthdbm;
	for (frequency =start_frequency ; frequency <= end_frequency; frequency += bandwidth) {
		pthread_mutex_lock(&mux_thr);
		error = Demodulator_acquireChannel( &it9130, chip, bandwidth, frequency);
		pthread_mutex_unlock(&mux_thr);
		if (error) {
			printf ("IT9133 acquireChannel failed, error = 0x%x.\n", error);

		}else{

			printf("frequency=%d,bandwidth=%d\n",frequency,bandwidth);
		}
		usleep(400000);
		pthread_mutex_lock(&mux_thr);
		error=Demodulator_getSignalStrengthDbm(&it9130, chip, &strengthdbm);
		pthread_mutex_unlock(&mux_thr);
		if(!error){

			printf("the signal strength is %ld\n",strengthdbm);
		}


	}
	return error;

}

uint32_t it9137_get_if_agc(uint8_t chip)
{
	uint32_t error=Error_NO_ERROR;
	uint8_t if_agc;
	pthread_mutex_lock(&mux_thr);
	error=Demodulator_getIfAgcGain(&it9130, chip,&if_agc);
	pthread_mutex_unlock(&mux_thr);
	if(error){
		printf("IT9133 get if_agc failed, error = 0x%x.\n", error);
	}else{

		printf("IT9133 get if_agc successful.if_agc=%d\n",if_agc);

	}
	return error;

}

uint32_t it9137_get_rf_agc_gain(uint8_t chip)
{
	uint32_t error=Error_NO_ERROR;
	uint8_t rf_agc_gain;
	pthread_mutex_lock(&mux_thr);
	error=Demodulator_getRfAgcGain(&it9130, chip,&rf_agc_gain);
	pthread_mutex_unlock(&mux_thr);
	if(error){
		printf("IT9133 get rf_agc gain failed, error = 0x%x.\n", error);
	}else{

		printf("IT9133 get rf_agc gain successful.rf_agc_gain=%d\n",rf_agc_gain);

	}
	return error;

}

//0 disable  1 enable

uint32_t it9137_control_pid_filter(uint8_t chip,uint8_t control)
{
	uint32_t error=Error_NO_ERROR;
	pthread_mutex_lock(&mux_thr);
	error=Demodulator_controlPidFilter(&it9130, chip,  control);
	pthread_mutex_unlock(&mux_thr);
	if(error){
		printf("IT9133 control_pidfilter failed, error = 0x%x.\n", error);
	}
	return error;

}

uint32_t it9137_reset_filter(uint8_t chip)
{
	uint32_t error=Error_NO_ERROR;
	pthread_mutex_lock(&mux_thr);
	error=Demodulator_resetPidFilter(&it9130, chip);
	pthread_mutex_unlock(&mux_thr);
	if(error){
		printf("IT9133 reset_filter failed, error = 0x%x.\n", error);

	}
	return error;
}

uint32_t it9137_add_pid_filter(uint8_t chip,uint8_t index,Pid pid)
{
	uint32_t error=Error_NO_ERROR;
	pthread_mutex_lock(&mux_thr);
	error=Demodulator_addPidToFilter(&it9130, chip,  index, pid);
	pthread_mutex_unlock(&mux_thr);
	if(error){
		printf("IT9133 add_pidfilter failed, error = 0x%x.\n", error);
	}
	return error;

}

//control=0 power down
uint32_t it9137_control_power_saving(uint8_t chip,uint8_t control)
{
	uint32_t error=Error_NO_ERROR;
	pthread_mutex_lock(&mux_thr);
	error=Demodulator_controlPowerSaving(&it9130, chip, control);
	pthread_mutex_unlock(&mux_thr);
	if(error){
		printf("IT9133 controlPowerSaving failed, error = 0x%x.\n", error);

	}

	return error;
}

uint32_t it9137_check_tpslocked(uint8_t chip)
{

	uint32_t error=Error_NO_ERROR;
	Booll istpslocked;
	pthread_mutex_lock(&mux_thr);
	error=Demodulator_isTpsLocked(&it9130, chip, &istpslocked);
	pthread_mutex_unlock(&mux_thr);
	if(!error){
		if(istpslocked) printf("TPS is locked.\n");
		else printf("TPS is not locked.\n");
	}
	return error;
}

uint32_t it9137_check_mpeg2locked(uint8_t chip)
{

	uint32_t error=Error_NO_ERROR;
	Booll islocked;
	pthread_mutex_lock(&mux_thr);
	error=Demodulator_isMpeg2Locked(&it9130, chip, &islocked);
	pthread_mutex_unlock(&mux_thr);
	if(!error){
		if(islocked) printf("MPEG is locked.\n");
		else printf("MPEG is not locked.\n");
	}
	return error;
}


uint32_t it9137_set_streamtype( StreamType  streamType)
{

	uint32_t error=Error_NO_ERROR;
	Booll islocked;
	pthread_mutex_lock(&mux_thr);
	error=Demodulator_setStreamType(&it9130, streamType);
	pthread_mutex_unlock(&mux_thr);
	if(!error){
		printf("set stream type ok.\n");
	}
	return error;
}

uint32_t it9137_set_architecture( Architecture      architecture)
{

	uint32_t error=Error_NO_ERROR;
	Booll islocked;
	pthread_mutex_lock(&mux_thr);
	error=Demodulator_setArchitecture(&it9130, architecture);
	pthread_mutex_unlock(&mux_thr);
	if(!error){
		printf("set architecture ok.\n");
	}
	return error;
}

uint32_t it9137_get_channel_modulation(uint8_t chip)
{
	uint32_t error=Error_NO_ERROR;
	ChannelModulation *info;
	pthread_mutex_lock(&mux_thr);
	error=Demodulator_getChannelModulation(&it9130, chip, info);
	pthread_mutex_unlock(&mux_thr);
	if(!error){

		printf("Frequency is %dkHz\n", info->frequency);

		if (info->bandwidth == Bandwidth_6M)
			printf ("Bandwidth is 6MHz.\n");
		else if (info->bandwidth == Bandwidth_7M)
			printf ("Bandwidth is 7MHz.\n");
		else if (info->bandwidth == Bandwidth_8M)
			printf ("Bandwidth is 8MHz.\n");
		else if (info->bandwidth == Bandwidth_5M)
			printf ("Bandwidth is 5MHz.\n");
		else
			printf ("Get Bandwidth failed.\n");

		if (info->priority == Priority_HIGH)
			printf ("DVB-T - identifies high-priority stream.\n");
		else if (info->priority == Priority_LOW)
			printf ("DVB-T - identifies low-priority stream.\n");

		if (info->constellation == Constellation_QPSK)
			printf ("Constellation is QPSK.\n");
		else if (info->constellation == Constellation_16QAM)
			printf ("Constellation is 16QAM.\n");
		else if (info->constellation == Constellation_64QAM)
			printf ("Constellation is 64QAM.\n");
		else
			printf ("Get Constellation failed.\n");

		if (info->highCodeRate == CodeRate_1_OVER_2)
			printf ("Signal uses FEC coding ratio of 1/2.\n");
		else if (info->highCodeRate == CodeRate_2_OVER_3)
			printf ("Signal uses FEC coding ratio of 2/3.\n");
		else if (info->highCodeRate == CodeRate_3_OVER_4)
			printf ("Signal uses FEC coding ratio of 3/4.\n");
		else if (info->highCodeRate == CodeRate_5_OVER_6)
			printf ("Signal uses FEC coding ratio of 5/6.\n");
		else if (info->highCodeRate == CodeRate_7_OVER_8)
			printf ("Signal uses FEC coding ratio of 7/8.\n");
		else if (info->highCodeRate == CodeRate_NONE)
			printf ("None, NXT doesn't have this one.\n");
		else
			printf ("Get HighCodeRate failed.\n");

		if (info->lowCodeRate == CodeRate_1_OVER_2)
			printf ("Signal uses FEC coding ratio of 1/2\n");
		else if (info->lowCodeRate == CodeRate_2_OVER_3)
			printf ("Signal uses FEC coding ratio of 1/2\n");
		else if (info->lowCodeRate == CodeRate_3_OVER_4)
			printf ("Signal uses FEC coding ratio of 1/2\n");
		else if (info->lowCodeRate == CodeRate_5_OVER_6)
			printf ("Signal uses FEC coding ratio of 5/6\n");
		else if (info->lowCodeRate == CodeRate_7_OVER_8)
			printf ("Signal uses FEC coding ratio of 7/8\n");
		else if (info->lowCodeRate == CodeRate_NONE)
			printf ("None, FEC doesn't have this code rate\n");
		else
			printf ("Get LowCodeRate failed");

		if (info->interval == Interval_1_OVER_32)
			printf ("Guard interval is 1/32 of symbol length.\n");
		else if (info->interval == Interval_1_OVER_16)
			printf ("Guard interval is 1/16 of symbol length.\n");
		else if (info->interval == Interval_1_OVER_8)
			printf ("Guard interval is 1/8 of symbol length.\n");
		else if (info->interval == Interval_1_OVER_4)
			printf ("Guard interval is 1/4 of symbol length\n");
		else
			printf ("Get Interval failed.\n");

		if (info->transmissionMode == TransmissionMode_2K )
			printf ("OFDM frame consists of 2048 subcarriers (2K FFT mode).\n");
		else if (info->transmissionMode == TransmissionMode_8K )
			printf ("OFDM frame consists of 8192 subcarriers (8K FFT mode).\n");
		else if (info->transmissionMode == TransmissionMode_4K )
			printf ("OFDM frame consists of 4096 subcarriers (4K FFT mode).\n");
		else
			printf ("Get TransmissionMode failed.\n");

		if (info->hierarchy == Hierarchy_NONE)
			printf ("Signal is non-hierarchical.\n");
		else if (info->hierarchy == Hierarchy_ALPHA_1)
			printf ("Signalling format uses alpha of 1.\n");
		else if (info->hierarchy == Hierarchy_ALPHA_2)
			printf ("Signalling format uses alpha of 2.\n");
		else if (info->hierarchy == Hierarchy_ALPHA_4)
			printf ("Signalling format uses alpha of 4.\n");
		else
			printf ("Get Hierarchy failed.\n");

	}
	return error;
}

uint32_t it9137_get_signal_quality(uint8_t chip)
{
	uint32_t error=Error_NO_ERROR;
	uint8_t quality;
	pthread_mutex_lock(&mux_thr);
	error=Demodulator_getSignalQuality(&it9130, chip, &quality);
	pthread_mutex_unlock(&mux_thr);
	if(!error){

		printf("the signal quality is %d.\n",quality);
	}
	return error;
}

uint32_t it9137_get_signal_quality_indication(uint8_t chip)
{
	uint32_t error=Error_NO_ERROR;
	uint8_t quality;
	pthread_mutex_lock(&mux_thr);
	error=Demodulator_getSignalQualityIndication(&it9130, chip, &quality);
	pthread_mutex_unlock(&mux_thr);
	if(!error){

		printf("the signal quality indication is %d.\n",quality);
	}
	return error;
}

//no use
uint32_t it9137_get_signal_strength(uint8_t chip)
{
	uint32_t error=Error_NO_ERROR;
	uint8_t strength;
	pthread_mutex_lock(&mux_thr);
	error=Demodulator_getSignalStrength(&it9130, chip, &strength);
	pthread_mutex_unlock(&mux_thr);
	if(!error){

		printf("the signal strength is %d.\n",strength);
	}
	return error;
}

//no use
uint32_t it9137_get_signal_strength_indication(uint8_t chip)
{
	uint32_t error=Error_NO_ERROR;
	uint8_t strength;
	pthread_mutex_lock(&mux_thr);
	error=Demodulator_getSignalStrengthIndication(&it9130, chip, &strength);
	pthread_mutex_unlock(&mux_thr);
	if(!error){

		printf("the signal strength is %d.\n",strength);
	}
	return error;
}



uint32_t it9137_get_signal_strength_dbm(uint8_t chip, long *sigdbm)
{
	uint32_t error=Error_NO_ERROR;
	long strengthdbm;
	pthread_mutex_lock(&mux_thr);
	error=Demodulator_getSignalStrengthDbm(&it9130, chip, &strengthdbm);
	pthread_mutex_unlock(&mux_thr);
	if(!error){
		if (!sigdbm)
			printf("the signal strength is %ld\n",strengthdbm);
		else  *sigdbm = strengthdbm;
	}
	return error;
}

uint32_t it9137_get_snr(uint8_t chip)
{

	uint32_t error=Error_NO_ERROR;
	uint8_t snr=0;
	pthread_mutex_lock(&mux_thr);
	error=Demodulator_getSNR(&it9130, chip, &snr);
	pthread_mutex_unlock(&mux_thr);
	if(!error){

		printf("signal snr is %d DBm.\n",snr);
	}
	return error;

}


uint32_t it9137_get_postviterbi_bit_error_rate(uint8_t chip, double *ptvitber)
{
	uint32_t post_error_count;
	uint32_t post_bit_count;
	uint16_t about_count;
	double postvitber;
	uint32_t error=Error_NO_ERROR;
	pthread_mutex_lock(&mux_thr);
	error=Demodulator_getPostVitBer(&it9130, chip, & post_error_count, &post_bit_count, &about_count);
	pthread_mutex_unlock(&mux_thr);
	if(!error){

		postvitber=(double)post_error_count/(double)post_bit_count;
		if (!ptvitber)
			printf ("the post viterbi BER is %.3E.post_error_count=%d,post_bit_count=%d\n", postvitber,post_error_count,post_bit_count);
		else  *ptvitber = postvitber;

	}
	return error;
}


/*

   Get the statistic values of demodulator, it includes Pre-Viterbi BER,Post-Viterbi BER,
   Abort Count, Signal Presented Flag, Signal Locked Flag, Signal Quality, Signal Strength,
   Delta-T for DVB-H time slicin

*/
uint32_t it9137_get_statistic(uint8_t chip)
{

	uint32_t error = Error_NO_ERROR;
	Statistic statistic;
	pthread_mutex_lock(&mux_thr);
	error = Demodulator_getStatistic ( &it9130, 0, &statistic);
	pthread_mutex_unlock(&mux_thr);
	if (error)
		printf ("IT9133 get static failed.error = %x.\n", error);
	else
		printf ("IT9133 get static success.\n");

	if (statistic.signalPresented == True)
		printf ("Signal Presented = True.\n");
	else
		printf ("Signal Presented = False.\n");
	if (statistic.signalLocked == True)
		printf ("Signal Locked = True.\n");
	else
		printf ("Signal Locked = False.\n");
	printf ("Signal Quality = %d\n", statistic.signalQuality);
	printf ("Signal Strength = %d\n", statistic.signalStrength);

	return error;

}

#include <stdarg.h>
  void write_v_formatted(FILE * stream, const char * format, ...) {
	  va_list args;
	  va_start (args, format);
	  vfprintf (stream, format, args);
	  va_end (args);
  }
uint32_t it9137_read_ofdm_registers(FILE *dump) {
	uint32_t error, post_error, post_bit;
	long strengthdbm ;
	uint16_t abort;
	uint8_t regs_read[8];

#define OFDM_REGS_API(addr, numb) \
	pthread_mutex_lock(&mux_thr); \
	error=Standard_readRegisters(&it9130, 0, Processor_OFDM, addr, numb, regs_read); \
	pthread_mutex_unlock(&mux_thr); \
	if (error) goto failed ; \
	switch(numb) { \
		case 1: write_v_formatted(dump, "0x%04x: 0x%02x\n", addr, *regs_read); \
						break; \
		case 2: write_v_formatted(dump, "0x%04x: 0x%02x, 0x%02x\n", addr, *regs_read,*(regs_read+1)); \
						break; \
		case 7: write_v_formatted(dump, "0x%04x: 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x,\n", \
																addr, *regs_read, *(regs_read+1), *(regs_read+2), *(regs_read+3), *(regs_read+4), *(regs_read+5), *(regs_read+6)); \
						break; \
	}

	OFDM_REGS_API(0x0046, 1)
	OFDM_REGS_API(0xF21B, 1)
	OFDM_REGS_API(0x0150, 1)
	OFDM_REGS_API(0x003C, 1)
	OFDM_REGS_API(0xF999, 1)
	OFDM_REGS_API(0xF27C, 2)
	OFDM_REGS_API(0xF2A6, 2)
	OFDM_REGS_API(0x005E, 1)
	OFDM_REGS_API(0x00C7, 1)
	OFDM_REGS_API(0x0043, 2)
	OFDM_REGS_API(0x00C1, 1)
	OFDM_REGS_API(0x00AF, 1)
	OFDM_REGS_API(0xF279, 1)
	OFDM_REGS_API(0xF10E, 7)
	OFDM_REGS_API(0x00A7, 1)
	OFDM_REGS_API(0xF02B, 1)
	OFDM_REGS_API(0xFD37, 1)
	OFDM_REGS_API(0x01A4, 1)
	OFDM_REGS_API(0x01A4, 1)
#undef OFDM_REGS_API

	pthread_mutex_lock(&mux_thr);
	error=Demodulator_getPostVitBer(&it9130, 0, & post_error, &post_bit, &abort);
	pthread_mutex_unlock(&mux_thr);
	if (error) goto failed ;
		write_v_formatted(dump, "p_err: %d, p_bit: %d, abort: %d\n", post_error, post_bit, abort);

	pthread_mutex_lock(&mux_thr);
	error=Demodulator_getSignalStrengthDbm(&it9130, 0, &strengthdbm);
	pthread_mutex_unlock(&mux_thr);
	if (error) goto failed ; \
		write_v_formatted(dump, "s_dbm: %d\n", strengthdbm);

failed:
	fflush(dump);

	if (error) {
		fputs("\n",dump); fputs("\n",dump); fputs("\n",dump);
	}
	else
		fputs(".................................................................................\n",dump);
	return error;
}
