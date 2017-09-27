#include "platform_it9137.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h> // for thread-safe design

/*static*/ pthread_mutex_t mux_thr; // protect at 1st tier (user level)
IT9130 it9130;

extern long dbg_rssi;
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

uint32_t it9137_scan_channel(uint8_t chip,
																	uint32_t start_frequency,
																	uint32_t end_frequency,
																	uint16_t bandwidth,
																	uint16_t bw_total, // include guard band
																	long *strengthdbm)
{
	uint32_t error=Error_NO_ERROR;

	uint32_t  frequency;
	for (frequency =start_frequency ; frequency <= end_frequency; frequency += bw_total) {
		pthread_mutex_lock(&mux_thr);
		error = Demodulator_acquireChannel( &it9130, chip, bandwidth, frequency);
		pthread_mutex_unlock(&mux_thr);
		if (error) {
			printf ("IT9133 acquireChannel failed, error = 0x%x.\n", error);

		}else{

			printf("frequency=%d,bandwidth=%d\n",frequency,bandwidth);
		}
		usleep(/*400000*/250000); // is this magic # to wait for stable signal strength?
		pthread_mutex_lock(&mux_thr);
		error=Demodulator_getSignalStrengthDbm(&it9130, chip, strengthdbm);
		pthread_mutex_unlock(&mux_thr);
		if(!error){

			printf("the signal strength is %ld\n",*strengthdbm);
		}
		strengthdbm += 1;

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

uint32_t it9137_check_tpslocked(uint8_t chip, void* istpslocked)
{

	uint32_t error=Error_NO_ERROR;
	pthread_mutex_lock(&mux_thr);
	error=Demodulator_isTpsLocked(&it9130, chip, (Booll*)istpslocked);
	pthread_mutex_unlock(&mux_thr);
	if(!error){
		if(*(Booll*)istpslocked) printf("TPS is locked.\n");
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

uint32_t it9137_get_snr(uint8_t chip, uint8_t *snr1)
{

	uint32_t error=Error_NO_ERROR;
	uint8_t snr=0;
	pthread_mutex_lock(&mux_thr);
	error=Demodulator_getSNR(&it9130, chip, &snr);
	pthread_mutex_unlock(&mux_thr);
	if(!error){
		if (snr1)
		   *snr1 = snr;
		else
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

#include <libusb.h>
#include "usb_rx.h"
#include "timer.h"
#if USE_MAVLINK
 #include "MavLink.h"
#endif

extern struct libusb_device_handle *devh;
extern pthread_mutex_t mux;
extern volatile int do_exit ;
extern unsigned char radio_tpacket[RADIO_USR_TX_LEN],
												radio_rpacket[RADIO_USR_RX_LEN+RADIO_INFO_LEN];

uint32_t it9137_video_channel_scan(Booll ran_once) {
	puts("...... scanning available video channels ......");
  static long vif= -1, tif, chdbm[NUM_OF_VID_CH];
  	static short tch, chidx[NUM_OF_VID_CH];
  	static short vch = -1; /*invalid channel #*/
  	bool istpslocked;
  uint32_t error;
  	int i, ch, ch1, size;
	if (!ran_once) {
	  	for (ch=0; ch<NUM_OF_VID_CH; ch++)
	  		chidx[ch] = ch;	// setup ch in sequential order at start
retry1:
		error=it9137_scan_channel(0, VID_IF_CH_BASE, VID_IF_CH_CEIL, VID_CH_BW, VID_CH_TTL, chdbm);
		if (error && 1==do_exit) {
			short_sleep(0.02);
			goto retry1;
		}
		// sort thru chan strength vector
	#define SWAP(a,b,t) t=a; a=b; b=t;
		for (ch=0; ch<NUM_OF_VID_CH-1; ch++)
			for (ch1=0; ch1<NUM_OF_VID_CH-1; ch1++)
				if (chdbm[ch1] > chdbm[ch1+1]) {
					SWAP(chidx[ch1], chidx[ch1+1], tch);
					SWAP(chdbm[ch1], chdbm[ch1+1], tif);
				}
	#undef SWAP
	}
	for (ch=0; ch<NUM_OF_VID_CH; ch++)
		if (VID_CH_STR_THR >= chdbm[ch]) {
			ch1 = (long)chidx[ch]*VID_CH_TTL+VID_IF_CH_BASE;
retry2:
			error = it9137_acquire_channel(0, ch1, VID_CH_BW);
			if (error && 1==do_exit) {
				short_sleep(0.02);
				goto retry2;
			}
retry3:
			error = it9137_check_tpslocked(0, &istpslocked);
			if(error && 1==do_exit) {
				short_sleep(0.02);
				goto retry3;
			}
			if (False == istpslocked &&
					vch != chidx[ch]) { // avoid choosing current channel, dynamic channel selection at any time
				vch = chidx[ch];
				vif = ch1; // available channel found
				break ;
			}
			vif = -1L;
		}
		else break;
	if (-1L == vif) {
		puts("failed to find available video channel, using one with lowest exisitng signal strength");
		//return (uint32_t)-1 ;
		vch = chidx[0];
		ch1 = (long)vch*VID_CH_TTL+VID_IF_CH_BASE;
retry6:
			error = it9137_acquire_channel(0, ch1, VID_CH_BW);
			if (error && 1==do_exit) {
				short_sleep(0.02);
				goto retry6;
			}
retry7:
			error = it9137_check_tpslocked(0, &istpslocked);
			if(error && 1==do_exit) {
				short_sleep(0.02);
				goto retry7;
			}
		printf("expect inadequate video quality, video channel IF = %d @ %d selected\n", ch1, vch);
	} else {
		printf("...... video channel IF = %d @ %d selected......\n", vif, vch);
	}
	// send video selected channel index to Tx side
	for(i=0;i<RADIO_USR_TX_LEN;i++) {
		radio_tpacket[i]= \
			RADIO_USR_TX_LEN-i-1; // signature of vid ch sel packet
	}
	radio_tpacket [RADIO_USR_TX_LEN/2] =vch;
	i = 0;
#if USE_MAVLINK
	uint8_t pending[255+2+MAVLINK_HDR_LEN]; // mavlink arch for efficiency
   Build_Mavlink_Data_Packet(pending, RADIO_USR_TX_LEN, radio_tpacket);
#endif
	do {
#if USE_MAVLINK
		size = MAVLINK_HDR_LEN +((MavLinkPacket*)pending)->length+ MAVLINK_CHKSUM_LEN; // accommodate mavlink var msg len,
		pthread_mutex_lock(&mux);
		 libusb_control_transfer(devh,CTRL_OUT, USB_RQ,RADIO_COMM_VAL,RADIO_MAVLEN_OUT_IDX,&size, 2, 0);
		 libusb_control_transfer(devh,
		 													CTRL_OUT,
		 													USB_RQ,
															RADIO_COMM_VAL,
															RADIO_DATA_TX_IDX,
															pending,
															MAVLINK_USB_LEN,
															0);
		pthread_mutex_unlock(&mux);
		usleep(10*CTRL_SEND_POLLPERIOD);
#else
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
#endif
	} while ( 100 > i++);  // flood Tx with vch sel msg
	if (1 == do_exit)
		short_sleep(10);  // allow sufficient time for tx to startup video
	// wait for video coming from TX
	while (1 == do_exit) {
retry4:
		error = it9137_acquire_channel(0, ch1, 6000);
		if (error && 1==do_exit) {
			short_sleep(0.02);
			goto retry4;
		}
retry5:
		error = it9137_check_tpslocked(0, &istpslocked);
		if(error && 1==do_exit) {
			short_sleep(0.02);
			goto retry5;
		}
		if (istpslocked)
			break ;	// video started @ tx
		else {
#if 0	// disable acknowledge process, liyenho
			bool vack;
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
			for(i=0;i<RADIO_USR_RX_LEN;i++) { // RADIO_USR_RX_LEN==RADIO_USR_TX_LEN
				if ( RADIO_USR_RX_LEN/2 != i &&
					RADIO_USR_TX_LEN-i-1 != radio_rpacket[i])
					break; // signature of vid ch ack packet
			}
#endif
			if (1 == do_exit)
#if 0
				usleep(CTRL_SEND_POLLPERIOD); // send again asap
#else
				usleep(1.0); // send again after a sec
#endif
#if 0	// disable acknowledge process, liyenho
			if (RADIO_USR_RX_LEN != i) {
				continue;
			}
			vack = True==radio_rpacket[RADIO_USR_RX_LEN/2];
			if (!vack)
#endif
			{
				i = 0;
				do {
#if USE_MAVLINK
		      pthread_mutex_lock(&mux);
		 		libusb_control_transfer(devh,CTRL_OUT, USB_RQ,RADIO_COMM_VAL,RADIO_MAVLEN_OUT_IDX,&size, 2, 0);
		       libusb_control_transfer(devh,
		       													CTRL_OUT,
		       													USB_RQ,
															      RADIO_COMM_VAL,
															      RADIO_DATA_TX_IDX,
															      pending,
															      MAVLINK_USB_LEN,
															      0);
		      pthread_mutex_unlock(&mux);
		      usleep(10*CTRL_SEND_POLLPERIOD);
#else
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
#endif
				} while ( 100 > i++);  // flood Tx with vch sel msg, and to keep ctrl radio locked!!! liyenho
			}
		}
	}
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
