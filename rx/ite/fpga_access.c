#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <math.h>
#include <libusb.h>
#include "fpga_access.h"  // definitions for host/fpga access @ tx

#define CTRL_OUT							(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT)
#define CTRL_IN								(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN)

#define COMM_ATMEL_DEV // enable atmel<->fpga/6612 comm test

#define true								1
#define false								0
#define print_usage                    puts("lgdst 0 rx r/w/bm/Va/[Uc]/ns/s/pair-id/pair-locked/loc-gps/MDst/temp/ctune/calib/calib-qry/hopless/setFEC/SiGetProp/RSSI/ant-sw/setCtrlPwr/use915/use869/setRadioChannel/set_vch fpath [chidx] [bsz] [val0,val1,...], all numbers are in hex");
#define RAED_SETUP	\
							shmLgdst_proc->type = ACS; \
							shmLgdst_proc->tag.wDir = CTRL_OUT; \
							shmLgdst_proc->tag.wValue = USB_HOST_MSG_TX_VAL; \
							shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX; \
    	  	 				goto _read;
typedef int bool;

#ifdef LIB
 extern volatile ipcLgdst *shmLgdst_proc;
#else
 int shmid_Lgdst = -1;
 volatile ipcLgdst *shmLgdst_proc = 0;
#endif
static volatile int do_exit = 0;	// main loop breaker
static bool work_mode = (bool)-1;

static void at_exit(int status) {
	if (do_exit) {
		do_exit = -1;
  }
#ifndef LIB
  if (shmLgdst_proc)
  	shmdt(shmLgdst_proc);
#endif
  exit(status);
}

static void perror_exit(char *message, int status)
{
  char fail[160], cs[8];
  sprintf(cs, ", %d",status);
  strncpy(fail, message, sizeof(fail));
    strcat(fail, cs);
  perror(fail);
  at_exit(status);
}

static int short_sleep(double sleep_time)
{
  struct timeval tv;

  tv.tv_sec = (time_t) floor(sleep_time);
  tv.tv_usec = (time_t) ((sleep_time - floor(sleep_time)) * 1.0e6);

  return(select(0, NULL, NULL, NULL, &tv));
}

static int htoi(char s[])
{
    int hexdigit,i,inhex,n;
    i = 0;
    if( s[i] == '0')
    {
        ++i;
        if(s[i] == 'x' || s[i] == 'X')
            ++i;
    }

    n = 0;
    inhex = true;

    for(;inhex==true;++i)
    {
        if(s[i] >='0' && s[i] <='9')
            hexdigit= s[i] - '0';
        else if(s[i] >='a' && s[i] <='f')
            hexdigit= s[i] -'a' + 10;
        else if(s[i] >='A' && s[i] <='F')
            hexdigit= s[i] -'A' + 10;
        else
            inhex = false;

        if(inhex == true)
            n = 16 * n + hexdigit;
    }
    return n;
}

static void print_ctrl_bits(BW_CTRL x, char *t) {
	switch(x) {
		case NEUTRAL:
				printf("%s : NEUTRAL\n", t);
				break;
		case LONG_RNG:
				printf("%s : LONG_RNG\n", t);
				break;
		case SHORT_RNG:
				printf("%s : SHORT_RNG\n", t);
				break;
		default :
				printf("%s is Invalid\n", t);
				break;
	}
}

static void ctrl_chsel_func(int entry) {
	for (int j=0; j<2/*2 param sets per chsel cmd*/; j++) {
		uint8_t *ch_param = chtbl_ctrl_rdo[entry*2+j];
				shmLgdst_proc->type = CMD1;
				shmLgdst_proc->len = chtbl_ctrl_len[j];
				shmLgdst_proc->tag.wDir = CTRL_OUT;
				shmLgdst_proc->tag.wValue = RADIO_COMM_VAL;
				shmLgdst_proc->tag.wIndex = RADIO_CHSEL_IDX;
				memcpy(shmLgdst_proc->access.hdr.data, ch_param, chtbl_ctrl_len[j]);
			// send the parameter set
			shmLgdst_proc->active = 1;  // wait for transaction complete
			while (1==shmLgdst_proc->active) ;
	}
}

#ifdef LIB
  int lgdst_access_rx(int argc,char **argv, void **ret)
#else
  int main(int argc,char **argv)
#endif
{
	int r = 1, i;
	do_exit = 1;
	if (3+1>argc) {
		print_usage
		return -1;
	}
	if (!strcasecmp(argv[2],"tx"))
		work_mode = true;
	else if (!strcasecmp(argv[2],"rx"))
		work_mode = false;
	else {
		puts("invalid work mode");
		print_usage
		return -2;
	}
#ifndef LIB
	while (0>=shmid_Lgdst) {
		if (work_mode) // Tx
	  	  shmid_Lgdst = shmget(SHMKEY_TX, sizeof(ipcLgdst), 0666);
		else // Rx
	  	  shmid_Lgdst = shmget(SHMKEY_RX, sizeof(ipcLgdst), 0666);
  }
  	if (0>(shmLgdst_proc=(ipcLgdst*)shmat(shmid_Lgdst, NULL, 0)))
	  perror_exit("get shmLgdst_proc shmem failed",-1);
#else //android doesn't have IPC facility, replaced with normal static global allocation
	while (0>=shmLgdst_proc) ; //
#endif
	/* we shouldn't bother issues of USB hw setup/config */
 #include <time.h>
 #include <sys/time.h>
 extern struct timeval tstart,tend,tdelta;
  extern void print_time(struct timeval ttime);
  extern int time_diff(struct timeval *time1,
  								struct timeval *time2,
                                struct timeval *diffTime);
  extern int get_time(struct timeval *time);
    extern int short_sleep(double sleep_time);
  	static int32_t sz, fc, tmp; // access buffer

  	dev_access *acs = (dev_access*)&shmLgdst_proc->access.hdr;
	while(-1 != shmLgdst_proc->active);
	get_time(&tstart);

   #ifdef COMM_ATMEL_DEV
     { 	// prepend access header
			if (3+1>argc ||(strcasecmp(argv[3],"ns") && strcasecmp(argv[3],"s") &&
	        strcasecmp(argv[3],"Uc") /*cpld*/&&
	        strcasecmp(argv[3],"Ua") /*atmel*/&&
	        /*strcasecmp(argv[3],"Vc") &&*/ /*cpld*/
	        strcasecmp(argv[3],"Va") /*atmel*/&&
			  strcasecmp(argv[3],"bm") /*atm boot mode*/&&
	        /*strcasecmp(argv[3],"Cst") &&*/
	        strcasecmp(argv[3], MDst) &&
	        /*strcasecmp(argv[3],"Cch") &&*/
	        strcasecmp(argv[3], tempCMD) &&
	        strcasecmp(argv[3],	ctune) &&
	        strcasecmp(argv[3], calib) &&
	        strcasecmp(argv[3], calibqry) &&
	        strcasecmp(argv[3], hopless) &&
	        strcasecmp(argv[3], pairID) &&
	        strcasecmp(argv[3], pairLocked)&&
			strcasecmp(argv[3], locGPS)&&
			strcasecmp(argv[3], RSSI)&&
			strcasecmp(argv[3], setFEC)&&
			strcasecmp(argv[3], setCtrlPwr)&&
			strcasecmp(argv[3], SiGetProp)&&
			strcasecmp(argv[3], use869)&&
			strcasecmp(argv[3], use915)&&
			strcasecmp(argv[3], setRadioChannel)&&
			strcasecmp(argv[3], ant_sw)&&
			strcasecmp(argv[3], "set_vch")&&
			 strcasecmp(argv[3],"r")&& strcasecmp(argv[3],"w")))
			{
    	  	  	puts("invalid access mode...");
					print_usage
    	  	  	goto _exit;
			}
			else if (!strcasecmp(argv[3],"Cst")) { // control radio states readout
				shmLgdst_proc->type = CMD1;
				shmLgdst_proc->len = RADIO_STATS_LEN;
				shmLgdst_proc->tag.wDir = CTRL_IN;
				shmLgdst_proc->tag.wValue = RADIO_COMM_VAL;
				shmLgdst_proc->tag.wIndex = RADIO_STATS_IDX;
			}
			else if (!strcasecmp(argv[3], MDst)) { // control modem states readout
				shmLgdst_proc->type = CMD1;
				shmLgdst_proc->len = RADIO_MODEM_LEN;
				shmLgdst_proc->tag.wDir = CTRL_IN;
				shmLgdst_proc->tag.wValue = RADIO_COMM_VAL;
				shmLgdst_proc->tag.wIndex = RADIO_MODEM_IDX;
			}
			else if (!strcasecmp(argv[3], ant_sw)) { // toggle video antenna pair on drone
				shmLgdst_proc->type = CMD0;
				shmLgdst_proc->tag.wValue = USB_ANT_SW_VAL;
				shmLgdst_proc->tag.wIndex = 0x1; // current cdc data interface
			}
			else if (!strcasecmp(argv[3], "set_vch")) { // set video IF frequency by ch index
				uint8_t vch = htoi(argv[4]);
				if (10< vch) {
					puts("video channel index must be within [1,10]");
					goto _exit ;
				}
			  	shmLgdst_proc->type = CMD1;
				shmLgdst_proc->len = sizeof(vch); // 1 byte vid ch idx
				shmLgdst_proc->tag.wDir = CTRL_OUT;
				shmLgdst_proc->tag.wValue = VIDEO_SETVCH_VAL;
				shmLgdst_proc->tag.wIndex = VIDEO_SETVCH_IDX;
				memcpy(shmLgdst_proc->access.hdr.data, &vch, sizeof(vch));
			}
			else if (false/*rx*/==work_mode && !strcasecmp(argv[3],"Cch")) { // control RF channel selection
				uint8_t *ch_param = NULL;
				int ch_sel = htoi(argv[4]);
				if (5 != argc) {
					perror_exit("invalid command line parameters, lgdst 0 rx Cch ctrl-ch#",-5);
				} else if (0>ch_sel || sizeof(chtbl_ctrl_rdo)/sizeof(ch_param)<= 2*ch_sel) {
						perror_exit("invalid channel #, beyond available ch or negative",-6);
					}
				ctrl_chsel_func(ch_sel) ;
				goto _exit0;
			}
			else if (!strcasecmp(argv[3], tempCMD)) { // // control radio temperature readout
				shmLgdst_proc->type = CMD1;
				shmLgdst_proc->len = RADIO_TEMP_LEN;
				shmLgdst_proc->tag.wDir = CTRL_IN;
				shmLgdst_proc->tag.wValue = RADIO_COMM_VAL;
				shmLgdst_proc->tag.wIndex = RADIO_TEMP_IDX;
			}
			else if (!strcasecmp(argv[3], ctune)) { // tune cap bank on ctrl radio chip
	    	  tmp = htoi(argv[4]);
	    	  if (0>tmp || 127<tmp) {
 					puts("invalid capacitor bank tuning value...");
		     		goto _exit; }
		     	shmLgdst_proc->type = CMD1;
				shmLgdst_proc->len = RADIO_CTUNE_LEN;
				shmLgdst_proc->tag.wDir = CTRL_OUT;
				shmLgdst_proc->tag.wValue = RADIO_COMM_VAL;
				shmLgdst_proc->tag.wIndex = RADIO_CTUNE_IDX;
				memcpy(shmLgdst_proc->access.hdr.data, &tmp, RADIO_CTUNE_LEN);
			}
			else if (!strcasecmp(argv[3], pairID)) {
				if (4+HOP_ID_LEN > argc) {
					puts("invalid params, missing 10 byte ID...");
					goto _exit;
				}
				shmLgdst_proc->type = CMD1;
				shmLgdst_proc->len = HOP_ID_LEN;
				shmLgdst_proc->tag.wDir = CTRL_OUT;
				shmLgdst_proc->tag.wValue = RADIO_COMM_VAL;
				shmLgdst_proc->tag.wIndex = RADIO_PAIRID_IDX;
        	 char *pc = (char*)shmLgdst_proc->access.hdr.data;
				for (i=0; i<shmLgdst_proc->len; i++) {
					*pc++ = htoi(argv[4+i]);
				} *pc = 0x0;
   				 puts("pairid called"); // liyenho
			}
			else if (!strcasecmp(argv[3], pairLocked)) {
				shmLgdst_proc->type = CMD1;
				shmLgdst_proc->len = RADIO_PAIR_LOCKED_LEN;
				shmLgdst_proc->tag.wDir = CTRL_IN;
				shmLgdst_proc->tag.wValue = RADIO_COMM_VAL;
				shmLgdst_proc->tag.wIndex = RADIO_PAIR_LOCKED_IDX;
			}
			else if (!strcasecmp(argv[3],"bm")) {
				if (5 != argc) {
					perror_exit("lgdst 0 rx bm 1/0 (1:to main, 0:upgrade)",-7);
				}
				uint8_t mode = atoi(argv[4]);
				if (1!=mode && 0!=mode) {
					perror_exit("lgdst 0 rx bm 1/0 (1:to main, 0:upgrade)",-7);
				}
				shmLgdst_proc->type = CMD1;
				shmLgdst_proc->len = sizeof(mode);
				shmLgdst_proc->tag.wDir = CTRL_OUT;
				shmLgdst_proc->tag.wValue = USB_BOOT_APP_VAL;
				shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;
				char *pc = (char*)shmLgdst_proc->access.hdr.data;
					*pc = mode;
			}
			else if (!strcasecmp(argv[3],"Ua")) {
				if (5 != argc) {
					perror_exit("invalid command line parameters, lgdst 0 rx Ua bin-file-path",-3);
				}
				shmLgdst_proc->type = CMD1;
				shmLgdst_proc->len = ((HOST_BUFFER_SIZE*2)<(strlen(argv[4])+1))?(HOST_BUFFER_SIZE*2):(strlen(argv[4])+1);
				shmLgdst_proc->tag.wDir = CTRL_OUT;
				shmLgdst_proc->tag.wValue = USB_ATMEL_UPGRADE_VAL;
				shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;
				char *pc = (char*)shmLgdst_proc->access.hdr.data;
				for (i=0; i<shmLgdst_proc->len-1; i++) {
					*pc++ = argv[4][i];
				} *pc = 0x0;
			}
			else if (!strcasecmp(argv[3],"Uc")) {
				if (5 != argc) {
					perror_exit("invalid command line parameters, lgdst 0 rx Uc cpld-file-path",-4);
				}
				shmLgdst_proc->type = CMD1;
				shmLgdst_proc->len = ((HOST_BUFFER_SIZE*2)<(strlen(argv[4])+1))?(HOST_BUFFER_SIZE*2):(strlen(argv[4])+1);
				shmLgdst_proc->tag.wDir = CTRL_OUT;
				shmLgdst_proc->tag.wValue = USB_CPLD_UPGRADE_VAL;
				shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;
				char *pc = (char*)shmLgdst_proc->access.hdr.data;
				for (i=0; i<shmLgdst_proc->len-1; i++) {
					*pc++ = argv[4][i];
				} *pc = 0x0;
			}
#if false
			else if (!strcasecmp(argv[3],"Vc")) {
		      acs->access = READ_BY_ADDR;
    	  	 	acs->dcnt = 1; // 1 short read on cpld ver
    	  	 	acs->addr = 0x7f;
				RAED_SETUP
			}
#endif
			else {
			  if (!strcasecmp(argv[3],"ns") ||!strcasecmp(argv[3],"s")) {
				  bool stream = (0==strcasecmp(argv[3],"s")) ? 0: 1;
				  if (stream) {
						shmLgdst_proc->type = CMD0;
						shmLgdst_proc->tag.wValue = USB_STREAM_ON_VAL;
						shmLgdst_proc->tag.wIndex = USB_STREAM_IDX;
					}
					else {
						shmLgdst_proc->type = CMD0;
						shmLgdst_proc->tag.wValue = USB_STREAM_OFF_VAL;
						shmLgdst_proc->tag.wIndex = USB_STREAM_IDX;
					}
				}
				else if (!strcasecmp(argv[3], calib)) { // factory cap value calibration for frequency correction
					shmLgdst_proc->type = CMD0;
					shmLgdst_proc->tag.wValue = RADIO_COMM_VAL;
					shmLgdst_proc->tag.wIndex = RADIO_CAL_IDX;
				}
				else if (!strcasecmp(argv[3],"Va")) {
					shmLgdst_proc->type = CMD1;
					shmLgdst_proc->tag.wDir = CTRL_IN;
					shmLgdst_proc->len = 3;
					shmLgdst_proc->tag.wValue = USB_ATMEL_VER_VAL;
					shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;
				}
				else if (!strcasecmp(argv[3], calibqry)) { // query result of factory cap value calibration
					shmLgdst_proc->type = CMD1;
					shmLgdst_proc->tag.wDir = CTRL_IN;
					shmLgdst_proc->len = 2*sizeof(*acs->data); // boolean var + cbv value
					shmLgdst_proc->tag.wValue = RADIO_COMM_VAL;
					shmLgdst_proc->tag.wIndex = RADIO_CAL_DONE_IDX;
				}
			  if (!strcasecmp(argv[3],"r") ||!strcasecmp(argv[3],"w")) {
				  shmLgdst_proc->type = ACS;
					shmLgdst_proc->tag.wIndex =htoi(argv[4]); // ofdm or link proc
    	  	 		int i, tmp, sz = htoi(argv[6]);
		    	  	if (1>sz || (HOST_BUFFER_SIZE+1)<sz) {
			    	 	puts("invalid block size...");
			    	 	goto _exit; }
    	  	 		acs->dcnt = sz; // must be less than HOST_BUFFER_SIZE+1
			    	acs->addr = htoi(argv[5]); // start addr
				  if ( 0==strcasecmp(argv[3],"w")) {
						shmLgdst_proc->tag.wDir = CTRL_OUT;
			    	   for (i=0; i<acs->dcnt; i++) {
				    	   sscanf(argv[7+i], "%x", &tmp);
			    		  acs->data[i] = tmp;
		    		   }
					}
					else { // read access
						shmLgdst_proc->tag.wDir = CTRL_IN;
					}
				}
				else if (!strcasecmp(argv[3], hopless)) { // special radio mode for FCC test
		    	  int32_t sect = htoi(argv[4]);
		    	  if (0>sect || 3<sect) {
			    	 puts("invalid section, 0: disable, 1: low, 2: mid, 3: high");
			     	goto _exit; }
					shmLgdst_proc->type = CMD1;
					shmLgdst_proc->len = sizeof(sect);
					shmLgdst_proc->tag.wDir = CTRL_OUT;
					shmLgdst_proc->tag.wValue = RADIO_COMM_VAL;
					shmLgdst_proc->tag.wIndex = RADIO_HOPLESS_IDX;
					memcpy(shmLgdst_proc->access.hdr.data, &sect, sizeof(sect));
				}
				else if (!strcasecmp(argv[3], locGPS)) {
					// get base station gps
	    	        if (4+(BASE_GPS_LEN/sizeof(float)) > argc) {
						puts("Invalid number of params, requires latitude and longitude...");
						goto _exit;
					}

					float latitude = atof(argv[4]);
					float longitude = atof(argv[5]);
	    	      	//check that supplied values are within the correct range
	    	        if ((-180>latitude) || (180<latitude) ) {
 						puts("Invalid latitude: must be within (-180,180)..");
		     			goto _exit;
		     		}
		     		 if ((-180>longitude) || (180<longitude) ) {
 						puts("Invalid longitude: must be within (-180,180)..");
		     			goto _exit;
		     		}

					shmLgdst_proc->type = CMD1;
					shmLgdst_proc->len = BASE_GPS_LEN;
					shmLgdst_proc->tag.wDir = CTRL_OUT;
					shmLgdst_proc->tag.wValue = RADIO_COMM_VAL;
					shmLgdst_proc->tag.wIndex = BASE_GPS_IDX;

 					float *pc = (float*)shmLgdst_proc->access.hdr.data;
					for (i=0; i<BASE_GPS_LEN/sizeof(float); i++) {
						*pc = atof(argv[4+i]);
						*pc++;
					}
					*pc = 0x0;
			    }
			      else if (!strcasecmp(argv[3], RSSI)){
					puts("Getting RSSI\n");
			    	shmLgdst_proc->type = CMD1;
					shmLgdst_proc->tag.wDir = CTRL_IN;
					shmLgdst_proc->len =4;
					shmLgdst_proc->tag.wValue = RADIO_COMM_VAL;
					shmLgdst_proc->tag.wIndex = RADIO_GET_RSSI_IDX;
			    }
			    else if (!strcasecmp(argv[3], setFEC)){
					if (4+1 > argc) {
						puts("invalid params, missing FEC status...");
						goto _exit;
					}
					printf("Setting FEC status\n");
					uint8_t FEC_option = htoi(argv[4]);

					shmLgdst_proc->type = CMD1;
					shmLgdst_proc->len = SET_FEC_LEN;
					shmLgdst_proc->tag.wDir = CTRL_OUT;
					shmLgdst_proc->tag.wValue = RADIO_COMM_VAL;
					shmLgdst_proc->tag.wIndex = SET_FEC_IDX;
					memcpy(shmLgdst_proc->access.hdr.data, &FEC_option, 1);
				}
				else if (!strcasecmp(argv[3], SiGetProp)){
					//get Si4463 properties
					if (4+3 > argc) {
						puts("invalid number of params, usuage is SiGetProp group num_props start_idx ");
						goto _exit;
					}

					shmLgdst_proc->type = CMD1;
					shmLgdst_proc->len = RADIO_GET_PROPERTY_HOST_LEN;
					shmLgdst_proc->tag.wDir = CTRL_OUT;
					shmLgdst_proc->tag.wValue = RADIO_COMM_VAL;
					shmLgdst_proc->tag.wIndex = RADIO_GET_PROPERTY_IDX;

					char *pc = (char*)shmLgdst_proc->access.hdr.data;
					for (i=0; i<shmLgdst_proc->len; i++) {
						*pc++ = htoi(argv[4+i]);
					} *pc = 0x0;

					printf("Sending property request to Atmel\n");
				}
				else if (!strcasecmp(argv[3], setCtrlPwr)){
					uint8_t pwr_lvl = htoi(argv[4]);
					const uint8_t max_pwr = 0x7f;
					if (max_pwr < pwr_lvl){
						printf("Error: requested power is too high!\n");
						printf("Maximum value is %#X\n", max_pwr);
						goto _exit;
					}

					puts("Setting Control Radio Power\n");
					printf("%u\n", pwr_lvl);
			    	shmLgdst_proc->type = CMD1;
					shmLgdst_proc->len = 1;
					shmLgdst_proc->tag.wDir = CTRL_OUT;
					shmLgdst_proc->tag.wValue = RADIO_COMM_VAL;
					shmLgdst_proc->tag.wIndex = RADIO_SET_PWR_LVL_IDX;
					memcpy(shmLgdst_proc->access.hdr.data, &pwr_lvl, 1);

				}
				else if (!strcasecmp(argv[3], use915)){
					printf("Switching to 915 MHz\n");
					shmLgdst_proc->type = CMD1;
					shmLgdst_proc->tag.wValue = RADIO_COMM_VAL;
					shmLgdst_proc->tag.wIndex = RADIO_SET_REGIME_915_IDX; // current cdc data interface
				}
				else if (!strcasecmp(argv[3], use869)){
					printf("Switching to 869 MHz\n");
					shmLgdst_proc->type = CMD1;
					shmLgdst_proc->tag.wValue = RADIO_COMM_VAL;
					shmLgdst_proc->tag.wIndex = RADIO_SET_REGIME_869_IDX; // current cdc data interface
				}
				else if (!strcasecmp(argv[3], setRadioChannel)){
					if (4+1 > argc) {
						puts("Error, channel not specified!\n");
						goto _exit;
					}
					uint8_t channel = htoi(argv[4]);

					printf("Setting Control Radio Channel: %u\n", channel);

			    	shmLgdst_proc->type = CMD1;
					shmLgdst_proc->len = RADIO_SET_CHANNEL_LEN;
					shmLgdst_proc->tag.wDir = CTRL_OUT;
					shmLgdst_proc->tag.wValue = RADIO_COMM_VAL;
					shmLgdst_proc->tag.wIndex = RADIO_SET_CHANNEL_IDX;
					memcpy(shmLgdst_proc->access.hdr.data, &channel, 1);

				}




			}
     }
_read:
	shmLgdst_proc->active = 1;  // wait for transaction complete
	while (1==shmLgdst_proc->active) ;
	  {
		  if (false/*rx*/==work_mode && !strcasecmp(argv[3], pairLocked)) {
				  uint8_t *pv = (uint8_t*)acs->data;
				  printf("Pairing Ctrl-Lock is %s\n", (1==*pv)?"On":"Off");
				if (ret)
					*(uint8_t**)ret = pv;
		  }
			else if (!strcasecmp(argv[3],"Cst")) {
				ctrl_radio_stats *pv = (uint8_t*)acs->data;
				{
					print_ctrl_bits(pv->bw_ctrl_bits, "bw_ctrl_bits");
		     	 	for (i=0; i<CTRL_CTX_LEN; i++) {
					  char tag[32] ;
			     	 	sprintf(tag, "ctrl_bits_ctx[%d]", i);
		     	 		print_ctrl_bits(pv->ctrl_bits_ctx[i], tag);
		     	 	}
		     	 	printf("errPerAcc = %d\n", pv->errPerAcc);
		     	 	printf("loop_cnt = %d\n", pv->loop_cnt);
	     	 	}
				if (ret)
					*(ctrl_radio_stats**)ret = pv;
			}
			else if (!strcasecmp(argv[3], MDst)) {
				si446x_get_modem_status *pv = (uint8_t*)acs->data;
					//printf("Ctrl Modem Intr pending bits = 0x%02x\n", pv->MODEM_PEND);
					printf("Ctrl Modem Status bits = 0x%02x\n", pv->MODEM_STATUS);
					int16_t curr_dBm = pv->CURR_RSSI/2 - pv->RSSI_COMP - 70;
					printf("Ctrl Modem current RF dBm measure = %d\n", curr_dBm);
					int16_t latched_dBm = pv->LATCH_RSSI/2 - pv->RSSI_COMP - 70;
					printf("Ctrl Modem RF dBm last measured = %d\n", latched_dBm);
					int16_t ant1_dBm = pv->ANT1_RSSI/2 - pv->RSSI_COMP - 70;
					printf("Ctrl Modem RF dBm from antenna 1 = %d\n", ant1_dBm);
					int16_t ant2_dBm = pv->ANT2_RSSI/2 - pv->RSSI_COMP - 70;
					printf("Ctrl Modem RF dBm from antenna 2 = %d\n", ant2_dBm);
					printf("Ctrl Modem AFC offset = %d\n", pv->AFC_FREQ_OFFSET);
					if (ret)
						*(si446x_get_modem_status**)ret = pv;
			}
			else if (!strcasecmp(argv[3], tempCMD)) {
				int16_t temp, *pv = (int16_t*)acs->data;
				if (1358<=*pv && 1722>=*pv) {
					temp = (899.0/4096.0)*(*pv) - 293;
					printf("current Si4463 temperature = %d C\n", temp);
				} else
				puts("current Si4463 temperature reading is bad");
				if (ret)
					*(int16_t*)ret = temp;
			}
#if false
		  else if (false/*rx*/==work_mode && !strcasecmp(argv[3],"Vc")) {
					short_sleep(0.1); // wait 0.1 sec before readback
					shmLgdst_proc->type = ACS;
					shmLgdst_proc->tag.wDir = CTRL_IN;
					shmLgdst_proc->tag.wValue = USB_HOST_MSG_RX_VAL;
					shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;

					shmLgdst_proc->active = 1; // process next ctrl xfer
					while(1==shmLgdst_proc->active) ;

		     	 for (i=0; i<1; i++) {
		   	 	printf("xxxxxx CPLD ver[%d] 0x%02x received xxxxxx\n", i, (uint8_t)acs->data[i]);
	   	 	}
				if (ret)
					*(uint16_t**)ret = acs->data;
			}
#endif
			else if (!strcasecmp(argv[3],"Va")) {
				uint8_t *pv = (uint8_t*)acs->data;
		     	 for (i=0; i<shmLgdst_proc->len; i++)
		   	 	printf("xxxxxx ATMEL ver[%d] 0x%02x received xxxxxx\n", i, pv[i]);
				if (ret)
					*(uint16_t**)ret = acs->data;
			}
			else if (!strcasecmp(argv[3],"calib-qry")) {
				bool done = (bool)*(acs->data);
				if (done)
		   		printf("Cap bank tuning process is done, cbv = 0x%02x\n",*(acs->data+1));
		   	else
			   	printf("Cap bank tuning process is not done\n");
				if (ret)
					*(bool*)ret = done;
			}
			else if (!strcasecmp(argv[3], SiGetProp)) {

				shmLgdst_proc->type = CMD1;
				shmLgdst_proc->len = RADIO_GET_PROPERTY_ATMEL_LEN;
				shmLgdst_proc->tag.wDir = CTRL_IN;
				shmLgdst_proc->tag.wValue = RADIO_COMM_VAL;
				shmLgdst_proc->tag.wIndex = RADIO_GET_PROPERTY_REPLY_IDX;

				printf("Requesting reply from Atmel\n");
				shmLgdst_proc->active = 1;  // wait for transaction complete
				while (1==shmLgdst_proc->active) ;


				uint8_t * prop_vals = (uint8_t*)acs->data;
				printf("The requested properties: \n");
				for (int i =0; i<RADIO_GET_PROPERTY_ATMEL_LEN;i++){
					printf("%#0X\n", *(prop_vals+i));
				}
				if (ret)
					*(uint8_t**)ret = acs->data;
			}
			else if (!strcasecmp(argv[3], RSSI)){
				uint8_t * val = (uint8_t*)acs->data;
				printf("RSSI value: %u\n", *val);
				if (ret)
					*(uint8_t**)ret = acs->data;
			}
			else if (false/*rx*/==work_mode && !strcasecmp(argv[3],"r")) {
				uint32_t adr =acs->addr ;
		     	 for (i=0; i<acs->dcnt; i++)
		   	 	printf("xxxxxx 0x%02x received @ 0x%02x xxxxxx\n",
		   	 					acs->data[i], adr+i);
				 if (ret)
				 	*(uint16_t**)ret = acs->data;
			}
     }
   #endif
_exit0:
   get_time(&tend);
   time_diff(&tend, &tstart, &tdelta);
   print_time(tdelta);
_exit:
	fprintf(stderr, "Wrap up/Leaving fpga_access app...\n");
	if (0 > do_exit) return do_exit;  // return from error
   else do_exit = 4;
#ifndef LIB
  if (shmLgdst_proc)
  	shmdt(shmLgdst_proc);
#endif
	// now we exit, thank you......
	return 0;
}
