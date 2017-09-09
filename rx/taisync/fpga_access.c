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
#define print_usage                    puts("lgdst 0 rx bm/Va/[Us]/[Uf]/pair-id/pair-locked/loc-gps//setFEC/RSSI/setCtrlPwr/use915/use869/fpath [chidx] [bsz] [val0,val1,...], all numbers are in hex");
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
			if (3+1>argc || strcasecmp(argv[3],"Cst") &&
				strcasecmp(argv[3],"Us") /*stm32*/&&
	        strcasecmp(argv[3],"Uf") /*fpga*/&&
	        strcasecmp(argv[3],"Va") /*atmel*/&&
			  strcasecmp(argv[3],"bm") /*atm boot mode*/&&
	        strcasecmp(argv[3], pairID) &&
	        strcasecmp(argv[3], pairLocked)&&
			strcasecmp(argv[3], locGPS)&&
			strcasecmp(argv[3], RSSI)&&
			strcasecmp(argv[3], setFEC)&&
			strcasecmp(argv[3], setCtrlPwr)&&
			strcasecmp(argv[3], use869)&&
			strcasecmp(argv[3], use915))
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
			else if (!strcasecmp(argv[3],"Uf")) {
				if (5 != argc) {
					perror_exit("invalid command line parameters, lgdst 0 rx Us fpga-file-path",-4);
				}
				shmLgdst_proc->type = CMD1;
				shmLgdst_proc->len = ((HOST_BUFFER_SIZE*2)<(strlen(argv[4])+1))?(HOST_BUFFER_SIZE*2):(strlen(argv[4])+1);
				shmLgdst_proc->tag.wDir = CTRL_OUT;
				shmLgdst_proc->tag.wValue = USB_FPGA_UPGRADE_VAL;
				shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;
				char *pc = (char*)shmLgdst_proc->access.hdr.data;
				for (i=0; i<shmLgdst_proc->len-1; i++) {
					*pc++ = argv[4][i];
				} *pc = 0x0;
			}
			else if (!strcasecmp(argv[3],"Us")) {
				if (5 != argc) {
					perror_exit("invalid command line parameters, lgdst 0 rx Us stm32-file-path",-4);
				}
				shmLgdst_proc->type = CMD1;
				shmLgdst_proc->len = ((HOST_BUFFER_SIZE*2)<(strlen(argv[4])+1))?(HOST_BUFFER_SIZE*2):(strlen(argv[4])+1);
				shmLgdst_proc->tag.wDir = CTRL_OUT;
				shmLgdst_proc->tag.wValue = USB_STM_UPGRADE_VAL;
				shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;
				char *pc = (char*)shmLgdst_proc->access.hdr.data;
				for (i=0; i<shmLgdst_proc->len-1; i++) {
					*pc++ = argv[4][i];
				} *pc = 0x0;
			}
			else {
				if (!strcasecmp(argv[3],"Va")) {
					shmLgdst_proc->type = CMD1;
					shmLgdst_proc->tag.wDir = CTRL_IN;
					shmLgdst_proc->len = 3;
					shmLgdst_proc->tag.wValue = USB_ATMEL_VER_VAL;
					shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;
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
			else if (!strcasecmp(argv[3],"Va")) {
				uint8_t *pv = (uint8_t*)acs->data;
		     	 for (i=0; i<shmLgdst_proc->len; i++)
		   	 	printf("xxxxxx ATMEL ver[%d] 0x%02x received xxxxxx\n", i, pv[i]);
				if (ret)
					*(uint16_t**)ret = acs->data;
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
