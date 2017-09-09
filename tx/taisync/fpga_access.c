#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <math.h>
#include <libusb.h>
#include <assert.h>

 #define CTRL_OUT		(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT)
 #define CTRL_IN		(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN)

  #include "fpga_access.h"  // definitions for host/fpga access @ tx
  #define COMM_ATMEL_DEV // enable atmel<->fpga/6612 comm test

#define true													1
#define false													0
 #define print_usage                    puts("lgdst 0 tx bm/Cst/Va/fbm/[Uf]/pair-id/pair-locked/loc-gps/droneyaw/camyaw/[rfch]/[atten]/RSSI fpath/adr [chidx] [atten] [bsz] [val0,val1,...], all numbers are in hex");
#define RAED_SETUP	\
							shmLgdst_proc->type = ACS; \
							shmLgdst_proc->tag.wDir = CTRL_OUT; \
							shmLgdst_proc->tag.wValue = USB_HOST_MSG_TX_VAL; \
							shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX; \
    	  	 				goto _read;

 int shmid_Lgdst = -1;
 volatile ipcLgdst *shmLgdst_proc = 0;
static volatile int do_exit = 0;	// main loop breaker
static bool work_mode = (bool)-1;
static rf_params Rf_Params;

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
bool open_tx_ini(uint16_t *setting_ch,uint16_t *setting_pwr)
{
	FILE *fp = NULL;
        char ini_buff[120];

        fp = fopen("usb_tx.ini", "r+");
	if (fp == NULL) {
	  // File doesn't exist, setup ini with default
       	  fp = fopen("usb_tx.ini", "w+");
	  if (fp == NULL) {
		printf("ERROR: Fail to creat 'usb_tx.ini' !!");
		return false;
	  }
	  fprintf(fp, "TX_CH=%d\n", *setting_ch);
	  fprintf(fp, "TX_PWR=%d\n", *setting_pwr);
	  fclose(fp);
          return true;
	}
	int ch, pwr;
        while(fgets(ini_buff, 120, fp) != NULL)
        {
           if (strstr(ini_buff, "TX_CH") > 0)
           {
               sscanf(ini_buff, "TX_CH=%d", &ch);
               *setting_ch = ch;
           }
	   if (strstr(ini_buff, "TX_PWR") > 0)
           {
               sscanf(ini_buff, "TX_PWR=%d", &pwr);
               *setting_pwr = pwr;
           }
        }
        printf("setting_ch = %d  setting_pwr  = %d\n", *setting_ch
, *setting_pwr);
	fclose(fp);
	return true;
}

bool update_ini(int mode,uint16_t *setting_ch,uint16_t *setting_att)
{
	FILE *fp;
    	char *myLine;
    	int maximumLineLength = 128;

       fp = fopen("usb_tx.ini", "w+");
	  if (fp == NULL) {
		printf("ERROR: Fail to create 'usb_tx.ini' !!");
		return false;
	  }
	  if (mode) {// Tx
	    //if (setting_ch)
	  		fprintf(fp, "TX_CH=%d\n", *setting_ch);
	    //if (setting_att)
	  		fprintf(fp, "TX_PWR=%d\n", *setting_att);
  		}
  		else {
	    //if (setting_ch)
	  		fprintf(fp, "6612_CH=%d\n", *setting_ch);
		}
	  fclose(fp);
          return true;
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
  int lgdst_access_tx(int argc,char **argv)
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
		printf("Getting memory\n");
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
  	static int32_t sz, adr, tmp; // access buffer

  	dev_access *acs = (dev_access*)&shmLgdst_proc->access.hdr;
	while(-1 != shmLgdst_proc->active);
	get_time(&tstart);

   #ifdef COMM_ATMEL_DEV
     { 	// prepend access header
			if (3+1>argc || strcasecmp(argv[3],"rfch") &&
	        strcasecmp(argv[3],"atten") &&
	        strcasecmp(argv[3],"Us") /*stm32*/&&
	        strcasecmp(argv[3],"Uf") /*fpga*/&&
#ifdef DBG_BOOTSTRAP_BYPASS
	        strcasecmp(argv[3],"bm") /*atm boot mode*/&&
#endif
	        strcasecmp(argv[3],"Ua") /*atmel*/&&
	        strcasecmp(argv[3],"Va") /*atmel*/&&
	        strcasecmp(argv[3],"Cst") &&
	        strcasecmp(argv[3],"pair-id") &&
	        strcasecmp(argv[3],"pair-locked")&&
			strcasecmp(argv[3],"loc-gps")&&
			strcasecmp(argv[3],"droneyaw")&&
			strcasecmp(argv[3],"camyaw")&&
			strcasecmp(argv[3],"RSSI"))
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
				goto _read;
			}
	   else if ((true/*Tx*/==work_mode) && !strcasecmp(argv[3],"rfch")) {
		   	if (5 != argc) {
			   	puts("invalid or no param passed, usage: lgdst 0 tx rfch ch");
			   	goto _exit;
		   	}
		    	open_tx_ini(&Rf_Params.params_tx.chan_idx,
		    								&Rf_Params.params_tx.pwr_att);
		    	Rf_Params.params_tx.chan_idx = htoi(argv[4]);
		    	if (1>Rf_Params.params_tx.chan_idx || 6<Rf_Params.params_tx.chan_idx){
			    	puts("invalid channel index, between [1, 6]");
			    	Rf_Params.params_tx.chan_idx = 0;
		    	}
				update_ini(1, &Rf_Params.params_tx.chan_idx,
										&Rf_Params.params_tx.pwr_att);
				shmLgdst_proc->type = CMD1;
				shmLgdst_proc->len = sizeof(Rf_Params.params_tx);
				shmLgdst_proc->tag.wDir = CTRL_OUT;
				shmLgdst_proc->tag.wValue = RF_TX_FREQ_VAL;
				shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;
				memcpy(shmLgdst_proc->access.hdr.data, &Rf_Params.params_tx, sizeof(Rf_Params.params_tx));
				goto _read;
		}
	    else if ((true/*Tx*/==work_mode) && !strcasecmp(argv[3],"atten")) {
		    	open_tx_ini(&Rf_Params.params_tx.chan_idx,
		    								&Rf_Params.params_tx.pwr_att);
		    	Rf_Params.params_tx.pwr_att = htoi(argv[4]);
		    	if (0>Rf_Params.params_tx.pwr_att ){
			    	puts("invalid power attenuation...");
			    	goto _exit;
		    	}
				update_ini(1, &Rf_Params.params_tx.chan_idx,
										&Rf_Params.params_tx.pwr_att);
				shmLgdst_proc->type = CMD1;
				shmLgdst_proc->len = sizeof(Rf_Params.params_tx);
				shmLgdst_proc->tag.wDir = CTRL_OUT;
				shmLgdst_proc->tag.wValue = RF_TX_ATTN_VAL;
				shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;
				memcpy(shmLgdst_proc->access.hdr.data, &Rf_Params.params_tx, sizeof(Rf_Params.params_tx));
				goto _read;
		}
			else if (!strcasecmp(argv[3],"pair-id")) {
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
				goto _read;
			}
			else if (!strcasecmp(argv[3],"pair-locked")) {
				shmLgdst_proc->type = CMD1;
				shmLgdst_proc->len = RADIO_PAIR_LOCKED_LEN;
				shmLgdst_proc->tag.wDir = CTRL_IN;
				shmLgdst_proc->tag.wValue = RADIO_COMM_VAL;
				shmLgdst_proc->tag.wIndex = RADIO_PAIR_LOCKED_IDX;
				goto _read;
			}
#ifdef DBG_BOOTSTRAP_BYPASS
			else if (!strcasecmp(argv[3],"bm")) {
				if (5 != argc ) {
					perror_exit("lgdst 0 tx bm 1/0 (1:to main, 0:upgrade)",-7);
				}
				uint8_t mode = htoi(argv[4]);
				if (1!=mode && 0!=mode) {
					perror_exit("lgdst 0 tx bm 1/0 (1:to main, 0:upgrade)",-7);
				}
				shmLgdst_proc->type = CMD1;
				shmLgdst_proc->len = sizeof(mode);
				shmLgdst_proc->tag.wDir = CTRL_OUT;
				shmLgdst_proc->tag.wValue = USB_BOOT_APP_VAL;
				shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;
				char *pc = (char*)shmLgdst_proc->access.hdr.data;
					*pc = mode;
					goto _read;
			}
#endif
			else if (!strcasecmp(argv[3],"Ua")) {
				if (5 != argc) {
					perror_exit("invalid command line parameters, lgdst 0 tx Ua bin-file-path",-3);
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
					goto _read;
			}
			else if (true/*tx*/==work_mode && !strcasecmp(argv[3],"Uf")) {
				if (6 != argc) {
					perror_exit("invalid command line parameters, lgdst 0 tx Uf rbf-file-path hex-file-path",-2);
				}
				shmLgdst_proc->type = CMD1;
				assert((HOST_BUFFER_SIZE*2) >= (strlen(argv[4])+1 + strlen(argv[5])+1));
				shmLgdst_proc->len = (strlen(argv[4])+1 + strlen(argv[5])+1);
				shmLgdst_proc->tag.wDir = CTRL_OUT;
				shmLgdst_proc->tag.wValue = USB_FPGA_UPGRADE_VAL;
				shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;
				char *pc = (char*)shmLgdst_proc->access.hdr.data;
				for (i=0; i<strlen(argv[4]); i++) {
					*pc++ = argv[4][i];
				} *pc++ = 0x0;
				for (i=0; i<strlen(argv[5]); i++) {
					*pc++ = argv[5][i];
				} *pc = 0x0;
					goto _read;
			}
			else if (false/*rx*/==work_mode && !strcasecmp(argv[3],"Us")) {
				if (5 != argc) {
					perror_exit("invalid command line parameters, lgdst 0 rx Us cpld-file-path",-4);
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
					goto _read;
			}
			else {
				if (!strcasecmp(argv[3],"Va")) {
					shmLgdst_proc->type = CMD1;
					shmLgdst_proc->tag.wDir = CTRL_IN;
					shmLgdst_proc->len = 3;
					shmLgdst_proc->tag.wValue = USB_ATMEL_VER_VAL;
					shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;
				}
				else if (!strcasecmp(argv[3],"loc-gps")) { // get base station gps
	    	        if (4+(DRONE_GPS_LEN/sizeof(float)) > argc) {
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
					shmLgdst_proc->len = DRONE_GPS_LEN;
					shmLgdst_proc->tag.wDir = CTRL_OUT;
					shmLgdst_proc->tag.wValue = RADIO_COMM_VAL;
					shmLgdst_proc->tag.wIndex = DRONE_GPS_IDX;

 					float *pc = (float*)shmLgdst_proc->access.hdr.data;
					for (i=0; i<DRONE_GPS_LEN/sizeof(float); i++) {
						*pc = atof(argv[4+i]);
						*pc++;
					}
					*pc = 0x0;
					goto _read;

			    }
				else if (!strcasecmp(argv[3],"droneyaw")) { // get drone yaw
	    	        float yaw = atof(argv[4]);
	    	        if ( (-360>yaw) || (360<yaw)) {
 					puts("Invalid drone yaw value, must be within (-360,360)");
		     		goto _exit; }


					shmLgdst_proc->type = CMD1;
					shmLgdst_proc->len = DRONE_YAW_LEN;
					shmLgdst_proc->tag.wDir = CTRL_OUT;
					shmLgdst_proc->tag.wValue = RADIO_COMM_VAL;
					shmLgdst_proc->tag.wIndex = DRONE_YAW_IDX;
					memcpy(shmLgdst_proc->access.hdr.data, &yaw, DRONE_YAW_LEN);

			    }
				else if (!strcasecmp(argv[3],"camyaw")) { // get camera yaw
	    	        float yaw = atof(argv[4]);
	    	        if ( (-360>yaw) || (360<yaw)){
 					puts("Invalid camera yaw value, must be within (-360,360)");
		     		goto _exit; }


					shmLgdst_proc->type = CMD1;
					shmLgdst_proc->len = CAMERA_YAW_LEN;
					shmLgdst_proc->tag.wDir = CTRL_OUT;
					shmLgdst_proc->tag.wValue = RADIO_COMM_VAL;
					shmLgdst_proc->tag.wIndex = CAMERA_YAW_IDX;
					memcpy(shmLgdst_proc->access.hdr.data, &yaw, CAMERA_YAW_LEN);

			    }

			    else if (!strcasecmp(argv[3],"RSSI")){
					puts("Getting RSSI\n");
			    	shmLgdst_proc->type = CMD1;
					shmLgdst_proc->tag.wDir = CTRL_IN;
					shmLgdst_proc->len =4;
					shmLgdst_proc->tag.wValue = RADIO_COMM_VAL;
					shmLgdst_proc->tag.wIndex = RADIO_GET_RSSI_IDX;
			    }
				goto _read;
			}
    	  adr = htoi(argv[4]);
    	  if (0>adr || (HOST_BUFFER_SIZE)<adr) {
	    	 puts("invalid address...");
	     	goto _exit; }
    	  acs->addr = adr;
     }
_read:
	shmLgdst_proc->active = 1;  // wait for transaction complete
	while (1==shmLgdst_proc->active) ;
	  {
	      if (!strcasecmp(argv[3],"pair-locked")) {
				  uint8_t *pv = (uint8_t*)acs->data;
				  printf("Pairing Ctrl-Lock is %s\n", (1==*pv)?"On":"Off");
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
			}
			else if (!strcasecmp(argv[3],"Va")) {
				uint8_t *pv = (uint8_t*)acs->data;
		     	 for (i=0; i<shmLgdst_proc->len; i++)
		   	 	printf("xxxxxx ATMEL ver[%d] 0x%02x received xxxxxx\n", i, pv[i]);
			}
			else if (!strcasecmp(argv[3], "RSSI")){
				uint8_t* val = (uint8_t*)acs->data;
				printf("Latched RSSI Value: %u\n", *val);
				printf("FRR B: %u\n", *(val+1));
				printf("FRR C:%u\n", *(val+2));
				printf("FRR D:%u\n", *(val+3));
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
