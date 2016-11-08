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

 #define CTRL_OUT		(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT)
 #define CTRL_IN		(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN)


#define true													1
#define false													0
#define print_usage                    puts("lgdst 0 rx [Vc]/Va/[Uc]/ns/s/wbs/ws/wb/w/rb/r/[vscan]/[retune]/[recv]/[sig]/[locked]/[rfch]/temp/ctune/ fpath/adr [chidx] [bsz] [val0,val1,...], all numbers are in hex");
#define RAED_SETUP	\
							shmLgdst_proc->type = ACS; \
							shmLgdst_proc->tag.wDir = CTRL_OUT; \
							shmLgdst_proc->tag.wValue = USB_HOST_MSG_TX_VAL; \
							shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX; \
    	  	 				goto _read;
typedef int bool;  // don't remove or modify this, please

extern  volatile ipcLgdst *shmLgdst_proc;
static int do_exit = 0;	// main loop breaker
static bool work_mode = (bool)-1;
static rf_params Rf_Params;
#define LIB
#define REC
static void at_exit(int status) {
	if (do_exit) {
		do_exit = -1;
  }
  exit(status);
}

static void perror_exit(char *message, int status)
{
  char fail[80], cs[8];
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
static bool open_tx_ini(uint16_t *setting_ch,uint16_t *setting_pwr)
{
	FILE *fp = NULL;
        char ini_buff[120];

        fp = fopen("usb_rx.ini.h", "r+");
	if (fp == NULL) {
	  // File doesn't exist, setup ini with default
       	  fp = fopen("usb_rx.ini.h", "w+");
	  if (fp == NULL) {
		printf("ERROR: Fail to creat 'usb_rx.ini.h' !!");
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

static bool update_ini(int mode,uint16_t *setting_ch,uint16_t *setting_att)
{
	FILE *fp;
    	char *myLine;
    	int maximumLineLength = 128;

       fp = fopen("usb_rx.ini.h", "w+");
	  if (fp == NULL) {
		printf("ERROR: Fail to create 'usb_rx.ini.h' !!");
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

static void video_chan_switch(bool fast, int32_t ch, dev_access *acs) {
	int i, size = ARRAY_SIZE(ch);
   dev_cfg *regs_6612 = GET_ARRAY(ch);

	shmLgdst_proc->type = CMD1;
	shmLgdst_proc->len = sizeof(size);
	shmLgdst_proc->tag.wDir = CTRL_OUT;
	if (fast)
		shmLgdst_proc->tag.wValue = USB_6612_SIZE_VAL_F;
	else // normal mode
		shmLgdst_proc->tag.wValue = USB_6612_SIZE_VAL;
	shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;
	memcpy(shmLgdst_proc->access.hdr.data, &size, sizeof(size));
	shmLgdst_proc->active = 1;  // wait for transaction complete
	while (1==shmLgdst_proc->active) ;

	dev_access echo; // atmel shall echo write cmd hdr back
   acs->dcnt = 1;  // send cfg val one by one

#define WRITE_SETUP(i)	\
			acs->access = WRITE_BY_ADDR; \
	    	acs->addr = regs_6612[i].addr; \
	    	*acs->data = regs_6612[i].data; \
	\
			shmLgdst_proc->type = ACS; \
			shmLgdst_proc->tag.wDir = CTRL_OUT; \
			shmLgdst_proc->tag.wValue = USB_HOST_MSG_TX_VAL; \
			shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX; \
	\
			shmLgdst_proc->active = 1; \
			while(1==shmLgdst_proc->active) ; /*complete for next transaction*/ \
	\
			memcpy(&echo, acs, sizeof(*acs)); /*keep original copy*/ \
			short_sleep(0.05); /*validate echo after 0.05 sec*/ \
			shmLgdst_proc->echo = true; \
			shmLgdst_proc->active = 1; \
	\
			while(shmLgdst_proc->echo || 1==shmLgdst_proc->active) ; \
	\
	 		if (echo.access != acs->access) \
	 			printf("xxxxxx invalid access mode echoed, " \
	 				"(sent(%d), echo(%d)) xxxxxxx\n", echo.access, acs->access); \
			if (echo.dcnt != acs->dcnt) \
	 			printf("xxxxxx invalid data count echoed, " \
	 				"(sent(%d), echo(%d)) xxxxxx\n", echo.dcnt, acs->dcnt); \
			printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",echo.data[0],echo.addr);

   if (fast) { // only two registers written, don't really work this way...
   	WRITE_SETUP(2)
   	WRITE_SETUP(23)
	} else {
	   for (i=0; i<size; i++) {
		   WRITE_SETUP(i)
		}
	}
}

static void query_video_short_rx(dev_access *acs) {
	memset(acs, 0x0, sizeof(*acs)-sizeof(uint16_t));
	acs->access = SIGNAL_STATUS;
	acs->dcnt = sizeof(Short_Statistics_ST)/sizeof(uint16_t);
	shmLgdst_proc->type = ACS;
	shmLgdst_proc->tag.wDir = CTRL_OUT;
	shmLgdst_proc->tag.wValue = USB_HOST_MSG_TX_VAL;
	shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;
}
static void read_video_short_rx(dev_access *acs) {
	shmLgdst_proc->type = ACS;
	shmLgdst_proc->tag.wDir = CTRL_IN;
	shmLgdst_proc->tag.wValue = USB_HOST_MSG_RX_VAL;
	shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;
	acs->dcnt = sizeof(Short_Statistics_ST)/sizeof(uint16_t);

	shmLgdst_proc->active = 1; // process next ctrl xfer
	while(1==shmLgdst_proc->active) ;
}

#ifdef LIB
  int lgdst_access_rx_vid(int argc,char **argv, void **ret)
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
	while (0>=shmLgdst_proc) ; //

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
  	static int32_t sz, fc, adr, tmp; // access buffer

  	dev_access *acs = (dev_access*)&shmLgdst_proc->access.hdr;
	while(-1 != shmLgdst_proc->active);
	get_time(&tstart);

   #ifdef COMM_ATMEL_DEV
     { 	// prepend access header
			if (3+1>argc ||(strcasecmp(argv[3],"wb") && strcasecmp(argv[3],"wbs") &&
				strcasecmp(argv[3],"w") && strcasecmp(argv[3],"ws") &&
				strcasecmp(argv[3],"rb") && strcasecmp(argv[3],"r") &&
				strcasecmp(argv[3],"ns") && strcasecmp(argv[3],"s") &&
	        /*strcasecmp(argv[3],"existed") &&*/
	        strcasecmp(argv[3],"sig") &&
	        /*strcasecmp(argv[3],"tran") &&*/
	        strcasecmp(argv[3],"recv") &&
	        strcasecmp(argv[3],"locked") &&
	        strcasecmp(argv[3],"rfch") &&
	        strcasecmp(argv[3],"atten") &&
	        strcasecmp(argv[3],"vscan") &&
	        strcasecmp(argv[3],"retune") &&
	        strcasecmp(argv[3],"Uc") /*cpld*/&&
	        strcasecmp(argv[3],"Uf") /*fpga*/&&
	        strcasecmp(argv[3],"Ua") /*atmel*/&&
	        strcasecmp(argv[3],"Vn") /*nois*/&&
	        strcasecmp(argv[3],"Vf") /*fpga*/&&
	        strcasecmp(argv[3],"Vc") /*cpld*/&&
	        strcasecmp(argv[3],"Va") /*atmel*/&&
	        strcasecmp(argv[3],"temp") &&
	        strcasecmp(argv[3],"ctune")))
			{
    	  	  	puts("invalid access mode...");
					print_usage
    	  	  	goto _exit;
			}
			else if (false/*rx*/==work_mode && !strcasecmp(argv[3],"vscan")) {
				Short_Statistics_ST *pret = NULL;
				if (ret)
					pret = *(Short_Statistics_ST**)ret ;
				for (i=0; i<AVAILABLE_VCHANS; i++) {
					video_chan_switch(true/*false*/, i, acs);
					short_sleep(/*1*/0.5);  // wait a bit to inquire rx channel status

					query_video_short_rx(acs);
					shmLgdst_proc->active = 1;  // wait for transaction complete
					while (1==shmLgdst_proc->active) ;
					read_video_short_rx(acs);

					Short_Statistics_ST *psts = &acs->data[0];
						printf("DVBT Ch%d Signal Status,\n"
									"\tInBandPwr = %d,\n"
									"\tSNR = %d,\n",
									i, // current video channel
									psts->InBandPwr,
									psts->SNR);
					if (pret) {
						*pret++ = *psts;
					}
				}
				int32_t size = 0;  // to finish vchan scan process
				shmLgdst_proc->type = CMD1;
				shmLgdst_proc->len = sizeof(size);
				shmLgdst_proc->tag.wDir = CTRL_OUT;
				shmLgdst_proc->tag.wValue = USB_6612_SIZE_VAL_F;
				shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;
				memcpy(shmLgdst_proc->access.hdr.data, &size, sizeof(size));
				shmLgdst_proc->active = 1;  // wait for transaction complete
				while (1==shmLgdst_proc->active) ;

				FILE *fp = NULL;
			        char ini_buff[120];

			        fp = fopen("usb_rx.ini.h", "r+");
				  if (fp == NULL) {
					puts("ERROR: Fail to open 'usb_rx.ini.h' !!");
					goto _exit0;
				  }

			        while(fgets(ini_buff, 120, fp) != NULL)
			        {
			           if (strstr(ini_buff, "6612_CH") > 0)
			           {
			               sscanf(ini_buff, "6612_CH=%d", &i/*current video channel*/);
			           }
			        }
				fclose(fp);
				video_chan_switch(false, i, acs);

				goto _exit0;
			}
			else if (false/*rx*/==work_mode && !strcasecmp(argv[3],"retune")) {
	    	  fc = htoi(argv[4]);
	    	  if (54000000>fc || 862000000<fc) { // from siano sms4470 spec
		    	 puts("invalid tuning frequency...");
		     	goto _exit; }
				shmLgdst_proc->type = CMD1;
				shmLgdst_proc->len = sizeof(fc);
				shmLgdst_proc->tag.wDir = CTRL_OUT;
				shmLgdst_proc->tag.wValue = USB_RX_TUNE_VAL;
				shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;
				memcpy(shmLgdst_proc->access.hdr.data, &fc, sizeof(fc));
					goto _read;
			}
#if false
			else if (false/*rx*/==work_mode && !strcasecmp(argv[3],"existed")) {
				memset(acs, 0x0, sizeof(*acs)-sizeof(uint16_t));
				acs->access = SIGNAL_DETECTED;
				acs->dcnt = 1; // boolean
				shmLgdst_proc->type = ACS;
				shmLgdst_proc->tag.wDir = CTRL_OUT;
				shmLgdst_proc->tag.wValue = USB_HOST_MSG_TX_VAL;
				shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;
				goto _read;
			}
#endif
			else if (false/*rx*/==work_mode && !strcasecmp(argv[3],"sig")) {
				query_video_short_rx(acs);
				goto _read;
			}
#if false
			else if (false/*rx*/==work_mode && !strcasecmp(argv[3],"tran")) {
				memset(acs, 0x0, sizeof(*acs)-sizeof(uint16_t));
				acs->access = TRANS_STATUS;
				acs->dcnt = sizeof(TRANSMISSION_STATISTICS_ST)/sizeof(uint16_t);
				shmLgdst_proc->type = ACS;
				shmLgdst_proc->tag.wDir = CTRL_OUT;
				shmLgdst_proc->tag.wValue = USB_HOST_MSG_TX_VAL;
				shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;
				goto _read;
			}
#endif
			else if (false/*rx*/==work_mode && !strcasecmp(argv[3],"recv")) {
				memset(acs, 0x0, sizeof(*acs)-sizeof(uint16_t));
				acs->access = RECV_STATUS;
				acs->dcnt = sizeof(RECEPTION_STATISTICS_ST)/sizeof(uint16_t);
				shmLgdst_proc->type = ACS;
				shmLgdst_proc->tag.wDir = CTRL_OUT;
				shmLgdst_proc->tag.wValue = USB_HOST_MSG_TX_VAL;
				shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;
				goto _read;
			}
			else if (false/*rx*/==work_mode && !strcasecmp(argv[3],"locked")) {
				memset(acs, 0x0, sizeof(*acs)-sizeof(uint16_t));
				acs->access = LOCKED_STATUS;
				acs->dcnt = 1; // boolean
				shmLgdst_proc->type = ACS;
				shmLgdst_proc->tag.wDir = CTRL_OUT;
				shmLgdst_proc->tag.wValue = USB_HOST_MSG_TX_VAL;
				shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;
				goto _read;
			}
			else if (!strcasecmp(argv[3],"temp")) { // // control radio temperature readout
				shmLgdst_proc->type = CMD1;
				shmLgdst_proc->len = RADIO_TEMP_LEN;
				shmLgdst_proc->tag.wDir = CTRL_IN;
				shmLgdst_proc->tag.wValue = RADIO_COMM_VAL;
				shmLgdst_proc->tag.wIndex = RADIO_TEMP_IDX;
				goto _read;
			}
			else if (!strcasecmp(argv[3],"ctune")) { // tune cap bank on ctrl radio chip
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
				goto _read;
			}
    	  else if (!strcasecmp(argv[3],"wb") ||!strcasecmp(argv[3],"wbs")) {
    	  	 	acs->access = BURST_MEM_WRITE;
    	  	 	sz = htoi(argv[5]);
    	  	 	if (1>sz || (HOST_BUFFER_SIZE+1)<sz) {
	    	  	 	puts("invalid block size...");
	    	  	 	goto _exit; }
    	  	 	acs->dcnt = sz; // must be less than HOST_BUFFER_SIZE+1
	    	   for (i=0; i<acs->dcnt; i++) {
		    	   sscanf(argv[6+i], "%x", &tmp);
	    		  acs->data[i] = tmp;
    		   }
    		   acs->toflash = (0==strcasecmp(argv[3], "wbs"));
    	  }
	    	else if (!strcasecmp(argv[3],"w") ||!strcasecmp(argv[3],"ws")) {
    	  	 	acs->access = WRITE_BY_ADDR;
    	  	 	acs->dcnt = 1;
		    	sscanf(argv[5], "%x", &tmp);
	    		acs->data[0] = tmp;
    		   	acs->toflash = (0==strcasecmp(argv[3], "ws"));
		}
	   else if ((true/*Tx*/==work_mode) && !strcasecmp(argv[3],"rfch")) {
		   	if (5 != argc) {
			   	puts("invalid or no param passed, usage: lgdst 0 tx rfch ch");
			   	goto _exit;
		   	}
		    	open_tx_ini(&Rf_Params.params_tx.chan_idx,
		    								&Rf_Params.params_tx.pwr_att);
		    	Rf_Params.params_tx.chan_idx = htoi(argv[4]);
		    	if (1>Rf_Params.params_tx.chan_idx || 20<Rf_Params.params_tx.chan_idx){
			    	puts("invalid channel index, between [1, 20]");
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
		else if ((false/*Rx*/==work_mode) && !strcasecmp(argv[3],"rfch")) {
    	  	 	acs->access = WRITE_BY_ADDR;
    	  	 	acs->dcnt = 1;
		}
		else if (!strcasecmp(argv[3],"rb")) {
		      acs->access = BURST_MEM_READ;
    	  	 	sz = htoi(argv[5]);
    	  	 	if (1>sz || (HOST_BUFFER_SIZE+1)<sz) {
	    	  	 	puts("invalid block size...");
	    	  	 	goto _exit; }
    	  	 	acs->dcnt = sz; // must be less than HOST_BUFFER_SIZE+1
			}
			else if (!strcasecmp(argv[3],"r")) {
    	  	 	acs->access = READ_BY_ADDR;
    	  	 	acs->dcnt = 1;
			}
			else if (!strcasecmp(argv[3],"Ua")) {
				if (5 != argc) {
					perror_exit("invalid command line parameters given, lgdst 0 tx/rx Ua bin-file-path",-3);
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
				if (5 != argc) {
					perror_exit("invalid command line parameters given, lgdst 0 tx Uf rbf-file-path",-2);
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
					goto _read;
			}
			else if (true/*tx*/==work_mode && !strcasecmp(argv[3],"Vn")) {
		      acs->access = BURST_MEM_READ;
    	  	 	acs->dcnt = 3; // 3 bytes read on nios ver
    	  	 	acs->addr = 0x4;
				RAED_SETUP
			}
			else if (true/*tx*/==work_mode && !strcasecmp(argv[3],"Vf")) {
		      acs->access = BURST_MEM_READ;
    	  	 	acs->dcnt = 3; // 3 bytes read on fpga ver
    	  	 	acs->addr = 0x8;
				RAED_SETUP
			}
			else if (false/*rx*/==work_mode && !strcasecmp(argv[3],"Uc")) {
				if (5 != argc) {
					perror_exit("invalid command line parameters given, lgdst 0 rx Uc cpld-file-path",-4);
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
					goto _read;
			}
			else if (false/*rx*/==work_mode && !strcasecmp(argv[3],"Vc")) {
		      acs->access = READ_BY_ADDR;
    	  	 	acs->dcnt = 1; // 1 short read on cpld ver
    	  	 	acs->addr = 0x7f;
				RAED_SETUP
			}
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
				else if (!strcasecmp(argv[3],"Va")) {
					shmLgdst_proc->type = CMD1;
					shmLgdst_proc->tag.wDir = CTRL_IN;
					shmLgdst_proc->len = 3;
					shmLgdst_proc->tag.wValue = USB_ATMEL_VER_VAL;
					shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;
				}
				goto _read;
			}
    	  adr = htoi(argv[4]);
    	  if (0>adr || (HOST_BUFFER_SIZE)<adr) {
	    	 puts("invalid address...");
	     	goto _exit; }
    	  acs->addr = adr;
		  if (!strcasecmp(argv[3],"wb") ||!strcasecmp(argv[3],"wbs") ||
		  		!strcasecmp(argv[3],"w") ||!strcasecmp(argv[3],"ws")) {
				shmLgdst_proc->type = ACS;
				shmLgdst_proc->tag.wDir = CTRL_OUT;
				shmLgdst_proc->tag.wValue = USB_HOST_MSG_TX_VAL;
				shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;
		    	  for (i=0; i<acs->dcnt; i++)
		    		 printf("xxxxxx sent 0x%02x @ 0x%02x xxxxx\n",
		    		 				acs->data[i], adr+i);
		  }
		  else if (!strcasecmp(argv[3],"rb") ||!strcasecmp(argv[3],"r")) {
				RAED_SETUP
		  }
		  else if ((false/*Rx*/==work_mode) && (!strcasecmp(argv[3],"rfch"))) {
		   	if (5 != argc) {
			   	puts("invalid or no param passed, usage: lgdst 0 rx rfch ch");
			   	goto _exit;
		   	}
		        // Kevin : RF Channel Selection (atmel change: setup_6612())
			printf("rf rx ch:%d \n", adr);
			update_ini(0, &adr, NULL); // chan info in 'adr'

	 		dev_access echo; // atmel shall echo write cmd hdr back
			video_chan_switch(false, adr, acs);
		  }
			else {
			  goto _exit;  // never reached
			}
     }
_read:
	shmLgdst_proc->active = 1;  // wait for transaction complete
	while (1==shmLgdst_proc->active) ;
	  {
		  if (!strcasecmp(argv[3],"wb") ||!strcasecmp(argv[3],"wbs") ||
		  		!strcasecmp(argv[3],"w") ||!strcasecmp(argv[3],"ws")) {
	 	dev_access echo; // atmel shall echo write cmd hdr back, liyenho

					memcpy(&echo, acs, sizeof(*acs)); // keep original copy
					shmLgdst_proc->type = ACS;
					shmLgdst_proc->tag.wDir = CTRL_IN;
					shmLgdst_proc->tag.wValue = USB_HOST_MSG_RX_VAL;
					shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;

					short_sleep(0.1); // validate echo after 0.1 sec
					shmLgdst_proc->echo = true;
					shmLgdst_proc->active = 1; // process next ctrl xfer
					while(shmLgdst_proc->echo || 1==shmLgdst_proc->active) ;

				 if (echo.access != acs->access)
				 	printf("xxxxxx invalid access mode echoed, "
				 		"(sent(%d), echo(%d)) xxxxxxx\n", echo.access, acs->access);
				 if (echo.dcnt != acs->dcnt)
				 	printf("xxxxxx invalid data count echoed, "
				 		"(sent(%d), echo(%d)) xxxxxx\n", echo.dcnt, acs->dcnt);
			}
		  else if (!strcasecmp(argv[3],"rb") ||!strcasecmp(argv[3],"r")) {
					short_sleep(0.1); // wait 0.1 sec before readback
					shmLgdst_proc->type = ACS;
					shmLgdst_proc->tag.wDir = CTRL_IN;
					shmLgdst_proc->tag.wValue = USB_HOST_MSG_RX_VAL;
					shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;

					shmLgdst_proc->active = 1; // process next ctrl xfer
					while(1==shmLgdst_proc->active) ;

		     	 for (i=0; i<acs->dcnt; i++)
		   	 	printf("xxxxxx 0x%02x received @ 0x%02x xxxxxx\n",
		   	 					acs->data[i], adr+i);
				 if (ret)
				 	*(uint16_t**)ret = acs->data;
			}
#if false
			else if (false/*rx*/==work_mode && !strcasecmp(argv[3],"existed")) {

					shmLgdst_proc->type = ACS;
					shmLgdst_proc->tag.wDir = CTRL_IN;
					shmLgdst_proc->tag.wValue = USB_HOST_MSG_RX_VAL;
					shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;
					acs->dcnt = 1; // boolean

					shmLgdst_proc->active = 1; // process next ctrl xfer
					while(1==shmLgdst_proc->active) ;

						printf("DVBT/2 signal may be %s presented\n",
									(0!=acs->data[0])?"not":"");
			}
#endif
			else if (false/*rx*/==work_mode && !strcasecmp(argv[3],"sig")) {

					read_video_short_rx(acs);
					Short_Statistics_ST *psts = &acs->data[0];
						printf("DVBT (specific) Signal Status,\n"
									"\tIsDemodLocked = %d,\n"
									"\tInBandPwr = %d,\n"
									"\tBER = %d (k),\n"
									"\tSNR = %d,\n"
									"\tTotalTSPackets = %d,\n"
									"\tErrorTSPackets = %d\n",
									psts->IsDemodLocked,
									psts->InBandPwr,
									psts->BER,
									psts->SNR,
									psts->TotalTSPackets,
									psts->ErrorTSPackets );
				 if (ret)
				 	*(Short_Statistics_ST**)ret = psts;
			}
#if false
			else if (false/*rx*/==work_mode && !strcasecmp(argv[3],"tran")) {

					shmLgdst_proc->type = ACS;
					shmLgdst_proc->tag.wDir = CTRL_IN;
					shmLgdst_proc->tag.wValue = USB_HOST_MSG_RX_VAL;
					shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;
					acs->dcnt = sizeof(TRANSMISSION_STATISTICS_ST)/sizeof(uint16_t);

					shmLgdst_proc->active = 1; // process next ctrl xfer
					while(1==shmLgdst_proc->active) ;

					TRANSMISSION_STATISTICS_ST *psts = &acs->data[0];
						printf("Rx Transmission Status,\n"
									"\tFrequency = %d,\n"
									"\tBandwidth = %d,\n"
									"\tTransmissionMode = %d (k),\n"
									"\tGuardInterval = %d,\n"
									"\tCodeRate = %d,\n"
									"\tLPCodeRate = %d,\n"
									"\tHierarchy = %d,\n"
									"\tConstellation = %d,\n"
									"\tIsDemodLocked = %d\n",
									psts->Frequency,
									psts->Bandwidth,
									psts->TransmissionMode,
									psts->GuardInterval,
									psts->CodeRate,
									psts->LPCodeRate,
									psts->Hierarchy,
									psts->Constellation,
									psts->IsDemodLocked );
			}
#endif
			else if (false/*rx*/==work_mode && !strcasecmp(argv[3],"recv")) {

					shmLgdst_proc->type = ACS;
					shmLgdst_proc->tag.wDir = CTRL_IN;
					shmLgdst_proc->tag.wValue = USB_HOST_MSG_RX_VAL;
					shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;
					acs->dcnt = sizeof(RECEPTION_STATISTICS_ST)/sizeof(uint16_t);

					shmLgdst_proc->active = 1; // process next ctrl xfer
					while(1==shmLgdst_proc->active) ;

					RECEPTION_STATISTICS_ST *psts = &acs->data[0];
					// Kevin : Add single item query (not atmel change)
					/*if (!strcasecmp(argv[4],"RSSI")) {
						printf("Rx Receive Status,\n"
									"\tRSSI = %d\n",
									psts->RSSI);
					} else if (!strcasecmp(argv[4],"IsDemodLocked")) {
						printf("Rx Receive Status,\n"
									"\tIsDemodLocked = %d\n",
									psts->IsDemodLocked);
					} else if (!strcasecmp(argv[4],"CarrierOffset")) {
						printf("Rx Receive Status,\n"
									"\tCarrierOffset = %d\n",
									psts->CarrierOffset);
					} else*/
						printf("Rx Receive Status,\n"
									"\tIsRfLocked = %d,\n"
									"\tIsDemodLocked = %d,\n"
									"\tIsExternalLNAOn = %d,\n"
									"\tModemState = %d,\n"
									"\tSNR = %d,\n"
									"\tBER = %d,\n"
									"\tBERErrorCount = %d,\n"
									"\tBERBitCount = %d,\n"
									"\tTS_PER = %d,\n"
									"\tMFER = %d,\n"
									"\tRSSI = %d,\n"
									"\tInBandPwr = %d,\n"
									"\tCarrierOffset = %d,\n"
									"\tErrorTSPackets = %d,\n"
									"\tTotalTSPackets = %d,\n"
									"\tRefDevPPM = %d,\n"
									"\tFreqDevHz = %d,\n"
									"\tMRC_SNR = %d,\n"
									"\tMRC_RSSI = %d,\n"
									"\tMRC_InBandPwr = %d,\n"
									"\tErrorTSPacketsAfterReset = %d,\n"
									"\tTotalTSPacketsAfterReset = %d\n",
									psts->IsRfLocked,
									psts->IsDemodLocked,
									psts->IsExternalLNAOn,
									psts->ModemState,
									psts->SNR,
									psts->BER,
									psts->BERErrorCount,
									psts->BERBitCount,
									psts->TS_PER,
									psts->MFER,
									psts->RSSI,
									psts->InBandPwr,
									psts->CarrierOffset,
									psts->ErrorTSPackets,
									psts->TotalTSPackets,
									psts->RefDevPPM,
									psts->FreqDevHz,
									psts->MRC_SNR,
									psts->MRC_RSSI,
									psts->MRC_InBandPwr,
									psts->ErrorTSPacketsAfterReset,
									psts->TotalTSPacketsAfterReset );
					 if (ret)
					 	*(RECEPTION_STATISTICS_ST**)ret = psts;
			}
			else if (false/*rx*/==work_mode && !strcasecmp(argv[3],"locked")) {

					shmLgdst_proc->type = ACS;
					shmLgdst_proc->tag.wDir = CTRL_IN;
					shmLgdst_proc->tag.wValue = USB_HOST_MSG_RX_VAL;
					shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;
					acs->dcnt = 1; // boolean

					shmLgdst_proc->active = 1; // process next ctrl xfer
					while(1==shmLgdst_proc->active) ;

						printf("DVBT/2 reception has been %s locked\n",
									(1!=acs->data[0])?"not":"");
					 if (ret)
					 	*(uint16_t*)ret = *acs->data;
			}
			else if (!strcasecmp(argv[3],"temp")) {
				int16_t temp, *pv = (int16_t*)acs->data;
				if (1358<=*pv && 1722>=*pv) {
				temp = (899.0/4096.0)*(*pv) - 293;
				printf("current Si4463 temperature = %d C\n", temp);
				} else
				puts("current Si4463 temperature reading is bad");
				 if (ret)
				 	*(int16_t*)ret = temp;
			}
		  else if (true/*tx*/==work_mode && !strcasecmp(argv[3],"Vn")) {
					short_sleep(0.1); // wait 0.1 sec before readback
					shmLgdst_proc->type = ACS;
					shmLgdst_proc->tag.wDir = CTRL_IN;
					shmLgdst_proc->tag.wValue = USB_HOST_MSG_RX_VAL;
					shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;

					shmLgdst_proc->active = 1; // process next ctrl xfer
					while(1==shmLgdst_proc->active) ;

		     	 for (i=0; i<3; i++) {
		   	 	printf("xxxxxx NIOS ver[%d] 0x%02x received xxxxxx\n", i, (uint8_t)acs->data[i]);
	   	 	}
				 if (ret)
				 	*(uint16_t**)ret = acs->data;
			}
		  else if (true/*tx*/==work_mode && !strcasecmp(argv[3],"Vf")) {
					short_sleep(0.1); // wait 0.1 sec before readback
					shmLgdst_proc->type = ACS;
					shmLgdst_proc->tag.wDir = CTRL_IN;
					shmLgdst_proc->tag.wValue = USB_HOST_MSG_RX_VAL;
					shmLgdst_proc->tag.wIndex = USB_HOST_MSG_IDX;

					shmLgdst_proc->active = 1; // process next ctrl xfer
					while(1==shmLgdst_proc->active) ;

		     	 for (i=0; i<3; i++) {
		   	 	printf("xxxxxx FPGA ver[%d] 0x%02x received xxxxxx\n", i, (uint8_t)acs->data[i]);
	   	 	}
				 if (ret)
				 	*(uint16_t**)ret = acs->data;
			}
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
			else if (!strcasecmp(argv[3],"Va")) {
				uint8_t *pv = (uint8_t*)acs->data;
		     	 for (i=0; i<shmLgdst_proc->len; i++)
		   	 	printf("xxxxxx ATMEL ver[%d] 0x%02x received xxxxxx\n", i, pv[i]);
				 if (ret)
				 	*(uint8_t**)ret = pv;
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
#if false //android doesn't have IPC facility, replaced with normal static global allocation
  if (shmLgdst_proc)
  	shmdt(shmLgdst_proc);
#endif
	// now we exit, thank you......
	return 0;
}
