//###########################################################
//# usb_tx_prod.c
//###########################################################

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <math.h>

#define DEBUG_CTRLRX                    1/*0*/
#define DEBUG_CTRLTX                    0
#define DEBUG_VIDEO                     1

#include <libusb.h>
#include <assert.h>
#define true														1
#define false														0

static volatile uint8_t system_upgrade = 0;
static volatile uint8_t upgrade_from_lgdst = 0;
static char upgrade_fwm_path[160]; // upgrade firmware path
#define ATMEL_END2END
#ifdef ATMEL_END2END
	static volatile uint8_t main_loop_on = false; // run time indicator, liyenho
  #define CTRL_OUT		(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT)
  #define CTRL_IN		(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN)
  #define USB_RQ			0x04
 #include "usb_tx.h"  // definitions for host/fpga access @ tx
#undef SRC_FRM_ENET  // disable network access in lib mode
 static int shmid_Lgdst = -1;
 /*static*/ volatile ipcLgdst *shmLgdst_proc = 0;
 #ifdef RADIO_SI4463
  static volatile bool si4463_radio_up = false ;
 #endif
 //#define COMM_ATMEL_DEV // enable atmel<->fpga/6612 comm test
 volatile bool ready_wait_for_mloop= false;
 	#ifdef RADIO_SI4463
 	static unsigned short ctrl_port_base;
  static char servIP[32];
 	static int ctrlsnd_socket, ctrlrcv_socket; //clnt_socket;
  static struct timeval ctrlsnd_tv;
  static fd_set ctrlsnd_fd;
  static int ctrlsnd_len;
 	static struct sockaddr_in ctrlsnd, ctrlrcv; //clnt;
 	#endif
	#ifdef SRC_FRM_ENET // extract ts packets from socket
	 #define UDP_PACKET_MAX 1880
		static int udpin_len, udpout_len;
		static int udpin_socket, udpout_socket;
		static fd_set udpin_fd;
		static struct timeval udpin_tv= {0};
		static struct sockaddr_in udpin, udpout;
		static struct ip_mreq udpin_mreq;
		static int udpin_init(void);
	  static int udpout_init(void);
	  static int receive(int usec, unsigned char *pb, uint32_t bytes);
	#endif
#endif
#define USB_DEV_VENDER			0x03eb
#define USB_DEV_PRODUCT			0x2404

// figure it out via lsusb -v -d, choose high speed (2.0) one
//#define USB_DEV_INTF20				0
#define USB_DEV_INTF				/*0*/1  // cdc comm intf included
// atmel CDC data device intf 1
  //#define USB_DEV_EP20						2
  #define USB_DEV_EP						2	// atmel CDC data ep 2 out
  //#define EP_DATA			(USB_DEV_EP20 | LIBUSB_ENDPOINT_OUT)
  #define EP_DATA			(USB_DEV_EP | LIBUSB_ENDPOINT_OUT)

#define TEST												2	// synchronous file based block transfer

#ifdef MEDIA_ON_FLASH
 #define FILE_LEN									63*1880  // 200 kbytes reserved on flash, liyenho
#else
 #define FILE_LEN									/*6422528*/ /*12540164*/ /*10388880*/ 118440
#endif
  #define FILE_NAME								/*"NativeMedia.ts"*/ "movebox118440.ts" /*"DVB_v2.ts"*/
#define FRAME_SIZE_A					1880  // 16 bit stereo @ 480 Khz
#define FRAME_SIZE_V					307200	// 320x240 dim expected
#define FRAME_SIZE_V2					(FRAME_SIZE_V*2)
#define ITERS												(FILE_LEN/FRAME_SIZE_A)
#define FRAME_BUFFS						5
#define TIMEOUT									1000		// audio time out @ 10 msec

static FILE *file = NULL;
static unsigned char audbuf[FRAME_SIZE_A*FRAME_BUFFS]__attribute__((aligned(8)));
static unsigned char vidbuf[FRAME_SIZE_V2*FRAME_BUFFS]__attribute__((aligned(8)));
#ifdef RADIO_SI4463
  static unsigned char radio_tpacket[RADIO_USR_TX_LEN]__attribute__((aligned(8)));
  static unsigned char radio_rpacket[RADIO_USR_RX_LEN+RADIO_INFO_LEN]__attribute__((aligned(8)));
#endif
#ifdef RFFE_PARAMS
	static rf_params Rf_Params= {0};	// shared between tx/rx platforms from host
#endif

static bool detached = false;
static struct libusb_device_handle *devh = NULL;
static volatile int do_exit = 0;	// main loop breaker
static pthread_t poll_thread= 0,
									lgdst_thread = 0,
									ctrl_thr_recv = 0,
									ctrl_thr_send = 0;
 static pthread_mutex_t mux;
 static int tag=0; //ts packet count

//function prototyping


static void DieWithError(char *errorMessage)  /* Error handling function */
{
    perror(errorMessage);
    exit(1);
}

static void at_exit(int status) {
	if (do_exit) {
		do_exit = -1;
		if (poll_thread)
			pthread_join(poll_thread, NULL);
		if (lgdst_thread)
			pthread_join(lgdst_thread, NULL);
#if defined(RADIO_SI4463)
		pthread_join(ctrl_thr_recv, NULL);
		pthread_join(ctrl_thr_send, NULL);
#endif
#ifdef SRC_FRM_ENET
		close(udpin_socket);
		close(udpout_socket);
#endif
#if defined(RADIO_SI4463) && !defined(CTRL_RADIO_TEST)
	close(ctrlsnd_socket);
  close(ctrlrcv_socket);
#endif
  }
  if (0 <= shmid_Lgdst ) {
  		shmdt(shmid_Lgdst );
		if (shmctl(shmid_Lgdst, IPC_RMID, NULL))
		    perror("Error deleting shmid_Lgdst shared memory segment");
	}
	if (file)
		fclose(file);
	if (devh) {
	  //libusb_release_interface(devh, USB_DEV_INTF20);
	  libusb_release_interface(devh, USB_DEV_INTF);
		if (detached)
			//libusb_attach_kernel_driver(devh,USB_DEV_INTF20);
			libusb_attach_kernel_driver(devh,USB_DEV_INTF);
	  libusb_close(devh);
		libusb_exit(NULL);
	}
   pthread_mutex_destroy(&mux);
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
#if (TEST==2)
static int stream_block(unsigned char *tsbuf) {
	int r = 0, transferred;
	r=libusb_bulk_transfer(
		devh,
		EP_DATA,
		tsbuf,
		FRAME_SIZE_A,
		&transferred,
#ifdef MEDIA_ON_FLASH
		1000); // in case erase sector occurred which take long time, liyenho
#else
 #ifdef RECV_SMS4470
		0); // realtime recv no delay allowed
 #else
		50);
 #endif
#endif
	if (r<0) return r;
skip:
	if (FRAME_SIZE_A!= transferred) {
		//if (0!=transferred)
		  //printf("xxx transferred = %d xxx\n",transferred);
		return -2; }
	return 0;
}
#endif

static int short_sleep(double sleep_time)
{
  struct timeval tv;

  tv.tv_sec = (time_t) floor(sleep_time);
  tv.tv_usec = (time_t) ((sleep_time - floor(sleep_time)) * 1.0e6);

  return(select(0, NULL, NULL, NULL, &tv));
}

#ifdef RADIO_SI4463
static void ctrl_poll_recv(void *arg) {
	int r, i;
  unsigned char validdataflag=0;
   long pv_wrbyte=0;
	while (1==do_exit) {
 	bool ctrl_sckt_ok = *(bool*)arg;
  		if (ctrl_sckt_ok) {
			pthread_mutex_lock(&mux);
			r = libusb_control_transfer(devh,
					CTRL_IN, USB_RQ,
					RADIO_COMM_VAL,
					RADIO_DATA_RX_IDX,
					radio_rpacket, sizeof(radio_rpacket), 0);
				pthread_mutex_unlock(&mux);

          validdataflag=(radio_rpacket[RADIO_USR_RX_LEN]&0x01);
	#ifdef SRC_FRM_ENET
          if(validdataflag==0x01) //got valid payload
          {
				  pv_wrbyte = sendto(ctrlrcv_socket,radio_rpacket,RADIO_USR_RX_LEN,0,
				  											(struct sockaddr *)&ctrlrcv, sizeof(ctrlrcv));
          }
	#endif
			 if(DEBUG_CTRLRX)
          {
            if(validdataflag)
            {
            printf("radio link Rx: ");
                     for(i=0;i<sizeof(radio_rpacket);i++)
                     printf("%02x ",radio_rpacket[i]);
                     printf("\n"); }


           {// user packet dataloss ---------------------------------
             static unsigned char val_exp;
             unsigned char        val_cur;
             static int ilcnt=0, becnt=0;
              unsigned char byteerror=0;
             if(validdataflag==0x01){
               for(i=6;i<RADIO_USR_RX_LEN;i++)
                 if(radio_rpacket[i]!=0xb5) {
                    byteerror++; becnt++;}
               if(byteerror > 0) printf("User packet corruption = %d. ++++++++++\n", becnt);
               val_cur = radio_rpacket[5];
               val_exp = (unsigned char)(val_exp+1);
               if(val_exp != val_cur) {
                 ilcnt++;
                 printf("User packet dataloss = %d. !!!!!!!!!!!!!!!!!!!!!!!!!!!!\n",ilcnt);
               }
               val_exp = val_cur;
             }//end valid data
           }// user packet dataloss check
        }//DEBUG_CTRLRX

			usleep(CTRL_RECV_POLLPERIOD);
  		} //ctrl_sckt_ok
	}//(1==do_exit)
}
void lgdst_ctl_rec_tx(unsigned char *rpacket)
{
	if (rpacket) {
		// rpacket buffer must be at least the same length:RADIO_USR_RX_LEN+RADIO_INFO_LEN
		// user should always check for buffer information (RADIO_INFO_LEN) in regards to access status
		memcpy(rpacket, radio_rpacket, sizeof(radio_rpacket));
	}
}

static void ctrl_poll_send(void *arg) {  //usb send to module
	int r,tx_cnt= 0, err, rcvsize;
  uint32_t ctrl_send_fifolvl_data_l=CTRL_SEND_FIFODEPTH;
  bool ctrl_sckt_ok = *(bool*)arg;

	while (1==do_exit) {

    rcvsize = 0;

    // clientudptx based user data --------------------
    //ctrlsnd_tv.tv_sec = 2; ctrlsnd_tv.tv_usec = 0;
    //FD_ZERO(&ctrlsnd_fd); FD_SET(ctrlsnd_socket,&ctrlsnd_fd);
    //err = select(ctrlsnd_socket+1,&ctrlsnd_fd,0,0,&ctrlsnd_tv);
    //if(err<0) {printf("Warning: ctrlsnd: select failed\n");return err;}
    //if (FD_ISSET(ctrlsnd_socket,&ctrlsnd_fd)) {
		//	rcvsize= recvfrom(ctrlsnd_socket , radio_tpacket, sizeof(radio_tpacket),0,
    //      (struct sockaddr *)&ctrlsnd, &ctrlsnd_len);
    //}
    // ------------------------------------------------

    //direct based user data --------------------------
    {
        //Insert user data here
        //Can stop sending anytime. No need to fill up
        static uint32_t echocnt=0;
        int i;
        for(i=0;i<sizeof(radio_tpacket);i++)
          radio_tpacket[i]=0xb5;
   	    radio_tpacket[2] = ((unsigned char *)&echocnt)[3];
        radio_tpacket[3] = ((unsigned char *)&echocnt)[2];
        radio_tpacket[4] = ((unsigned char *)&echocnt)[1];
        radio_tpacket[5] = ((unsigned char *)&echocnt)[0];
        echocnt++;
        rcvsize = sizeof(radio_tpacket);
        if(DEBUG_CTRLTX) {
          printf("radio link Tx: ");
          for(i=0;i<sizeof(radio_tpacket);i++)
                     printf("%02x ",radio_tpacket[i]);
                     printf("\n");
                     }
    }
    // ------------------------------------------------
		if (sizeof(radio_tpacket) == rcvsize) {
		  pthread_mutex_lock(&mux);
		  libusb_control_transfer(devh,CTRL_OUT, USB_RQ,RADIO_COMM_VAL,RADIO_DATA_TX_IDX,
      radio_tpacket, sizeof(radio_tpacket), 0);
		  pthread_mutex_unlock(&mux);
      }

      usleep(CTRL_SEND_POLLPERIOD);
	} //do_exit
}
void lgdst_ctl_snd_tx(unsigned char *tpacket)
{
	if (tpacket) {
		// tpacket buffer must be the same length:RADIO_USR_TX_LEN
		memcpy(radio_tpacket, tpacket, sizeof(radio_tpacket));
	}
}
#endif //RADIO_SI4463

static void *poll_thread_main(void *arg)
{
	int r=0, s, video, frms = 0, radio_cnt=0;
   long pa_wrbyte=0, pv_wrbyte=0;
	bool ctrl_sckt_ok = false;
	printf("poll thread running\n");
#ifdef RADIO_SI4463
	printf("Setup Ctrl Radio Sockets... ip=%s portTx=%d portRx=%d\n",
    servIP, ctrl_port_base+1, ctrl_port_base);
	int ctrlsnd_pt = htons(ctrl_port_base);
  int ctrlrcv_pt = htons(ctrl_port_base+1);
	int tx_cnt= 0, rx_cnt= 0;

	r = socket(AF_INET,SOCK_DGRAM,0);
  s = socket(AF_INET,SOCK_DGRAM,0);

	if ((-1 != r) &&(-1!=s)) {
		ctrlsnd.sin_family = AF_INET;
		ctrlsnd.sin_addr.s_addr = htonl(INADDR_ANY);
		ctrlsnd.sin_port = ctrlsnd_pt; //ctrl_pt+1, for output
		ctrlsnd_socket = r;
    ctrlsnd_len = sizeof(ctrlsnd);
		if (bind(r, (struct sockaddr *) &ctrlsnd, sizeof(ctrlsnd)) < 0)
			{	DieWithError("ctrlsnd: bind() failed"); }

 		ctrlrcv.sin_family = AF_INET;
		ctrlrcv.sin_addr.s_addr = inet_addr(servIP);
		ctrlrcv.sin_port = ctrlrcv_pt; //ctrl_pt+1, for socket output port
		ctrlrcv_socket = s;

			ctrl_sckt_ok = true; // validate socket open
	}
#endif
		while (!ready_wait_for_mloop) ;
		pthread_mutex_lock(&mux);
 		do {
			 libusb_control_transfer(devh,
					CTRL_IN, USB_RQ,
					USB_STREAM_ON_VAL,
					USB_QUERY_IDX,
					&main_loop_on, sizeof(main_loop_on), 0);
			if (!main_loop_on) {
				short_sleep(1); 	// setup & settle in 1 sec
			} else
			break;
		} while (1==do_exit);
		pthread_mutex_unlock(&mux);
#if defined(RADIO_SI4463)
		si4463_radio_up = true;
   r=0;
	r = pthread_create(&ctrl_thr_recv, NULL, ctrl_poll_recv, &ctrl_sckt_ok);
	if (0 != r)
		perror_exit("ctrl recv thread creation error", r);
	r = pthread_create(&ctrl_thr_send, NULL, ctrl_poll_send, &ctrl_sckt_ok);
	if (0 != r)
		perror_exit("ctrl send thread creation error", r);
#endif
	while (1==do_exit) {
		struct timeval tv = { 0, 10000 };
		r = libusb_handle_events_timeout(NULL, &tv);
		if (r < 0) {
			perror_exit("event handler failed", r);
			break;
		}
	}
#if defined(RADIO_SI4463)
	pthread_join(ctrl_thr_recv, NULL);
	pthread_join(ctrl_thr_send, NULL);
#endif
	printf("poll thread shutting down\n");
	return NULL;
}

static void *lgdst_thread_main(void *arg)
{
#if false // android doesn't have IPC facility, replaced with normal static global allocation
	if (0>(shmid_Lgdst = shmget(SHMKEY_TX,
		sizeof(ipcLgdst),IPC_CREAT|0666))) {
		perror_exit("failed to create IPC memory for lgdst process, bailed out...", -7);
	}
	if (0>(shmLgdst_proc=shmat(shmid_Lgdst, NULL, 0))) {
	  perror_exit("get shmLgdst_proc shmem failed",0);
  }
	memset(shmLgdst_proc, 0, sizeof(ipcLgdst));
#else
	shmLgdst_proc = (ipcLgdst*)calloc(1, sizeof(ipcLgdst));
	if (!shmLgdst_proc)
		perror_exit("allocate shmLgdst_proc shmem failed",0);
#endif
	shmLgdst_proc->active = -1; // shm signalling flag to lgdst client
	static dAccess lclMem ;
	while (1==do_exit) {
		dev_access *acs = &lclMem.hdr;
		if (1 == shmLgdst_proc->active) {
			memcpy(&lclMem, &shmLgdst_proc->access, sizeof(dAccess));
			switch(shmLgdst_proc->type) {
				case CMD0:
					if (USB_FPGA_NEW_VAL == shmLgdst_proc->tag.wValue) {
    					system_upgrade = 5;
						puts("user requests fpga firmware switch...");
						break;
					}
	printf("CMD0: wValue = %d, wIndex = %d\n", shmLgdst_proc->tag.wValue,
											shmLgdst_proc->tag.wIndex);  // for debug
					pthread_mutex_lock(&mux);
					libusb_control_transfer(devh,
								CTRL_OUT,
								USB_RQ,
								shmLgdst_proc->tag.wValue,
								shmLgdst_proc->tag.wIndex,
								NULL, 0, 0);
					pthread_mutex_unlock(&mux);
					break;
				case CMD1:
					if (USB_CPLD_UPGRADE_VAL == shmLgdst_proc->tag.wValue ||
							USB_FPGA_UPGRADE_VAL == shmLgdst_proc->tag.wValue) {
						assert(sizeof(upgrade_fwm_path) >= shmLgdst_proc->len);
						memcpy(upgrade_fwm_path, (char*)acs->data, shmLgdst_proc->len);
    					system_upgrade = 1*(USB_CPLD_UPGRADE_VAL == shmLgdst_proc->tag.wValue) +
    															2*(USB_FPGA_UPGRADE_VAL == shmLgdst_proc->tag.wValue) ;
						puts("user requests system firmware upgrade...");
						break;
					}
	printf("CMD1: wDir = %d, wValue = %d, wIndex = %d, len= %d, data = %d\n",
											shmLgdst_proc->tag.wDir,
											shmLgdst_proc->tag.wValue,
											shmLgdst_proc->tag.wIndex,
											shmLgdst_proc->len,
											*(int*)acs->data);  // for debug
					pthread_mutex_lock(&mux);
					libusb_control_transfer(devh,
								shmLgdst_proc->tag.wDir,
								USB_RQ,
								shmLgdst_proc->tag.wValue,
								shmLgdst_proc->tag.wIndex,
								acs->data,
								shmLgdst_proc->len, 0);
					pthread_mutex_unlock(&mux);
					if (CTRL_IN==shmLgdst_proc->tag.wDir)
						memcpy(&shmLgdst_proc->access.hdr.data,
										&lclMem.hdr.data, shmLgdst_proc->len);
					break;
				case ACS:
					if (shmLgdst_proc->echo) {
	printf("ACS/echo: access= %d, dcnt= %d\n",
											(int)acs->access,
											(int)acs->dcnt);  // for debug
						pthread_mutex_lock(&mux);
			  	 		while(0==libusb_control_transfer(devh,
							  	CTRL_IN, USB_RQ,
							  	USB_HOST_MSG_RX_VAL,
					  			USB_HOST_MSG_IDX,
					  			acs, sizeof(*acs)-sizeof(acs->data[0]), 0))
					  		short_sleep(0.0005);
						pthread_mutex_unlock(&mux);
						memcpy(&shmLgdst_proc->access.hdr,
										&lclMem.hdr, sizeof(lclMem.hdr)-sizeof(lclMem.hdr.data[0]));
					  	shmLgdst_proc->echo = false;
					}
					else {
	printf("ACS: type = %d, access = %d, dcnt=%d\n",
											(int)shmLgdst_proc->type,
											(int)acs->access,
											(int)acs->dcnt);  // for debug
						pthread_mutex_lock(&mux);
						libusb_control_transfer(devh,
									shmLgdst_proc->tag.wDir,
									USB_RQ,
									shmLgdst_proc->tag.wValue,
									shmLgdst_proc->tag.wIndex,
									acs, sizeof(*acs)+(acs->dcnt-1)*sizeof(uint16_t), 0);
						pthread_mutex_unlock(&mux);
						if (CTRL_IN==shmLgdst_proc->tag.wDir)
							memcpy(&shmLgdst_proc->access.hdr,
											&lclMem.hdr, sizeof(lclMem.hdr)+(acs->dcnt-1)*sizeof(lclMem.hdr.data[0]));
					}
					break;
				default:
					perror_exit("shmLgdst_proc->type failed",0);
					break;
			}
			// turn off flag must be @ the end of process
			shmLgdst_proc->active = -1;
		}
	}
  if (0 <= shmid_Lgdst ) {
  		shmdt(shmid_Lgdst );
		if (shmctl(shmid_Lgdst, IPC_RMID, NULL))
		    perror("Error deleting shmid_Lgdst shared memory segment");
	}
}

/*static*/ void sigint_handler_tx(int signum)
{
   do_exit = 4; // this shall terminate poll thread
   pthread_mutex_unlock(&mux);
}
#ifdef RFFE_PARAMS
static bool open_ini(int *setting_ch,int *setting_pwr)
{
	FILE *fp = NULL;
        char ini_buff[120];

        fp = fopen("usb_tx.ini.h", "r+");
	if (fp == NULL) {
	  // File doesn't exist, setup ini with default
       	  fp = fopen("usb_tx.ini.h", "w+");
	  if (fp == NULL) {
		printf("ERROR: Fail to create 'usb_tx.ini.h' !!");
		return false;
	  }
	  fprintf(fp, "TX_CH=%d\n", *setting_ch);
	  fprintf(fp, "TX_PWR=%d\n", *setting_pwr);
	  fclose(fp);
          return true;
	}

        while(fgets(ini_buff, 120, fp) != NULL)
        {
           if (strstr(ini_buff, "TX_CH") > 0)
           {
               sscanf(ini_buff, "TX_CH=%d", setting_ch);
           }
	   if (strstr(ini_buff, "TX_PWR") > 0)
           {
               sscanf(ini_buff, "TX_PWR=%d",setting_pwr );
           }
        }
        printf("setting_ch = %d  setting_pwr  = %d\n", *setting_ch, *setting_pwr);
	fclose(fp);
	return true;
}
#endif

#ifdef PES_HDR_PROT
 //#define PHP_DBG
  	static void checksum_pes_hdr(uint32_t size, uint8_t *tsbuf)
 		{
 #ifdef PHP_DBG
 		static FILE *fdbg = 0;
 		if (!fdbg) fdbg = fopen("php_tx.dbg", "w");
 #endif
	 		uint8_t adp, *pbp, *pb = tsbuf;
	 		uint16_t chksm = 0;
	 		uint32_t i, n;
			for (i=0; i<size/188; chksm = 0, i++) {
				adp = 0x30 & *(pb+0x3) ;
				if ((0x40 == *(pb+0x1)) && (0x10 & adp)) {
					if (*(pb+0x4) && 0x0==*(pb+0x5) && 0xff==*(pb+0x6)) {
						if (2<*(pb+0x4)) {
							// packets from pre-defined tables, all 0xff came after 0x0 are stuffing bytes
							pbp = pb + 0x6; // jump to 1st stuff byte
							for (n=0; n<188; n+=sizeof(uint16_t)) {
								chksm ^= (uint16_t)*(pb+n);
							}
							*(uint16_t*)pbp = chksm;
						}
					}
					else {
						if (0x10 == adp ) {
							pbp = pb + 0x4; // jump to start adr of pes
							if (0x0 != *pbp || 0x0 != *(pbp+1) || 0x1 != *(pbp+2)) {
 #ifdef PHP_DBG
 			fprintf(stderr, "0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x\n",
 							*(pb+0), *(pb+1), *(pb+2), *(pb+3), *(pb+4), *(pb+5), *(pb+6));
#endif
								//perror_exit("invalid PES start code prefix found\n", -3);
								puts("invalid PES start code prefix found");
							}
							for (n=0; n<188; n+=sizeof(uint16_t)) {
								chksm ^= (uint16_t)*(pb+n);
							}
							*(uint16_t*)pbp = chksm;
						} else { // with adaptation field
							if (2<*(pb+0x4)) {
								pbp = (pb+0x4)+ *(pb+0x4)+1; // jump to start adr of pes
								if (0x0 != *pbp || 0x0 != *(pbp+1) || 0x1 != *(pbp+2)) {
	 #ifdef PHP_DBG
	 			fprintf(stderr, "0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x\n",
	 							*(pbp+0), *(pbp+1), *(pbp+2), *(pbp+3), *(pbp+4), *(pbp+5), *(pbp+6));
	#endif
									perror_exit("invalid PES start code prefix found\n", -3);
								}
								for (n=0; n<188; n+=sizeof(uint16_t)) {
									chksm ^= (uint16_t)*(pb+n);
								}
								for (n=0; n<sizeof(uint16_t); n++)
								*pbp++ = 0xff& (chksm>>(n*8));
							}
						}
					}
				}
 #ifdef PHP_DBG
 		fwrite(pb, 188, 1, fdbg);
 		fflush(fdbg);
#endif
				pb += 188;
			}
		}
#endif
#ifdef PES_FRM_PROT
 //#define TSP_DBG
  	static void checksum_pkt_ts(uint32_t size, uint8_t *tsbuf)
 		{
	 		uint8_t tmp, *pb = tsbuf;
	 		uint16_t chksm = 0;
	 		bool tp_skip ;
	 		uint32_t i, n;
 #ifdef TSP_DBG
 		static FILE *fdbg = 0;
 		if (!fdbg) fdbg = fopen("tsp_tx.dbg", "wb");
 #endif
 			for (i=0; i<size/188; chksm = 0, i++) {
	 			uint16_t pid = *(pb+0x2) | (0x1f00 & (((uint16_t)*(pb+0x1))<<8));
	 			tmp = (~0xc0 & *(pb+3));
	 			tp_skip= false ;
	 			switch(pid) {
		 			case PAT_PID:
		 				// cannot protect pat as it may look the same as scrambling ctrl bits 0!!! liyenho
		 				tp_skip = true; break;
		 			case PMT_PID:
		 				*(pb+3) = PMT | tmp; break;
		 			case VIDEO_PID:
		 				*(pb+3) = VIDEO | tmp; break;
		 			case AUDIO_PID:
		 				*(pb+3) = AUDIO| tmp; break;
		 			default : tp_skip = true; break;
				}
				if (tp_skip) {
					// flag transport priority bit to indicate chksm is NOT added
					*(pb+1) |= 0x20;
				}
				else {
					// unflag transport priority bit to indicate chksm is added
					 tmp= *(pb+1) & 0xc0;
					 *(pb+1) &= 0xdf;
					for (n=0; n<188; n+=sizeof(uint16_t)) {
						chksm ^= *(uint16_t*)(pb+n);
					}
					 // produce 13 bit checksum in place of pid
					*(pb+1) = tmp | (0x1f & (chksm>>8));
					*(pb+2) = chksm;
				}
 #ifdef TSP_DBG
 		fwrite(pb, 188, 1, fdbg);
 		fflush(fdbg);
#endif
				pb += 188;
			}
		}
#endif
  int lgdst_init_tx(int argc,char **argv)
{
	int r = 1, chsel_tx=0, pwr_attn=10000;
	do_exit = 1;
   pthread_mutex_init(&mux, NULL);
	r = libusb_init(NULL);
	if (r < 0)
		perror_exit("failed to initialise libusb",1);
	libusb_set_debug(NULL, 2);
	while (1) { // wait for atmel usb ready
#if (1/*0*/)
	devh = libusb_open_device_with_vid_pid(NULL,
					USB_DEV_VENDER, USB_DEV_PRODUCT);
#else  // multi-atmel dev support
    if (1+1 >= argc) {
    	puts("missing device address number, bailed out...");
    	libusb_exit(NULL);
    	return -1; }
    uint32_t dev_addr = atoi(argv[1]);
    if (0>dev_addr || 127<dev_addr) {
    	puts("invalid device address number, bailed out...");
    	libusb_exit(NULL);
    	return -2; }
	devh = libusb_open_device_with_vid_pid_adr(NULL,
					USB_DEV_VENDER, USB_DEV_PRODUCT,
					/*USB_DEV_ADR*/(uint8_t)dev_addr);
#endif
	// check for FW upgrade request from user cmdline, liyenho
    system_upgrade = (!strcasecmp(argv[2],"Uf0"))?5: // fpga image switch
    						((!strcasecmp(argv[2],"Ua0"))?4: // direct boot atmel
    						((!strcasecmp(argv[2],"Ua1"))?3:	// fwm upgrade atmel
    						((!strcasecmp(argv[2],"Uf"))?2:
    						((!strcasecmp(argv[2],"Uc"))?1: 0 ))));
		if (system_upgrade) {
				if (4>system_upgrade) {
					if (argc <4) {
						puts("missing upgrade firmware filepath, bailed out...");
						return -3; 	}
					strncpy(upgrade_fwm_path, argv[3], sizeof(upgrade_fwm_path));
				}
				goto libusb_next;
		}
  #if defined(SRC_FRM_ENET) && defined(RADIO_SI4463) && !defined(CTRL_RADIO_TEST)
  if (4 != argc)  { // udp setup require ip addr
	  printf("usage: sudo %s 0 ctrl-ip ctrl-rx-pt#, bailed out...\n",argv[0]);
  return -4; }
  ctrl_port_base = atoi(argv[3]);
  sprintf(servIP,"%s",argv[2]);
  #endif
libusb_next:
		if (devh <= 0) {
			//libusb_exit(NULL);
			//perror_exit("could not find/open USB device",2);
			puts("wait for atmel usb ready...");
		}
		else break;
	}
	r = libusb_claim_interface(devh,USB_DEV_INTF);
	if (r < 0) {
		r = libusb_detach_kernel_driver(devh,USB_DEV_INTF);
		if (r < 0)
			perror_exit("libusb_detach_kernel_driver error", r);
		else {
			detached = true;
			r = libusb_claim_interface(devh,USB_DEV_INTF);
			if (r < 0)
				perror_exit("usb_claim_interface error", r);
		}
	}
	printf("claimed interface\n");
	// send system restart command...
	libusb_control_transfer(devh,
						CTRL_OUT, USB_RQ,
						USB_SYSTEM_RESTART_VAL,
						USB_HOST_MSG_IDX,
						NULL, 0, 0);
	// bring up all others after main system restart command sent...
	r = pthread_create(&poll_thread, NULL, poll_thread_main, NULL);
	if (0 != r)
		perror_exit("poll thread creation error", r);
   /*if (SIG_ERR == signal(SIGINT, sigint_handler)) {
     perror("FAIL: assigning signal handler");
   }*/
   if (system_upgrade)
   	return 1;	// system upgrade or atmel reboot...
	r = pthread_create(&lgdst_thread, NULL, lgdst_thread_main, NULL);
	if (0 != r)
		perror_exit("lgdst thread creation error", r);
  #ifdef ATMEL_END2END
    extern int short_sleep(double sleep_time);
  	static int32_t i, sz, msg[80]; // access buffer
  #endif
#ifdef RFFE_PARAMS
   open_ini(&chsel_tx, &pwr_attn);  // get default chan/pwr settings
 	Rf_Params.params_tx.chan_idx = chsel_tx;
 	Rf_Params.params_tx.pwr_att = pwr_attn; // in mB scale
	libusb_control_transfer(devh, CTRL_OUT, USB_RQ,
		RF_TX_VAL, USB_HOST_MSG_IDX,
		&Rf_Params.params_tx, sizeof(Rf_Params.params_tx), 0);
	printf("check nios done bit...\n");
try_again:
	  	 libusb_control_transfer(devh,
					  	CTRL_IN, USB_RQ,
					  	RF_TX_NIOS_DONE,
					  	USB_HOST_MSG_IDX,
					  	&i, sizeof(i), 0);
		if (0x81 != i) {
			short_sleep(1); 	// read back result after 1 sec
			goto try_again;  // wait for nios being ready
		}
	printf("nios done bit is set...\n");
#endif
#ifdef SND
	//file = fopen(FILE_NAME,"rb");
#endif
#ifndef SRC_FRM_ENET
	/*if (!file)
		perror_exit("test file open failed, bail out",-3);*/
#endif
#ifdef MEDIA_ON_FLASH
   int mda_len = FILE_LEN;
	libusb_control_transfer(
		devh,
		CTRL_OUT,
		USB_RQ,
		USB_LOAD_MEDIA,
		USB_LOADM_IDX,
		&mda_len,
		USB_LOADM_LEN,
		0);
#endif
	tag = 0;
#if defined(ATMEL_END2END)
  	dev_access *acs = (dev_access*)msg;
#endif
#ifdef SND
 #ifndef SRC_FRM_ENET // extract ts packets from socket
	//fread(audbuf, FRAME_SIZE_A, 1,file);
 #else
 	ready_wait_for_mloop = true;
   /************************************************************************/
  udpin_init(); // initialize socket intf
   /************************************************************************/
	 if(0>receive(2549/*usec*/, audbuf, UDP_PACKET_MAX))
	  	perror_exit("invalid socket read, bailed out...", -6);
 #endif
#endif
	return 0;
}

void lgdst_deinit_tx(int rcode)
{
	if (0 > rcode || true==(bool) rcode) goto _fail;
#ifdef SND
	fprintf(stderr, "Wrap up/Leaving test_snd app...\n");
#endif
	if (0 > do_exit) return do_exit;  // return from error
   else do_exit = 4;

   pthread_join(poll_thread, NULL);
	pthread_join(lgdst_thread, NULL);
#ifdef SRC_FRM_ENET
		close(udpin_socket);
		close(udpout_socket);
#endif
#if defined(RADIO_SI4463) && !defined(CTRL_RADIO_TEST)
	close(ctrlsnd_socket);
	close(ctrlrcv_socket);
#endif
_fail:
	//libusb_release_interface(devh,USB_DEV_INTF20);
	libusb_release_interface(devh,USB_DEV_INTF);
	if (detached)
		//libusb_attach_kernel_driver(devh,USB_DEV_INTF20);
		libusb_attach_kernel_driver(devh,USB_DEV_INTF);
	libusb_close(devh);
	libusb_exit(NULL);
   pthread_mutex_destroy(&mux);
}

bool lgdst_upgrade_tx(int argc, char **argv)  // return -1 when failed, liyenho
{
  	float delay ;
	int r = 1, blksz=0;
 	uint32_t n, size, fw_info[3+1]; /*len = SMS_FW_HDR_LEN*/
  if (system_upgrade) { // user request firmware upgrade
  	FILE *file_up ;
  	int end, cur, len;
  	bool first_file = true;
	if (5/*fpga switch*/== system_upgrade) {
		do_exit = 4; // notify all other threads to exit
		short_sleep(2);
		pthread_mutex_lock(&mux);
		libusb_control_transfer(devh,
					CTRL_OUT,
					USB_RQ,
					USB_FPGA_NEW_VAL,
					USB_HOST_MSG_IDX,
					NULL, 0, 0);
		pthread_mutex_unlock(&mux);
		return false;
	}
upgrade_firmware:
	if (4>system_upgrade) {
		file_up = fopen(upgrade_fwm_path, "rb");
  		if (!file_up) {
	  		puts("failed to open upgrade firmware, bail out....");
	  		return (bool)-1;
  		}
	  	fread(&fw_info[0], 1, sizeof(int), file_up);// using 1st 4 bytes as sync word
	  	printf("upgrade firmware header: 0x%08x\n", fw_info[0]);
		cur = ftell(file_up);
		fseek(file_up,0,SEEK_END);
		end = ftell(file_up);
	  	fw_info[1] = len =end - cur + sizeof(int);
	  	printf("upgrade firmware length: %d bytes\n", fw_info[1]);
	  	fseek(file_up,0,SEEK_SET);
  	}
	if (!upgrade_from_lgdst) {
  		if (1/*cpld*/== system_upgrade) {
			//blksz = ???;
  			//...
	  	}
  		else if (2/*fpga*/== system_upgrade) {
	  		 {
	  		fw_info[2] = 0x00295700;  // constant fpga download address
	  			goto download;
  			}
#ifndef NON_NIOS
  next_file: // required two files to upgrade fpga/nios content
#endif
  			{
	  			if (!upgrade_from_lgdst)
	  				strncpy(upgrade_fwm_path, argv[4], sizeof(upgrade_fwm_path));
	  			else {
		  			char *ptr = upgrade_fwm_path;
		  			while (*ptr++) ;
	  				strncpy(upgrade_fwm_path, ptr, sizeof(upgrade_fwm_path));
  				}
	  			file_up = fopen(upgrade_fwm_path, "rb");
			  	fread(&fw_info[0], 1, sizeof(int), file_up);// using 1st 4 bytes as sync word
			  	printf("upgrade firmware header: 0x%08x\n", fw_info[0]);
				cur = ftell(file_up);
				fseek(file_up,0,SEEK_END);
				end = ftell(file_up);
			  	fw_info[1] = len =end - cur + sizeof(int);
			  	printf("upgrade firmware length: %d bytes\n", fw_info[1]);
			  	fseek(file_up,0,SEEK_SET);
			  	fw_info[2] = 0x0052ae00;  // constant nios download address
			  	first_file = false;
  			}
download:
	  		printf("upgrade firmware address: 0x%08x\n", fw_info[2]);
		 libusb_control_transfer(devh, CTRL_OUT, USB_RQ,
			USB_FPGA_UPGRADE_VAL, USB_HOST_MSG_IDX,
			fw_info, FW_UPGRADE_HDR_LEN, 0);
			blksz = 15000;
			delay = /*3.5*/0.05;
  		}
  		else if (3/*atmel*/== system_upgrade) {
		 libusb_control_transfer(devh, CTRL_OUT, USB_RQ,
			USB_FWM_UPDATE_VAL, USB_HOST_MSG_IDX,
			fw_info, ATMEL_UPGRADE_HDR_LEN, 0);
			blksz = 7200;
			delay = 0.84;
  		}
  		else if (4/*atmel boot*/== system_upgrade) {
		 libusb_control_transfer(devh, CTRL_OUT, USB_RQ,
			USB_FWM_BOOTUP_VAL, USB_HOST_MSG_IDX,
			NULL, 0, 0);
			short_sleep(0.01);
			return true; // it is done
  		}
		else {
			puts("invalid target for upgrade request, bail out....");
			return (bool)-1; }
	}
	// no more 8192 (divisible into 64) if we'll need to perform usb read after write verification!!!
	// a big bug in atmel usb core!!! liyenho
	if (blksz*2 > len) {
		fclose(file_up);
		perror_exit("upgrade firmware data is too short, bail out",-5);
	}
	// begin download process thru usb
 #ifdef FWUP_DNLD_DBG
 	static FILE *fdmp;
	long rem, cur1;
	char fd[128], *dfn, ta;
  strncpy(fd, upgrade_fwm_path, sizeof(fd));
 	dfn = strrchr(fd, '.');
 	strcpy(dfn, ".dmp");
 	fdmp = fopen(fd,"wb");
 	if (!fdmp) {
	 	fclose(file_up);
	 	perror_exit("can't open fw read back dump file",-8);
 	}
 #endif
		cur = blksz;
 		do {
			 libusb_control_transfer(devh,
					CTRL_IN, USB_RQ,
					USB_STREAM_ON_VAL,
					USB_QUERY_IDX,
					&main_loop_on, sizeof(main_loop_on), 0);
			if (!main_loop_on) {
				short_sleep(1); 	// setup & settle in 1 sec
			} else
			break;
		} while (1);
 #ifdef FWUP_DNLD_DBG
 		rem = len;
 		do {
 			libusb_control_transfer(devh, CTRL_IN, USB_RQ, 0x7d, 0, &ta, sizeof(ta), 0);
 			short_sleep(0.005);
		} while (0==ta);
 		libusb_control_transfer(devh, CTRL_OUT, USB_RQ, 0xd7, 0, NULL, 0, 0);
 #endif
		n = 1;
		// send 1st chunk outside of loop to align xfer at target side
			fread(vidbuf, cur, 1,file_up);
				 r=libusb_bulk_transfer(
				 devh, 2 | LIBUSB_ENDPOINT_OUT,
				 vidbuf,
				 cur,
				 &size,
				 0);
			printf("fw data block #%d sent (%d)...\n", n++,size);
			len -= cur;
		short_sleep(1); // chk[] search may take time?!
		while(1) {
			if (1 != do_exit) break;
			fread(vidbuf, cur, 1,file_up);
 #ifdef FWUP_DNLD_DBG
 		do {
 			libusb_control_transfer(devh, CTRL_IN, USB_RQ, 0x7d, 0, &ta, sizeof(ta), 0);
 			short_sleep(0.005);
		} while (0==ta);
 		libusb_control_transfer(devh, CTRL_OUT, USB_RQ, 0xd7, 0, NULL, 0, 0);
 #endif
				 r=libusb_bulk_transfer(
				 devh, 2 | LIBUSB_ENDPOINT_OUT,
				 vidbuf,
				 cur,
				 &size,
				 0);
			printf("fw data block #%d sent (%d)...\n", n++,size);
 #ifdef FWUP_DNLD_DBG
 		if (blksz>cur) {
			fread(vidbuf, cur, 1,file_up);
			fclose(file_up);
			 r=libusb_bulk_transfer(
				 devh, 2 | LIBUSB_ENDPOINT_OUT,
				 vidbuf,
				 10*EXTRA,  // send something to flush usb bulk pipe, liyenho
				 &size,
				 0);
 		}
 #endif
		short_sleep(delay); // sync up with target/flash access on max timing
 #ifdef FWUP_DNLD_DBG
  		if (blksz<=rem) {
  			cur1 = blksz;
  			rem -= blksz;
	 		libusb_control_transfer(devh, CTRL_OUT, USB_RQ, 0xd7, 0, NULL, 0, 0);
			do {
			 r = libusb_bulk_transfer(
				 devh, (1 | LIBUSB_ENDPOINT_IN),
				 vidbuf+blksz,
				 cur1,
				 &size,
				 50);
			} while (1 == do_exit &&((size!=cur1) || r<0));
	    	printf("dumping read fw data block(%d)...\n",n-2);
	    	fwrite(vidbuf+blksz, size, 1, fdmp);
 		}
 #endif
			len -= cur;
			if (!len) break;
			cur =(len>=blksz)?blksz:len;
		}
 #ifndef FWUP_DNLD_DBG
		fread(vidbuf, cur, 1,file_up);
		fclose(file_up);
		 r=libusb_bulk_transfer(
			 devh, 2 | LIBUSB_ENDPOINT_OUT,
			 vidbuf,
			 10*EXTRA,  // send something to flush usb bulk pipe, liyenho
			 &size,
			 0);
 #else //FWUP_DNLD_DBG
	  		cur1 = rem;
	  		rem = 0; // last run
			short_sleep(delay); // sync up with target/flash access on max timing
	 		libusb_control_transfer(devh, CTRL_OUT, USB_RQ, 0xd7, 0, NULL, 0, 0);
			do {
			 r = libusb_bulk_transfer(
				 devh, (1 | LIBUSB_ENDPOINT_IN),
				 vidbuf+blksz,
				 cur1,
				 &size,
				 50);
				 											// can't always get full packet, perhaps just get as much as we have...
			} while (1 == do_exit &&((size/*!=cur1*/<100) || r<0));
	    	printf("dumping read fw data block(%d)...\n",n-1);
	    	fwrite(vidbuf+/*7200*/blksz, size, 1, fdmp);
    // close readback dump file
	fclose(fdmp);
 #endif
 	if (2<system_upgrade || (4== argc && !strcasecmp(argv[3], upgrade_fwm_path)))
		 return true; // finished regardless if there's error
	else {
#ifndef NON_NIOS
		if (2 == system_upgrade && first_file)
			goto next_file;
	else
#endif
		return false; // normal shutdown
	}
  }
}

void lgdst_ts_tx(uint8_t *tsbuf)
{
	tag += 1;
	while(0>stream_block(tsbuf));
#ifdef SND
      if(DEBUG_VIDEO)
                      { static int tpbprtcnt=0;
                        tpbprtcnt++; if(tpbprtcnt>100){ tpbprtcnt=0;
			printf("sent a transport packet block, %d.......\n",tag);
                      }
                      }
#endif
#ifdef SND
  #ifndef SRC_FRM_ENET // extract ts packets from socket
			;
  #else
	 if(0>receive(2549/*usec*/, tsbuf, UDP_PACKET_MAX))
	  	perror_exit("invalid socket read, bailed out...", -6);
  #endif
#endif
		if (ITERS==tag)
			tag = 0; // make it endless, liyenho
}

/**********************************************************************************************************/
#ifdef SRC_FRM_ENET
static int udpin_init(void)
{
  int rr,rc,i;
  u_int port_common_flag = 1;
  struct ifreq   *ifr_ipsrc;
  struct ifreq   ifrr_ipsrc;
  struct sockaddr_in sa;
  char ipsrc[32];

  struct sched_param  schparm, schparm_obs;
  int policy;
  char multip[20];
 #define UDPIN_PORT 5553
 #define UDPIN_MULTIADDR "232.168.54.200"

/* Setup UDP Socket
   sprintf(multip, "%d.%d.%d.%d", (shmem_snfdec_local->dec_mult_ipaddr) & 0xff,
							((shmem_snfdec_local->dec_mult_ipaddr) >> 8) & 0xff,
							((shmem_snfdec_local->dec_mult_ipaddr) >> 16) & 0xff,
							((shmem_snfdec_local->dec_mult_ipaddr) >> 24) & 0xff);
   printf("multip = %s\n",multip);
  */
  udpin.sin_family = AF_INET;
  udpin.sin_port = htons(UDPIN_PORT);
  udpin.sin_addr.s_addr = htonl(INADDR_ANY);
  udpin_len = sizeof(udpin);
  udpin_socket = socket(AF_INET,SOCK_DGRAM,0);
  if(udpin_socket <0)
  { printf("Fail to open udpin socket\n"); return -1;}

  /* Setup Common PORT use (for multicasting) */
  if(setsockopt(udpin_socket,SOL_SOCKET,SO_REUSEADDR,
                &port_common_flag, sizeof(port_common_flag))<0)
  { printf("Fail to set common port state.\n"); return -1;}

  /* Binding Socket */
  rr=bind(udpin_socket,(struct sockaddr*)&udpin,udpin_len);
  if(rr<0)
  { printf("Fail to bind udpin socket.\n"); return -1;}

  /* Enable Multicast read */
  udpin_mreq.imr_multiaddr.s_addr = inet_addr(UDPIN_MULTIADDR);
   ifr_ipsrc = &ifrr_ipsrc;
  ifrr_ipsrc.ifr_addr.sa_family = AF_INET;
  strncpy(ifrr_ipsrc.ifr_name, "eth1", sizeof(ifrr_ipsrc.ifr_name));
  ioctl(udpin_socket,SIOCGIFADDR, ifr_ipsrc);
  strncpy(ipsrc,inet_ntoa(
     (*(struct in_addr *) &ifr_ipsrc->ifr_addr.sa_data[sizeof sa.sin_port])),
     16);
  udpin_mreq.imr_interface.s_addr = inet_addr(ipsrc);
  rr = setsockopt(udpin_socket, IPPROTO_IP,IP_ADD_MEMBERSHIP,
             &udpin_mreq,sizeof(udpin_mreq));
  if(rr <0)
  printf("ipmulticast = %x %x\n",udpin_mreq.imr_multiaddr.s_addr,
                             udpin_mreq.imr_interface.s_addr );
  return 0;
}

static int udpout_init(void)
{
  #define UDPOUT_ADDR "192.168.3.101"
  #define UDPOUT_PORT          5588
  memset(&udpout,0,sizeof(udpout));
  udpout.sin_family = AF_INET;
  udpout.sin_port = htons(UDPOUT_PORT);
  udpout.sin_addr.s_addr = inet_addr(UDPOUT_ADDR);
  udpout_len = sizeof(udpout);
  udpout_socket = socket(AF_INET, SOCK_DGRAM,0);
  if(udpout_socket <0)
  { printf("Fail to open udpout socket\n"); return -1; }
  return 0;
}

static int receive(int usec, unsigned char *pb, uint32_t bytes) {
  static unsigned int net_loc_recv_buf[2*UDP_PACKET_MAX/sizeof(unsigned int)];
  static unsigned int loc_buf_ptr = 0;
  unsigned char *pbf = (unsigned char*)net_loc_recv_buf+loc_buf_ptr;
	int err, recvsize;
   udpin_tv.tv_sec = 0;
   udpin_tv.tv_usec = usec;
   while(/*UDP_PACKET_MAX*/bytes>loc_buf_ptr) {
    FD_ZERO(&udpin_fd);
    FD_SET(udpin_socket,&udpin_fd);
    err = select(udpin_socket+1,&udpin_fd,0,0,&udpin_tv);
    if(err<0) {
		  printf("select failed\n");
  		  return err;
	  }
    if (FD_ISSET(udpin_socket,&udpin_fd)) {
      // Ethernet input data processing
    	  recvsize= recvfrom(udpin_socket,pbf,UDP_PACKET_MAX,0,
										  (struct sockaddr *)&udpin,&udpin_len);
      loc_buf_ptr += recvsize;
      pbf += recvsize;
    }
   }
   pbf = (unsigned char*)net_loc_recv_buf;
   memcpy(pb, pbf, bytes);
   loc_buf_ptr -= bytes;
   memmove(pbf,
           pbf+bytes,
           loc_buf_ptr);
  return 0;
}
#endif

/*********************** fpga_access source *************************/
