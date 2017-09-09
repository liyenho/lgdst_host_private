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
 #include <time.h>
 #include <sys/time.h>

#include <libusb.h>
#include <assert.h>

#include "usb_tx.h"  // definitions for host/fpga access @ tx
#include "radio_link_quality.h"
#define true														1
#define false														0

//#define SENSITIVITY_TESTING //define to print out packet ID and payload statistics

volatile uint8_t system_upgrade = 0;
volatile uint8_t boot_and_app = 0;
volatile uint8_t upgrade_boot_and_app=0;
volatile uint8_t boot_only = 0;
volatile uint8_t app_only = 0;
char upgrade_fwm_path[160]; // upgrade firmware path

	volatile uint8_t main_loop_on = false; // run time indicator, liyenho
  #define CTRL_OUT		(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT)
  #define CTRL_IN		(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN)
  #define USB_RQ			0x04
 int shmid_Lgdst = -1;
 volatile ipcLgdst *shmLgdst_proc = 0;
 #ifdef RADIO_SI4463
  volatile bool si4463_radio_up = false ;
 #endif
 //#define COMM_ATMEL_DEV // enable atmel<->fpga/6612 comm test
#ifdef SRC_FRM_ENET
 volatile bool ready_wait_for_mloop= false;
#endif
 	#ifdef RADIO_SI4463
 	unsigned short ctrl_port_base;
  char servIP[32];
 	int ctrlsnd_socket, ctrlrcv_socket; //clnt_socket;
  struct timeval ctrlsnd_tv;
  fd_set ctrlsnd_fd;
  int ctrlsnd_len;
 	struct sockaddr_in ctrlsnd, ctrlrcv; //clnt;
 	#endif
	#ifdef SRC_FRM_ENET // extract ts packets from socket
	 #define UDP_PACKET_MAX 1880
		int udpin_len, udpout_len;
		int udpin_socket, udpout_socket;
		fd_set udpin_fd;
		struct timeval udpin_tv= {0};
		struct sockaddr_in udpin, udpout;
		struct ip_mreq udpin_mreq;
		int udpin_init(void);
	  int udpout_init(void);
	  int receive(int usec, unsigned char *pb, uint32_t bytes);
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

//#define SEND_TEST_PATTERN    //Generate a test pattern tracks individual frames
#ifdef SEND_TEST_PATTERN
	int GetTestPattern(unsigned char *pb, uint32_t num_bytes);
#endif

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
/*static*/volatile int do_exit = 0;	// main loop breaker
static pthread_t poll_thread= 0,
									lgdst_thread = 0,
									ctrl_thr_recv = 0,
									ctrl_thr_send = 0;
 static pthread_mutex_t mux;


//function prototyping
  extern struct timeval tstart,tend,tdelta;
  extern void print_time(struct timeval ttime);
  extern int time_diff(struct timeval *time1,
  								struct timeval *time2,
                                struct timeval *diffTime);
  extern int get_time(struct timeval *time);

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
static int stream_block() {
	int r = 0, transferred;
	r=libusb_bulk_transfer(
		devh,
		EP_DATA,
		audbuf,
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
if (r<0){
	printf("Bulk transfer C\t");
	printf("xxx r=%d, transferred=%d xxx\n", r, transferred);
	printf("Error transmitting in *stream_block*!!!!!\n");
}
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
          if(validdataflag==0x01) //got valid payload
          {
				  pv_wrbyte = sendto(ctrlrcv_socket,radio_rpacket,RADIO_USR_RX_LEN,0,
				  											(struct sockaddr *)&ctrlrcv, sizeof(ctrlrcv));
          }
		if(DEBUG_CTRLRX)
    {
     #if !defined SENSITIVITY_TESTING
      if(validdataflag)
      //if((radio_rpacket[0]==0xe5)&&(radio_rpacket[1]==0xe5))
      //if((radio_rpacket[0]!=0xee)||(radio_rpacket[1]!=0xee))
      {
        static int pcnt=0;
        //pcnt++; if(pcnt>5) pcnt=0;  //print every 5th packet
        if(pcnt==0){
          printf("radio link Rx: ");
          for(i=0;i<sizeof(radio_rpacket);i++){
          if(i==2) printf(" ");
          if(i==4) printf(" ");
          printf("%02x ",radio_rpacket[i]);}
          printf("\n");
        }
      }
      #endif

      {// idle packet dataloss ----------------------------------
        static unsigned char val_exp;
        unsigned char        val_cur;
        static int ilcnt=0, becnt=0;
         unsigned char byteerror=0;
        if((radio_rpacket[0]==0xe5)&&(radio_rpacket[1]==0xe5)){
          for(i=6;i<RADIO_USR_RX_LEN;i++)
            if(radio_rpacket[i]!=0xe5) {
               byteerror++; becnt++;}
          if(byteerror > 0) printf("Idle packet corruption = %d. ++++++++++\n", becnt);
          val_cur = radio_rpacket[2];
          val_exp = (unsigned char)(val_exp+1);
          if(val_exp != val_cur) {
            ilcnt++;
            printf("Idle packet dataloss = %d. !!!!!!!!!!!!!!!!!!!!!!!!!!!!\n",ilcnt);
          }
          val_exp = val_cur;
        }//end 0xe5 0xe5
      }// end idle packet loss check

      {// user packet dataloss ---------------------------------

        static unsigned char val_exp;
        unsigned char        val_cur;
        static int ilcnt=0, becnt=0;
         unsigned char byteerror=0;
         if((validdataflag==0x01)&&
          ((radio_rpacket[0]!=0xe5)||(radio_rpacket[1]!=0xe5)||(radio_rpacket[2]!=0xe5))
          ){
          	#ifdef SENSITIVITY_TESTING
          		check_packet(radio_rpacket);
        	#else
          		for(i=6;i<RADIO_USR_RX_LEN;i++)
            		if(radio_rpacket[i]!=0xb5) {
               			byteerror++; becnt++;
               		}
          		if(byteerror > 0) printf("User packet corruption = %d. ++++++++++\n", becnt);
          		val_cur = radio_rpacket[5];
          		val_exp = (unsigned char)(val_exp+1);
        		if(val_exp != val_cur) {
          		  ilcnt++;
          		  printf("User packet dataloss = %d. !!!!!!!!!!!!!!!!!!!!!!!!!!!!\n",ilcnt);
        		}
         		val_exp = val_cur;
            #endif
        }//end valid data
      }// user packet dataloss check
    }//DEBUG_CTRLRX

			usleep(CTRL_RECV_POLLPERIOD);
  		} //ctrl_sckt_ok
	}//(1==do_exit)
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
#ifdef SRC_FRM_ENET
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
		} while (1);
		pthread_mutex_unlock(&mux);
#endif
#if defined(RADIO_SI4463) && defined(SRC_FRM_ENET)
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
#if defined(RADIO_SI4463) && defined(SRC_FRM_ENET)
	pthread_join(ctrl_thr_recv, NULL);
	pthread_join(ctrl_thr_send, NULL);
#endif
	printf("poll thread shutting down\n");
	return NULL;
}

static void *lgdst_thread_main(void *arg)
{
	if (0>(shmid_Lgdst = shmget(SHMKEY_TX,
		sizeof(ipcLgdst),IPC_CREAT|0666))) {
		perror_exit("failed to create IPC memory for lgdst process, bailed out...", -7);
	}
	if (0>(shmLgdst_proc=shmat(shmid_Lgdst, NULL, 0))) {
	  perror_exit("get shmLgdst_proc shmem failed",0);
  }
	memset(shmLgdst_proc, 0, sizeof(ipcLgdst));
	shmLgdst_proc->active = -1; // shm signalling flag to lgdst client
	static dAccess lclMem ;
	while (1==do_exit) {
    short_sleep(0.1);
		dev_access *acs = &lclMem.hdr;
		if (1 == shmLgdst_proc->active) {
			memcpy(&lclMem, &shmLgdst_proc->access, sizeof(dAccess));
			switch(shmLgdst_proc->type) {
				case CMD0:
					if (USB_FPGA_NEW_VAL == shmLgdst_proc->tag.wValue) {
    					system_upgrade = 4;
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

static void sigint_handler(int signum)
{
   do_exit = 4; // this shall terminate poll thread
   pthread_mutex_unlock(&mux);
}
#ifdef RFFE_PARAMS
bool open_ini(int *setting_ch,int *setting_pwr)
{
	FILE *fp = NULL;
        char ini_buff[120];

        fp = fopen("usb_tx.ini", "r+");
	if (fp == NULL) {
	  // File doesn't exist, setup ini with default
       	  fp = fopen("usb_tx.ini", "w+");
	  if (fp == NULL) {
		printf("ERROR: Fail to create 'usb_tx.ini' !!");
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

void usb_open(int argc_sub, char **argv_sub)
{
  int  r = 1;
  	r = libusb_init(NULL);
	if (r < 0)
		perror_exit("failed to initialise libusb",1);
	libusb_set_debug(NULL, 2);
	while (1) { // wait for atmel usb ready
#ifdef _ONE_USB_
	devh = libusb_open_device_with_vid_pid(NULL,
					USB_DEV_VENDER, USB_DEV_PRODUCT);
#else
            if (1+1 >= argc_sub) {
            	puts("missing device address number, bailed out...");
            	libusb_exit(NULL);
            	return -1; }
            uint32_t dev_addr = atoi(argv_sub[1]);
            if (0>dev_addr || 127<dev_addr) {
             	puts("invalid device address number, bailed out...");
            	libusb_exit(NULL);
            	return -2; }
        	   devh = libusb_open_device_with_vid_pid_adr(NULL,
        					USB_DEV_VENDER, USB_DEV_PRODUCT,
        					/*USB_DEV_ADR*/(uint8_t)dev_addr);
#endif
	  	if (devh <= 0) {
			//libusb_exit(NULL);
			//perror_exit("could not find/open USB device",2);
			puts("wait for atmel usb ready...");
	  	}
		else break;
	}
	//r = libusb_claim_interface(devh,USB_DEV_INTF20);
	r = libusb_claim_interface(devh,USB_DEV_INTF);
	if (r < 0) {
		//r = libusb_detach_kernel_driver(devh,USB_DEV_INTF20);
		r = libusb_detach_kernel_driver(devh,USB_DEV_INTF);
		if (r < 0)
			perror_exit("libusb_detach_kernel_driver error", r);
		else {
			detached = true;
			//r = libusb_claim_interface(devh,USB_DEV_INTF20);
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
}

int system_upgrade_proc(int system_upgrade_v, int usbmsg_enable, char *upgrade_fwm_path_v)
{
 	  uint32_t n,r, blksz=0, size, fw_info[3+1]; /*len = SMS_FW_HDR_LEN*/
 	  bool first_file = true;
    float delay=0;
  	FILE *file_up ;
  	int end, cur, len;
		if (4/*fpga switch*/== system_upgrade_v) {
			do_exit = 4; // notify all other threads to exit
			short_sleep(2);
			pthread_mutex_lock(&mux);
			libusb_control_transfer(devh,
						CTRL_OUT,
						USB_RQ,
						shmLgdst_proc->tag.wValue,
						shmLgdst_proc->tag.wIndex,
						NULL, 0, 0);
			pthread_mutex_unlock(&mux);
			return 0;
		}
		file_up = fopen(upgrade_fwm_path_v, "rb");
  		if (!file_up) {
	  		puts("failed to open upgrade firmware, bail out....");
	  		return(1);
  		}
	  	fread(&fw_info[0], 1, sizeof(int), file_up);// using 1st 4 bytes as sync word
	  	printf("upgrade firmware header: 0x%08x\n", fw_info[0]);
		cur = ftell(file_up);
		fseek(file_up,0,SEEK_END);
		end = ftell(file_up);
	  	fw_info[1] = len =end - cur + sizeof(int);
	  	printf("upgrade firmware length: %d bytes\n", fw_info[1]);
	  	fseek(file_up,0,SEEK_SET);

	if (usbmsg_enable) {
  		if (1/*cpld*/== system_upgrade_v) {
			//blksz = ???;
  			//...
	  	}
  		else if (2/*fpga*/== system_upgrade_v) {
	  		 {
	  		fw_info[2] = 0x00295700;  // constant fpga download address
	  			goto download;
  			}
#ifndef NON_NIOS
  next_file: // required two files to upgrade fpga/nios content
#endif
  			{
	  			{
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
  		else if (3/*atmel*/== system_upgrade_v) {
		 libusb_control_transfer(devh, CTRL_OUT, USB_RQ,
			USB_FWM_UPDATE_VAL, USB_HOST_MSG_IDX,
			fw_info, ATMEL_UPGRADE_HDR_LEN, 0);
			blksz = 7200;
			delay = 0.84;
  		}
		else {
			puts("invalid target for upgrade request, bail out....");
			return(1); }
	}
	// no more 8192 (divisible into 64) if we'll need to perform usb read after write verification!!!
	// a big bug in atmel usb core!!! liyenho
	if (blksz*2 > len) {
		fclose(file_up);
		printf("upgrade firmware data is too short, bail out",-5);
		return(1);
	}
	// begin download process thru usb
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

				 r=libusb_bulk_transfer(
				 devh, 2 | LIBUSB_ENDPOINT_OUT,
				 vidbuf,
				 cur,
				 &size,
				 0);
			printf("fw data block #%d sent (%d)...\n", n++,size);

		short_sleep(delay); // sync up with target/flash access on max timing

			len -= cur;
			if (!len) break;
			cur =(len>=blksz)?blksz:len;
		}

		fread(vidbuf, cur, 1,file_up);
		fclose(file_up);
		 r=libusb_bulk_transfer(
			 devh, 2 | LIBUSB_ENDPOINT_OUT,
			 vidbuf,
			 10*EXTRA,  // send something to flush usb bulk pipe, liyenho
			 &size,
			 0);
 	if (2<system_upgrade_v)
		 return(1); // finished regardless if there's error
	else {
#ifndef NON_NIOS
		if (2 == system_upgrade_v && first_file)
			goto next_file;
	else
#endif
		return (0); // normal shutdown
	}
}

void usb_terminate(void)
{
  	//libusb_release_interface(devh,USB_DEV_INTF20);
	libusb_release_interface(devh,USB_DEV_INTF);
	if (detached)
		//libusb_attach_kernel_driver(devh,USB_DEV_INTF20);
		libusb_attach_kernel_driver(devh,USB_DEV_INTF);
	libusb_close(devh);
	libusb_exit(NULL);

}

void mutex_terminate(void){
     pthread_mutex_destroy(&mux);
}

void bl_jump_to_app(void) //bootLoader command to jusp to application start address
{
		 libusb_control_transfer(devh, CTRL_OUT, USB_RQ,
			USB_FWM_BOOTUP_VAL, USB_HOST_MSG_IDX,
			NULL, 0, 0);
			short_sleep(0.01);
			return(1); // it is done
 }

static void print_cmd_usage(char **argv)
{
	      printf("usage: sudo ./%s 0 ctrl-ip ctrl-rx-pt#, bailed out...\n",argv[0]);
        printf("usage: sudo ./%s 0 Ua0<ret>, leave BL start atmel app\n");
        printf("usage: sudo ./%s 0 Ua1 filename<ret>, upgrade, stay in BL\n");
        printf("usage: sudo ./%s 0 Bta leave BL, start atmel app, and start host app\n");
        printf("usage: sudo ./%s 0 Ubta upgrade,leave BL, start atmel app, and start host app\n");
}
int main(int argc,char **argv)
{
  int r=1, blksz=0, tag, chsel_tx=0, pwr_attn=10000;

  do_exit = 1;
  if (3>argc)  {
      print_cmd_usage(argv);
        exit(1); }

  //command parsing and integrity check ////////////////////////////////////////////////
  system_upgrade =
    						((!strcasecmp(argv[2],"Ua1"))?3:	// fwm upgrade atmel
    						((!strcasecmp(argv[2],"Uf"))?2:
    						((!strcasecmp(argv[2],"Uc"))?1: 0 )));
  boot_only = (!strcasecmp(argv[2],"Ua0"))?1:0;
  boot_and_app = (!strcasecmp(argv[2],"Bta"))?1:0; //single command boot and run app
  upgrade_boot_and_app = (!strcasecmp(argv[2],"Ubta"))?1:0; //single upgrade boot, and app
  app_only = ((boot_and_app==0)&
              (boot_only==0) &
              (upgrade_boot_and_app==0)&
              (system_upgrade==0)); //no boot application only

  if(app_only){
      if (4 != argc)  { // udp setup require ip addr
      print_cmd_usage(argv);
        exit(1); }
      ctrl_port_base = atoi(argv[3]);
      sprintf(servIP,"%s",argv[2]);
  }

  if(boot_and_app){
      if (5 != argc)  { // udp setup require ip addr
      print_cmd_usage(argv);
        exit(1); }
      ctrl_port_base = atoi(argv[4]);
      sprintf(servIP,"%s",argv[3]);
  }

  if ((system_upgrade)||(upgrade_boot_and_app)) {
		  if (argc <4) {
						puts("missing upgrade firmware filepath, bailed out...");
						exit(1); 	}
		  strncpy(upgrade_fwm_path, argv[3], sizeof(upgrade_fwm_path));
  }

  if(upgrade_boot_and_app){
    if(6 != argc){
      print_cmd_usage(argv);
      exit(1);}
    ctrl_port_base = atoi(argv[5]);
    sprintf(servIP,"%s",argv[4]);

  }



  //global initializations ////////////////////////////////////////////////////////////
  pthread_mutex_init(&mux, NULL);
  if (SIG_ERR == signal(SIGINT, sigint_handler)) {
     perror("FAIL: assigning signal handler");
  }
  usb_open(argc, argv);

  //upgrade routines //////////////////////////////////////////////////////////////////
  if(system_upgrade) {
    r = system_upgrade_proc(system_upgrade, 1, upgrade_fwm_path);
	usb_terminate();
  mutex_terminate();
	exit(1);
  }

  if(boot_only)
  {
    bl_jump_to_app();
	  usb_terminate();
    mutex_terminate();
	  exit(1);
  }

  if(boot_and_app)
  {
	//single call to bl loader to jump to app, and run host usb code
	bl_jump_to_app();
  sleep(1);
  usb_terminate();
  sleep(5);
  usb_open(argc,argv);
	sleep(5);
	libusb_control_transfer(devh,
				CTRL_OUT,
				USB_RQ,
				USB_FPGA_NEW_VAL,
				USB_HOST_MSG_IDX,
				NULL, 0, 0);
	sleep(1);
  usb_terminate();
  sleep(5);
  usb_open(argc,argv);
  }

  if(upgrade_boot_and_app)
  {
	system_upgrade = 3;
	r = system_upgrade_proc(system_upgrade, 1, upgrade_fwm_path);
	system_upgrade = 0;
  sleep(5);
	bl_jump_to_app();
  sleep(1);
  usb_terminate();
  sleep(5);
  usb_open(argc,argv);
  }

//Setup Continous process routines (threads) /////////////////////////////////////////
  r = pthread_create(&poll_thread, NULL, poll_thread_main, NULL);
  if (0 != r) {printf("poll thread creation error"); usb_terminate(); mutex_terminate();}

	r = pthread_create(&lgdst_thread, NULL, lgdst_thread_main, NULL);
	if (0 != r)
		perror_exit("lgdst thread creation error", r);

	/* we shouldn't bother issues of USB hw setup/config */
    extern int short_sleep(double sleep_time);
  	static int32_t i, sz, msg[80]; // access buffer

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
			printf("Trying again\n");
			goto try_again;  // wait for nios being ready
		}
	printf("nios done bit is set...\n");
#endif

	file = fopen(FILE_NAME,"rb");
 #ifdef TEST_BITSTREAM
 	FILE *fdbg = fopen("loopback.dbg","wb");
 #endif

#ifndef SRC_FRM_ENET
	if (!file)
		perror_exit("test file open failed, bail out",-3);
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

  	dev_access *acs = (dev_access*)msg;

	get_time(&tstart);

 #ifndef SRC_FRM_ENET // extract ts packets from socket
	fread(audbuf, FRAME_SIZE_A, 1,file);
 #else
 	ready_wait_for_mloop = true;
   /************************************************************************/
  udpin_init(); // initialize socket intf
  #ifdef TEST_BITSTREAM
  	 udpout_init();
  #endif
   /************************************************************************/
	 #ifdef SEND_TEST_PATTERN
	 	GetTestPattern(audbuf,UDP_PACKET_MAX);
	 #else
	 	if(0>receive(2549/*usec*/, audbuf, UDP_PACKET_MAX))
	  		perror_exit("invalid socket read, bailed out...", -6);
	 #endif
 #endif

	tag += 1;
	while (ITERS>=tag) {
     if (1 != do_exit) break; // we can't fail here......
     if(system_upgrade) //can only be triggered by external lgdst call
	 {
	   // TODO: turn off video
	   r = system_upgrade_proc(system_upgrade, 1, upgrade_fwm_path);
	   if(r>0) {usb_terminate(); mutex_terminate(); exit(1);}
	   system_upgrade = 0;
	 }


 #if defined(PES_HDR_PROT) || defined(PES_FRM_PROT)
  #ifdef SRC_FRM_ENET
		size = UDP_PACKET_MAX;
  #else
 		size = FRAME_SIZE_A;
  #endif
  	 #ifdef PES_HDR_PROT
  		checksum_pes_hdr(size, audbuf);
  	 #else //PES_FRM_PROT
  	 	checksum_pkt_ts(size, audbuf);
  	 #endif
 #endif
		if (0>stream_block()) continue;

		else /*printf("sent an audio block, %d.......\n",tag);*/
      if(DEBUG_VIDEO)
                      { static int tpbprtcnt=0;
                        tpbprtcnt++; if(tpbprtcnt>100){ tpbprtcnt=0;
			printf("sent a transport packet block, %d.......\n",tag);
                      }
                      }
		if (ITERS==tag)
			break;	// done


  #ifdef TEST_BITSTREAM
    #ifndef SRC_FRM_ENET
		short_sleep((3000)*10e-6);
	 #endif
  #else
    #ifndef SRC_FRM_ENET
  		// approxmate rate @ 4.6 mb/s, half of it was spent in actual transmission, liyenho
		short_sleep((2506/1.25)*10e-6);
	 #endif
  #endif



	#ifdef TEST_BITSTREAM
		static int ep = (1 | LIBUSB_ENDPOINT_IN);
		unsigned char tmpbuf[FRAME_SIZE_A];
		int rr,  td;
			 rr=libusb_bulk_transfer(
			 devh, ep,
			 tmpbuf,
			 FRAME_SIZE_A,
			 &td,
			 50);
			//printf("xxx rr=%d, td=%d xxx\n", rr, td);
	#endif


		if (ITERS-1==tag)
	      #ifndef MEDIA_ON_FLASH
			r = FILE_LEN - (tag+1)*FRAME_SIZE_A;
		  #else
			r = FRAME_SIZE_A; // len is guaranteed to be multiple of block
		  #endif
		else
			r = FRAME_SIZE_A;

#ifdef SND
  #ifndef SRC_FRM_ENET // extract ts packets from socket
			fread(audbuf, r, 1,file);
  #else
	#ifdef SEND_TEST_PATTERN
			GetTestPattern(audbuf,UDP_PACKET_MAX);
	#else
		 if(0>receive(2549/*usec*/, audbuf, UDP_PACKET_MAX))
		  	perror_exit("invalid socket read, bailed out...", -6);
	#endif
  #endif
  #ifdef TEST_BITSTREAM
  			if (1880==td)
	  #ifndef SRC_FRM_ENET // forward to network peer
  				fwrite(tmpbuf, r, 1, fdbg);
		#else
	       {
	int frag, sentsize;
	unsigned char *pb = tmpbuf;
				for (frag=0; frag<5; frag++) {
					sentsize=sendto(udpout_socket, pb, r/5,0,(struct sockaddr *)&udpout,
	                  udpout_len);
	      if (sentsize < 0) printf("send pack Erorr\n");
	      else printf("send udp TS packet %d out..\n", tag);
	      		pb += r/5;
      		}
	       }
		#endif
  #endif
#endif
frm_inc:
     tag += 1;
#ifdef SRC_FRM_ENET
		if (ITERS==tag)
			tag = 0; // make it endless, liyenho
#endif

   }
   get_time(&tend);
   time_diff(&tend, &tstart, &tdelta);
   print_time(tdelta);
 #if defined(MEDIA_ON_FLASH) && defined(DEBUG_FL)
 		FILE *frb = fopen("media_readback.ts", "wb");
		static int ep = (1 | LIBUSB_ENDPOINT_IN);
		unsigned char tmpbuf[FRAME_SIZE_A];
		int fc=0, rr,  td;
		r = 0;
		do {
			if (1 != do_exit) break;
			 rr=libusb_bulk_transfer(
			 devh, ep,
			 tmpbuf,
			 FRAME_SIZE_A,
			 &td,
			 50);
			if (0!=td)
			  printf("xxx frame = %d, rr=%d, td=%d xxx\n",++fc,rr,td);
			if (1880==td) {
				fwrite(tmpbuf, td, 1, frb);
				r += td;
			}
		} while (r<FILE_LEN);
		fclose(frb);
 #endif
_exit:
#ifdef SND
	fprintf(stderr, "Wrap up/Leaving test_snd app...\n");
#else
	fprintf(stderr, "Wrap up/Leaving test_rec app...\n");
#endif
	if (0 > do_exit) return do_exit;  // return from error
   else do_exit = 4;

	if (file) fclose(file);
#if defined(TEST_BITSTREAM) && defined(SND)
	if (fdbg) fclose(fdbg);
#endif
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
	return 0;
}
/**********************************************************************************************************/
#ifdef SRC_FRM_ENET
int udpin_init(void)
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

int udpout_init(void)
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

int receive(int usec, unsigned char *pb, uint32_t bytes) {
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
    	  recvsize= recvfrom(udpin_socket,pbf,/*UDP_PACKET_MAX*/bytes,0,
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


int GetTestPattern(unsigned char *pb, uint32_t num_bytes){

	uint32_t const frame_size = 188;  //frame size in bytes
	uint32_t elem_per_frame = frame_size/sizeof(uint32_t);
	uint32_t test_frame_hdr =  0x10000147; //frame header template

	uint32_t * data_buffer = (uint32_t *)pb; //to write as 4 byte chunks
	static uint32_t frame_id=0;  //for tracking individual frames

	uint32_t num_frames = num_bytes/frame_size;  //number of frames to be written

	//fill out first frame
	data_buffer[0] = ((frame_id%0x10) << 24 ) | test_frame_hdr; //header
	data_buffer[1] = frame_id;
	frame_id++;
	//fill out payload of frame
	for (int i = 2; i<elem_per_frame ;i++){
			data_buffer[i]= 0x00010010 | (i%0x10);
	}

	//copy first frame to rest of buffer
	for (int i =1;i<num_frames;i++){
		memcpy(data_buffer+i*elem_per_frame, data_buffer, frame_size);
		data_buffer[i*elem_per_frame] = ((frame_id%0x10) << 24 ) | test_frame_hdr;
		//embedding a frame counter
		data_buffer[i*elem_per_frame+1] = frame_id;
		frame_id++;
	}

	return 0;
}
