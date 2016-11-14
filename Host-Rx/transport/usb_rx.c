//##############################################################
//# usb_rx_prod.c
//##############################################################

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
#include <libusb.h>

#define DEBUG_CTRLRX                    1/*0*/
#define DEBUG_CTRLTX                    0
#define DEBUG_VIDEO                     1

#include <libusb.h>
#define true														1
#define false														0

volatile uint8_t system_upgrade = 0;
volatile uint8_t upgrade_from_lgdst = 0;
char upgrade_fwm_path[160]; // upgrade firmware path
#define ATMEL_END2END
#ifdef ATMEL_END2END
	volatile uint8_t main_loop_on = false;  // run time indicator
  #define CTRL_OUT		(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT)
  #define CTRL_IN		(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN)
  #define USB_RQ			0x04
 #include "usb_rx.h"  // definitions for host/fpga access @ rx
 int shmid_Lgdst = -1;
 volatile ipcLgdst *shmLgdst_proc = 0;
 #ifdef RADIO_SI4463
 volatile bool si4463_radio_up = false ;
 #endif
 //#define COMM_ATMEL_DEV // enable atmel<->fpga/6612 comm test
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
		int udpout_len;
		int udpin_socket, udpout_socket;
		fd_set udpin_fd;
		struct sockaddr_in udpout;
	  int udpout_init(char* udpaddr);
	#endif
#endif

#define USB_DEV_VENDER			 0x03eb
#define USB_DEV_PRODUCT			 0x2404
  #define UDPOUT_PORT          5558
// figure it out via lsusb -v -d, choose high speed (2.0) one
//#define USB_DEV_INTF20				0
#define USB_DEV_INTF				/*0*/1  // cdc comm intf included
// atmel CDC data device intf 1
  //#define USB_DEV_EP20						3	// not control endpoint
  #define USB_DEV_EP						1	// atmel CDC data ep 1 in
  //#define EP_DATA			(USB_DEV_EP20 | LIBUSB_ENDPOINT_IN)
  #define EP_DATA			(USB_DEV_EP | LIBUSB_ENDPOINT_IN)

#define TEST												2	// synchronous file based block transfer

#ifdef MEDIA_ON_FLASH
 #define FILE_LEN									63*1880  // 200 kbytes reserved on flash, liyenho
#else
 #define FILE_LEN									/*6422528*/ 12540164 /*63*1880*/ /*10388880*/
#endif
  #define FILE_NAME								/*"php_rx.ts"*/ /*"tsp_rx.ts"*/ "NativeMedia-rec.ts" /*movebox41frm-rec.ts*/
#define FRAME_SIZE_A					1880  // 16 bit stereo @ 480 Khz
#define FRAME_SIZE_V					307200	// 320x240 dim expected
#define FRAME_SIZE_V2					(FRAME_SIZE_V*2)
#define ITERS												(FILE_LEN/FRAME_SIZE_A)
#define FRAME_BUFFS						5
#define TIMEOUT									1000		// audio time out @ 10 msec

typedef int bool;

static FILE *file = NULL;
static unsigned char audbuf[FRAME_SIZE_A*FRAME_BUFFS]__attribute__((aligned(8)));
static unsigned char vidbuf[FRAME_SIZE_V2*FRAME_BUFFS]__attribute__((aligned(8)));
  #if defined(PES_FRM_PROT) || defined(PES_FRM_PROT1)
	 static unsigned char retbuf[FRAME_SIZE_A*ERR_CHK_BUFFS /*+ 188*/]__attribute__((aligned(8)));
	//#define DBG_PROT1
    #ifdef DBG_PROT1
    	typedef struct {
	    	uint8_t *buf_end;
	    	bool first_access;
	    	uint8_t *wrptr;
	    	uint8_t *rdptr;
    	} dbg_frm_prot1;
    	#define DBG_PROT1_CHECK \
    		if (dbg_fp.first_access) { \
				memcpy(dbg_fp.buf_end-size, tsbuf, size); \
				dbg_fp.first_access = false; \
			} else { \
				memcpy(dbg_fp.wrptr, tsbuf, size); \
			} \
			dbg_fp.wrptr += size; \
			if (dbg_fp.buf_end<=dbg_fp.wrptr) \
				dbg_fp.wrptr = retbuf;
		#define DBG_PROT1_CHECK1(size, sbuf) \
			remain = (int32_t)(dbg_fp.buf_end -dbg_fp.wrptr); \
			if (0>remain) perror_exit("invalid size",-255); \
			if ((size) < remain) { \
				memcpy(dbg_fp.wrptr, (sbuf), (size)); \
				dbg_fp.wrptr += (size); \
			} else { \
				memcpy(dbg_fp.wrptr, (sbuf), remain); \
				memcpy(retbuf, (sbuf)+remain, (size)-remain); \
				dbg_fp.wrptr = retbuf+ (size)-remain; \
			}
		static dbg_frm_prot1 dbg_fp = {
			.buf_end = retbuf+sizeof(retbuf),
			.first_access = true,
			.wrptr = retbuf+sizeof(retbuf), // not finished yet
			.rdptr = retbuf,
		};
    #endif
  #endif
#ifdef RADIO_SI4463
static unsigned char radio_tpacket[RADIO_USR_TX_LEN]__attribute__((aligned(8)));
static unsigned char radio_rpacket[RADIO_USR_RX_LEN+RADIO_INFO_LEN]__attribute__((aligned(8)));
#endif

static bool detached = false;
static struct libusb_device_handle *devh = NULL;
/*static*/volatile int do_exit = 0;	// main loop breaker
static pthread_t poll_thread= 0,
									lgdst_thread = 0,
									ctrl_thr_recv = 0,
									ctrl_thr_send = 0;
 static pthread_mutex_t mux;
 int tag=0; //ts packet count

 //////////////////////////////////////////////////////
 // function prototype
 //////////////////////////////////////////////////////
  extern void print_time(struct timeval ttime);
  extern int time_diff(struct timeval *time1,
  								struct timeval *time2,
                                struct timeval *diffTime);
  extern int get_time(struct timeval *time);
 const unsigned char nullts[188]={
    0x47,0x1f,0xff,0x10,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
  };
 static struct timeval time_orig; // as starting time ref

//function prototyping
 int tsptsadj(unsigned char* buff, int len, int pidid, int pcrid);

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
//#if !defined(PES_HDR_PROT) && !defined(PES_FRM_PROT)
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
		70); // realtime recv no delay allowed
 #else
		750);
 #endif
#endif
	if (r<0) return r;
skip:
	if (FRAME_SIZE_A!= transferred) {
		//if (0!=transferred)
		  //printf("xxx transferred = %d xxx\n",transferred);
		return -2; }
/*#else
	static FILE *fdbg = 0;
 #if !defined(PES_FRM_PROT)
	if (!fdbg) fdbg = fopen("php_tx.dbg", "rb");
 #else
	if (!fdbg) fdbg = fopen("tsp_tx.dbg", "rb");
 #endif
	r = fread(audbuf, 1, FRAME_SIZE_A, fdbg);
	if (FRAME_SIZE_A != r)
		return -1;	// eof
#endif*/
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
  unsigned char ctrl_recv_fifolvl_data=0;
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
	} //socket init fail check

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
#ifdef RADIO_SI4463
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
#ifdef RADIO_SI4463
	pthread_join(ctrl_thr_recv, NULL);
	pthread_join(ctrl_thr_send, NULL);
#endif
	printf("poll thread shutting down\n");
	return NULL;
}

static void *lgdst_thread_main(void *arg)
{
	if (0>(shmid_Lgdst = shmget(SHMKEY_RX,
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
		dev_access *acs = &lclMem.hdr;
		if (1 == shmLgdst_proc->active) {
			memcpy(&lclMem, &shmLgdst_proc->access, sizeof(dAccess));
			switch(shmLgdst_proc->type) {
				case CMD0:
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
						strncpy(upgrade_fwm_path, (char*)acs->data, shmLgdst_proc->len);
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

static void rffe_write_regs(dev_cfg* pregs, int size)
{
	int32_t i, msg[80]; // access buffer
  	dev_access *acs = (dev_access*)msg;
	 	dev_access echo; // atmel shall echo write cmd hdr back
    	  acs->dcnt = 1;  // send cfg val one by one
    for (i=0; i<size; i++) {
retry:
    	  acs->access = WRITE_BY_ADDR;
    	  acs->addr = pregs[i].addr;
    	  *acs->data = pregs[i].data;
		libusb_control_transfer(devh,
						CTRL_OUT, USB_RQ,
						USB_HOST_MSG_TX_VAL,
						USB_HOST_MSG_IDX,
						acs, sizeof(*acs), 0);
		short_sleep(0.1); 	// validate echo after 0.1 sec
		printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",acs->data[0],acs->addr);
    	  acs->access = READ_BY_ADDR;
    	  acs->addr = pregs[i].addr;
		  libusb_control_transfer(devh,
						CTRL_OUT, USB_RQ,
						USB_HOST_MSG_TX_VAL,
						USB_HOST_MSG_IDX,
						acs, sizeof(*acs), 0);
		short_sleep(0.1); 	// receive readback after 0.1 sec
	  	 while(0==libusb_control_transfer(devh,
					  	CTRL_IN, USB_RQ,
					  	USB_HOST_MSG_RX_VAL,
					  	USB_HOST_MSG_IDX,
					  	&echo, sizeof(*acs), 0))
			short_sleep(0.0005);
		short_sleep(0.1); // gap after 0.1 sec
#if defined(REC)
		if (0x0==acs->addr) {
			// 6612 soft reset is write only
			continue;
		}
#endif
#if false
   	if(echo.data[0] != pregs[i].data) {
	   	printf("xxxxxx 0x%04x @ 0x%x mis-matched xxxxxx\n",echo.data[0],acs->addr);
			goto retry;
		}
#else
		printf("xxxxxx 0x%04x @ 0x%x received xxxxxx\n",echo.data[0],acs->addr);
#endif
	}
}
static void sigint_handler(int signum)
{
   do_exit = 4; // this shall terminate poll thread
   pthread_mutex_unlock(&mux);
}
#ifdef RFFE_PARAMS
bool open_ini(int *setting_rec)
{
	FILE *fp = NULL;
        char ini_buff[120];

        fp = fopen("usb_rx.ini", "r+");
	if (fp == NULL) {
	  // File doesn't exist, setup ini with default
       	  fp = fopen("usb_rx.ini", "w+");
	  if (fp == NULL) {
		printf("ERROR: Fail to creat 'usb_rx.ini' !!");
		return false;
	  }
	  fprintf(fp, "6612_CH=%d\n", *setting_rec);
	  fclose(fp);
          return true;
	}

        while(fgets(ini_buff, 120, fp) != NULL)
        {
           if (strstr(ini_buff, "6612_CH") > 0)
           {
               sscanf(ini_buff, "6612_CH=%d", setting_rec);
           }
        }
        printf("setting_rec = %d\n", *setting_rec);
	fclose(fp);
	return true;
}
#endif
#ifdef PES_HDR_PROT
 //#define PHP_DBG
  	uint32_t chksm_exam_pes_hdr(uint32_t size, uint8_t *tsbuf)
 		{
	 		bool skip_pkt = false;
	 		uint32_t  frames = size;
	 		uint8_t adp, *pbp, *pb = tsbuf;
	 		uint16_t chk1, chksm=0;
	 		uint32_t i, n;
			for (i=0; i<size/188; chksm=0, i++) {
				if (skip_pkt) {
					memcpy(pb-188, pb, 188);
					frames -= 188;
				}
				adp = 0x30 & *(pb+0x3) ;
				if ((0x40 == *(pb+0x1)) && (0x10 & adp)) {
					if (*(pb+0x4) && 0x0==*(pb+0x5)) {
						if (2<*(pb+0x4)) {
							// packets from pre-defined tables, all 0xff came after 0x0 are stuffing bytes
							chk1 = *(uint16_t*)(pb+0x6);
							*(uint16_t*)(pb+0x6) = 0xffff;
							for (n=0; n<188; n+=sizeof(uint16_t)) {
								chksm ^= (uint16_t)*(pb+n);
							}
							if (chksm != chk1) {
	 #ifdef PHP_DBG
	 			fprintf(stderr, "0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x\n",
	 							*(pb+0), *(pb+1), *(pb+2), *(pb+3), *(pb+4), *(pb+5), *(pb+6), *(pb+7));
	 #endif
								skip_pkt = true;
							} else {
								skip_pkt =false;
							}
						}
					}
					else if (0x10 == adp ) {
						pbp = pb + 0x4; // jump to start adr of pes
						chk1 = *(uint16_t*)pbp;
						*(uint16_t*)pbp = 0x0000;
						for (n=0; n<188; n+=sizeof(uint16_t)) {
							chksm ^= (uint16_t)*(pb+n);
						}
						if (chksm != chk1) {
 #ifdef PHP_DBG
 			fprintf(stderr, "0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x\n",
 							*(pb+0), *(pb+1), *(pb+2), *(pb+3), *(pb+4), *(pb+5));
 #endif
							skip_pkt = true;
						} else {
							skip_pkt =false;
						}
					} else { // with adaptation field
						if (2<*(pb+0x4)) {
							pbp = (pb+0x4)+ *(pb+0x4)+1; // jump to start adr of pes
							for (chk1 = 0,n=0; n<sizeof(uint16_t); n++) {
								chk1 |= ((uint16_t)*pbp)<<(n*8);
								*pbp++ = 0x00;
							}
							for (n=0; n<188; n+=sizeof(uint16_t)) {
								chksm ^= (uint16_t)*(pb+n);
							}
							if (chksm != chk1) {
	 #ifdef PHP_DBG
	 			fprintf(stderr, "0x%02x, 0x%02x, 0x%02x, 0x%02x\n", *(pb+0), *(pb+1), *(pb+2), *(pb+3));
	 			uint32_t n1 = *(pb+4)>>2;
	 			for (n=0; n<n1; n++)
		 			fprintf(stderr, "0x%02x, 0x%02x, 0x%02x, 0x%02x\n",
	 							*(pb+4*n+4), *(pb+4*n+5), *(pb+4*n+6), *(pb+4*n+7));
	 			if (3 & *(pb+4)) {
		 			switch(3 & *(pb+4)) { // last two are of chksum
			 			case 1 : fprintf(stderr, "0x%02x, 0x%02x, 0x%02x\n",
			 								*(pb+4*n+4), *(pb+4*n+5), *(pb+4*n+6));
			 							break;
			 			case 2 : fprintf(stderr, "0x%02x, 0x%02x, 0x%02x, 0x%02x\n",
			 								*(pb+4*n+4), *(pb+4*n+5), *(pb+4*n+6),  *(pb+4*n+7));
			 							break;
			 			case 3 : fprintf(stderr, "0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x\n",
			 								*(pb+4*n+4), *(pb+4*n+5), *(pb+4*n+6), *(pb+4*n+7), *(pb+4*n+8));
			 							break;
		 			}
	 			} else { // these are of chksum
		 			fprintf(stderr, "0x%02x, 0x%02x\n", *(pb+4*n+4), *(pb+4*n+5));
		 		}
	 #endif
								skip_pkt = true;
							} else {
								skip_pkt = false;
							}
						}
					}
				} else
				skip_pkt = false;
				pb += 188;
			}
			return frames;
		}
#endif
#ifdef PES_FRM_PROT
 //#define TSP_DBG
	void func_chksm_internal(uint32_t size, uint8_t *pb, bool *skip_pkt) {
	 	uint16_t chk1, chksm=0;
		uint32_t i, n;
		uint8_t tmp;
			for (i=0; i<size/188; chksm=0, i++) {
				// scan only video packets
				tmp = 0xc0 & *(pb+3);
				if (VIDEO == tmp) {
					if (0x80 & *(pb+1)) {  // TEI bit flagged
						*skip_pkt = true;
					}
					else if (!(*(pb+1) & 0x20)) {
						chk1 = *(pb+2);  // fetch 13 bit checksum
						chk1 |= (0x1f & (uint16_t)*(pb+1)) << 8;
						// restore pid prior to calc chksm
						*(pb+0x2) = VIDEO_PID;
						*(pb+0x1) = (0xc0 & *(pb+0x1)) | (0x1f & (VIDEO_PID>>8));
						for (n=0; n<188; n+=sizeof(uint16_t)) {
							chksm ^= *(uint16_t*)(pb+n);
						}
						chksm &= 0x1fff;
						if (chksm != chk1 ) {
							*skip_pkt = true;
						}
					} else {
					*skip_pkt = true; // there must be error(s)
					}
				}
				pb += 188;
			}
		}
		uint32_t output_packets (uint8_t *chkbuf, uint8_t *retbuf, uint32_t *vb, uint32_t ve, bool *skip_pkt) {
				uint32_t i, j, retsz = 0;
				uint8_t tmp, *pb, *pbr;
				uint16_t chk1, chksm=0;
				pb = chkbuf + *vb;
				pbr = retbuf;
#define ADD_PACKET \
									*(pb+3) &= ~0xc0; \
									memcpy(pbr , pb, 188); \
									pbr += 188; \
									retsz += 188;
				for (i=0; i<(ve-*vb)/188; chksm=0, i++) {
					tmp = 0xc0 & *(pb+3);
					if (VIDEO != tmp) {
						if (!(0x80 & *(pb+1))) {
							if (!(*(pb+1) & 0x20)) {
								chk1 = *(pb+2);  // fetch 13 bit checksum
								chk1 |= (0x1f & (uint16_t)*(pb+1)) << 8;
								// restore pid prior to calc chksm
									uint16_t pid ;
									switch(tmp) {
										case PMT:
											pid = PMT_PID; break;
										case AUDIO:
											pid = AUDIO_PID; break;
										default: {
											goto tp_skip; // packet with errors, dropped
										}
									}
									*(pb+0x2) = pid;
									*(pb+0x1) = (0xc0 & *(pb+0x1)) | (0x1f & (pid>>8));
								for (j=0; j<188; j+=sizeof(uint16_t)) {
									chksm ^= *(uint16_t*)(pb+j);
								}
								chksm &= 0x1fff;
								if (chksm == chk1 ) {
									ADD_PACKET
								}
							} else {
								*(pb+1) &= ~0x20; // reset transport priority bit
								ADD_PACKET
							}
						}
					} else { // video packets
						if (!*skip_pkt) {
							ADD_PACKET
						}
					}
tp_skip:
					pb += 188;
				}
#undef ADD_PACKET
				*skip_pkt = false;
				*vb = ve;
			 	return retsz;
			}
  	uint32_t chksm_exam_pkt_ts(uint32_t size, uint8_t *tsbuf, uint8_t *retbuf)
 		{
//#include <assert.h>
	 		const int chkbuf_size = FRAME_SIZE_A*ERR_CHK_BUFFS ;
			static uint8_t got_video_frame = 0,
										chkbuf[FRAME_SIZE_A*ERR_CHK_BUFFS ]__attribute__((aligned(8)));
			static uint32_t vb= 0,  chk1idx =0, chk_index = 0;
	 		static bool skip_pkt/*[4]*/ = false; // four types of TS expected, concern only video
	 		uint8_t tmp, *pbr= retbuf, *pb = tsbuf;
	 		uint16_t pid;
	 		uint32_t i, n, ve, retsz =0;
	 		if (!got_video_frame) {
			 	// if we don't buffer enough TS packets yet
				for (i=0; i<size/188; i++) {
					tmp = 0xc0 & *(pb+3);
					if ((0x40 & *(pb+0x1)) && (VIDEO == tmp)) {
						vb = i * 188 ;
						got_video_frame = 1;
						i ++ ; pb += 188;
						goto video_start_found;
					}
					pb += 188;
				}
				return 0;  // not found any video packet yet
			} else {
				bool found ;
				i = 0;
				pb = chkbuf+chk_index;
video_start_found:
				found = false;
				//assert(chkbuf_size >= chk_index + size);
				if (chkbuf_size < chk_index + size) {
					chk1idx = chk_index = 0;
					got_video_frame = 0;
					return 0;	// error protection
				}
				memcpy(chkbuf+chk_index, tsbuf, size);
					for (; i<size/188; i++) {
						tmp = 0xc0 & *(pb+3);
						if ((0x40 & *(pb+0x1)) && (VIDEO == tmp)) {
							ve = chk_index + i*188;
							func_chksm_internal(ve-vb, chkbuf+chk1idx+vb, &skip_pkt);
							retsz += output_packets (chkbuf, pbr+retsz, &vb, ve, &skip_pkt);
							found = true;
						}
						pb += 188;
					}
				if (found) {
					chk1idx = chk_index;
				}
				chk_index += size;
				if (!retsz)  // not found entire video frame yet
					return 0;
			}
			if (chk_index > size) {
				memmove(chkbuf, chkbuf+chk_index-size, size);
			 	vb -= (chk_index-size);
			 	chk1idx = 0;
			 	chk_index = size;
		 	}
			if (1 & (retsz/188)) { // append a null packet if # of return packets is odd
			 	memcpy(pbr+retsz, nullts, sizeof(nullts));
			 	retsz += 188;
			}
			return retsz;
		}
#elif defined(PES_FRM_PROT1)
	uint32_t TEI_exam_pkt_ts(uint32_t size, uint8_t *tsbuf, uint8_t *retbuf)
	{
//#include <assert.h>
			const static int chkbuf_size = FRAME_SIZE_A*ERR_CHK_BUFFS ;
		  const static int pidvid = (TSTYPE==0)?0x40:
                     				((TSTYPE==1)?0x200:
                                0x100);
			static uint8_t got_video_frame = 0,
										chkbuf[FRAME_SIZE_A*ERR_CHK_BUFFS]__attribute__((aligned(8)));
			static int32_t vb= 0,  chk_index = 0;
	 		uint8_t *pbr= retbuf, *pb = tsbuf;
#ifdef DBG_PROT1
	 		int32_t remain ;
#endif
	 		int32_t i, n, ve, tmp, retsz0= 0, retsz =0;
	 		if (!got_video_frame) {
			 	// if we don't buffer enough TS packets yet
				for (i=0; i<size/188; i++) {
					if ((0x40 & *(pb+0x1)) &&
							((*(pb+0x1)&0x1f)==((pidvid>>8)&0x1f))  &&
          				((*(pb+0x2)&0xff)==((pidvid)   &0xff))) {
						vb = i * 188 ;
						got_video_frame = 1;
						retsz0 = i++ * 188;
						pb += 188;
#ifndef DBG_PROT1
						memcpy(pbr, tsbuf, retsz0);
						pbr += retsz0;
#else
						if (dbg_fp.first_access) {
						   dbg_fp.wrptr -= retsz0;
						   dbg_fp.first_access = false;
						}
						DBG_PROT1_CHECK1(retsz0, tsbuf)
#endif
						retsz += retsz0;
						goto video_start_found;
					}
					pb += 188;
				}
#ifndef DBG_PROT1
				memcpy(retbuf, tsbuf, size);
#else
				DBG_PROT1_CHECK
#endif
				return size;  // not found any video packet yet
			} else {
				i = 0;
				pb = chkbuf+chk_index;
video_start_found:
				//assert(chkbuf_size >= chk_index + size);
				if (chkbuf_size < chk_index + size) {
					puts("video TS buffer is too small");
					chk_index = 0;
					got_video_frame = 0;
#ifndef DBG_PROT1
					memcpy(retbuf, tsbuf, size);
#else
					DBG_PROT1_CHECK
#endif
					return size;	// error protection
				}
				memcpy(chkbuf+chk_index, tsbuf, size);
					for (; i<size/188; i++) {
						if ((0x40 & *(pb+0x1)) &&
							((pb[1]&0x1f)==((pidvid>>8)&0x1f))  &&
          				((pb[2]&0xff)==((pidvid)   &0xff))) {
							ve = chk_index + i*188;
							//assert(0==((ve-vb)%188));
							if ((ve-vb) != ((ve-vb)/188)*188)
								perror_exit("invalid video frame size, not divided into 188",-1);
							uint8_t *pb1 = chkbuf+vb;
							tmp = ve - vb; // total bytes go out
							bool bad = false;
							for (n=0; n<tmp/188; pb1+=188, n++) {
								if ((0x80 & *(pb1+1)) && // check upon only video packet
									((pb1[1]&0x1f)==((pidvid>>8)&0x1f))  &&
          						((pb1[2]&0xff)==((pidvid)   &0xff))) {
									 // if error flagged by Siano, replaced with null packet
									 bad = true;
									 break;
								}
							}
							if (bad) {
								pb1 = chkbuf+vb;
								for (n=0; n<tmp/188; pb1+=188, n++) {
									if (((pb1[1]&0x1f)==((pidvid>>8)&0x1f))  &&
	          						((pb1[2]&0xff)==((pidvid)   &0xff))) {
										 // if error flagged by Siano, replaced with null packet
										 memcpy(pb1, nullts, sizeof(nullts));
									}
								}
							}
#ifndef DBG_PROT1
							memcpy(pbr, chkbuf+vb, tmp);
							pbr += tmp;
#else
							DBG_PROT1_CHECK1(tmp, chkbuf+vb)
#endif
							retsz += tmp;
							vb = ve;
						}
						pb += 188;
					}
				chk_index += size;
				if (!retsz)  // not found entire video frame yet
					return 0;
				if (retsz0 == retsz) {
				  	 if (1 & (retsz0/188)) {
#ifndef DBG_PROT1
				  	 	memcpy(retbuf+retsz0, nullts, sizeof(nullts));
			 			retsz0 += 188;
#else
						DBG_PROT1_CHECK1(188, nullts)
#endif
				  	 }
				  	 return retsz0;
				}
			}
			if (chk_index > size) {
				memmove(chkbuf, chkbuf+chk_index-size, size);
			 	vb -= (chk_index-size);
			 	chk_index = size;
		 	}
			if (1 & (retsz/188)) { // append a null packet if # of return packets is odd
#ifndef DBG_PROT1
			 	memcpy(pbr, nullts, sizeof(nullts));
			 	retsz += 188;
#else
				DBG_PROT1_CHECK1(188, nullts)
#endif
			}
			return retsz;
	}
#endif
int main(int argc,char **argv)
{
   char udpaddr[20];
        int chsel_6612 = 2;
	int r = 2, blksz=0 ;
        int try_i = 0;
  	float delay ;
  const int pidvid = (TSTYPE==0)?0x40:
                     ((TSTYPE==1)?0x200:
                                0x100);
  const int pidpcr = (TSTYPE==0)?0x4F:
                     ((TSTYPE==1)?0x200:
                                0x100);
	do_exit = 1;
   pthread_mutex_init(&mux, NULL);

	r = libusb_init(NULL);
	if (r < 0)
		perror_exit("failed to initialise libusb",1);

	while (1) //for (int i=0; i < 10; i ++)
 	{
#if (1/*0*/)
		devh = libusb_open_device_with_vid_pid(NULL,
					USB_DEV_VENDER, USB_DEV_PRODUCT);
#else
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
    system_upgrade = (!strcasecmp(argv[2],"Ua0"))?4: // direct boot atmel
    						((!strcasecmp(argv[2],"Ua1"))?3:	// fwm upgrade atmel
    						((!strcasecmp(argv[2],"Uf"))?2:
    						((!strcasecmp(argv[2],"Uc"))?1: 0 )));
		if (system_upgrade) {
				if (4>system_upgrade) {
					if (argc <4) {
						puts("missing upgrade firmware filepath, bailed out...");
						goto _fail; 	}
					strncpy(upgrade_fwm_path, argv[3], sizeof(upgrade_fwm_path));
				}
				// bypass cmdline parsing for all other tasks
				goto libusb_next;
		}
        if(argc == 3) //ip address entered
          sprintf(udpaddr, "%s",argv[2]);
		  #if defined(RADIO_SI4463) && !defined(CTRL_RADIO_TEST)
		  if (5 != argc)  {
			  printf("usage: sudo %s 0 [video-ip] [ctrl_ip] ctrl-pt# (out), bailed out...\n",argv[0]);
		  goto _fail; }
		  else{
			  sprintf(udpaddr, "%s",argv[2]);
        sprintf(servIP, "%s",argv[3]);
		  		ctrl_port_base = atoi(argv[4]);
	  		}
		  #endif
        printf("Video udpaddr=%s port=%d\n",udpaddr,UDPOUT_PORT);
libusb_next:
		if (devh <= 0) {
			//libusb_exit(NULL);
			//perror_exit("could not find/open USB device",2);
			if (try_i == 10) {
			  printf("%c                                                                     %c", 0x0d, 0x0d);
			  try_i = 0;
			}
			printf("fail to aquire USB link. attempt:%d ",try_i);
			fflush(stdout);
			try_i ++;
			sleep(1);
			continue;
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
		printf("\n");
		break; // USB Device found!! exit waiting loop
	}
	if (devh <= 0) {
		libusb_exit(NULL);
		perror_exit("could not find/open USB device",2);
	}
	printf("claimed interface\n");
	// send system restart command...
	libusb_control_transfer(devh,
						CTRL_OUT, USB_RQ,
						USB_SYSTEM_RESTART_VAL,
						USB_HOST_MSG_IDX,
						NULL, 0, 0);
	if (system_upgrade) // no need to bring up other proc threads,
		goto upgrade_next;
	// bring up all others after main system restart command sent...
	r = pthread_create(&poll_thread, NULL, poll_thread_main, NULL);
	if (0 != r)
		perror_exit("poll thread creation error", r);
upgrade_next:
   if (SIG_ERR == signal(SIGINT, sigint_handler)) {
     perror("FAIL: assigning signal handler");
   }

	/* we shouldn't bother issues of USB hw setup/config */
#if (TEST==2)
 #include <time.h>
 #include <sys/time.h>
 extern struct timeval tstart,tend,tdelta;
 	uint32_t n, size, fw_info[3+1]; /*len = SMS_FW_HDR_LEN*/
  if (system_upgrade) { // user request firmware upgrade
  	FILE *file_up ;
  	int end, cur, len;
upgrade_firmware:
	if (4>system_upgrade) {
		file_up = fopen(upgrade_fwm_path, "rb");
  		if (!file_up) {
	  		puts("failed to open upgrade firmware, bail out....");
	  		goto _fail;
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
	 {
  		if (1/*cpld*/== system_upgrade) {
	  		fw_info[2] = 0x00003800;  // constant cpld download address
	  		printf("upgrade firmware address: 0x%08x\n", fw_info[2]);
		 libusb_control_transfer(devh, CTRL_OUT, USB_RQ,
			USB_CPLD_UPGRADE_VAL, USB_HOST_MSG_IDX,
			fw_info, FW_UPGRADE_HDR_LEN, 0);
			blksz = 7200; // similar size to atmel bin image, it should be fine
  			delay = 0.84;
	  	}
  		else if (2/*fpga*/== system_upgrade) {
	  		fw_info[2] = 0x00295700;  // constant fpga download address
	  		printf("upgrade firmware address: 0x%08x\n", fw_info[2]);
		 libusb_control_transfer(devh, CTRL_OUT, USB_RQ,
			USB_FPGA_UPGRADE_VAL, USB_HOST_MSG_IDX,
			fw_info, FW_UPGRADE_HDR_LEN, 0);
			blksz = 15000;
			delay = 3.5;
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
			goto _fail; // it is done
  		}
		else {
			puts("invalid target for upgrade request, bail out....");
			goto _fail; }
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
	    	fwrite(vidbuf+blksz, size, 1, fdmp);
    // close readback dump file
	fclose(fdmp);
 #endif
 	if (2<system_upgrade || (4== argc && !strcasecmp(argv[3], upgrade_fwm_path)))
		 goto _fail; // finished regardless if there's error
	else
		goto _exit; // normal shutdown
  }
	r = pthread_create(&lgdst_thread, NULL, lgdst_thread_main, NULL);
	if (0 != r)
		perror_exit("lgdst thread creation error", r);
  #ifdef ATMEL_END2END
    extern int short_sleep(double sleep_time);
  	static int32_t i, sz, msg[80]; // access buffer
  #endif
#ifdef REC
	open_ini(&chsel_6612);
 #ifdef CONFIG_ADI_6612
 	sz = ARRAY_SIZE(chsel_6612);
	libusb_control_transfer(devh, CTRL_OUT, USB_RQ,
		USB_6612_SIZE_VAL, USB_HOST_MSG_IDX,
		&sz, USB_6612_SIZE_LEN, 0);
	rffe_write_regs(GET_ARRAY(chsel_6612), sz);
	printf("rx chan sel is set...\n");
	short_sleep(0.5);
	// signify atmel to go ahead bootup
	libusb_control_transfer(devh, CTRL_OUT, USB_RQ,
		RF_RX_VAL, USB_HOST_MSG_IDX,
		&sz/*don't care*/, sizeof(sz), 0);
	printf("rx rffe is running...\n");
 #endif // CONFIG_ADI_6613
        //Kevin:  Put head here 0xAB0005 for Atmel alginments
			vidbuf[0] = 0xAB;
			vidbuf[1] = 0x00;
			vidbuf[2] = 0x05;
			 r=libusb_bulk_transfer(
			 devh, 2 | LIBUSB_ENDPOINT_OUT,
			 vidbuf,
			 0x03,
			 &sz,
			 0);
 #ifdef SMS_DVBT2_DOWNLOAD
	long len, end, cur;
 #ifdef FWM_DNLD_DBG
 	static FILE *fdmp;
	long rem, cur1;
	char fd[128], *dfn;
 #endif
 	file = fopen(SMS_FW_FNAME,"rb");
 #ifdef FWM_DNLD_DBG
  strncpy(fd,SMS_FW_FNAME,sizeof(fd));
 	dfn = strrchr(fd, '.');
	strcpy(dfn, ".dmp");
 	fdmp = fopen(fd,"wb");
 #endif
 	fread(fw_info, 3, sizeof(int), file);
#if (DVBT2==MODE)
 	fw_info[3] = (uint32_t)SMSHOSTLIB_DEVMD_DVBT2;  // user setup rec working mode here
#elif (DVBT==MODE)
 	fw_info[3] = (uint32_t)SMSHOSTLIB_DEVMD_DVBT;  // user setup rec working mode here
#endif

	 libusb_control_transfer(devh, CTRL_OUT, USB_RQ,
		USB_SMS_FW_VAL, USB_HOST_MSG_IDX,
		fw_info, SMS_FW_HDR_LEN, 0);
	printf("firmware header (0x%08x, 0x%08x, 0x%08x, 0x%08x) sent...\n",
			fw_info[0], fw_info[1], fw_info[2], fw_info[3]);
	cur = ftell(file);
	fseek(file,0,SEEK_END);
	end = ftell(file);
	len= end - cur;
	printf("sms4470 firmware binary length = %d\n", len);
	if (8192 > len) {
		fclose(file);
		perror_exit("sms4470 firmware is too short, bail out",-4);
	}
	// reverse load from end of fw file
	fseek(file,-8192,SEEK_END);
	cur = 8192;
 #ifdef FWM_DNLD_DBG
 	rem = len;
 #endif
	i = 1;
	while(1) {
		if (1 != do_exit) break;
		fread(vidbuf, cur, 1,file);
			 r=libusb_bulk_transfer(
			 devh, 2 | LIBUSB_ENDPOINT_OUT,
			 vidbuf,
			 cur,
			 &sz,
			 0);
		printf("firmware block #%d sent (%d)...\n", i++,sz);
		len -= cur;
		if (!len) break;
		short_sleep( // 42 msec xfer time+ 30 msec delay+ 20 msec delay+ 30 msec delay
			(42000+30000+20000+30000)/3 //too long so...
			*10e-6);
		fseek(file,-cur,SEEK_CUR);
		cur =(len>=8192)?8192:len;
		fseek(file,-cur,SEEK_CUR);
	}
	fread(vidbuf, cur, 1,file);
	fclose(file);
	 r=libusb_bulk_transfer(
		 devh, 2 | LIBUSB_ENDPOINT_OUT,
		 vidbuf,
		 10*EXTRA,  // send something to flush usb bulk pipe, liyenho
		 &sz,
		 0);
#ifdef FWM_DNLD_DBG
	i = 1;
	do {
  		if (8192<=rem) {
  			cur1 = /*sizeof(DownloadCommand)*/8+/*sizeof(_hdr)*/12+8192;
  			rem -= 8192;
		}
  		else {
  			cur1 = /*sizeof(DownloadCommand)*/8+/*sizeof(_hdr)*/12+rem;
  			rem = 0;
  		}
		do {
		 r = libusb_bulk_transfer(
			 devh, (1 | LIBUSB_ENDPOINT_IN),
			 vidbuf+8192,
			 cur1,
			 &sz,
			 50);
		} while (1 == do_exit &&((sz!=cur1) || r<0));
    printf("dumping read fw dnld block(%d)...\n",i++);
		fwrite(vidbuf+8192, sz, 1, fdmp);
	} while (0<rem);
	fclose(fdmp);
#endif
#if false // no need in single mode
	 // approximate delay in between downloads
	short_sleep((200000)*10e-6);
 	file = fopen(SMS_DATA_FNAME,"rb");
 	fread(fw_info, 3, sizeof(int), file);
 	fw_info[2] = 0xe00000; // hardwired data address, liyenho
	 libusb_control_transfer(devh, CTRL_OUT, USB_RQ,
		USB_SMS_DATA_VAL, USB_HOST_MSG_IDX,
		fw_info, SMS_FW_HDR_LEN, 0);
	printf("firmware data header (0x%08x, 0x%08x, 0x%08x, 0x%08x) sent...\n",
			fw_info[0], fw_info[1], fw_info[2], fw_info[3]);
	// approximate another 20 msec delay
	short_sleep((20000)*10e-6);
	cur = ftell(file);
	fseek(file,0,SEEK_END);
	end = ftell(file);
	len= end - cur;
	if (240 > len) {
		fclose(file);
		perror_exit("sms4470 firmware data is too short, bail out",-5);
	}
	fseek(file,12,SEEK_SET);
	cur = 240;
	i = 1;
	while(1) {
		if (1 != do_exit) break;
		fread(audbuf, cur, 1,file);
			 r=libusb_bulk_transfer(
			 devh, 2 | LIBUSB_ENDPOINT_OUT,
			 audbuf,
			 cur,
			 &sz,
			 0);
		printf("fw data block #%d sent (%d)...\n", i++,sz);
		len -= cur;
		if (!len) break;
		short_sleep((20000)*10e-6);
		cur =(len>=240)?240:len;
	}
	fread(audbuf, cur, 1,file);
	fclose(file);
	 r=libusb_bulk_transfer(
		 devh, 2 | LIBUSB_ENDPOINT_OUT,
		 audbuf,
		 EXTRA,  // send something to flush usb bulk pipe, liyenho
		 &sz,
		 0);
#endif
	file = NULL;
 #endif
	file = fopen(FILE_NAME,"wb");
 #ifdef TEST_BITSTREAM
 	static char nm[] = FILE_NAME;
 	char *pn = strstr(nm,"-rec");
 	if (!pn) puts("unexpected media filename?.....");
 	strcpy(pn,pn+strlen("-rec"));
 	FILE *fdbg = fopen(nm,"rb");
 #endif
#endif
#ifndef SRC_FRM_ENET
	if (!file)
		perror_exit("test file open failed, bail out",-3);
#endif
#if defined(ATMEL_END2END)
	libusb_control_transfer(devh, CTRL_OUT, USB_RQ, 0x07, 0, NULL, 0, 0);
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
	get_time(&tstart);
#ifdef REC
 #ifdef TEST_BITSTREAM
	static int ep = (2 | LIBUSB_ENDPOINT_OUT);
	unsigned char tmpbuf[FRAME_SIZE_A];
	int rr,  td;
	fread(tmpbuf, FRAME_SIZE_A, 1,fdbg);
	do {
		 rr=libusb_bulk_transfer(
		 devh, ep,
		 tmpbuf,
		 FRAME_SIZE_A,
		 &td,
		 7);
		//printf("xxx rr=%d, td=%d xxx\n", rr, td);
	} while (rr<0 || td!=FRAME_SIZE_A);
	short_sleep((3000)*10e-6);
 #endif
 #ifdef SRC_FRM_ENET
   udpout_init(udpaddr);
 #endif
#endif
	tag += 1;
	while (ITERS>=tag) {
     if (1 != do_exit) break; // we can't fail here......
     if (system_upgrade && !upgrade_from_lgdst) {
			upgrade_from_lgdst = 1;
     	 }
		if (0>stream_block() && (1!=upgrade_from_lgdst)) continue;
  #ifdef SRC_FRM_ENET
		sz = UDP_PACKET_MAX;
  #else
 		sz = FRAME_SIZE_A;
  #endif
#ifdef REC
 #ifdef RECV_SMS4470
 	uint64_t llw;
 	int ii;
 	for (ii=0, i=0; i<(sz/8); i++) {
		llw = *((uint64_t*)audbuf+i);
		*((uint16_t*)audbuf+ii++) = ((0xff & llw)<<8) | (0xff & (llw>>8));
		llw >>= 16;
		*((uint16_t*)audbuf+ii++) = ((0xff & llw)<<8) | (0xff & (llw>>8));
		llw >>= 16;
		*((uint16_t*)audbuf+ii++) = ((0xff & llw)<<8) | (0xff & (llw>>8));
		llw >>= 16;
		*((uint16_t*)audbuf+ii++) = ((0xff & llw)<<8) | (0xff & (llw>>8));
 	}
 #endif
 #if defined(PES_HDR_PROT) || defined(PES_FRM_PROT) || defined(PES_FRM_PROT1)
  		uint32_t frames_len;
  	 #ifdef PES_HDR_PROT
  		frames_len = chksm_exam_pes_hdr(sz, audbuf);
  	 #else
  	  #ifdef PES_FRM_PROT
  		frames_len = chksm_exam_pkt_ts(sz, audbuf, retbuf);
  	  #else // PES_FRM_PROT1
		 frames_len = TEI_exam_pkt_ts(sz, audbuf, retbuf);
     #endif
		if (!frames_len) goto frm_inc;
  	 #endif
 #endif
#endif // REC
#endif // TEST==2
	#ifdef REC
	  #if !defined(PES_FRM_PROT) && !defined(PES_FRM_PROT1)
    tsptsadj(audbuf, FRAME_SIZE_A, pidvid, pidpcr);
     #endif
		if (ITERS==tag)
		 #if !defined(PES_HDR_PROT) && !defined(PES_FRM_PROT) && !defined(PES_FRM_PROT1)
			r = FILE_LEN - tag*FRAME_SIZE_A;
		 #else
			r = frames_len;
		 #endif
  //#endif
	#else
		if (ITERS-1==tag)
	 #ifndef MEDIA_ON_FLASH
			r = FILE_LEN - (tag+1)*FRAME_SIZE_A;
		#else
			r = FRAME_SIZE_A; // len is guaranteed to be multiple of block
		#endif
	#endif
		else
	#ifndef REC
			r = FRAME_SIZE_A;
	#else
	  #if defined(PES_HDR_PROT) || defined(PES_FRM_PROT) || defined(PES_FRM_PROT1)
			r = frames_len;
		#else
			r = FRAME_SIZE_A;
		#endif
	#endif
  //#endif
#ifdef REC
	  #if defined(PES_FRM_PROT) || defined(PES_FRM_PROT1)
  #ifndef DBG_PROT1
    tsptsadj(retbuf, r, pidvid, pidpcr);
  #endif
     #endif
  #if defined(REALIGN_TS_PKT)
   	unsigned char chkbuf[FRAME_SIZE_A+188]__attribute__((aligned(8)));
   	static uint32_t chki = 0;
   	unsigned char tsbuf[FRAME_SIZE_A]__attribute__((aligned(8)));
   	uint8_t *pb1 = tsbuf, *pb0;
   	memcpy(chkbuf + chki, audbuf, r);
   	if (0== chki) {
   		pb0 = chkbuf;
   	  ii = 188;
		  for (i=0; i<188; i++)
 	  	  	 if (0x47 == pb0[i]) {
 	  	 		ii = i;
 	  	 		break;
 	  	  	}
 	  	  if (188==ii)
 	  	    perror_exit("can't find sync word within TS packet",-6);
 	  	  pb0 += ii; // first TS block or non re-alignment needed
 	  	}
 	  	else {
 	  		if (0x47 != audbuf[188-chki]) {
 	  			pb0 = &audbuf[188-chki]; // re-alignment failed @ 1st packet
			  ii = 188;
			  for (i=0; i<188; i++)
	 	  	  	 if (0x47 == pb0[i]) {
	 	  	 		ii = i;
	 	  	 		break;
	 	  	  	}
	 	  	  if (188==ii)
	 	  	    perror_exit("can't find sync word within TS packet",-6);
	 	  	  pb0 = chkbuf+188+ii;
	 	  	  ii += 188;
 	  		} else { // re-alignment still works @ 1st packet
	 	  	  memcpy(tsbuf, chkbuf, 188);
 	  			pb0 = chkbuf+188;
 	  			ii = 188;
 	  		}
 	  	}
   	uint32_t r0 = r- ii, r1=0;
   	while(0<=(r0-188)) {
 	  	  if (0x47 != *(pb0+188)) {
 	  		 pb0 += 1;
 	  	  }
 	  	  else {
 	  	  	 memcpy(pb1, pb0, 188);
 	  	  	 pb1 += 188;
 	  	    r1 += 188;
 	  	    pb0 += 188;
 	  	  	 r0 -= 188;
		  	 continue;
		  }
   	  ii = 188;
		  for (i=0; i<188; i++)
 	  	  	 if (0x47 == pb0[i]) {
 	  	 		ii = i;
 	  	 		break;
 	  	  	}
 	  	  if (188==ii)
 	  	    perror_exit("can't find sync word within TS packet",-6);
 	  	  r0 -= ii;
 	  	}
 	  	// copy leftover
 	  	memcpy(chkbuf, audbuf+(r-r0), r0);
 	  	 chki = r0;
   #endif
  #ifndef SRC_FRM_ENET // extract ts packets from socket
    #ifndef REALIGN_TS_PKT
      #if !defined(PES_FRM_PROT) && !defined(PES_FRM_PROT1)
			fwrite(audbuf, r, 1, file);
  		#else
			fwrite(retbuf, r, 1, file);
		#endif
	  #else
			fwrite(tsbuf, r1, 1, file);
	  #endif
  #else
	int frag, sentsize;
	unsigned char *pb = audbuf;
    #ifndef REALIGN_TS_PKT
    	#if !defined(PES_FRM_PROT) && !defined(PES_FRM_PROT1)
				for (frag=0; frag<5; frag++) {
					sentsize=sendto(udpout_socket, pb, r/5,0,(struct sockaddr *)&udpout,
	                  udpout_len);
	      if (sentsize < 0) printf("send pack ERorr\n");
	      		pb += r/5;
      		}
      #else
			       {
				pb = retbuf;
		#ifndef DBG_PROT1
						for (frag=0; frag<r/(188*2); frag++) {
							sentsize=sendto(udpout_socket, pb, 188*2,0,(struct sockaddr *)&udpout,
			                  udpout_len);
			      if (sentsize < 0) printf("send pack ERorr\n");
			      		pb += 188*2;
		      		}
		#else
						for (frag=0; frag<r/(188*2); frag++) {
							sentsize=sendto(udpout_socket, dbg_fp.rdptr, 188*2,0,(struct sockaddr *)&udpout,
			                  udpout_len);
			      if (sentsize < 0) printf("send pack ERorr\n");
			      		dbg_fp.rdptr += 188*2;
			      		if (dbg_fp.buf_end < dbg_fp.rdptr)
			      			dbg_fp.rdptr = retbuf;
		      		}
		#endif
			       }
      #endif
    #else
				sentsize=sendto(udpout_socket, tsbuf, r1,
						0, (struct sockaddr *)&udpout, udpout_len);
	      if (sentsize < 0) printf("send pack ERorr\n");
    #endif
  #endif
	#ifdef TEST_BITSTREAM
		fread(tmpbuf, FRAME_SIZE_A, 1,fdbg);
		int rr,  td;
		do {
			if (1 != do_exit) break;
			 rr=libusb_bulk_transfer(
			 devh, ep,
			 tmpbuf,
			 FRAME_SIZE_A,
			 &td,
			 50);
			//printf("xxx rr=%d, td=%d xxx\n", rr, td);
		} while (rr<0 || td!=FRAME_SIZE_A);
		short_sleep(3000*10e-6);
	#endif
#else
  #ifndef SRC_FRM_ENET // extract ts packets from socket
			fread(audbuf, r, 1,file);
  #else
	 if(0>receive(2549/*usec*/, audbuf, UDP_PACKET_MAX))
	  	perror_exit("invalid socket read, bailed out...", -6);
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
	      if (sentsize < 0) printf("send pack ERorr\n");
	      else {}//printf("send udp TS packet %d out..\n", tag);
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
     if (system_upgrade && 1==upgrade_from_lgdst) {
	     upgrade_from_lgdst = 2;
	     // now we can jump onto system upgrade process
     		goto upgrade_firmware; }
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
int udpout_init(char* udpaddr)
{
  memset(&udpout,0,sizeof(udpout));
  udpout.sin_family = AF_INET;
  udpout.sin_port = htons(UDPOUT_PORT);
  udpout.sin_addr.s_addr = inet_addr(udpaddr);
  udpout_len = sizeof(udpout);
  udpout_socket = socket(AF_INET, SOCK_DGRAM,0);
  if(udpout_socket <0)
  { printf("Fail to open udpout socket\n"); return -1; }
  return 0;
}
#endif

#define adaptation_field_no_payload 0x02
#define payload_only 0x01
#define adaptation_field_payload 0x03
#define stream_id 0xE0
#if(TSTYPE==0)
  #define PTSADD    750  //60fps in sps, ptsdelta=1500, but, ptsval have lsbit=0, so 750;
  #define STCDELTAMAX  9000
  #define STCDELTAMIN  2000
#else
  #define PTSADD    1500
  #define STCDELTAMAX  18000
  #define STCDELTAMIN  4000
#endif

int tsptsadj(unsigned char* buff, int len, int pidid, int pcrid)
{
  //adjust pcr/pts to continuous mode
  //looks for pid, pusi bit to identify pts/pcr
  //pcr is always 3 frames before pts
  //assume 60fps configuration
  //full restamping of pcr and pts
  //return: -1=fail, #=number of pts/pcr total corrections
  static unsigned int ptsval =  9000;   //lsbit=0 always
  static unsigned int dtsval =  3000;   //lsbit=0 always
  static unsigned int pcrval =  0;      //lsbit=0 always
  static unsigned int stcval =  0;
  static int start=1, ccerror=0, tagprev=0;
  static struct timeval time_prev;
  static unsigned int frmcnt=0;
  static unsigned char cc_exp;
  static int frmbadflg = 1;
  const int tspktos_pcr= (TSTYPE==0)?0:0;
  const int tspktos_pts= (TSTYPE==0)?0:0;

  struct timeval time_curr, time_delta;
  int i,j,pcrhit, pidhit,index,pusihit=0;
  int pcrptscnt=0, tagdelta;
  unsigned char * tspkt;
  if(start==1) { get_time(&time_orig); start=0;} // clock_gettime() use timespec not timeval, all calculations went wrong! liyenho
  if(len%188!=0) return(-1);
  get_time(&time_curr); // clock_gettime() use timespec not timeval, all calculations went wrong! liyenho
  time_diff(&time_curr, &time_orig, &time_delta) ;
  //printf("%u:%u (%u:%u)\n",time_delta.tv_sec, time_delta.tv_usec, time_curr.tv_sec, time_curr.tv_usec);
  																																		// be professional on integer/fixed point math!	// don't wrokaround mistake, fix it, liyenho
  stcval = ((int64_t)time_delta.tv_sec*STC_HALF_RATE)+ ( ((time_delta.tv_usec*(STC_HALF_RATE/1000)+500)/1000)/*/1000*/);

  if((abs(time_curr.tv_sec - time_prev.tv_sec) > 15)) {
    get_time(&time_prev); // clock_gettime() use timespec not timeval, all calculations went wrong! liyenho
    get_time(&time_curr); // clock_gettime() use timespec not timeval, all calculations went wrong! liyenho
 }

  if((time_curr.tv_sec > (time_prev.tv_sec+1))||
     ( (time_curr.tv_sec == (time_prev.tv_sec+1)) &&
       (time_curr.tv_usec > time_prev.tv_usec)
     )
    )
  {
    //printf("frame count/10sec = %d (t=%ds:%dus) stc=%d\n", frmcnt,time_curr.tv_sec, time_curr.tv_usec, stcval);

    if(tag< tagprev) tagdelta = tag+ITERS - tagprev;
    else             tagdelta = tag - tagprev;
    if(DEBUG_VIDEO)
      printf("Video Status: ts_cnt=%d ts_discontinuity=%d \n",tagdelta, ccerror);
        tagprev = tag;
        ccerror =0;
    frmcnt=0;
    get_time(&time_prev); // clock_gettime() use timespec not timeval, all calculations went wrong! liyenho
  }

  for(i=0;i<len/188;i++)
  {
    tspkt = buff+(i*188);
    if((tspkt[1]&0x80)==0x80)
    {
      frmbadflg=1;
      memcpy(tspkt, nullts, sizeof(nullts));
    }
    if((tspkt[1]&0x40)==0x40) pusihit=1;
    else                      pusihit=0;
    if(   ((tspkt[1]&0x1f)==((pidid>>8)&0x1f))  &&
          ((tspkt[2]&0xff)==((pidid)   &0xff))
      )  pidhit =1;
    else pidhit =0;
    if(   ((tspkt[1]&0x1f)==((pcrid>>8)&0x1f))  &&
          ((tspkt[2]&0xff)==((pcrid)   &0xff))
      )  pcrhit =1;
    else pcrhit =0;
    if((pcrhit==1)|| (pidhit==1)||(pusihit==1)) ;
    //printf("hit:%d %d %d( %x %x %x %x)\n", pcrhit,pidhit,pusihit, tspkt[0],tspkt[1],tspkt[2],tspkt[3]);

    if(pcrhit!=0)
    {
      if( (TSTYPE==0) ||
          (((TSTYPE==1)||(TSTYPE==2)) &&(pusihit==1))
        )
        {
      //restamp PCR
      //printf("restamp pcr1: %x %x %x %x pusi=%d\n",tspkt[0],tspkt[1],tspkt[2],tspkt[3],pusihit);

      tspkt[6+tspktos_pcr] = (stcval>>24)&0xff;
      tspkt[7+tspktos_pcr] = (stcval>>16)&0xff;
      tspkt[8+tspktos_pcr] = (stcval>>8 )&0xff;
      tspkt[9+tspktos_pcr] = (stcval>>0 )&0xff;
      tspkt[10+tspktos_pcr] = tspkt[10]&0x7f; //pcr lsbit always 0

      tspkt[10] = tspkt[10]&0xfe; //pcr low 300cnt always 0
      tspkt[11] = 0;
      pcrptscnt++;
      }
    }

    if(pidhit==1)
    {
      //cc check
      if((tspkt[3]&0x0f)!= cc_exp) ccerror++;
      cc_exp = tspkt[3]&0x0f;
      cc_exp = (cc_exp+1)&0x0f;
    }

    if((pidhit==1) && (pusihit==1))
    {
      //printf("frm: t=%ds %dus (%d %d)\n",time_curr.tv_sec, time_curr.tv_usec, time_prev.tv_sec, time_prev.tv_usec);
      frmbadflg = 0;
      pcrptscnt++;
      frmcnt++;

      ptsval += PTSADD;
      pcrval += PTSADD;


      if(ptsval< stcval)  ptsval = stcval + STCDELTAMAX;  //reset to center
      else if( (ptsval - stcval)< STCDELTAMIN) {
        ptsval = stcval+STCDELTAMIN;
        //printf("ptsadj(toosmall):stc=%d pts=%d\n",stcval,ptsval);
        }
      else if( (ptsval - stcval)> 18000) {
        ptsval = stcval+18000;
        //printf("ptsadj(toobig):stc=%d pts=%d\n",stcval,ptsval);
        }

      //printf("frm: stc=%d,pts=%d, delta=%d time=%ds,%dns\n",
      //  stcval, ptsval, ptsval-stcval, time_curr.tv_sec, time_curr.tv_usec);

        if ((tspkt[3] >> 4) == adaptation_field_no_payload)
        {//no payload ie: adaptation_field_control == '10'
          return(-2); //should not have no payload.
        }
        // check condition of adaptation_field_control
        if ((tspkt[3] >> 4) == payload_only)
        { //adaptation_field_control == '01'
          if (((tspkt[4] == 0x00) //packet_start_code_prfix
            && (tspkt[5] == 0x00)
            && (tspkt[6] == 0x01))
            && (tspkt[7] == stream_id))
          {// mask stream id of UDP
            index = 13 + tspkt[12]+tspktos_pts;

	    tspkt[index-1]= tspkt[index-1]&0xfd; //bit0 =0
	    tspkt[index-1]= (tspkt[index-1]&0x3) | ((ptsval&0x7f)<<2);  //bits 6:1 (6bits)
      tspkt[index-2]= (ptsval>>6)&0xff;  //bits 14:7 (8bits)
	    tspkt[index-3]= (tspkt[index-3]&0x1) | ((ptsval>>13)&0xfe); //bits 21:15 (7bits)
	    tspkt[index-4]= (ptsval>>21)&0xff; //bits32:24 (8bits)
      tspkt[index-5]= (tspkt[index-5]&0xf1) | ((ptsval>>28)&0xfe); //(3bits)
          }
          else {
          }

        }//end if adaptation_field_control == '01'
        else if ((tspkt[3] >> 4) == adaptation_field_payload)
        {//adaptation_field_control == '11'
          //5 bytes = header(4 bytes) + 1 byte of adaptation field length + length of adaptation field length
          index = (5) + tspkt[4];// right now, index begin of PES
          //check packet_start_code_prefix and stream_id
          if (((tspkt[index] == 0x00)
            && (tspkt[index + 1] == 0x00)
            && (tspkt[index + 2] == 0x01))
            && (tspkt[index + 3] == stream_id))
          {// stream id of UDP
            index = index + 9 + tspkt[index + 8]+tspktos_pts;

	    tspkt[index-1]= tspkt[index-1]&0xfd; //bit0 =0
	    tspkt[index-1]= (tspkt[index-1]&0x3) | ((ptsval&0x7f)<<2);  //bits 6:1 (6bits)
      tspkt[index-2]= (ptsval>>6)&0xff;  //bits 14:7 (8bits)
	    tspkt[index-3]= (tspkt[index-3]&0x1) | ((ptsval>>13)&0xfe); //bits 21:15 (7bits)
	    tspkt[index-4]= (ptsval>>21)&0xff; //bits32:24 (8bits)
      tspkt[index-5]= (tspkt[index-5]&0xf1) | ((ptsval>>28)&0xfe); //(3bits)
          }

        }//end adaptation_field_control == '11'
        else
        {//adaptation_field_control == '00'
	  //do nothing
        }
  }
  if(frmbadflg==1)
    memcpy(tspkt, nullts, sizeof(nullts));
  }//i
  return(pcrptscnt);
}
