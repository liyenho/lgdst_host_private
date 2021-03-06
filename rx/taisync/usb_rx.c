#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>
#include <sys/time.h>
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
#include "usb_rx.h"
#include "timer.h"
#include "MavLink.h"

#define true							1
#define false							0

volatile uint8_t system_upgrade = 0;
volatile uint8_t upgrade_from_lgdst = 0;
char upgrade_fwm_path[160]; // upgrade firmware path
#ifdef LIB
  /*static*/ volatile ipcLgdst *shmLgdst_proc = 0;
#else
  int shmid_Lgdst = -1;
  ipcLgdst *shmLgdst_proc = 0;
  void*shm = NULL;
#endif
volatile uint8_t main_loop_on = false;  // run time indicator
volatile bool ready_wait_for_mloop= false;

#ifdef RADIO_TAISYNC
volatile bool taisync_radio_up = false ;
int ctrlsnd_socket, ctrlrcv_socket; //clnt_socket;
struct timeval ctrlsnd_tv;
fd_set ctrlsnd_fd;
socklen_t ctrlsnd_len;
struct sockaddr_in ctrlsnd, ctrlrcv; //clnt;

#if USE_MAVLINK
unsigned char radio_tpacket[RADIO_USR_TX_LEN]__attribute__((aligned(8)));
unsigned char radio_rpacket[MAVLINK_USB_LEN]__attribute__((aligned(8)));
#else
unsigned char radio_tpacket[RADIO_USR_TX_LEN]__attribute__((aligned(8)));
unsigned char radio_rpacket[RADIO_USR_RX_LEN+RADIO_INFO_LEN]__attribute__((aligned(8)));
#endif

#endif

int udpout_len;
int  udpout_socket;
fd_set udpin_fd;
struct sockaddr_in udpout;

bool stream_on = false ; // ctrl xfer access speed flag, liyenho

static FILE *file = NULL;
unsigned char audbuf[FRAME_SIZE_A*FRAME_BUFFS]__attribute__((aligned(8)));
unsigned char vidbuf[FRAME_SIZE_V2]__attribute__((aligned(8)));
unsigned char retbuf[FRAME_SIZE_A*ERR_CHK_BUFFS + 188]__attribute__((aligned(8)));


static bool detached = false;
struct libusb_device_handle *devh = NULL;

volatile int do_exit = 0;	       // main loop breaker
static pthread_t poll_thread= 0,
				 lgdst_thread = 0,
				 ctrl_thr_recv = 0,
				 ctrl_thr_send = 0;
pthread_mutex_t mux_thr; // (user level)
/*static*/ pthread_mutex_t mux;
int tag=0;

int chsel_2072 = 0;
const int pidvid = 0x100;
const int pidpcr = 0x100;
static struct timeval time_orig;    // as starting time ref
uint32_t n, size, fw_info[3+1];

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
//DBG_USB_RX related ///////////////////////////////
static int dbg_frmcnt=0;
static int dbg_tseicnt=0;
static int dbg_tscnt=0;
static int dbg_tsvidcnt=0;
static int dbg_ccerror=0;
static int dbg_tdmalock=0;
static int dbg_ctrlcnt_uplink=0;
static int dbg_ctrlcnt_vid188=0;
static int dbg_rssi=9;
static uint8_t dbg_snr=9;
static int dbg_antpos=9;

 void at_exit(int status)
{
	if (do_exit) {
		do_exit = -1;

		if (poll_thread)
			pthread_join(poll_thread, NULL);
		if (lgdst_thread)
			pthread_join(lgdst_thread, NULL);
#if defined(RADIO_TAISYNC)
		pthread_join(ctrl_thr_recv, NULL);
		pthread_join(ctrl_thr_send, NULL);
#endif
		//close(udpin_socket);
		close(udpout_socket);

#if defined(RADIO_TAISYNC)
		close(ctrlsnd_socket);
		close(ctrlrcv_socket);
#endif
	}
#ifndef LIB
	if (0 <= shmid_Lgdst ) {
		shmdt(shmid_Lgdst );
		if (shmctl(shmid_Lgdst, IPC_RMID, NULL))
			perror("Error deleting shmid_Lgdst shared memory segment");
	}
#endif
	if (file)
		fclose(file);
	if (devh) {
		libusb_release_interface(devh, USB_DEV_INTF);
		if (detached)
			libusb_attach_kernel_driver(devh,USB_DEV_INTF);
		libusb_close(devh);
		libusb_exit(NULL);
	}
	pthread_mutex_destroy(&mux);
	exit(status);
}

 void perror_exit(char *message, int status)
{
	char fail[80], cs[8];
	sprintf(cs, ", %d",status);
	strncpy(fail, message, sizeof(fail));
	strcat(fail, cs);
	perror(fail);
	at_exit(status);
}

 int stream_block(unsigned char*pdata,int length)
{
	int r = 0, transferred;
	r=libusb_bulk_transfer(devh,EP_DATA,pdata,length,&transferred,70);

	if (r<0) return r;
	if (FRAME_SIZE_A!= transferred) {
		return -2;
	}
	return 0;
}

 int short_sleep(double sleep_time)
{
	struct timeval tv;

	tv.tv_sec = (time_t) floor(sleep_time);
	tv.tv_usec = (time_t) ((sleep_time - floor(sleep_time)) * 1.0e6);

	return(select(0, NULL, NULL, NULL, &tv));
}


char time_buffer[26];
time_t timer;
struct tm* tm_info;
uint8_t socket_send_buffer[MAVLINK_USB_LEN] = {0};
int socket_send_len =0;

#ifdef RADIO_TAISYNC
void *ctrl_poll_recv(void *arg)
{
	int r, i;
	unsigned char ctrl_recv_fifolvl_data = 0;
	unsigned char validdataflag = 0;
	long pv_wrbyte = 0;
	while (1 == do_exit) {
		bool ctrl_sckt_ok = *(bool*)arg;
		if (ctrl_sckt_ok) {
			pthread_mutex_lock(&mux);
			libusb_control_transfer(devh, CTRL_IN, USB_RQ, RADIO_COMM_VAL, RADIO_DATA_RX_IDX, radio_rpacket, sizeof(radio_rpacket), 0);
			pthread_mutex_unlock(&mux);

			bool filler_flag = ((radio_rpacket[0]==0xee) && (radio_rpacket[1]==0xee));
			validdataflag = !filler_flag && ((radio_rpacket[0]!=0xe5) && (radio_rpacket[1]!=0xe5));

#if USE_MAVLINK
			if (!filler_flag) {
				MavLinkPacket recvMav;
				memcpy(&recvMav, radio_rpacket, MAVLINK_USB_LEN);
				printf("Radio Rx: ");
				PrintMavLink(recvMav);
				uint16_t expec_chksum = Compute_Mavlink_Checksum(recvMav);
				bool checksum_ok = Check_Mavlink_Checksum(recvMav);
				printf(" Checksum %s", checksum_ok ? "OK" : "Error");
				if (!checksum_ok) {
					printf("\t Expected %x", expec_chksum);
				}
				printf("\n\n");

				//prepare for socket send
				MavLink_PackData(recvMav, socket_send_buffer);
				socket_send_len = recvMav.length+MAVLINK_HDR_LEN+MAVLINK_CHKSUM_LEN;
			}

#endif //end USE_MAVLINK


			if (validdataflag) //got valid payload
			{
				pv_wrbyte = sendto(ctrlrcv_socket, socket_send_buffer, socket_send_len, 0, (struct sockaddr *)&ctrlrcv, sizeof(ctrlrcv));
			}

			usleep(CTRL_RECV_POLLPERIOD);
		} //ctrl_sckt_ok
	}//(1==do_exit)
}
#ifdef LIB
void lgdst_ctl_rec_rx(unsigned char *rpacket)
 {
	if (rpacket) {
		// rpacket buffer must be at least the same length:RADIO_USR_RX_LEN+RADIO_INFO_LEN
		// user should always check for buffer information (RADIO_INFO_LEN) in regards to access status
		memcpy(rpacket, radio_rpacket, sizeof(radio_rpacket));
	}
 }
#endif

void *ctrl_poll_send(void *arg)
{  //usb send to module
	int r,tx_cnt= 0, err;
	uint32_t ctrl_send_fifolvl_data_l=CTRL_SEND_FIFODEPTH;
	bool ctrl_sckt_ok = *(bool*)arg;
	srand(time(NULL));

	while (1==do_exit) {

		{
			static uint32_t echocnt=0;
			int i;
			for(i=0;i<sizeof(radio_tpacket);i++)
				radio_tpacket[i]=0xb5;
			radio_tpacket[2] = ((unsigned char *)&echocnt)[3];
			radio_tpacket[3] = ((unsigned char *)&echocnt)[2];
			radio_tpacket[4] = ((unsigned char *)&echocnt)[1];
			radio_tpacket[5] = ((unsigned char *)&echocnt)[0];
			echocnt++;
			if(DBG_USB_TX) {
				printf("radio link Tx: ");
				for(i=0;i<sizeof(radio_tpacket);i++)
					printf("%02x ",radio_tpacket[i]);
				printf("\n");
			}
		}
		#if USE_MAVLINK
		uint8_t data[255] ={0};
		for (int j=0;j<255;j++){
			data[j] = j;
		}

		uint32_t size = rand()%120;
		MavLinkPacket pkt = Build_Mavlink_Data_Packet(size, data);
		uint8_t pending[255+2+MAVLINK_HDR_LEN];
		memcpy(pending, &pkt, pkt.length+MAVLINK_HDR_LEN);
		memcpy(pending +pkt.length+MAVLINK_HDR_LEN, pkt.checksum, 2);
		/*printf("Radio Tx: ");
		PrintMavLink(pkt);
		printf("\n");*/
		#endif

		// ------------------------------------------------

		pthread_mutex_lock(&mux);
		#if USE_MAVLINK
			libusb_control_transfer(devh,CTRL_OUT, USB_RQ,RADIO_COMM_VAL,RADIO_DATA_TX_IDX,pending, MAVLINK_USB_LEN, 0);
		#else
			libusb_control_transfer(devh,CTRL_OUT, USB_RQ,RADIO_COMM_VAL,RADIO_DATA_TX_IDX,radio_tpacket, sizeof(radio_tpacket), 0);
		#endif
		pthread_mutex_unlock(&mux);


		usleep(10*CTRL_SEND_POLLPERIOD);
	}
}
#ifdef LIB
void lgdst_ctl_snd_rx(unsigned char *tpacket)
{
	if (tpacket) {
		// tpacket buffer must be the same length:RADIO_USR_TX_LEN
		memcpy(radio_tpacket, tpacket, sizeof(radio_tpacket));
	}
}
#endif
#endif //RADIO_TAISYNC



 void *poll_thread_main(void *arg)
{
	int r=0, s, video, frms = 0, radio_cnt=0;
	long pa_wrbyte=0, pv_wrbyte=0;
	bool ctrl_sckt_ok = false;
	int tx_cnt= 0, rx_cnt= 0;

	printf("poll thread running\n");
	printf("Setup Ctrl Radio Sockets... ip=%s portTx=%d portRx=%d\n",servIP, ctrl_port_base-1, ctrl_port_base);
	int ctrlsnd_pt = htons(ctrl_port_base);
	int ctrlrcv_pt = htons(ctrl_port_base-1);

	r = socket(AF_INET,SOCK_DGRAM,0);
	s = socket(AF_INET,SOCK_DGRAM,0);

	if ((-1 != r) &&(-1!=s)) {
		ctrlsnd.sin_family = AF_INET;
		ctrlsnd.sin_addr.s_addr = htonl(INADDR_ANY);
		ctrlsnd.sin_port = ctrlsnd_pt; //ctrl_pt+1, for output
		ctrlsnd_socket = r;
		ctrlsnd_len = sizeof(ctrlsnd);
		if (bind(r, (struct sockaddr *) &ctrlsnd, sizeof(ctrlsnd)) < 0)
		{
			perror("ctrlsnd: bind() failed");
			exit(1);
		}

		ctrlrcv.sin_family = AF_INET;
		ctrlrcv.sin_addr.s_addr = inet_addr(servIP);
		ctrlrcv.sin_port = ctrlrcv_pt; //ctrl_pt+1, for socket output port
		ctrlrcv_socket = s;

		ctrl_sckt_ok = true; // validate socket open
	} //socket init fail check
		while (!ready_wait_for_mloop) ;

	taisync_radio_up = true;
	r=0;
	r = pthread_create(&ctrl_thr_recv, NULL, ctrl_poll_recv, &ctrl_sckt_ok);
	if (0 != r)
		perror_exit("ctrl recv thread creation error", r);
	r = pthread_create(&ctrl_thr_send, NULL, ctrl_poll_send, &ctrl_sckt_ok);
	if (0 != r)
		perror_exit("ctrl send thread creation error", r);


	while (1==do_exit) {
		struct timeval tv = { 0, 10000 };
		r = libusb_handle_events_timeout(NULL, &tv);
		if (r < 0) {
			perror_exit("event handler failed", r);
			break;
		}
	}

	pthread_join(ctrl_thr_recv, NULL);
	pthread_join(ctrl_thr_send, NULL);

	printf("poll thread shutting down\n");
	return NULL;
}

 void *lgdst_thread_main(void *arg)
{
	static dAccess lclMem ;
#ifdef LIB
	shmLgdst_proc = (ipcLgdst*)calloc(1, sizeof(ipcLgdst));
	if (!shmLgdst_proc)
		perror_exit("allocate shmLgdst_proc shmem failed",0);
#else
	if (0>(shmid_Lgdst = shmget(SHMKEY_RX,sizeof(ipcLgdst),IPC_CREAT|0666))) {
		perror_exit("failed to create IPC memory for lgdst process, bailed out...", -7);
	}
	if (0>(shmLgdst_proc=shmat(shmid_Lgdst, NULL, 0))) {
		perror_exit("get shmLgdst_proc shmem failed",0);
	}
	memset(shmLgdst_proc, 0, sizeof(ipcLgdst));
#endif

	shmLgdst_proc->active = -1; // shm signalling flag to lgdst client
	while (1==do_exit) {
		short_sleep(0.1);
		dev_access *acs = &lclMem.hdr;
		if (1 == shmLgdst_proc->active) {
			memcpy(&lclMem, &shmLgdst_proc->access, sizeof(dAccess));
			switch(shmLgdst_proc->type) {
			case CMD0:
				printf("CMD0: wValue = %d, wIndex = %d\n", shmLgdst_proc->tag.wValue,shmLgdst_proc->tag.wIndex);  // for debug
				pthread_mutex_lock(&mux);
				libusb_control_transfer(devh,CTRL_OUT,USB_RQ,shmLgdst_proc->tag.wValue,shmLgdst_proc->tag.wIndex,NULL, 0, 0);
				pthread_mutex_unlock(&mux);
				break;
			case CMD1:
				if (USB_STM_UPGRADE_VAL == shmLgdst_proc->tag.wValue ) {
					strncpy(upgrade_fwm_path, (char*)acs->data, shmLgdst_proc->len);
					system_upgrade = 1*(USB_STM_UPGRADE_VAL == shmLgdst_proc->tag.wValue)  ;
					puts("user requests stm32 system firmware upgrade...");
					break;
				}
				if (USB_FPGA_UPGRADE_VAL == shmLgdst_proc->tag.wValue ) {
					strncpy(upgrade_fwm_path, (char*)acs->data, shmLgdst_proc->len);
					system_upgrade = 2*(USB_FPGA_UPGRADE_VAL == shmLgdst_proc->tag.wValue)  ;
					puts("user requests fpga system firmware upgrade...");
					break;
				}
				printf("CMD1: wDir = %d, wValue = %d, wIndex = %d, len= %d, data = %d\n",
						shmLgdst_proc->tag.wDir,shmLgdst_proc->tag.wValue,shmLgdst_proc->tag.wIndex,shmLgdst_proc->len,*(int*)acs->data);  // for debug
				pthread_mutex_lock(&mux);
				libusb_control_transfer(devh,shmLgdst_proc->tag.wDir,USB_RQ,shmLgdst_proc->tag.wValue,shmLgdst_proc->tag.wIndex,(unsigned char*)acs->data,shmLgdst_proc->len, 0);
				pthread_mutex_unlock(&mux);
				if (CTRL_IN==shmLgdst_proc->tag.wDir)
					memcpy(&shmLgdst_proc->access.hdr.data,&lclMem.hdr.data, shmLgdst_proc->len);
				break;
			case ACS:
				printf("ACS: direction = %d, processor = %d\n",(int)shmLgdst_proc->tag.wDir,(int)shmLgdst_proc->tag.wIndex);  // for debug
				if (CTRL_IN==shmLgdst_proc->tag.wDir) {
					pthread_mutex_lock(&mux_thr);
/*
					Demodulator_readRegisters (
							NULL, // not used...
					      0,
					      shmLgdst_proc->tag.wIndex, // ofdm/link proc
					      (uint32_t) acs->addr,
					      acs->dcnt,
					      acs->data) ;
*/
					pthread_mutex_unlock(&mux_thr);
				}
				else if (CTRL_OUT==shmLgdst_proc->tag.wDir) {
					pthread_mutex_lock(&mux_thr);
/*
					Demodulator_writeRegisters (
							NULL, // not used...
					      0,
					      shmLgdst_proc->tag.wIndex, // ofdm/link proc
					      (uint32_t) acs->addr,
					      acs->dcnt,
					      acs->data) ;
*/
					pthread_mutex_unlock(&mux_thr);
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
#ifdef LIB
	if (shmLgdst_proc) {
		free (shmLgdst_proc);
		shmLgdst_proc = NULL;
	}
#else
	if (0 <= shmid_Lgdst ) {
		shmdt(shmid_Lgdst );
		if (shmctl(shmid_Lgdst, IPC_RMID, NULL))
			perror("Error deleting shmid_Lgdst shared memory segment");
	}
#endif
}

#ifdef LIB
 void sigint_handler_rx(int signum)
#else
 void sigint_handler(int signum)
#endif
{
	do_exit = 4; // this shall terminate poll thread
	pthread_mutex_unlock(&mux);
}


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
		fprintf(fp, "2072_CH=%d\n", *setting_rec);
		fclose(fp);
		return true;
	}

	while(fgets(ini_buff, 120, fp) != NULL)
	{
		if (strstr(ini_buff, "2072_CH") > 0)
		{
			sscanf(ini_buff, "2072_CH=%d", setting_rec);
		}
	}
	printf("setting_rec = %d\n", *setting_rec);
	fclose(fp);
	return true;
}


bool update_ini(int *setting_rec)
{
	FILE *fp;
	char *myLine;
	int maximumLineLength = 128;

	fp = fopen("usb_rx.ini", "w+");
	if (fp == NULL) {
		printf("ERROR: Fail to creat 'usb_test.ini' !!");
		return false;
	}
	fprintf(fp, "2072_CH=%d\n", *setting_rec);

	fclose(fp);
	return true;
}


#define adaptation_field_no_payload 	0x02
#define payload_only 					0x01
#define adaptation_field_payload 		0x03
#define stream_id 						0xE0

#define PTSADD    						1500
#define STCDELTAMAX  					18000
#define STCDELTAMIN  					4000


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
  const int tspktos_pcr= 0;
  const int tspktos_pts= 0;

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
  /*
  if((time_curr.tv_sec > (time_prev.tv_sec+1))||
     ( (time_curr.tv_sec == (time_prev.tv_sec+1)) &&
       (time_curr.tv_usec > time_prev.tv_usec)
     )
    )
  {
    //printf("frame count/10sec = %d (t=%ds:%dus) stc=%d\n", frmcnt,time_curr.tv_sec, time_curr.tv_usec, stcval);
    if(tag< tagprev) tagdelta = tag+ITERS - tagprev;
    else             tagdelta = tag - tagprev;
    if(1) //(ccerror) don't print out unless cc encounters error
      printf("Video Status: ts_cnt=%d ts_discontinuity=%d \n",tagdelta, ccerror);
        tagprev = tag;
        ccerror =0;
    frmcnt=0;
    get_time(&time_prev); // clock_gettime() use timespec not timeval, all calculations went wrong! liyenho
  }
  */

  for(i=0;i<len/188;i++)
  {
    tspkt = buff+(i*188);
    if((tspkt[1]&0x80)==0x80)
    {
      frmbadflg=1;
      dbg_tseicnt += 1; // liyenho
      memcpy(tspkt, nullts, sizeof(nullts));
    }
	 dbg_tscnt += 1;  // liyenho
    if((tspkt[1]&0x40)==0x40) pusihit=1;
    else                      pusihit=0;
    if(   ((tspkt[1]&0x1f)==((pidid>>8)&0x1f))  &&
          ((tspkt[2]&0xff)==((pidid)   &0xff))
      ) {
	      pidhit =1;
      }
    else pidhit =0;
    if(   ((tspkt[1]&0x1f)==((pcrid>>8)&0x1f))  &&
          ((tspkt[2]&0xff)==((pcrid)   &0xff))
      )  pcrhit =1;
    else pcrhit =0;
    if((pcrhit==1)|| (pidhit==1)||(pusihit==1)) ;
      dbg_tsvidcnt++;
    //printf("hit:%d %d %d( %x %x %x %x)\n", pcrhit,pidhit,pusihit, tspkt[0],tspkt[1],tspkt[2],tspkt[3]);

    if(pcrhit!=0)
    {
      if( pusihit==1)
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
      dbg_frmcnt++;

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

#if (/*1*/0)
  int init_device(void)
#else
  int init_device(int argc,char **argv)
#endif
{
	int r = 2, blksz=0 ;
	int try_i = 0;

	do_exit = 1;
	pthread_mutex_init(&mux, NULL);
	r = libusb_init(NULL);
	if (r < 0){
		perror_exit("failed to initialise libusb",1);
	}

	while (1) //for (int i=0; i < 10; i ++)
	{
#ifdef _ONE_USB_
		devh = libusb_open_device_with_vid_pid(NULL,USB_DEV_VENDER, USB_DEV_PRODUCT);
#else
   		if (1+1 > argc) {
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
	if (2 < argc) {
		// check for FW upgrade request from user cmdline, liyenho
    	system_upgrade = ((!strcasecmp(argv[2],"Ua0"))?4: // direct boot atmel
    						((!strcasecmp(argv[2],"Ua1"))?3:	// fwm upgrade atmel
    						((!strcasecmp(argv[2],"Uf"))?2:
    						((!strcasecmp(argv[2],"Us"))?1: 0 ))));
		if (system_upgrade) {
				if (4>system_upgrade) {
					if (argc <4) {
						puts("missing upgrade firmware filepath, bailed out...");
					return -3; 	}
					strncpy(upgrade_fwm_path, argv[3], sizeof(upgrade_fwm_path));
				}
		}
	}
		if (devh <= 0) {
			if (try_i == 10) {
				printf("%c   %c", 0x0d, 0x0d);
				try_i = 0;
			}
			printf("fail to aquire USB link. attempt:%d ",try_i);
			fflush(stdout);
			try_i ++;
			sleep(1);
			continue;
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
		printf("\n");
		break; // USB Device found!! exit waiting loop
	}
	if (devh <= 0) {
		libusb_exit(NULL);
		perror_exit("could not find/open USB device",2);
	}
	printf("claimed interface\n");
	if (system_upgrade) // no need to bring up other proc threads,
		goto upgrade_next;
	// send system restart command...
	pthread_mutex_lock(&mux);
	libusb_control_transfer(devh,CTRL_OUT, USB_RQ,USB_SYSTEM_RESTART_VAL,USB_HOST_MSG_IDX,NULL, 0, 0);
	pthread_mutex_unlock(&mux);
 		do {
	 		pthread_mutex_lock(&mux);
			 libusb_control_transfer(devh,
					CTRL_IN, USB_RQ,
					USB_STREAM_ON_VAL,
					USB_QUERY_IDX,
					&main_loop_on, sizeof(main_loop_on), 0);
			pthread_mutex_unlock(&mux);
			if (!main_loop_on) {
				short_sleep(1); 	// setup & settle in 1 sec
			} else
			break;
		} while (1);

	// bring up all others after main system restart command sent...
	r = pthread_create(&poll_thread, NULL, poll_thread_main, NULL);
	if (0 != r)
		perror_exit("poll thread creation error", r);
upgrade_next:
#ifndef LIB
	if (SIG_ERR == signal(SIGINT, sigint_handler)) {
		perror("FAIL: assigning signal handler");
	}
#endif
   if (system_upgrade)
   	return 1;	// system upgrade or atmel reboot...
	r = pthread_create(&lgdst_thread, NULL, lgdst_thread_main, NULL);
	if (0 != r)
		perror_exit("lgdst thread creation error", r);

	return 0;

}

#ifdef LIB
  void lgdst_ts_rx(uint8_t *tsbuf0)
#else
  int transfer_video(void)
#endif
{
	int i,r, msg[80]; // access buffer
	uint64_t llw;
	int ii;
	uint32_t frames_len;
	int frag, sentsize;
	stream_on = true; // slow down ctrl xfer access at real time, liyenho
#ifndef LIB
	tag = 0;
	dev_access *acs = (dev_access*)msg;
	acs->access = TS_VID_ACTIVE;
	acs->dcnt = 0; // no param
	acs->addr = 0x0; // by wire not addr
	pthread_mutex_lock(&mux);
	libusb_control_transfer(devh,CTRL_OUT,USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs), 0);
	pthread_mutex_unlock(&mux);
	short_sleep(0.2);
	pthread_mutex_lock(&mux);
	libusb_control_transfer(devh, CTRL_OUT, USB_RQ, 0x07, 0, NULL, 0, 0);
	pthread_mutex_unlock(&mux);
#endif
	tag += 1;
#ifndef LIB
	while (ITERS>=tag) {
		if (1 != do_exit) break; // we can't fail here......
		if (0>stream_block(audbuf,FRAME_SIZE_A)) continue;
#else
	while(1==do_exit && 0>stream_block(audbuf,FRAME_SIZE_A));
	if (1!=do_exit) return;
#endif
 	for (ii=0, i=0; i<(FRAME_SIZE_A/8); i++) {
		llw = *((uint64_t*)audbuf+i);
		*((uint16_t*)audbuf+ii++) = ((0xff & llw)<<8) | (0xff & (llw>>8));
		llw >>= 16;
		*((uint16_t*)audbuf+ii++) = ((0xff & llw)<<8) | (0xff & (llw>>8));
		llw >>= 16;
		*((uint16_t*)audbuf+ii++) = ((0xff & llw)<<8) | (0xff & (llw>>8));
		llw >>= 16;
		*((uint16_t*)audbuf+ii++) = ((0xff & llw)<<8) | (0xff & (llw>>8));
 	}

		 tsptsadj(audbuf, FRAME_SIZE_A, pidvid, pidpcr);
#ifndef LIB
		if (ITERS==tag)
			r = FILE_LEN - tag*FRAME_SIZE_A;
         else
			r = FRAME_SIZE_A;


		unsigned char *pb = audbuf;

		for (frag=0; frag<5; frag++) {
			sentsize=sendto(udpout_socket, pb, r/5,0,(struct sockaddr *)&udpout,udpout_len);
	      if (sentsize < 0) printf("send pack ERorr\n");
	      		pb += r/5;
      		}

frm_inc:
		tag += 1;

		if (ITERS==tag)
			tag = 0; // make it endless, liyenho
#else  // in LIB mode
		memcpy(tsbuf0, audbuf, FRAME_SIZE_A);
#endif
#ifndef LIB
	}
#endif
}
#ifdef LIB
  void lgdst_deinit_rx(int rcode)
#else
  void deinit_device(void)
#endif
{
#ifdef LIB
	if (0 > rcode || true==(bool) rcode) goto _fail;
#endif
	fprintf(stderr, "Wrap up/Leaving test_rec app...\n");

	if (0 > do_exit) ;//return do_exit;  // return from error
	else do_exit = 4;

	if (file) fclose(file);

	pthread_join(poll_thread, NULL);
	pthread_join(lgdst_thread, NULL);
	//close(udpin_socket);
	close(udpout_socket);
#if defined(RADIO_TAISYNC)
	close(ctrlsnd_socket);
	close(ctrlrcv_socket);
#endif
_fail:
	libusb_release_interface(devh,USB_DEV_INTF);
	if (detached)
		libusb_attach_kernel_driver(devh,USB_DEV_INTF);
	libusb_close(devh);
	libusb_exit(NULL);
	pthread_mutex_destroy(&mux);
	return ;

}
#ifndef LIB
int udpout_init(char* udpaddr)
{
	memset(&udpout,0,sizeof(udpout));
	udpout.sin_family = AF_INET;
	udpout.sin_port = htons(UDPOUT_PORT);
	udpout.sin_addr.s_addr = inet_addr(udpaddr);
	udpout_len = sizeof(udpout);
	udpout_socket = socket(AF_INET, SOCK_DGRAM,0);
	if(udpout_socket <0)
	{
		printf("Fail to open udpout socket.\n");
		return -1;
	}
	return 0;
}
#endif
#ifdef LIB
  void lgdst_vid_stats_rx(long *sigdbm, double *ptvitber) {
	 int32_t 		i, r, msg[80]; // access buffer
	 dev_access *acs = (dev_access*)msg;
	  uint32_t error;

	acs->access = RDO_RSSI;  // set ofdm bandwidth
	acs->dcnt = sizeof(uint8_t);

		 	pthread_mutex_lock(&mux);
		r = libusb_control_transfer(devh,
						CTRL_OUT, USB_RQ,
						USB_HOST_MSG_TX_VAL,
						USB_HOST_MSG_IDX,
						acs, sizeof(*acs)+(acs->dcnt-1), 0);
			pthread_mutex_unlock(&mux);

        short_sleep (/*0.1*/0.01);

			while(1) {
		 		pthread_mutex_lock(&mux);
				if (libusb_control_transfer(devh,CTRL_IN, USB_RQ,USB_HOST_MSG_RX_VAL,USB_HOST_MSG_IDX,
																			(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0)){
					pthread_mutex_unlock(&mux);
					break;
				}
				pthread_mutex_unlock(&mux);
				short_sleep(0.0005);
			}

		printf("rf rssi reading = %d\n", *(uint8_t*)acs->data);

	acs->access = RDO_RSSI;  // set ofdm bandwidth
	acs->dcnt = sizeof(uint32_t);

		 	pthread_mutex_lock(&mux);
		r = libusb_control_transfer(devh,
						CTRL_OUT, USB_RQ,
						USB_HOST_MSG_TX_VAL,
						USB_HOST_MSG_IDX,
						acs, sizeof(*acs)+(acs->dcnt-1), 0);
			pthread_mutex_unlock(&mux);

        short_sleep (/*0.1*/0.01);

			while(1) {
		 		pthread_mutex_lock(&mux);
				if (libusb_control_transfer(devh,CTRL_IN, USB_RQ,USB_HOST_MSG_RX_VAL,USB_HOST_MSG_IDX,
																			(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0)){
					pthread_mutex_unlock(&mux);
					break;
				}
				pthread_mutex_unlock(&mux);
				short_sleep(0.0005);
			}

		printf("ldpc failed reading = %d\n", *(uint32_t*)acs->data);
  }

bool lgdst_upgrade_rx(int argc, char **argv)  // return -1 when failed, liyenho
{
  if (system_upgrade) { // user request firmware upgrade
	uint32_t n, size, fw_info[3+1]; /*len = SMS_FW_HDR_LEN*/
  	FILE *file_up ;
  	int r, end, cur, len, blksz=0 ;
  	float delay ;
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
  		if (1/*stm32*/== system_upgrade) {
			pthread_mutex_lock(&mux);
		 libusb_control_transfer(devh, CTRL_OUT, USB_RQ,
			USB_STM_UPGRADE_VAL, USB_HOST_MSG_IDX,
			fw_info, FW_UPGRADE_HDR_LEN, 0);
			pthread_mutex_unlock(&mux);
			blksz = 7200;
			delay = 0.84;
	  	}
  		else if (2/*fpga*/== system_upgrade) {
	  		/*fw_info[2] = 0x00295700;  // constant fpga download address
	  		printf("upgrade firmware address: 0x%08x\n", fw_info[2]);*/
			pthread_mutex_lock(&mux);
		 libusb_control_transfer(devh, CTRL_OUT, USB_RQ,
			USB_FPGA_UPGRADE_VAL, USB_HOST_MSG_IDX,
			fw_info, FW_UPGRADE_HDR_LEN, 0);
			pthread_mutex_unlock(&mux);
			blksz = 15000;
			delay = 3.5;
  		}
  		else if (3/*atmel*/== system_upgrade) {
			pthread_mutex_lock(&mux);
		 libusb_control_transfer(devh, CTRL_OUT, USB_RQ,
			USB_FWM_UPDATE_VAL, USB_HOST_MSG_IDX,
			fw_info, ATMEL_UPGRADE_HDR_LEN, 0);
			pthread_mutex_unlock(&mux);
			blksz = 7200;
			delay = 0.84;
  		}
  		else if (4/*atmel boot*/== system_upgrade) {
			pthread_mutex_lock(&mux);
		 libusb_control_transfer(devh, CTRL_OUT, USB_RQ,
			USB_FWM_BOOTUP_VAL, USB_HOST_MSG_IDX,
			NULL, 0, 0);
			pthread_mutex_unlock(&mux);
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
			pthread_mutex_lock(&mux);
			 libusb_control_transfer(devh,
					CTRL_IN, USB_RQ,
					USB_STREAM_ON_VAL,
					USB_QUERY_IDX,
					&main_loop_on, sizeof(main_loop_on), 0);
			pthread_mutex_unlock(&mux);
			if (!main_loop_on) {
				short_sleep(1); 	// setup & settle in 1 sec
			} else
			break;
		} while (1);
 #ifdef FWUP_DNLD_DBG
 		rem = len;
 		do {
			pthread_mutex_lock(&mux);
 			libusb_control_transfer(devh, CTRL_IN, USB_RQ, 0x7d, 0, &ta, sizeof(ta), 0);
 			pthread_mutex_unlock(&mux);
 			short_sleep(0.005);
		} while (0==ta);
		pthread_mutex_lock(&mux);
 		libusb_control_transfer(devh, CTRL_OUT, USB_RQ, 0xd7, 0, NULL, 0, 0);
 		pthread_mutex_unlock(&mux);
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
	 		pthread_mutex_lock(&mux);
 			libusb_control_transfer(devh, CTRL_IN, USB_RQ, 0x7d, 0, &ta, sizeof(ta), 0);
 			pthread_mutex_unlock(&mux);
 			short_sleep(0.005);
		} while (0==ta);
		pthread_mutex_lock(&mux);
 		libusb_control_transfer(devh, CTRL_OUT, USB_RQ, 0xd7, 0, NULL, 0, 0);
 		pthread_mutex_unlock(&mux);
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
  			pthread_mutex_lock(&mux);
	 		libusb_control_transfer(devh, CTRL_OUT, USB_RQ, 0xd7, 0, NULL, 0, 0);
	 		pthread_mutex_unlock(&mux);
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
			pthread_mutex_lock(&mux);
	 		libusb_control_transfer(devh, CTRL_OUT, USB_RQ, 0xd7, 0, NULL, 0, 0);
	 		pthread_mutex_unlock(&mux);
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
		 return true; // finished regardless if there's error
	else
		return false; // normal shutdown is required
  }
}
#endif
//set channel value = set channel -1583000
#ifdef LIB
  int lgdst_init_vid_rx()
#else
  uint32_t  main(int argc,char **argv)
#endif
{
	 int32_t 		i, r, msg[80]; // access buffer
	 dev_access *acs = (dev_access*)msg;
	uint32_t error=Error_NO_ERROR;
#ifndef LIB
	#if (/*1*/0)
		init_device();
	#else
		init_device(argc, argv);
	   if (system_upgrade)
	   	return 1;	// system upgrade or atmel reboot...
	#endif
#endif
#if /*true*/false  // test atmel encapsulation of asic host
	// initialize video subsystem inside atmel
		pthread_mutex_lock(&mux);
		 libusb_control_transfer(devh,
		 													CTRL_OUT,
		 													USB_RQ,
															USB_INIT_VID_SUBSYS,
															0x1,
															NULL,
															0,
															0);
		pthread_mutex_unlock(&mux);
	// startup video subsystem inside atmel
		pthread_mutex_lock(&mux);
		 libusb_control_transfer(devh,
		 													CTRL_OUT,
		 													USB_RQ,
															USB_START_VID_SUBSYS,
															0x1,
															NULL,
															0,
															0);
		pthread_mutex_unlock(&mux);
#else
	acs->access = OFDM_BW;  // set ofdm bandwidth
	acs->dcnt = sizeof(uint8_t);
	*((uint8_t*)acs->data) = (uint8_t)TEN_MHZ;

		 	pthread_mutex_lock(&mux);
		r = libusb_control_transfer(devh,
						CTRL_OUT, USB_RQ,
						USB_HOST_MSG_TX_VAL,
						USB_HOST_MSG_IDX,
						acs, sizeof(*acs)+(acs->dcnt-1), 0);
			pthread_mutex_unlock(&mux);

        short_sleep (/*0.1*/0.01);

	acs->access = OFDM_CARR;  // set center frequency
	acs->dcnt = sizeof(uint16_t);
	*((uint16_t*)acs->data) = 2392;  // 2392 mhz rf carr

		 	pthread_mutex_lock(&mux);
		r = libusb_control_transfer(devh,
						CTRL_OUT, USB_RQ,
						USB_HOST_MSG_TX_VAL,
						USB_HOST_MSG_IDX,
						acs, sizeof(*acs)+(acs->dcnt-1), 0);
			pthread_mutex_unlock(&mux);

        short_sleep (/*0.1*/0.01);

	acs->access = RDO_ANT;  // enable rx antenna
	acs->dcnt = sizeof(uint8_t);  // boolean var
	*((uint8_t*)acs->data) = 1;  // enable

		 	pthread_mutex_lock(&mux);
		r = libusb_control_transfer(devh,
						CTRL_OUT, USB_RQ,
						USB_HOST_MSG_TX_VAL,
						USB_HOST_MSG_IDX,
						acs, sizeof(*acs)+(acs->dcnt-1), 0);
			pthread_mutex_unlock(&mux);

        short_sleep (/*0.1*/0.01);
#endif
ready_wait_for_mloop = true;
#ifndef LIB
	udpout_init(servIP);
	transfer_video();
	deinit_device();
#else
	acs->access = TS_VID_ACTIVE;
	acs->dcnt = 0; // no param
	acs->addr = 0x0; // by wire not addr
	pthread_mutex_lock(&mux);
	libusb_control_transfer(devh,CTRL_OUT,USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs), 0);
	pthread_mutex_unlock(&mux);
	short_sleep(0.2);
	pthread_mutex_lock(&mux);
	libusb_control_transfer(devh, CTRL_OUT, USB_RQ, 0x07, 0, NULL, 0, 0);
	pthread_mutex_unlock(&mux);
#endif
_exit:
	return error;

}
#ifdef LIB
  int lgdst_init_rx(int argc, char **argv) {
#if (/*1*/0)
	init_device();
#else
	init_device(argc, argv);
   if (system_upgrade)
   	return 1;	// system upgrade or atmel reboot...
#endif
  }
#endif




