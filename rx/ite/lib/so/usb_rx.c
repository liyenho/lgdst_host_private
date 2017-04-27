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
#include "rf2072_set.h"
#include "usb_rx.h"
#include "platform_it9137.h"
#include "type.h"
#include "timer.h"

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

#ifdef RADIO_SI4463
volatile bool si4463_radio_up = false ;
int ctrlsnd_socket, ctrlrcv_socket; //clnt_socket;
struct timeval ctrlsnd_tv;
fd_set ctrlsnd_fd;
socklen_t ctrlsnd_len;
struct sockaddr_in ctrlsnd, ctrlrcv; //clnt;

unsigned char radio_tpacket[RADIO_USR_TX_LEN]__attribute__((aligned(8)));
unsigned char radio_rpacket[RADIO_USR_RX_LEN+RADIO_INFO_LEN]__attribute__((aligned(8)));

#endif

int udpout_len;
int  udpout_socket;
fd_set udpin_fd;
struct sockaddr_in udpout;

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
static pthread_mutex_t mux;
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

intmax_t get_file_size(const char* file_path)
{
	struct stat statbuf;

	if(stat(file_path, &statbuf) == -1)
		return -1;
	return (intmax_t) statbuf.st_size;
}


 void DieWithError(char *errorMessage)  /* Error handling function */
{
	perror(errorMessage);
	exit(1);
}

 void at_exit(int status)
{
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
		//close(udpin_socket);
		close(udpout_socket);

#if defined(RADIO_SI4463)
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

#ifdef RADIO_SI4463
 void *ctrl_poll_recv(void *arg)
{
	int r, i;
	unsigned char ctrl_recv_fifolvl_data=0;
	unsigned char validdataflag=0;
	long pv_wrbyte=0;
	while (1==do_exit) {
		bool ctrl_sckt_ok = *(bool*)arg;
		if (ctrl_sckt_ok) {
			pthread_mutex_lock(&mux);
			libusb_control_transfer(devh,CTRL_IN, USB_RQ,RADIO_COMM_VAL,RADIO_DATA_RX_IDX,radio_rpacket, sizeof(radio_rpacket), 0);
			pthread_mutex_unlock(&mux);

			validdataflag=(radio_rpacket[RADIO_USR_RX_LEN]&0x01);
			if(validdataflag==0x01) //got valid payload
			{
				pv_wrbyte = sendto(ctrlrcv_socket,radio_rpacket,RADIO_USR_RX_LEN,0,(struct sockaddr *)&ctrlrcv, sizeof(ctrlrcv));
			}
			if(validdataflag)
			{
				printf("radio link Rx: ");
				for(i=0;i<sizeof(radio_rpacket);i++)
					printf("%02x ",radio_rpacket[i]);
				printf("\n");
			}

			{// user packet dataloss ---------------------------------
				static unsigned char val_exp;
				unsigned char        val_cur;
				static int ilcnt=0, becnt=0;
				unsigned char byteerror=0;
				if(validdataflag==0x01){
					for(i=6;i<RADIO_USR_RX_LEN;i++)
						if(radio_rpacket[i]!=0xb5) {
							byteerror++;
							becnt++;
						}
					if(byteerror > 0) printf("User packet corruption = %d. ++++++++++\n", becnt);
					val_cur = radio_rpacket[5];
					val_exp = (unsigned char)(val_exp+1);
					if(val_exp != val_cur)
					{
						ilcnt++;
						printf("User packet dataloss = %d. !!!!!!!!!!!!!!!!!!!!!!!!!!!!\n",ilcnt);
					}
					val_exp = val_cur;
				}//end valid data
			}// user packet dataloss check


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
	int r,tx_cnt= 0, err, rcvsize;
	uint32_t ctrl_send_fifolvl_data_l=CTRL_SEND_FIFODEPTH;
	bool ctrl_sckt_ok = *(bool*)arg;

	while (1==do_exit) {

		rcvsize = 0;
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
			rcvsize = sizeof(radio_tpacket);
			if(0) {
				printf("radio link Tx: ");
				for(i=0;i<sizeof(radio_tpacket);i++)
					printf("%02x ",radio_tpacket[i]);
				printf("\n");
			}
		}
		// ------------------------------------------------
		if (sizeof(radio_tpacket) == rcvsize) {
			pthread_mutex_lock(&mux);
			libusb_control_transfer(devh,CTRL_OUT, USB_RQ,RADIO_COMM_VAL,RADIO_DATA_TX_IDX,radio_tpacket, sizeof(radio_tpacket), 0);
			pthread_mutex_unlock(&mux);
		}

		usleep(CTRL_SEND_POLLPERIOD);
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
#endif //RADIO_SI4463



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
			DieWithError("ctrlsnd: bind() failed");
		}

		ctrlrcv.sin_family = AF_INET;
		ctrlrcv.sin_addr.s_addr = inet_addr(servIP);
		ctrlrcv.sin_port = ctrlrcv_pt; //ctrl_pt+1, for socket output port
		ctrlrcv_socket = s;

		ctrl_sckt_ok = true; // validate socket open
	} //socket init fail check
		while (!ready_wait_for_mloop) ;

	si4463_radio_up = true;
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
				if (USB_CPLD_UPGRADE_VAL == shmLgdst_proc->tag.wValue ) {
					strncpy(upgrade_fwm_path, (char*)acs->data, shmLgdst_proc->len);
					system_upgrade = 1*(USB_CPLD_UPGRADE_VAL == shmLgdst_proc->tag.wValue)  ;
					puts("user requests system firmware upgrade...");
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
				if (shmLgdst_proc->echo) {
					printf("ACS/echo: access= %d, dcnt= %d\n",(int)acs->access,(int)acs->dcnt);  // for debug
					pthread_mutex_lock(&mux);
					while(0==libusb_control_transfer(devh,CTRL_IN, USB_RQ,USB_HOST_MSG_RX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs)-sizeof(acs->data[0]), 0))
						short_sleep(0.0005);
					pthread_mutex_unlock(&mux);
					memcpy(&shmLgdst_proc->access.hdr,&lclMem.hdr, sizeof(lclMem.hdr)-sizeof(lclMem.hdr.data[0]));
					shmLgdst_proc->echo = false;
				}
				else {
					printf("ACS: type = %d, access = %d, dcnt=%d\n",(int)shmLgdst_proc->type,(int)acs->access,(int)acs->dcnt);  // for debug
					pthread_mutex_lock(&mux);
					libusb_control_transfer(devh,shmLgdst_proc->tag.wDir,USB_RQ,shmLgdst_proc->tag.wValue,shmLgdst_proc->tag.wIndex,(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1)*sizeof(uint16_t), 0);
					pthread_mutex_unlock(&mux);
					if (CTRL_IN==shmLgdst_proc->tag.wDir)
						memcpy(&shmLgdst_proc->access.hdr,&lclMem.hdr, sizeof(lclMem.hdr)+(acs->dcnt-1)*sizeof(lclMem.hdr.data[0]));
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

 void rffe_write_regs(dev_cfg* pregs, int size)
{
	int r;
	int32_t i, msg[80]; // access buffer
	dev_access *acs = (dev_access*)msg;
	uint16_t *conv= (uint16_t*)acs->data;

	acs->dcnt = sizeof(pregs[i].data);
	for (i=0; i<size; i++) {

		acs->access = RF2072_WRITE;
		acs->addr = pregs[i].addr;
		*conv = pregs[i].data;
		libusb_control_transfer(devh,CTRL_OUT, USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0);

		short_sleep(0.2); 	// validate echo after 0.1 sec
		printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",*(uint16_t*)acs->data,acs->addr);
	}

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

  if((time_curr.tv_sec > (time_prev.tv_sec+1))||
     ( (time_curr.tv_sec == (time_prev.tv_sec+1)) &&
       (time_curr.tv_usec > time_prev.tv_usec)
     )
    )
  {
    //printf("frame count/10sec = %d (t=%ds:%dus) stc=%d\n", frmcnt,time_curr.tv_sec, time_curr.tv_usec, stcval);

    if(tag< tagprev) tagdelta = tag+ITERS - tagprev;
    else             tagdelta = tag - tagprev;
    if(ccerror) // don't print out unless cc encounters error
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

//1=cpld firmware update  Uc   blksz = 7200;  delay = 0.84;
//3=atmel firmware update Ua1  blksz = 7200;  delay = 0.84;
//4=atmel boot direct   Ua0
int cpld_firmware_update(int mode, const char*file_name)
{
	uint32_t n,fw_info[3+1];
	int cur, len,size,blksz=0,nread=0;
	float delay;
	FILE *file_up=NULL ;
	static unsigned char vidbuf[FRAME_SIZE_V2]__attribute__((aligned(8)));
	memset(vidbuf,0,sizeof(vidbuf));
	if((file_up = fopen(file_name, "rb"))!=NULL)
	{
		fread(&fw_info[0], 1, sizeof(int), file_up);// using 1st 4 bytes as sync word
		printf("upgrade firmware header: 0x%08x\n", fw_info[0]);
		len=get_file_size(file_name);
		//printf("file_size =%d\n",len);
		fw_info[1] = len ;
		printf("upgrade firmware length: %d bytes\n", fw_info[1]);
		//confirm system_upgrade and blksz  delay
		fseek(file_up,0,SEEK_SET);

		if (1/*cpld*/== mode) {
			fw_info[2] = 0x00003800;  // constant cpld download address
			printf("upgrade firmware address: 0x%08x\n", fw_info[2]);
			libusb_control_transfer(devh, CTRL_OUT, USB_RQ,	USB_CPLD_UPGRADE_VAL, USB_HOST_MSG_IDX,	(unsigned char*)fw_info, FW_UPGRADE_HDR_LEN, 0);
			blksz = 7200; // similar size to atmel bin image, it should be fine
			delay = 0.84;
		}
		else if (3/*atmel*/== mode) {
			libusb_control_transfer(devh, CTRL_OUT, USB_RQ,	USB_FWM_UPDATE_VAL, USB_HOST_MSG_IDX,(unsigned char*)fw_info, ATMEL_UPGRADE_HDR_LEN, 0);
			blksz = 7200;
			delay = 0.84;
		}
		else if(4/*atmel boot*/==mode){

			libusb_control_transfer(devh, CTRL_OUT, USB_RQ,USB_FWM_BOOTUP_VAL, USB_HOST_MSG_IDX,NULL, 0, 0);
			short_sleep(0.01);

		}
		else {
			printf("invalid target for upgrade request !\n");
			return -1;
		}
		// no more 8192 (divisible into 64) if we'll need to perform usb read after write verification!!!
		// a big bug in atmel usb core!!! liyenho
		if (blksz*2 > len) {
			fclose(file_up);
			perror_exit("upgrade firmware data is too short, bail out",-5);
		}
		cur = blksz;
		do {
			libusb_control_transfer(devh,CTRL_IN, USB_RQ,USB_STREAM_ON_VAL,USB_QUERY_IDX,(unsigned char*)&main_loop_on, sizeof(main_loop_on), 0);
			//printf("main_loop_on=%d\n",main_loop_on);
			if (!main_loop_on) {
				short_sleep(1); 	// setup & settle in 1 sec
			} else
				break;
		} while (1);
		n = 1;
		short_sleep(1); // chk[] search may take time?!

		while(1) {
			if (1 != do_exit) break;
			nread=fread(vidbuf, 1, cur,file_up);
			if(libusb_bulk_transfer(devh, 2 | LIBUSB_ENDPOINT_OUT,(unsigned char*)vidbuf,cur,&size,0)<0){
				printf("libusb_bulk_transfer failed !\n");
			}
			if(nread!=size)printf(" transfer mismatch !\n");
			printf("fw data block #%d sent (%d)...\n", n++,size);
			short_sleep(delay); // sync up with target/flash access on max timing
			len -= cur;
			if (!len) break;
			cur =(len>=blksz)?blksz:len;
		}
		fread(vidbuf, cur, 1,file_up);
		fclose(file_up);
		// send something to flush usb bulk pipe, liyenho
		if(libusb_bulk_transfer(devh, 2 | LIBUSB_ENDPOINT_OUT,(unsigned char*)vidbuf,10*EXTRA,&size,0)<0){
			printf("libusb transfer failed !\n");
		}
		if(nread!=size)printf(" transfer mismatch !\n");

	}else{
		printf("failed to open upgrade firmware !\n");
		return -2;
	}
	return 0;
}

#if (/*1*/0)
  int init_device(void)
#else
  int init_device(int argc,char **argv)
#endif
{
	int r = 2, blksz=0 ;
	int try_i = 0;
	float delay ;

	do_exit = 1;
	pthread_mutex_init(&mux, NULL);
	r = libusb_init(NULL);
	if (r < 0){
		perror_exit("failed to initialise libusb",1);
	}

	while (1) //for (int i=0; i < 10; i ++)
	{
#if (/*1*/0)
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
    	system_upgrade = (!strcasecmp(argv[2],"Ua0"))?4: // direct boot atmel
    						((!strcasecmp(argv[2],"Ua1"))?3:	// fwm upgrade atmel
    						/*((!strcasecmp(argv[2],"Uf"))?2:*/
    						((!strcasecmp(argv[2],"Uc"))?1: 0 )/*)*/);
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

	// send system restart command...
	libusb_control_transfer(devh,CTRL_OUT, USB_RQ,USB_SYSTEM_RESTART_VAL,USB_HOST_MSG_IDX,NULL, 0, 0);

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

	// bring up all others after main system restart command sent...
	r = pthread_create(&poll_thread, NULL, poll_thread_main, NULL);
	if (0 != r)
		perror_exit("poll thread creation error", r);
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

int set_frequency_rf2072(uint32_t f_lo_KHz)
{

	uint16_t writeValue, P2_FREQ1,P2_FREQ2,P2_FREQ3;

	int i;
	uint32_t error;

	uint32_t dw_temp, dw_temp1000, dw_temp2, dw_ndiv1000,dw_fvco_KHz;
	uint8_t n_lo2, lodiv2,fbkdiv2, numlsb2,p2presc2,p2lodiv2;
	uint16_t n2 ,nummsb2 ;


	dw_temp = 5400000;
	dw_temp2 = dw_temp/(f_lo_KHz);
	dw_temp = log2(dw_temp2);
	n_lo2 = (uint8_t)dw_temp;
	p2lodiv2 = n_lo2;
	//lodiv =pow (2.0, n_lo);// 2^n_lo;
	lodiv2 = 1;
	for(i=0;i<n_lo2;i++)
		lodiv2 = lodiv2 * 2;


	dw_fvco_KHz = lodiv2 * f_lo_KHz;

	if(dw_fvco_KHz>3200000){
		fbkdiv2 = 4;

		p2presc2 = 2;

		//pllcpl to 3 to do ???
	}else{
		fbkdiv2 = 2;

		p2presc2 = 1;

	}
	dw_ndiv1000 = (dw_fvco_KHz*10)/fbkdiv2/26;

	n2 =  (uint16_t) (dw_fvco_KHz/fbkdiv2/26000);
	dw_temp1000 = (65536*(dw_ndiv1000-n2*10000));
	nummsb2 = (uint16_t)((65536*(dw_ndiv1000-n2*10000))/10000);
	numlsb2 = (uint8_t)((256*(dw_temp1000-nummsb2*10000))/10000);
//-------------------------------------------


	P2_FREQ1 = n2<<7 | p2lodiv2<<4 | p2presc2<<2 | 0x02;
	P2_FREQ2 = nummsb2;
	P2_FREQ3 = numlsb2<<8;

//	printf("P2_FREQ1=%x,P2_FREQ2=%x,P2_FREQ3=%x\n",P2_FREQ1,P2_FREQ2,P2_FREQ3);

	int32_t  msg[80]; // access buffer
	dev_access *acs = (dev_access*)msg;
	uint16_t *conv= (uint16_t*)acs->data;


		acs->dcnt = sizeof(*conv);

        acs->access = RF2072_WRITE;
		acs->addr = 0x08;
		*conv =(0xFC06 & 0x7FFE) | 0x8000;	//ct_min=0 ct_max=127
		libusb_control_transfer(devh,CTRL_OUT, USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0);

		short_sleep(0.2); 	// validate echo after 0.1 sec
		printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",*(uint16_t*)acs->data,acs->addr);

		acs->dcnt = sizeof(*conv);

        acs->access = RF2072_WRITE;
		acs->addr = 0x0F;
		*conv =P2_FREQ1;
		libusb_control_transfer(devh,CTRL_OUT, USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0);

		short_sleep(0.2); 	// validate echo after 0.1 sec
		printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",*(uint16_t*)acs->data,acs->addr);

		acs->dcnt = sizeof(*conv);

        acs->access = RF2072_WRITE;
		acs->addr = 0x10;
		*conv =P2_FREQ2;
		libusb_control_transfer(devh,CTRL_OUT, USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0);

		short_sleep(0.2); 	// validate echo after 0.1 sec
		printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",*(uint16_t*)acs->data,acs->addr);

		acs->dcnt = sizeof(*conv);

        acs->access = RF2072_WRITE;
		acs->addr = 0x11;
		*conv =P2_FREQ3;
		libusb_control_transfer(devh,CTRL_OUT, USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0);

		short_sleep(0.2); 	// validate echo after 0.1 sec
		printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",*(uint16_t*)acs->data,acs->addr);

		return 0;


}

int init_rf2072(void)
{
	dev_cfg* pregs ;
	int32_t i, sz, msg[80]; // access buffer
	dev_access *acs = (dev_access*)msg;
   uint16_t *conv= (uint16_t*)acs->data;

	open_ini(&chsel_2072);
	acs->access = RF2072_RESET;
	acs->dcnt = 0; // no param
	acs->addr = 0x0; // by wire not addr
	libusb_control_transfer(devh,CTRL_OUT,USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs), 0);
	short_sleep(0.2);

	rffe_write_regs(pregs=GET_ARRAY(chsel_2072), sz=ARRAY_SIZE(chsel_2072));
	short_sleep(0.2);
     set_frequency_rf2072(LO_Frequency);
	 printf("rx rffe is running...\n");
#if 0
	 //if not pull up rxena need write 0x15 at here
	acs->dcnt = sizeof(uint16_t);
        acs->access = RF2072_WRITE;
		acs->addr = 0x15;
		*conv =(0x8000 | 0x0008);
		libusb_control_transfer(devh,CTRL_OUT, USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0);
		short_sleep(0.2);
		printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",*(uint16_t*)acs->data,acs->addr);
#endif
#if 1
		//uint16_t *conv=(uint16_t*)acs->data;
		short_sleep(0.5);
		acs->dcnt = sizeof(uint16_t);
        acs->access = RF2072_WRITE;
		acs->addr = 0x09;
		*conv =((0x8224&0xFFF7) | 0x0008);
		libusb_control_transfer(devh,CTRL_OUT, USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0);
		printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",*(uint16_t*)acs->data,acs->addr);

		short_sleep(0.5);
#if 0
//read action after 2072 all write
	acs->dcnt = sizeof(uint16_t);
	acs->access = RF2072_READ;
    	  acs->addr = 0x00;
	libusb_control_transfer(devh,
						CTRL_OUT, USB_RQ,
						USB_HOST_MSG_TX_VAL,
						USB_HOST_MSG_IDX,
						acs, sizeof(*acs)+(acs->dcnt-1), 0);
	short_sleep(0.2);
	  	 while(0==libusb_control_transfer(devh,
					  	CTRL_IN, USB_RQ,
					  	USB_HOST_MSG_RX_VAL,
					  	USB_HOST_MSG_IDX,
					  	acs, sizeof(*acs)+(acs->dcnt-1), 0))
				short_sleep(0.0005);
	short_sleep(0.2);
	printf("reg 0x%x = 0x%04x\n",acs->addr,*(uint16_t*)acs->data);
#endif
#endif

#if 1
/*{
	puts("read from 0x00");
	char tmpb[8], *pt=tmpb;
	unsigned char ln= sizeof(tmpb);
	getline(&pt, &ln, stdin);
}*/
    	  acs->access = RF2072_WRITE;
    	  acs->dcnt = sizeof(uint16_t);
    	  acs->addr = 0x1D;
    	  *conv = 0x1001;
		libusb_control_transfer(devh,
						CTRL_OUT, USB_RQ,
						USB_HOST_MSG_TX_VAL,
						USB_HOST_MSG_IDX,
						acs, sizeof(*acs)+(acs->dcnt-1), 0);
	short_sleep(0.2);
		printf("setup device control = 0x%04x\n",*(uint16_t*)acs->data);
    	  acs->access = RF2072_READ;
    	  acs->addr = 0x1F;
		  libusb_control_transfer(devh,
						CTRL_OUT, USB_RQ,
						USB_HOST_MSG_TX_VAL,
						USB_HOST_MSG_IDX,
						acs, sizeof(*acs)+(acs->dcnt-1), 0);
	short_sleep(0.2);
	  	 while(0==libusb_control_transfer(devh,
					  	CTRL_IN, USB_RQ,
					  	USB_HOST_MSG_RX_VAL,
					  	USB_HOST_MSG_IDX,
					  	acs, sizeof(*acs)+(acs->dcnt-1), 0))
				short_sleep(0.0005);
	short_sleep(0.2);
		printf("lock state = 0x%x\n",(0x8000&*(uint16_t*)acs->data)?1:0);


#endif
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
	tag = 0;
#ifndef LIB
	dev_access *acs = (dev_access*)msg;
	acs->access = TS_VID_ACTIVE;
	acs->dcnt = 0; // no param
	acs->addr = 0x0; // by wire not addr
	libusb_control_transfer(devh,CTRL_OUT,USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs), 0);
	short_sleep(0.2);
	libusb_control_transfer(devh, CTRL_OUT, USB_RQ, 0x07, 0, NULL, 0, 0);
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
  uint32_t deinit_device(void)
#endif
{
	uint32_t error=Error_NO_ERROR;
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
#if defined(RADIO_SI4463)
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

	error=it9137_deinit();
	return error;

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
  void lgdst_vid_stats_rx() {
	  uint32_t error;
		error = it9137_get_signal_strength_dbm(0);
		if (error) {
			printf("error from it9137_get_signal_strength_dbm, 0x%08x\n", error);
			return ;
		}
		error = it9137_get_postviterbi_bit_error_rate(0);
		if (error) {
			printf("error from it9137_get_postviterbi_bit_error_rate, 0x%08x\n", error);
			return ;
		}
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
  		if (1/*cpld*/== system_upgrade) {
			//blksz = ???;
  			//...
	  	}
#if false  // not needed
  		else if (2/*fpga*/== system_upgrade) {
	  		fw_info[2] = 0x00295700;  // constant fpga download address
	  		printf("upgrade firmware address: 0x%08x\n", fw_info[2]);
		 libusb_control_transfer(devh, CTRL_OUT, USB_RQ,
			USB_FPGA_UPGRADE_VAL, USB_HOST_MSG_IDX,
			fw_info, FW_UPGRADE_HDR_LEN, 0);
			blksz = 15000;
			delay = 3.5;
  		}
#endif
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
  int lgdst_init_rx0(int argc, char **argv)
#else
  uint32_t  main(int argc,char **argv)
#endif
{
	Pid pid;
	pid.value=0x100;
	uint32_t error=Error_NO_ERROR;
#if (/*1*/0)
	init_device();
#else
	init_device(argc, argv);
   if (system_upgrade)
   	return 1;	// system upgrade or atmel reboot...
#endif
#if /*true*/false  // test atmel encapsulation of asic host
	// initialize video subsystem inside atmel
		 libusb_control_transfer(devh,
		 													CTRL_OUT,
		 													USB_RQ,
															USB_INIT_VID_SUBSYS,
															0x1,
															NULL,
															0,
															0);
	// startup video subsystem inside atmel
		 libusb_control_transfer(devh,
		 													CTRL_OUT,
		 													USB_RQ,
															USB_START_VID_SUBSYS,
															0x1,
															NULL,
															0,
															0);
#else
	init_rf2072();

#if 1
//	error= it9137_reset();
//	if(error)goto _exit;
//	error=it9137_reboot();
//	if(error)goto _exit;
//	error= it9137_control_power_saving(0,0);
//	if(error)goto _exit;
//	error= it9137_control_power_saving(0,1);
//	if(error)goto _exit;
//	error=it9137_reboot();
//	if(error)goto _exit;

	error=it9137_init();
	if(error)goto _exit;

	//error=it9137_get_firmwareversion();
	//if(error)goto _exit;
//     error= it9137_scan_channel(0,747000,832000, 6000);
  //  if(error)goto _exit;
	error=it9137_acquire_channel(0,/*809000*/750000,6000); //avoid conflict with wifi, liyenho
	if(error)goto _exit;
	//error=it9137_get_if_agc(0);
//	if(error)goto _exit;
//	error=it9137_get_rf_agc_gain(0);
//	if(error)goto _exit;
#if 0
	error=it9137_control_pid_filter(0,1);
	if(error)goto _exit;
	error=it9137_add_pid_filter(0,0,pid);
	if(error)goto _exit;
	error= it9137_control_power_saving(0,1);
	if(error)goto _exit;
#endif

	int i=0;
	while(i<1){
	error=it9137_check_tpslocked(0);
	if(error)goto _exit;
	error=it9137_check_mpeg2locked(0);
	if(error)goto _exit;
	//do not call this api interface ,some error inside
	//error=it9137_get_channel_modulation(0);
	//if(error)goto _exit;
	error=it9137_get_signal_quality(0);
	if(error)goto _exit;
	error=it9137_get_signal_quality_indication(0);
	if(error)goto _exit;
	error=it9137_get_signal_strength(0);
	if(error)goto _exit;
	error=it9137_get_signal_strength_indication(0);
	if(error)goto _exit;
	error=it9137_get_statistic(0);
	if(error)goto _exit;
	error=it9137_get_signal_strength_dbm(0);
	if(error)goto _exit;
	error=it9137_get_postviterbi_bit_error_rate(0);
	if(error)goto _exit;
	error=it9137_get_snr(0);
	if(error)goto _exit;
	//error= it9137_reset();
	//if(error)goto _exit;
	//error=it9137_reboot();
	//if(error)goto _exit;
	i++;
	}
#endif
#endif
ready_wait_for_mloop = true;
#ifndef LIB
	udpout_init(servIP);
	transfer_video();
	deinit_device();
#else
int32_t msg[80]; // access buffer
	dev_access *acs = (dev_access*)msg;
	acs->access = TS_VID_ACTIVE;
	acs->dcnt = 0; // no param
	acs->addr = 0x0; // by wire not addr
	libusb_control_transfer(devh,CTRL_OUT,USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs), 0);
	short_sleep(0.2);
	libusb_control_transfer(devh, CTRL_OUT, USB_RQ, 0x07, 0, NULL, 0, 0);
#endif
_exit:
	return error;

}
#ifdef LIB
  int lgdst_init_rx(int argc, char **argv) {
  	return lgdst_init_rx0(argc, argv);
  }
#endif




