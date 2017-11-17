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
#include "MavLink.h"

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

#if USE_MAVLINK
unsigned char radio_tpacket[RADIO_USR_TX_LEN]__attribute__((aligned(8))); //still used in video chan select
volatile uint8_t pending[255+2+MAVLINK_HDR_LEN]; // for efficiency
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

bool stream_on = false ; // ctrl xfer access speed flag

static FILE *file = NULL;
unsigned char audbuf[FRAME_SIZE_A*FRAME_BUFFS]__attribute__((aligned(8)));
unsigned char vidbuf[FRAME_SIZE_V2]__attribute__((aligned(8)));
#ifdef VIDEO_DUAL_BUFFER
	ctx_dual_stream dual_str_ctx = {0};
#endif
const uint32_t vch_tbl[] = {
	2392, 2406, 2413, 2420,
	2427, 2434, 2441, 2448,
	2455, 2462, 2469
};

static bool detached = false;
struct libusb_device_handle *devh = NULL;

volatile int do_exit = 0;	       // main loop breaker
static pthread_t poll_thread= 0,
				 lgdst_thread = 0,
				 ctrl_thr_recv = 0,
				 ctrl_thr_send = 0;
extern pthread_mutex_t mux_thr; // (user level)
/*static*/ pthread_mutex_t mux;
int tag=0;

int chsel_2072 = 0;
const int pidvid = 0x100;
const int pidpcr = 0x100;
static struct timeval time_orig;    // as starting time ref
uint32_t n, size, fw_info[3+1];

const unsigned char nullts_spec[188]={
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
	0xff,0xff,0xff,0xff,0xf8,0xf9,0xfa,0xfb,0xfc,0xfd,0xfe,0xff
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

void mon_print(void)
{
  static int prtcnt=0;
  static int ctrlcnt_uplink_prev=0;
  static int ctrlcnt_vid188_prev=0;
  static int err_loop = 0/*3*/;
  int ctrlcnt_uplink_inc;
  int ctrlcnt_vid188_inc;
  uint32_t err0, err1;
  {
    prtcnt++; if(prtcnt>99999) prtcnt=0;
    if (0 == err_loop) {
      err0 = it9137_get_signal_strength_dbm(0,&dbg_rssi);
      err1 = it9137_get_snr(0, &dbg_snr);
      if (err0 || err1)
       { err_loop = 3; } // NACK was asserted, wait a while
    }
    else { err_loop -= 1;
    	printf("mon_print: wait err_loop =%d\n",err_loop);
 	}
    if(ctrlcnt_uplink_prev <= dbg_ctrlcnt_uplink)
      ctrlcnt_uplink_inc = dbg_ctrlcnt_uplink - ctrlcnt_uplink_prev;
    else
      ctrlcnt_uplink_inc = 256+dbg_ctrlcnt_uplink-ctrlcnt_uplink_prev;
    if(ctrlcnt_vid188_prev <= dbg_ctrlcnt_vid188)
      ctrlcnt_vid188_inc = dbg_ctrlcnt_vid188 - ctrlcnt_vid188_prev;
    else
      ctrlcnt_vid188_inc = 256+dbg_ctrlcnt_vid188-ctrlcnt_vid188_prev;
    ctrlcnt_uplink_prev = dbg_ctrlcnt_uplink;
    ctrlcnt_vid188_prev = dbg_ctrlcnt_vid188;
    printf("DBRX(%5d) rssi:%3d ts:%5d tsei:%3d tsvid:%5d frm:%4d cc:%2d ctrlup:%3d ctrldwn:%3d lock:%1d ant:%1d\n",
      prtcnt,dbg_rssi,dbg_tscnt, dbg_tseicnt, dbg_tsvidcnt,dbg_frmcnt,dbg_ccerror,ctrlcnt_uplink_inc,
      ctrlcnt_vid188_inc,dbg_tdmalock,dbg_antpos);
    dbg_frmcnt=0;
    dbg_tseicnt=0;
    dbg_tscnt=0;
    dbg_tsvidcnt=0;
    dbg_ccerror=0;
    dbg_tdmalock=9;
    dbg_antpos=9;

  }
}

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


FILE  *ctrl_file_handle = 0;
char time_buffer[26];
time_t timer;
struct tm* tm_info;
uint8_t socket_send_buffer[MAVLINK_USB_LEN] = {0};
int socket_send_len =0;

#ifdef RADIO_SI4463
void *ctrl_poll_recv(void *arg)
{
	int r, i;
	bool ctrl_sckt_ok = *(bool*)arg;
	unsigned char ctrl_recv_fifolvl_data = 0;
	unsigned char validdataflag = 0;
	long pv_wrbyte = 0;
	while (1 == do_exit) {
		if (ctrl_sckt_ok) {
			pthread_mutex_lock(&mux);
#if USE_MAVLINK
			libusb_control_transfer(devh, CTRL_IN, USB_RQ, RADIO_COMM_VAL, RADIO_MAVLEN_IN_IDX, &pv_wrbyte, 2, 0);
			libusb_control_transfer(devh, CTRL_IN, USB_RQ, RADIO_COMM_VAL, RADIO_DATA_RX_IDX, radio_rpacket, pv_wrbyte, 0);
#else
			libusb_control_transfer(devh, CTRL_IN, USB_RQ, RADIO_COMM_VAL, RADIO_DATA_RX_IDX, radio_rpacket, sizeof(radio_rpacket), 0);
#endif
			pthread_mutex_unlock(&mux);

			bool filler_flag = ((radio_rpacket[0]==0xee) && (radio_rpacket[1]==0xee)) ||
													((radio_rpacket[0]==0x00) && (radio_rpacket[1]==0x00));
			validdataflag = !filler_flag && ((radio_rpacket[0]!=0xe5) && (radio_rpacket[1]!=0xe5));

			if (validdataflag) {
				//not filler data
				if (ctrl_file_handle == 0) {
					ctrl_file_handle = fopen("ControlLog.txt", "w");
					printf("Opened ControlLog.txt\n");
				}

				if (ctrl_file_handle != 0) {
					//get timestamp
					time(&timer);
					tm_info = localtime(&timer);
					strftime(time_buffer, 26, "%Y-%m-%d %H:%M:%S", tm_info);
					//write time followed by comma and tab to file
					fprintf(ctrl_file_handle, "%s,\t", time_buffer);

					for (int i = 0; i < sizeof(radio_rpacket); i++) {
						fprintf(ctrl_file_handle, "%02X,\t", radio_rpacket[i]);
					}
					fprintf(ctrl_file_handle, "\n");
				}
			}


#if USE_MAVLINK
			socket_send_len = 0;
			if (validdataflag) {
				memcpy(socket_send_buffer, radio_rpacket, pv_wrbyte); // straight memcpy entire pkt
				printf("Radio Rx: ");
				PrintMavLink(socket_send_buffer);
				Set_Mavlink_Checksum(socket_send_buffer) ; //write chksm into right place
				uint16_t expec_chksum = Compute_Mavlink_Checksum((MavLinkPacket*)socket_send_buffer);
				bool checksum_ok = Check_Mavlink_Checksum((MavLinkPacket*)socket_send_buffer);
				printf(" Checksum %s", checksum_ok ? "OK" : "Error");
				if (!checksum_ok) {
					printf("\t Expected %x", expec_chksum);
				}
				printf("\n\n");

				//prepare for socket send,
				socket_send_len = ((MavLinkPacket*)socket_send_buffer)->length+MAVLINK_HDR_LEN+MAVLINK_CHKSUM_LEN;
			}

#else
			if (validdataflag)
			{
				//prepare for socket send
				socket_send_len = RADIO_USR_RX_LEN;
				memcpy(socket_send_buffer, radio_rpacket, socket_send_len);


				printf("radio link Rx: ");
				for (i = 0; i < sizeof(radio_rpacket); i++)
					printf("%02x ", radio_rpacket[i]);
				printf("\n");
			}
			if (1) //all packets are forced restamped in test code at vender_specific()
			{
				dbg_tdmalock = (int)(radio_rpacket[23] & 0x01);
				dbg_ctrlcnt_uplink = (int)(radio_rpacket[22]);
				dbg_ctrlcnt_vid188 = (int)(radio_rpacket[24]);
				dbg_antpos = (int)(radio_rpacket[23] & 0xc0);
			}

			{	// user packet dataloss ---------------------------------
				static unsigned char val_exp;
				unsigned char        val_cur;
				static int ilcnt = 0, becnt = 0;
				unsigned char byteerror = 0;
				if (validdataflag == 0x01) {
					for (i = 6; i < RADIO_USR_RX_LEN; i++)
						if (radio_rpacket[i] != 0xb5) {
							byteerror++;
							becnt++;
						}
					if (byteerror > 0) printf("User packet corruption = %d. ++++++++++\n", becnt);
					val_cur = radio_rpacket[5];
					val_exp = (unsigned char)(val_exp + 1);
					if (val_exp != val_cur)
					{
						ilcnt++;
						printf("User packet dataloss = %d. !!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", ilcnt);
					}
					val_exp = val_cur;
				}//end valid data
			}// user packet dataloss check

#endif //end USE_MAVLINK


			if (validdataflag) //got valid payload
			{
				sendto(ctrlrcv_socket, socket_send_buffer, socket_send_len, 0, (struct sockaddr *)&ctrlrcv, sizeof(ctrlrcv));
			}

			#if !(USE_MAVLINK)
			usleep(CTRL_RECV_POLLPERIOD);
			#else  // make data rate as close to maximum as possible
			usleep(CTRL_RECV_POLLPERIOD*(pv_wrbyte+RADIO_USR_RX_LEN/2)/RADIO_USR_RX_LEN);
			#endif
		} //ctrl_sckt_ok
	}//(1==do_exit)
}
#ifdef LIB
void lgdst_ctl_rec_rx(unsigned char *rpacket)
 {
	if (rpacket) {
		#if !(USE_MAVLINK)
		// rpacket buffer must be at least the same length:RADIO_USR_RX_LEN+RADIO_INFO_LEN
		// user should always check for buffer information (RADIO_INFO_LEN) in regards to access status
		memcpy(rpacket, radio_rpacket, sizeof(radio_rpacket));
		#endif
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
		#if !(USE_MAVLINK)
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
			if(DBG_USB_CTRL_TX) { // turn on/off ctrl tx debug print
				printf("radio link Tx: ");
				for(i=0;i<sizeof(radio_tpacket);i++)
					printf("%02x ",radio_tpacket[i]);
				printf("\n");
			}
		}
		#endif
		#if USE_MAVLINK
		uint8_t data[255] ;
		for (int j=0;j<255;j++){
			data[j] = j;
		}

		uint32_t mav_size = rand() &255;
		Build_Mavlink_Data_Packet(pending, mav_size, data);

		if (DBG_USB_CTRL_TX) { // turn on/off ctrl tx debug print
			printf("Radio Tx: ");
			PrintMavLink(pending);
			printf("\n");
		}
		#endif

		// ------------------------------------------------

		pthread_mutex_lock(&mux);
		#if USE_MAVLINK
			mav_size = MAVLINK_HDR_LEN +((MavLinkPacket*)pending)->length+ MAVLINK_CHKSUM_LEN; // accommodate mavlink var msg len,
			libusb_control_transfer(devh,CTRL_OUT, USB_RQ,RADIO_COMM_VAL,RADIO_MAVLEN_OUT_IDX,&mav_size, 2, 0);
			libusb_control_transfer(devh,CTRL_OUT, USB_RQ,RADIO_COMM_VAL,RADIO_DATA_TX_IDX,pending, mav_size, 0);
		#else
			libusb_control_transfer(devh,CTRL_OUT, USB_RQ,RADIO_COMM_VAL,RADIO_DATA_TX_IDX,radio_tpacket, sizeof(radio_tpacket), 0);
		#endif
		pthread_mutex_unlock(&mux);

		#if !(USE_MAVLINK)
		usleep(CTRL_SEND_POLLPERIOD);
		#else  // make data rate as close to maximum as possible
		usleep(CTRL_SEND_POLLPERIOD*(mav_size+RADIO_USR_TX_LEN/2)/RADIO_USR_TX_LEN);
		#endif
	}
}
#if defined(LIB)  // accommodate mavlink opt
void lgdst_ctl_snd_rx(unsigned char *tpacket)
{
	if (tpacket) {
		#if !(USE_MAVLINK)
		// tpacket buffer must be the same length:RADIO_USR_TX_LEN
		memcpy(radio_tpacket, tpacket, sizeof(radio_tpacket));
		#endif
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
		if(DBG_USB_RX) {
			static int loop = 200;
			static int delay_init = 8; //delay until pair-id done, or, will see i2c errors
			static FILE *dump_reg_rx = NULL;

			if (!dump_reg_rx) {
				dump_reg_rx = fopen("./ofdm-registers-dump.txt", "wt");
				printf("INFO: open ofdm-registers-dump.txt for ITE status logging.\n");
			}
			if ( !loop-- ) {
			   if(delay_init>0) delay_init--;
			   if(delay_init==0) {
				  mon_print();
				  it9137_read_ofdm_registers(dump_reg_rx);
				}
				loop = 200;	// poll every 2 sec
			}
		}
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
				printf("CMD0: wValue = %d, wIndex = %d\n", shmLgdst_proc->tag.wValue,shmLgdst_proc->tag.wIndex);
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
						shmLgdst_proc->tag.wDir,shmLgdst_proc->tag.wValue,shmLgdst_proc->tag.wIndex,shmLgdst_proc->len,*(int*)acs->data);
				pthread_mutex_lock(&mux);
				libusb_control_transfer(devh,shmLgdst_proc->tag.wDir,USB_RQ,shmLgdst_proc->tag.wValue,shmLgdst_proc->tag.wIndex,(unsigned char*)acs->data,shmLgdst_proc->len, 0);
				pthread_mutex_unlock(&mux);
				if (VIDEO_SETVCH_VAL == shmLgdst_proc->tag.wValue) {
					uint8_t vch = *(uint8_t*)acs->data;
					if (sizeof(vch_tbl)>vch) {
						it9137_acquire_channel(0,vch_tbl[vch]*1000-LO_Frequency,6000);
						short_sleep(0.2);
						int32_t msg[80]; // access buffer
							dev_access *acs = (dev_access*)msg;
							acs->access = TS_VID_ACTIVE;
							acs->dcnt = 0; // no param
							acs->addr = 0x0; // by wire not addr
							pthread_mutex_lock(&mux);
							libusb_control_transfer(devh,CTRL_OUT,USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs), 0);
							pthread_mutex_unlock(&mux);
					}
				}
				if (CTRL_IN==shmLgdst_proc->tag.wDir)
					memcpy(&shmLgdst_proc->access.hdr.data,&lclMem.hdr.data, shmLgdst_proc->len);
				break;
			case ACS:
				printf("ACS: direction = %d, processor = %d\n",(int)shmLgdst_proc->tag.wDir,(int)shmLgdst_proc->tag.wIndex);
				if (CTRL_IN==shmLgdst_proc->tag.wDir) {
					pthread_mutex_lock(&mux_thr);
					Demodulator_readRegisters (
							NULL, // not used...
					      0,
					      shmLgdst_proc->tag.wIndex, // ofdm/link proc
					      (uint32_t) acs->addr,
					      acs->dcnt,
					      acs->data) ;
					pthread_mutex_unlock(&mux_thr);
				}
				else if (CTRL_OUT==shmLgdst_proc->tag.wDir) {
					pthread_mutex_lock(&mux_thr);
					Demodulator_writeRegisters (
							NULL, // not used...
					      0,
					      shmLgdst_proc->tag.wIndex, // ofdm/link proc
					      (uint32_t) acs->addr,
					      acs->dcnt,
					      acs->data) ;
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
		pthread_mutex_lock(&mux);
		libusb_control_transfer(devh,CTRL_OUT, USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0);
		pthread_mutex_unlock(&mux);
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
    get_time(&time_prev); // clock_gettime() use timespec not timeval, all calculations went wrong!
    get_time(&time_curr); // clock_gettime() use timespec not timeval, all calculations went wrong!
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
    get_time(&time_prev); // clock_gettime() use timespec not timeval, all calculations went wrong!
  }
  */

  for(i=0;i<len/188;i++)
  {
    tspkt = buff+(i*188);
    if((tspkt[1]&0x80)==0x80)
    {
      frmbadflg=1;
      dbg_tseicnt += 1; //added by  liyenho
      memcpy(tspkt, nullts_spec, sizeof(nullts_spec));
    }
	 dbg_tscnt += 1;  // added by liyenho
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
    if((pcrhit==1)|| (pidhit==1)||(pusihit==1))
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
    memcpy(tspkt, nullts_spec, sizeof(nullts_spec));
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
			pthread_mutex_lock(&mux);
			libusb_control_transfer(devh, CTRL_OUT, USB_RQ,	USB_CPLD_UPGRADE_VAL, USB_HOST_MSG_IDX,	(unsigned char*)fw_info, FW_UPGRADE_HDR_LEN, 0);
			pthread_mutex_unlock(&mux);
			blksz = 7200; // similar size to atmel bin image, it should be fine
			delay = 0.84;
		}
		else if (3/*atmel*/== mode) {
			pthread_mutex_lock(&mux);
			libusb_control_transfer(devh, CTRL_OUT, USB_RQ,	USB_FWM_UPDATE_VAL, USB_HOST_MSG_IDX,(unsigned char*)fw_info, ATMEL_UPGRADE_HDR_LEN, 0);
			pthread_mutex_unlock(&mux);
			blksz = 7200;
			delay = 0.84;
		}
		else if(4/*atmel boot*/==mode){
			pthread_mutex_lock(&mux);
			libusb_control_transfer(devh, CTRL_OUT, USB_RQ,USB_FWM_BOOTUP_VAL, USB_HOST_MSG_IDX,NULL, 0, 0);
			pthread_mutex_unlock(&mux);
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
			pthread_mutex_lock(&mux);
			libusb_control_transfer(devh,CTRL_IN, USB_RQ,USB_STREAM_ON_VAL,USB_QUERY_IDX,(unsigned char*)&main_loop_on, sizeof(main_loop_on), 0);
			pthread_mutex_unlock(&mux);
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
		pthread_mutex_lock(&mux);
		libusb_control_transfer(devh,CTRL_OUT, USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0);
		pthread_mutex_unlock(&mux);
		short_sleep(0.2); 	// validate echo after 0.1 sec
		printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",*(uint16_t*)acs->data,acs->addr);

		acs->dcnt = sizeof(*conv);

        acs->access = RF2072_WRITE;
		acs->addr = 0x0F;
		*conv =P2_FREQ1;
		pthread_mutex_lock(&mux);
		libusb_control_transfer(devh,CTRL_OUT, USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0);
		pthread_mutex_unlock(&mux);
		short_sleep(0.2); 	// validate echo after 0.1 sec
		printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",*(uint16_t*)acs->data,acs->addr);

		acs->dcnt = sizeof(*conv);

        acs->access = RF2072_WRITE;
		acs->addr = 0x10;
		*conv =P2_FREQ2;
		pthread_mutex_lock(&mux);
		libusb_control_transfer(devh,CTRL_OUT, USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0);
		pthread_mutex_unlock(&mux);
		short_sleep(0.2); 	// validate echo after 0.1 sec
		printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",*(uint16_t*)acs->data,acs->addr);

		acs->dcnt = sizeof(*conv);

        acs->access = RF2072_WRITE;
		acs->addr = 0x11;
		*conv =P2_FREQ3;
		pthread_mutex_lock(&mux);
		libusb_control_transfer(devh,CTRL_OUT, USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0);
		pthread_mutex_unlock(&mux);
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
	pthread_mutex_lock(&mux);
	libusb_control_transfer(devh,CTRL_OUT,USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs), 0);
	pthread_mutex_unlock(&mux);
	short_sleep(0.2);
													// always 0, for rffc 2072 main startup, liyenho
	rffe_write_regs(pregs=GET_ARRAY(0), sz=ARRAY_SIZE(0));
	short_sleep(0.2);
     set_frequency_rf2072(LO_Frequency);
	 printf("rx rffe is running...\n");
#if 0
	 //if not pull up rxena need write 0x15 at here
	acs->dcnt = sizeof(uint16_t);
        acs->access = RF2072_WRITE;
		acs->addr = 0x15;
		*conv =(0x8000 | 0x0008);
		pthread_mutex_lock(&mux);
		libusb_control_transfer(devh,CTRL_OUT, USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0);
		pthread_mutex_unlock(&mux);
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
		pthread_mutex_lock(&mux);
		libusb_control_transfer(devh,CTRL_OUT, USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0);
		pthread_mutex_unlock(&mux);
		printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",*(uint16_t*)acs->data,acs->addr);

		short_sleep(0.5);
#endif
#if 1
    	  acs->access = RF2072_WRITE;
    	  acs->dcnt = sizeof(uint16_t);
    	  acs->addr = 0x1D;
    	  *conv = 0x1001;
			pthread_mutex_lock(&mux);
		libusb_control_transfer(devh,
						CTRL_OUT, USB_RQ,
						USB_HOST_MSG_TX_VAL,
						USB_HOST_MSG_IDX,
						acs, sizeof(*acs)+(acs->dcnt-1), 0);
			pthread_mutex_unlock(&mux);
	short_sleep(0.2);
		printf("setup device control = 0x%04x\n",*(uint16_t*)acs->data);
    	  acs->access = RF2072_READ;
    	  acs->addr = 0x1F;
			pthread_mutex_lock(&mux);
		  libusb_control_transfer(devh,
						CTRL_OUT, USB_RQ,
						USB_HOST_MSG_TX_VAL,
						USB_HOST_MSG_IDX,
						acs, sizeof(*acs)+(acs->dcnt-1), 0);
			pthread_mutex_unlock(&mux);
	short_sleep(0.2);
		while(1) {
			pthread_mutex_lock(&mux);
			if (libusb_control_transfer(devh,CTRL_IN, USB_RQ,USB_HOST_MSG_RX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0)){
				pthread_mutex_unlock(&mux);
				break;
			}
			pthread_mutex_unlock(&mux);
			short_sleep(0.0005);
		}
	short_sleep(0.2);
		printf("lock state = 0x%x\n",(0x8000&*(uint16_t*)acs->data)?1:0);
#endif
	return 0;

}
#ifdef LIB
  void lgdst_ts_rx(uint8_t *tsbuf0, uint32_t *blks)
#else
  int transfer_video(void)
#endif
{
	int i,r, msg[80]; // access buffer
	uint64_t llw;
	int ii;
	uint32_t frames_len;
	int frag, sentsize;
	stream_on = true; // slow down ctrl xfer access at real time,
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
#ifndef LIB
		if (ITERS==tag)
			r = FILE_LEN - tag*FRAME_SIZE_A;
         else
			r = FRAME_SIZE_A;
	#define SEND_VIA_UDP(buff) \
	  unsigned char *pb = buff; \
		for (frag=0; frag<5; frag++) { \
			sentsize=sendto(udpout_socket, pb, r/5,0,(struct sockaddr *)&udpout,udpout_len); \
	      if (sentsize < 0) printf("send pack ERorr\n"); \
	      		pb += r/5; \
      		}
#endif
#ifndef VIDEO_DUAL_BUFFER
	 tsptsadj(audbuf, FRAME_SIZE_A, pidvid, pidpcr);
  #ifdef LIB
	 *blks = 1;
  #endif
#ifndef LIB
		SEND_VIA_UDP(audbuf)
frm_inc:
		tag += 1;
		if (ITERS==tag)
			tag = 0; // make it endless
#else  // in LIB mode
		memcpy(tsbuf0, audbuf, FRAME_SIZE_A);
#endif
#else  // dual stream scheme
	unsigned char *tspkt = audbuf;
	unsigned  pid, n, nn;
  #ifdef LIB
	*blks = 0;
  #endif
 #if false
	unsigned  char *cnt_seq_hi;  // use 6 bits
	unsigned  char *cnt_seq_lo;  //use 8 bits
	unsigned  ts_type;
	static unsigned char cc_pmt=0;
	static unsigned char  cc_pat=0;
	static unsigned char  cc_vid = 0;
	for (n=0; n<FRAME_SIZE_A/188; tspkt+=188, n++) {
		ts_type =( tspkt[1] & 0x1f) >>2;
		if(ts_type == 0) {pid = 0; tspkt[3]=(tspkt[3]&0xf0)| (cc_pat&0x0f); cc_pat++; }
		else if(ts_type == 1){ pid = 0xfff; tspkt[3]=(tspkt[3]&0xf0)| (cc_pmt&0x0f); cc_pmt++; }
		else if(ts_type ==2) { pid = 0x100; tspkt[3]=(tspkt[3]&0xf0)| (cc_vid&0x0f); cc_vid++; }
		else {pid=0x1fff; tspkt[3]= tspkt[3]&0xf0; }

		*cnt_seq_hi =(( tspkt[1]&0x3)<<4) |( tspkt[2]>>4);
		*cnt_seq_lo =(tspkt[2]<<4) | (tspkt[3]&0x0f);
//#define YENDO
#ifdef YENDO
		tspkt[1]=( tspkt[1]&0xe0)| (pid>>8);
		tspkt[2]=pid&0xff;
#endif
	}
 #endif
  #define FILL_SEARCH_BUFF \
					memcpy(dual_str_ctx.vid_sch_ptr, \
									tspkt, 188); \
					dual_str_ctx.vid_sch_ptr += 188; \
					if (dual_str_ctx.vid_sch_ptr >= \
						dual_str_ctx.vid_sch_buff_e) { \
						dual_str_ctx.vid_sch_ptr =dual_str_ctx.vid_sch_buff; \
						dual_str_ctx.sch_buff_full = true; \
					}
#ifdef LIB
  #define INSERT_A_PKT(ptr_pkt) \
						memcpy(dual_str_ctx.ptw_ts, ptr_pkt, 188); \
						dual_str_ctx.ptw_ts += 188; \
						if (dual_str_ctx.ptw_ts>= dual_str_ctx.ptw_ts_e) { \
							dual_str_ctx.ptw_ts = dual_str_ctx.ptw_ts_b; \
							if (1/*dual_str_ctx.sch_buff_full*/) { \
								tsptsadj(dual_str_ctx.ptw_ts_b, \
												FRAME_SIZE_A, pidvid, pidpcr); \
								memcpy(tsbuf0 + *blks *FRAME_SIZE_A, \
												dual_str_ctx.ptw_ts_b, \
												FRAME_SIZE_A); \
								*blks += 1; \
							} \
						}
#else  // not in lib mode
  #define INSERT_A_PKT(ptr_pkt) \
						memcpy(dual_str_ctx.ptw_ts, ptr_pkt, 188); \
						dual_str_ctx.ptw_ts += 188; \
						if (dual_str_ctx.ptw_ts>= dual_str_ctx.ptw_ts_e) { \
							dual_str_ctx.ptw_ts = dual_str_ctx.ptw_ts_b; \
							if (1/*dual_str_ctx.sch_buff_full*/) { \
								tsptsadj(dual_str_ctx.ptw_ts_b, \
												FRAME_SIZE_A, pidvid, pidpcr); \
								SEND_VIA_UDP(dual_str_ctx.ptw_ts_b) \
							} \
						}
#endif
 	static int ext_seq_cnt0 =-1;
	for (n=0; n<FRAME_SIZE_A/188; tspkt+=188, n++) {
	  if((tspkt[1]&0x80)!=0x80) {
		unsigned short ofst = 0, ext_seq_cnt, seq_del;
		pid =( ((unsigned)(tspkt[1]&0x1f))<<8) + tspkt[2];
		if (pidvid == pid) {
			if ((0x30== (0x30&tspkt[3])) &&
				(1+1+2 <= tspkt[4]) && (0x2== (0x2&tspkt[5]))) {
				ofst += (0x10 & *(tspkt+5))?6:0;
	 			ofst += (0x08 & *(tspkt+5))?6:0;
	 			ofst += (0x04 & *(tspkt+5))?1:0;
	 			if (2 == *(tspkt+6+ofst)) {// 2 byte ext seq cnt
					*((uint8_t*)(&ext_seq_cnt)+1) = *(tspkt+7+ofst);
					*((uint8_t*)(&ext_seq_cnt)+0) = *(tspkt+8+ofst);
					if (0>(int)ext_seq_cnt0)
						ext_seq_cnt0 = ext_seq_cnt;
					else {
						if (!dual_str_ctx.buf_flag) {
							// determine the dual stream sequence
							if (ext_seq_cnt0<ext_seq_cnt) {
								seq_del = ext_seq_cnt- ext_seq_cnt0;
								if (A_QUARTER_LESS/188<=seq_del) {
									dual_str_ctx.buf_flag = 1; // next one is later
									dual_str_ctx.ext_seq_cnt_n =
										(ext_seq_cnt0 + 1) & 0xffff;
								}
								else {
									if (1==seq_del) {
										// within 1 sec startup phase, can't determine sequence yet
										FILL_SEARCH_BUFF
									}
									else { // early stream wrapped
										dual_str_ctx.buf_flag = -1; // current one is later
										dual_str_ctx.ext_seq_cnt_n =
											(ext_seq_cnt) & 0xffff;
									}
								}
							}
							else {
								seq_del = ext_seq_cnt0- ext_seq_cnt;
								if (A_QUARTER_LESS/188<=seq_del) {
									dual_str_ctx.buf_flag = -1; // current one is later
									dual_str_ctx.ext_seq_cnt_n =
										(ext_seq_cnt) & 0xffff;
								}
								else {  // early stream wrapped
									dual_str_ctx.buf_flag = 1; // next one is later
									dual_str_ctx.ext_seq_cnt_n =
										(ext_seq_cnt0 + 1) & 0xffff;
								}
							}
						}
						if (0<dual_str_ctx.buf_flag) {
							if ((1==abs(ext_seq_cnt- ext_seq_cnt0)) &&
								(dual_str_ctx.ext_seq_cnt_n==ext_seq_cnt)) {
								goto reverse_late;
							}
reverse_early:
							FILL_SEARCH_BUFF
							if (dual_str_ctx.update_n)
								dual_str_ctx.ext_seq_cnt_n =
											(ext_seq_cnt0 + 1) & 0xffff;
							dual_str_ctx.buf_flag = -1;
						}
						else if (0>dual_str_ctx.buf_flag) {
							if ((1==abs(ext_seq_cnt- ext_seq_cnt0)) &&
								(A_QUARTER_LESS/188<abs(ext_seq_cnt-dual_str_ctx.ext_seq_cnt_n))) {
								dual_str_ctx.update_n = false;
								goto reverse_early;
							}
reverse_late:
							dual_str_ctx.update_n = true;
							dual_str_ctx.buf_flag = 1;
							// check upon ext seq cnt for pkt lost
							if (dual_str_ctx.ext_seq_cnt_n!=ext_seq_cnt) {
								uint8_t *pkt=(dual_str_ctx.sch_buff_full)? \
									dual_str_ctx.vid_sch_ptr+ 188 :dual_str_ctx.vid_sch_buff ;
								unsigned short cnt_next = dual_str_ctx.ext_seq_cnt_n,
																	exp_bn = cnt_next+SEQ_SCH_TOL ;
	printf("missed expected ext seq cnt, exp = %d, real = %d (%d, %d)\n",
				dual_str_ctx.ext_seq_cnt_n, ext_seq_cnt, ext_seq_cnt0, ext_seq_cnt)  ;
								if (A_QUARTER_LESS/188<abs(ext_seq_cnt-cnt_next)) {
									dual_str_ctx.buf_flag *= -1;
									exp_bn = cnt_next;
								}
								do {
									bool failed = true;
									while(1) {
										if (dual_str_ctx.vid_sch_buff_e ==pkt)
											pkt = dual_str_ctx.vid_sch_buff;
										unsigned short ofst1 = 0, ext_seq_cnt1;
										ofst1 += (0x10 & *(pkt+5))?6:0;
							 			ofst1 += (0x08 & *(pkt+5))?6:0;
							 			ofst1 += (0x04 & *(pkt+5))?1:0;
										*((uint8_t*)(&ext_seq_cnt1)+1) = *(pkt+7+ofst1);
										*((uint8_t*)(&ext_seq_cnt1)+0) = *(pkt+8+ofst1);
										if (cnt_next == ext_seq_cnt1) {
											INSERT_A_PKT(pkt)
											failed = false;
											break;
										}
										if (dual_str_ctx.vid_sch_ptr == pkt)
											break ;
										pkt += 188;
									}
									cnt_next = (cnt_next+1) ;
								} while (cnt_next != (exp_bn+1) &&
												cnt_next != ext_seq_cnt);
							}
							INSERT_A_PKT(tspkt)
							if (0<dual_str_ctx.buf_flag)
								dual_str_ctx.ext_seq_cnt_n =
											(ext_seq_cnt + 1) & 0xffff;
							else  // sequence reversed
								dual_str_ctx.ext_seq_cnt_n =
											(ext_seq_cnt0 + 1) & 0xffff;
						}
						ext_seq_cnt0 = ext_seq_cnt; // keep tracking on ext seq cnt
					}
				}
			}
		}
		else {
			INSERT_A_PKT(tspkt)
			if (0 !=dual_str_ctx.buf_flag) {
				// in case ctrl radio pkt leaked toward video stream, ctrl radio pid : 0x1000
				if (0x1000 ==pid)
					goto skip;
				// check for null pkt to tell whether if they're added in transmission or bit stream
				if (0x1fff ==pid) {
					for (nn=188-1; nn>180-1; nn--)
						if (nullts_spec[nn] != tspkt[nn])
							break;
					if (180-1 != nn)
						goto skip;
				}
#define _REALTIME_ // turn off if tested with file io
#ifdef _REALTIME_
	goto skip;  // not in file io, sequence order is speculated
#endif
				if (!dual_str_ctx.update_n)
					goto skip;  //in special sequence...
				if (0> dual_str_ctx.buf_flag) {
					ext_seq_cnt0 =
						(dual_str_ctx.ext_seq_cnt_n-1) & 0xffff;
				}
				else {
					; // there shall be no effect at all,
				}
				dual_str_ctx.buf_flag *= -1;
			}
skip: 	;
		}}
	}
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
  void lgdst_vid_stats_rx(long *sigdbm, double *ptvitber) {
	  uint32_t error;
		error = it9137_get_signal_strength_dbm(0, sigdbm);
		if (error) {
			printf("error from it9137_get_signal_strength_dbm, 0x%08x\n", error);
			return ;
		}
		error = it9137_get_postviterbi_bit_error_rate(0, ptvitber);
		if (error) {
			printf("error from it9137_get_postviterbi_bit_error_rate, 0x%08x\n", error);
			return ;
		}
  }
#endif

#define SCHEME_RETRY(it9517_func_call, err_val) \
	err_cnt = 0; \
	do { \
		error=it9517_func_call ; \
		if (5<=err_cnt++) \
			{error=err_val; goto _exit;} \
	} while(error );

#ifdef LIB
  int lgdst_init_vid_rx()
#else
  uint32_t  main(int argc,char **argv)
#endif
{
	Pid pid;
	pid.value=0x100;
	uint32_t error=Error_NO_ERROR, err_cnt;
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
	init_rf2072();

	SCHEME_RETRY(it9137_init(), 1)
#if DYNAMIC_VIDEO_CHANNEL_SCAN
	if (it9137_video_channel_scan(False))
		goto _exit;  // channel scan failed
#else // fixed frequency setting
  	bool istpslocked = true;
	SCHEME_RETRY(it9137_acquire_channel(0,vch_tbl[chsel_2072]*1000-LO_Frequency,6000), 2)

	int i=0;
	while(i<1){
	SCHEME_RETRY(it9137_check_tpslocked(0, &istpslocked), 3)
	SCHEME_RETRY(it9137_check_mpeg2locked(0), 4)
	SCHEME_RETRY(it9137_get_signal_quality(0), 5)
	SCHEME_RETRY(it9137_get_signal_quality_indication(0), 6)
	SCHEME_RETRY(it9137_get_signal_strength(0), 7)
	SCHEME_RETRY(it9137_get_signal_strength_indication(0), 8)
	SCHEME_RETRY(it9137_get_statistic(0), 9)
	SCHEME_RETRY(it9137_get_signal_strength_dbm(0, NULL), 10)
	SCHEME_RETRY(it9137_get_postviterbi_bit_error_rate(0, NULL), 11)
	SCHEME_RETRY(it9137_get_snr(0, NULL), 12)
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
	pthread_mutex_lock(&mux);
	libusb_control_transfer(devh,CTRL_OUT,USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs), 0);
	pthread_mutex_unlock(&mux);
	short_sleep(0.2);
	pthread_mutex_lock(&mux);
	libusb_control_transfer(devh, CTRL_OUT, USB_RQ, 0x07, 0, NULL, 0, 0);
	pthread_mutex_unlock(&mux);
 #ifdef VIDEO_DUAL_BUFFER
 	// init video dual stream context
	dual_str_ctx.vid_sch_buff = vidbuf;
	dual_str_ctx.vid_sch_buff_e =vidbuf+ ONE_SEC_WORTHY;
	dual_str_ctx.vid_sch_ptr = vidbuf;
	dual_str_ctx.ptw_ts_b = audbuf+ (FRAME_BUFFS-1)*FRAME_SIZE_A;
	dual_str_ctx.ptw_ts_e =dual_str_ctx.ptw_ts_b+FRAME_SIZE_A;
	dual_str_ctx.ptw_ts =dual_str_ctx.ptw_ts_b;
	dual_str_ctx.sch_buff_full = false;
	dual_str_ctx.update_n = true;
	dual_str_ctx.buf_flag = 0;  // invalidated
	dual_str_ctx.ext_seq_cnt_n = -1;
 #endif
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
 uint32_t lgdst_reacquire_vch();
  uint32_t lgdst_reacquire_vch() {
	return it9137_acquire_channel(0,/*809000*/699000/*797000*/,6000);
  }
#endif




