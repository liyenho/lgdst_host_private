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
#include <assert.h>
#include "rf2072_set.h"
#include "usb_tx.h"
#ifdef UART_COMM
  #include <termios.h>
#endif
#include "it9510.h"
#include "firmware.h"
#include "platform_it9517.h"
#define true						    1
#define false							0

volatile uint8_t system_upgrade = 0;
volatile uint8_t upgrade_from_lgdst = 0;
char upgrade_fwm_path[160]; // upgrade firmware path

int shmid_Lgdst = -1;
ipcLgdst *shmLgdst_proc = 0;
void*shm = NULL;
volatile uint8_t main_loop_on = false; // run time indicator, liyenho
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
volatile bool get_sensitivity= false ;
const char sensitivity_curve[] = "sensitivity_curve.csv";
static FILE *sensitivity_cur= 0 ;
#endif

int udpin_len, udpout_len;
int udpin_socket, udpout_socket;
fd_set udpin_fd;
struct timeval udpin_tv= {0};
struct sockaddr_in udpin, udpout;
struct ip_mreq udpin_mreq;

bool stream_on = false ;
void fsm_time_ant_sw() ; // extr declr for timer.c

static FILE *file = NULL;
unsigned char audbuf[FRAME_SIZE_A*FRAME_BUFFS]__attribute__((aligned(8)));
unsigned char vidbuf[FRAME_SIZE_V2*FRAME_BUFFS]__attribute__((aligned(8)));
static bool detached = false;
struct libusb_device_handle *devh = NULL;
#ifdef UART_COMM
 #define HANDSHAKE_UART(AT_cmd, ccnt, label_again) \
			pthread_mutex_lock(&mux); \
   label_again: \
			 usleep(/*300*/1000); /*just keep this long enough to serve sync event*/ \
			write (fd_uart, AT_cmd, ccnt); \
			 usleep(400/*5000*/); /*longer than 3 bytes went across serial line @ 115200 bauds*/ \
			 r = read (fd_uart, &i, 3); \
			if (3 ==r) { \
				char *pa = (char*)&i; \
				if (1==do_exit && strncmp("ATK",pa, 3)) { \
					goto label_again; /*we must see"ATA" as acknowledge*/ \
				} \
			} \
			else { \
				if (1==do_exit) \
					goto label_again; \
			} \
			pthread_mutex_unlock(&mux);
 #define HANDSHAKE_UART_LOG(AT_cmd, ccnt, label_again) \
			pthread_mutex_lock(&mux); \
   label_again:  \
			 usleep(600/*1000*/); /*longer than 2 bytes went across serial line @ 115200 bauds*/ \
			write (fd_uart, AT_cmd, ccnt); \
			 usleep(400/*1000*/); /*not sure if atmel has something to log out*/ \
			 r = read (fd_uart, &i, 3); \
			if (3 ==r) { \
				char *pa = (char*)&i; \
				if (1==do_exit && strncmp("ATK",pa, 3)) { \
					goto label_again; /*we must see"ATA" as acknowledge*/ \
				} \
			} \
			else { \
				if (1==do_exit) \
					goto label_again; \
			} \
			pthread_mutex_unlock(&mux);
 #define HANDSHAKE_ACK_WR \
 			usleep(200); \
			/*pthread_mutex_lock(&mux);*/ \
			write (fd_uart, "ATK", 3); \
			/*pthread_mutex_unlock(&mux);*/
 #define HANDSHAKE_ACK_RD(label) \
 			i = 0; \
 			while(3 > i++) { \
 				usleep(/*600*/300); \
				/*pthread_mutex_lock(&mux);*/ \
				r = read (fd_uart, &i, 3); \
				/*pthread_mutex_unlock(&mux);*/ \
				if (3 ==r) { \
					char *pa = (char*)&i; \
					if (!strncmp("ATK",pa, 3)) \
						break ; \
				} \
			} if (3 == i) {\
				pthread_mutex_unlock(&mux); \
				failed = true; \
				puts("failed to get acked"); \
				goto label ; \
			} \
			else failed = false;
  const char *portname = "/dev/ttyUSB0";
  int fd_uart = 0; // file handle of uart port
#endif
volatile int do_exit = 0;	// main loop breaker
pthread_t poll_thread= 0,
		  lgdst_thread = 0,
#ifdef UART_COMM
			logging_thread = 0,
#endif
		  ctrl_thr_recv = 0,
		  ctrl_thr_send = 0;
pthread_mutex_t mux; // to be accessed in user.c

int chsel_2072 = 0;

//function prototyping

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
 #if defined(DBG_UART_SND)
		pthread_join(ctrl_thr_recv, NULL);
 #endif
 #if defined(DBG_UART_REC)
		pthread_join(ctrl_thr_send, NULL);
 #endif
#endif
		close(udpin_socket);
		close(udpout_socket);
#ifdef UART_COMM
		close(fd_uart);
#endif
#if defined(RADIO_SI4463)
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
	r=libusb_bulk_transfer(devh,EP_DATA,pdata,length,&transferred,70); // realtime recv no delay allowed
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
	int r, r1, i;
	unsigned char validdataflag=0;
	long pv_wrbyte=0,
				/*for sensitivity meas*/
				pkt_cnt=0;
#ifdef UART_COMM
			uint8_t *pb, radio_rpacket1[RADIO_USR_RX_LEN+MAVLINK_HDR_LEN+CHKSUM_LEN];
			// constant payload length from atmel ctrl link
				struct timeval uart_tv, uart_tv1;
				uart_tv.tv_sec = 0;
		// this field shall be adapted to actual baud rate (115200) selected, also depends upon
		// uart traffic convention (8N1) selected, also allowed tolerance on timing constraint,
		// this field is most crucial setup!!! liyenho
				uart_tv.tv_usec = 3/*1.5*/*(1000000*10+115200-1)/115200*sizeof(radio_rpacket1);
/***************************************************************
 	#include <sched.h>
 		struct sched_param sp;
 		uint32_t sch_pol, sch_prio_x ;
 		int err ;
 		  err = sched_getscheduler(0);
		  if (0 > err)
		  	 perror_exit ("sched_getscheduler failed", err);
		  sch_pol = err;
		  err = sched_get_priority_max(SCHED_FIFO);
		  if (0 > err)
		  	 perror_exit ("sched_get_priority_max failed", err);
		  // get highest possible prio for SCHED_FIFO scheme
		  sch_prio_x = err;
***************************************************************/
#endif
	while (1==do_exit) {
		bool failed_to_get, ctrl_sckt_ok = *(bool*)arg;
		if (ctrl_sckt_ok) {
#ifndef UART_COMM
			pthread_mutex_lock(&mux);
			libusb_control_transfer(devh,CTRL_IN, USB_RQ,RADIO_COMM_VAL,RADIO_DATA_RX_IDX,radio_rpacket, sizeof(radio_rpacket), 0);
			pthread_mutex_unlock(&mux);
#else  //UART_COMM
	//		HANDSHAKE_UART("AT4", 3, again)
				uart_tv1 = uart_tv;
				failed_to_get = false;
				fd_set uart_fd;
				FD_ZERO(&uart_fd);
				FD_SET(fd_uart,&uart_fd);
			pthread_mutex_lock(&mux);
/****************************************************
			sp.sched_priority = sch_prio_x;
			err = sched_setscheduler(0,SCHED_FIFO,&sp);
		  if (0 > err)
		  	 perror_exit ("sched_setscheduler failed", err);
****************************************************/
			r = sizeof(radio_rpacket1);
			pb = radio_rpacket1;
			while(1==do_exit && 0!=r) {
				int err = select(fd_uart+1,&uart_fd,0,0,&uart_tv1);
				if(err<0) {
					perror_exit("uart select failed", err);
				} else if (0==err) {
					failed_to_get = true;
					puts("failed to get packet");
					break; // force to send ack to re-set send side on the other end
				}
				if (FD_ISSET(fd_uart,&uart_fd)) {
					// Ethernet input data processing
					r1 = read (fd_uart, pb, r);
					r -= r1;
					pb += r1;
				}
			}
				HANDSHAKE_ACK_WR
				pthread_mutex_unlock(&mux);
/****************************************************
			sp.sched_priority = 0;
			err = sched_setscheduler(0,sch_pol,&sp);
		  if (0 > err)
		  	 perror_exit ("sched_setscheduler failed", err);
****************************************************/
			if (!failed_to_get) {
				uint16_t crc, crc0 ;
				crc= crc_calculate(radio_rpacket1+START_SIGN_LEN,
											RADIO_USR_RX_LEN \
											+MAVLINK_HDR_LEN \
											-START_SIGN_LEN);
				crc0 = *(radio_rpacket1+sizeof(radio_rpacket1)-CHKSUM_LEN);
				crc0 |= ((uint16_t)*(radio_rpacket1+sizeof(radio_rpacket1)-(CHKSUM_LEN-1))<<8);
	//printf("crc = 0x%04x, crc0 = 0x%04x\n",crc, crc0);
				if (crc == crc0 ) {
			#if defined(DBG_UART_SND)
					printf("....... received a valid uart packet, %d ......\n",
								*(radio_rpacket1+2));
			#endif
					memcpy(radio_rpacket,
										radio_rpacket1+MAVLINK_HDR_LEN,
										RADIO_USR_RX_LEN);
				}
			}
#endif
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

			// user packet dataloss ---------------------------------
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
			// user packet dataloss check

			// process for control link sensitivity measurement, liyenho
			pkt_cnt += validdataflag==0x01;
			if (get_sensitivity) {
				int sigdbm ;
				  pthread_mutex_lock(&mux);
				  libusb_control_transfer(devh,CTRL_IN, USB_RQ,RADIO_COMM_VAL,RADIO_GET_RSSI_IDX,&sigdbm, 4, 0);
				  pthread_mutex_unlock(&mux);
				long err_cnt = byteerror+ilcnt*RADIO_USR_RX_LEN;
				double err_rate;
				err_rate = (double)err_cnt /(pkt_cnt*RADIO_USR_RX_LEN);
				if (sensitivity_cur)
					fprintf(sensitivity_cur,
									"%d, %f,\n",
									sigdbm, err_rate);
			}

			usleep(CTRL_RECV_POLLPERIOD);
		} //ctrl_sckt_ok
	}//(1==do_exit)
}

void *ctrl_poll_send(void *arg)
{  //usb send to module
	int i, r, tx_cnt= 0, err, rcvsize;
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
#ifndef UART_COMM
			pthread_mutex_lock(&mux);
			libusb_control_transfer(devh,CTRL_OUT, USB_RQ,RADIO_COMM_VAL,RADIO_DATA_TX_IDX,radio_tpacket, sizeof(radio_tpacket), 0);
			pthread_mutex_unlock(&mux);
#else  //UART_COMM
			uint8_t radio_tpacket1[RADIO_USR_TX_LEN+MAVLINK_HDR_LEN+CHKSUM_LEN];
			bool failed = false;
again:
	//		HANDSHAKE_UART("AT3", 3, again0)
			*(radio_tpacket1) = MAVLINK_START_SIGN;
			*(radio_tpacket1+1) =sizeof(radio_tpacket1) -START_SIGN_LEN;
			static uint8_t uart_sequ_sent_cnt;
			if (!failed)
				*(radio_tpacket1+2) = uart_sequ_sent_cnt++;
			memcpy(radio_tpacket1+MAVLINK_HDR_LEN,
								radio_tpacket, RADIO_USR_RX_LEN);
			uint16_t crc ;
			crc= crc_calculate(radio_tpacket1+START_SIGN_LEN,
										sizeof(radio_tpacket1)-START_SIGN_LEN-CHKSUM_LEN);
			*(radio_tpacket1+sizeof(radio_tpacket1)-CHKSUM_LEN) = (unsigned char)crc;
			*(radio_tpacket1+sizeof(radio_tpacket1)-(CHKSUM_LEN-1)) = (unsigned char)(crc>>8);
			pthread_mutex_lock(&mux);
	//		write (fd_uart, &r, sizeof(short)); // two bytes shall be sufficient for all cases
			write (fd_uart,
					radio_tpacket1,
					sizeof(radio_tpacket1));
			HANDSHAKE_ACK_RD(again)
			pthread_mutex_unlock(&mux);
		#if defined(DBG_UART_REC)
				printf("....... sent a acked uart packet, %d ......\n",
							*(radio_tpacket1+2));
		#endif
#endif
		}

		usleep(CTRL_SEND_POLLPERIOD);
	}
}
#endif //RADIO_SI4463
#ifdef UART_COMM
void *logging_poll(void *arg)
{  //recv module log mesgs
  char msg_buf[MAX_MSG_LEN];
  int i, r ;
	while (1==do_exit) {
		bool failed = false;
loop:
	//	HANDSHAKE_UART_LOG("AT0", 3, loop0)
		pthread_mutex_lock(&mux);
			 r = read (fd_uart,
								msg_buf,
								sizeof(msg_buf));
			if (0 != r) {
				HANDSHAKE_ACK_RD(loop)
				printf("%s", msg_buf);
			}
		pthread_mutex_unlock(&mux);
		short_sleep(0.25);
	}
}
#endif

void *poll_thread_main(void *arg)
{
	int i, r=0, s, video, frms = 0, radio_cnt=0;
	long pa_wrbyte=0, pv_wrbyte=0;
	bool ctrl_sckt_ok = false;
	int tx_cnt= 0, rx_cnt= 0;
	printf("poll thread running\n");

	printf("Setup Ctrl Radio Sockets... ip=%s portTx=%d portRx=%d\n",servIP, ctrl_port_base+1, ctrl_port_base);
	int ctrlsnd_pt = htons(ctrl_port_base);
	int ctrlrcv_pt = htons(ctrl_port_base+1);

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
	}

	while (!ready_wait_for_mloop) ;

	si4463_radio_up = true;
#ifdef UART_COMM
	puts("send uart cmd to start up ctrl uart process");
			HANDSHAKE_UART("AT0", 3, uart_port_start)
#endif
	r=0;
#ifdef DBG_UART_SND
	r = pthread_create(&ctrl_thr_recv, NULL, ctrl_poll_recv, &ctrl_sckt_ok);
#endif
	if (0 != r)
		perror_exit("ctrl recv thread creation error", r);
#ifdef DBG_UART_REC
	r = pthread_create(&ctrl_thr_send, NULL, ctrl_poll_send, &ctrl_sckt_ok);
#endif
	if (0 != r)
		perror_exit("ctrl send thread creation error", r);
#ifdef UART_COMM
	//r = pthread_create(&logging_thread, NULL, logging_poll, NULL);
	if (0 != r)
		perror_exit("atmel logging thread creation error", r);
#endif
//#define TIME_ANT_SW
 #ifdef TIME_ANT_SW
	struct timeval tstart, tend, tdelta,
								  tsim = { 0, 166667 };  // every 1/6 sec
	get_time(&tstart);
 #endif
	while (1==do_exit) {
		struct timeval tv = { 0, 10000 };
		r = libusb_handle_events_timeout(NULL, &tv);
		if (r < 0) {
			perror_exit("event handler failed", r);
			break;
		}
 #ifdef TIME_ANT_SW
  			get_time(&tend);
  			time_diff(&tend,&tstart,&tdelta);
  			if(compare_time(&tsim, &tdelta)<0){
	  			/////////////////////////////////////////////////////////
	struct timeval start ,end;
	uint32_t diff;
  				gettimeofday(&start,NULL);
  				////////////////////////////////////////////////////////
				fsm_time_ant_sw();
				tstart = tend;
				////////////////////////////////////////////////////////
				gettimeofday(&end,NULL);
				diff= 1000*((int)end.tv_sec-
													(int)start.tv_sec)+
								0.001*((int)end.tv_usec-(int)start.tv_usec);
				printf("time taken = %d ms\n", diff);
				////////////////////////////////////////////////////////
			}
 #endif
	}
#if defined(DBG_UART_REC)
	pthread_join(ctrl_thr_send, NULL);
#endif
#if defined(DBG_UART_SND)
	pthread_join(ctrl_thr_recv, NULL);
#endif
	printf("poll thread shutting down\n");
	return NULL;
}

void *lgdst_thread_main(void *arg)
{
	static dAccess lclMem ;
	if (0>(shmid_Lgdst = shmget(SHMKEY_TX,sizeof(ipcLgdst),IPC_CREAT|0666))) {
		perror_exit("failed to create IPC memory for lgdst process, bailed out...", -7);
	}
	if (0>(shmLgdst_proc=shmat(shmid_Lgdst, NULL, 0))) {
		perror_exit("get shmLgdst_proc shmem failed",0);
	}
	memset(shmLgdst_proc, 0, sizeof(ipcLgdst));
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
					assert(sizeof(upgrade_fwm_path) >= shmLgdst_proc->len);
					memcpy(upgrade_fwm_path, (char*)acs->data, shmLgdst_proc->len);
					system_upgrade = 1*(USB_CPLD_UPGRADE_VAL == shmLgdst_proc->tag.wValue) ;
					puts("user requests system firmware upgrade...");
					break;
				}
				printf("CMD1: wDir = %d, wValue = %d, wIndex = %d, len= %d, data = %d\n",
						shmLgdst_proc->tag.wDir,shmLgdst_proc->tag.wValue,shmLgdst_proc->tag.wIndex,shmLgdst_proc->len,*(int*)acs->data);  // for debug
#ifdef RADIO_SI4463
				if (RADIO_SENS_VAL == shmLgdst_proc->tag.wValue) {
					memcpy(&get_sensitivity, (char*)acs->data, shmLgdst_proc->len);
					if (get_sensitivity) {
						unlink(sensitivity_curve);
						sensitivity_cur =fopen(sensitivity_curve, "wt");
					}
					else {
						if (sensitivity_cur) {
							fclose (sensitivity_cur);
							sensitivity_cur = NULL;
						}
					}
					break ;
				}
#endif
				pthread_mutex_lock(&mux);
				libusb_control_transfer(devh,shmLgdst_proc->tag.wDir,USB_RQ,shmLgdst_proc->tag.wValue,shmLgdst_proc->tag.wIndex,acs->data,shmLgdst_proc->len, 0);
				pthread_mutex_unlock(&mux);
				if (CTRL_IN==shmLgdst_proc->tag.wDir)
					memcpy(&shmLgdst_proc->access.hdr.data,&lclMem.hdr.data, shmLgdst_proc->len);
				break;
			case ACS:
				if (shmLgdst_proc->echo) {
					printf("ACS/echo: access= %d, dcnt= %d\n",(int)acs->access,(int)acs->dcnt);  // for debug
					while(1) {
						pthread_mutex_lock(&mux);
						if (libusb_control_transfer(devh,CTRL_IN, USB_RQ,USB_HOST_MSG_RX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs)-sizeof(acs->data[0]), 0)){
							pthread_mutex_unlock(&mux);
							break;
						}
						pthread_mutex_unlock(&mux);
						short_sleep(0.0005);
					}
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
	if (0 <= shmid_Lgdst ) {
		shmdt(shmid_Lgdst );
		if (shmctl(shmid_Lgdst, IPC_RMID, NULL))
			perror("Error deleting shmid_Lgdst shared memory segment");
	}
#ifdef RADIO_SI4463
	if (sensitivity_cur)
		fclose (sensitivity_cur);
#endif
}


void sigint_handler(int signum)
{
	do_exit = 4; // this shall terminate poll thread
	pthread_mutex_unlock(&mux);
}

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

//1=cpld firmware update  Uc   blksz = 7200;  delay = 0.84;
//3=atmel firmware update Ua1  blksz = 7200;  delay = 0.84;
//4=atmel boot direct   Ua0
int cpld_firmware_update(int mode, const char*file_name)
{
	uint32_t n,  fw_info[3+1];
	int cur,len,size,blksz=0,nread=0;
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
		fseek(file_up,0,SEEK_SET);


		if (1/*cpld*/== mode) {
			fw_info[2] = 0x00003800;  // constant cpld download address
			printf("upgrade firmware address: 0x%08x\n", fw_info[2]);
			pthread_mutex_lock(&mux);
			libusb_control_transfer(devh, CTRL_OUT, USB_RQ,USB_CPLD_UPGRADE_VAL, USB_HOST_MSG_IDX,	(unsigned char*)fw_info, FW_UPGRADE_HDR_LEN, 0);
			pthread_mutex_unlock(&mux);
			blksz = 7200; // similar size to atmel bin image, it should be fine
			delay = 0.84;
		}
		else if (3/*atmel*/== mode) {
			pthread_mutex_lock(&mux);
			libusb_control_transfer(devh, CTRL_OUT, USB_RQ,USB_FWM_UPDATE_VAL, USB_HOST_MSG_IDX,(unsigned char*)fw_info, ATMEL_UPGRADE_HDR_LEN, 0);
			pthread_mutex_unlock(&mux);
			blksz = 7200;
			delay = 0.84;
		}
		else if (4/*atmel boot*/== mode) {
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
		// begin download process thru usb
		cur = blksz;
		do {
			pthread_mutex_lock(&mux);
			libusb_control_transfer(devh,CTRL_IN, USB_RQ,USB_STREAM_ON_VAL,USB_QUERY_IDX,(unsigned char*)&main_loop_on, sizeof(main_loop_on), 0);
			pthread_mutex_unlock(&mux);
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
#ifdef UART_COMM
int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                perror_exit ("error from tcgetattr",-1);
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                perror_exit ("error from tcsetattr",-2);
        }
        return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                perror_exit ("error from tggetattr",-3);
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                perror_exit ("error setting term attributes",-4);
}
#endif
#if 0
int init_device(void)
#else
init_device(int argc,char **argv)
#endif
{
	int r = 1, blksz=0, tag;

	do_exit = 1;
	pthread_mutex_init(&mux, NULL);
	r = libusb_init(NULL);
	if (r < 0)
		perror_exit("failed to initialise libusb",1);
	libusb_set_debug(NULL, 2);
	while (1) { // wait for atmel usb ready
#if (/*1*/0)
	devh = libusb_open_device_with_vid_pid(NULL,
					USB_DEV_VENDER, USB_DEV_PRODUCT);
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
    	system_upgrade = /*(!strcasecmp(argv[2],"Uf0"))?5:*/ // fpga image switch
    						((!strcasecmp(argv[2],"Ua0"))?4: // direct boot atmel
    						((!strcasecmp(argv[2],"Ua1"))?3:	// fwm upgrade atmel
    						/*((!strcasecmp(argv[2],"Uf"))?2:*/
    						((!strcasecmp(argv[2],"Uc"))?1: 0 ))/*)*/);
		if (system_upgrade) {
				if (4>system_upgrade) {
					if (argc <4) {
						perror_exit("missing upgrade firmware filepath, bailed out...",-3); }
					strncpy(upgrade_fwm_path, argv[3], sizeof(upgrade_fwm_path));
				}
				// bypass cmdline parsing for all other tasks
				//goto libusb_next;
		}
	}
		if (devh <= 0) {
			//libusb_exit(NULL);
			//perror_exit("could not find/open USB device",2);
			printf("wait for atmel usb ready...\n");
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
	if (system_upgrade) // no need to bring up other proc threads,
		goto upgrade_next;
#ifdef UART_COMM
	fd_uart = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd_uart < 0)
	{
		char err_msg[80]= {"error opening: "};
			strcat(err_msg, portname); strcat(err_msg,"\n");
		perror_exit (err_msg,-5 );
	}
	set_interface_attribs (fd_uart, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (fd_uart, 0);                // set no blocking
 #if false
	write (fd_uart, "hello!\n", 7);	// send 7 character greeting
	short_sleep(0.1); // validate echo after 0.1 sec
	char buf [16];
	r = read (fd_uart, buf, sizeof buf);  // read up to 100 characters if ready to read
	printf("echo from atmel uart: %s\n",buf);
 #endif
#endif
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
	if (SIG_ERR == signal(SIGINT, sigint_handler)) {
		perror("FAIL: assigning signal handler");
	}
	r = pthread_create(&lgdst_thread, NULL, lgdst_thread_main, NULL);
	if (0 != r)
		perror_exit("lgdst thread creation error", r);
printf("line # = %d\n", __LINE__);

	return 0;

}

uint32_t deinit_device(int rcode)
{
	uint32_t error=ModulatorError_NO_ERROR;
	if (0 > rcode || true==(bool) rcode) goto _fail;
	fprintf(stderr, "Wrap up/Leaving test_snd app...\n");
	if (0 > do_exit) ;//return do_exit;  // return from error
	else do_exit = 4;

	if (file) fclose(file);

	pthread_join(poll_thread, NULL);
	pthread_join(lgdst_thread, NULL);
	close(udpin_socket);
	close(udpout_socket);
#if defined(RADIO_SI4463)
	close(ctrlsnd_socket);
	close(ctrlrcv_socket);
#endif
#ifdef UART_COMM
	close(fd_uart);
#endif
_fail:
	libusb_release_interface(devh,USB_DEV_INTF);
	if (detached)
		libusb_attach_kernel_driver(devh,USB_DEV_INTF);
	libusb_close(devh);
	libusb_exit(NULL);
	pthread_mutex_destroy(&mux);
	return 0;


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
		short_sleep(0.01); 	// validate echo after 0.1 sec
		printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",*(uint16_t*)acs->data,acs->addr);
	}
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

	//printf("P2_FREQ1=%x,P2_FREQ2=%x,P2_FREQ3=%x\n",P2_FREQ1,P2_FREQ2,P2_FREQ3);

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
	short_sleep(0.01); 	// validate echo after 0.1 sec
	printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",*(uint16_t*)acs->data,acs->addr);

	acs->dcnt = sizeof(*conv);

	acs->access = RF2072_WRITE;
	acs->addr = 0x0F;
	*conv =P2_FREQ1;
	pthread_mutex_lock(&mux);
	libusb_control_transfer(devh,CTRL_OUT, USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0);
	pthread_mutex_unlock(&mux);
	short_sleep(0.01); 	// validate echo after 0.1 sec
	printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",*(uint16_t*)acs->data,acs->addr);

	acs->dcnt = sizeof(*conv);

	acs->access = RF2072_WRITE;
	acs->addr = 0x10;
	*conv =P2_FREQ2;
	pthread_mutex_lock(&mux);
	libusb_control_transfer(devh,CTRL_OUT, USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0);
	pthread_mutex_unlock(&mux);
	short_sleep(0.01); 	// validate echo after 0.1 sec
	printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",*(uint16_t*)acs->data,acs->addr);

	acs->dcnt = sizeof(*conv);

	acs->access = RF2072_WRITE;
	acs->addr = 0x11;
	*conv =P2_FREQ3;
	pthread_mutex_lock(&mux);
	libusb_control_transfer(devh,CTRL_OUT, USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0);
	pthread_mutex_unlock(&mux);
	short_sleep(0.01); 	// validate echo after 0.1 sec
	printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",*(uint16_t*)acs->data,acs->addr);

	return 0;
}

int init_rf2072(void)
{
	int chsel_tx=0, pwr_attn=10000;
	dev_cfg* pregs ;
	int32_t i, sz, msg[80]; // access buffer
	dev_access *acs = (dev_access*)msg;
	uint16_t *conv= (uint16_t*)acs->data;

	open_ini(&chsel_tx, &pwr_attn);  // get default chan/pwr settings

	acs->access = RF2072_RESET;
	acs->dcnt = 0; // no param
	acs->addr = 0x0; // by wire not addr
	pthread_mutex_lock(&mux);
	libusb_control_transfer(devh,CTRL_OUT,USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs), 0);
	pthread_mutex_unlock(&mux);
	short_sleep(0.1);

	rffe_write_regs(pregs=GET_ARRAY(chsel_2072), sz=ARRAY_SIZE(chsel_2072));

	set_frequency_rf2072(LO_Frequency);

	//uint16_t *conv=(uint16_t*)acs->data;
//	short_sleep(0.1);
	acs->dcnt = sizeof(uint16_t);
	acs->access = RF2072_WRITE;
	acs->addr = 0x09;
	*conv =((0x8224&0xFFF7) | 0x0008);
	pthread_mutex_lock(&mux);
	libusb_control_transfer(devh,CTRL_OUT, USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0);
	pthread_mutex_unlock(&mux);
	printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",*(uint16_t*)acs->data,acs->addr);

	short_sleep(0.01);

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
	short_sleep(0.01);
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
	short_sleep(0.01);
	while(1) {
		pthread_mutex_lock(&mux);
		if (libusb_control_transfer(devh,CTRL_IN, USB_RQ,USB_HOST_MSG_RX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0)){
			pthread_mutex_unlock(&mux);
			break;
		}
		pthread_mutex_unlock(&mux);
		short_sleep(0.0005);
	}
	short_sleep(0.01);
	printf("lock state = 0x%x\n",(0x8000&*(uint16_t*)acs->data)?1:0);

	printf("tx rffe is running...\n");

	return 0;

}

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
	while(UDP_PACKET_MAX>loc_buf_ptr) {
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
	memmove(pbf,
			pbf+bytes,
			loc_buf_ptr- bytes);
	loc_buf_ptr -= bytes;
	return 0;
}


int main(int argc,char **argv)
{
	struct timeval start ,end1,end2;
	unsigned long diff,diff1;
	int i, r=0;

	gettimeofday(&start,NULL);
	uint16_t bandwidth = 6000;
	uint16_t sawBandwidth = 8000;
	static IQtable IQ_tableEx [65536];
	uint32_t error = ModulatorError_NO_ERROR;
	TPS tps;
	TransportLayer  layer;
	ChannelModulation      channel_Modulation;
	int tag=0;
#if 0 // never use this option, can't do upgrade
	init_device();
#else
printf("line # = %d\n", __LINE__);
	init_device(argc, argv);
printf("line # = %d\n", __LINE__);
 	uint32_t n, size, blksz, fw_info[3+1]; /*len = SMS_FW_HDR_LEN*/
 	float delay ;
  if (system_upgrade) { // user request firmware upgrade
printf("line # = %d\n", __LINE__);
  	FILE *file_up ;
  	int end, cur, len;
  	bool first_file = true;
upgrade_firmware:
 #if false  // not used
	if (5/*fpga switch*/== system_upgrade) {
		pthread_mutex_lock(&mux);
		libusb_control_transfer(devh,
					CTRL_OUT,
					USB_RQ,
					USB_FPGA_NEW_VAL,
					USB_HOST_MSG_IDX,
					NULL, 0, 0);
		pthread_mutex_unlock(&mux);
		goto exit; // normal shutdown
	}
 #endif
	if (4>system_upgrade) {
		file_up = fopen(upgrade_fwm_path, "rb");
  		if (!file_up) {
	  		puts("failed to open upgrade firmware, bail out....");
	  		deinit_device(1);
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
	  		pthread_mutex_lock(&mux);
		 libusb_control_transfer(devh, CTRL_OUT, USB_RQ,
			USB_CPLD_UPGRADE_VAL, USB_HOST_MSG_IDX,
			fw_info, FW_UPGRADE_HDR_LEN, 0);
			pthread_mutex_unlock(&mux);
			blksz = 7200; // similar size to atmel bin image, it should be fine
  			delay = 0.84;
	  	}
#if false // not used
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
	  		pthread_mutex_lock(&mux);
		 libusb_control_transfer(devh, CTRL_OUT, USB_RQ,
			USB_FPGA_UPGRADE_VAL, USB_HOST_MSG_IDX,
			fw_info, FW_UPGRADE_HDR_LEN, 0);
			pthread_mutex_unlock(&mux);
			blksz = 15000;
			delay = /*3.5*/0.05;
  		}
#endif
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
			deinit_device(1); // it is done
  		}
		else {
			puts("invalid target for upgrade request, bail out....");
			deinit_device(1); }
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
	    	fwrite(vidbuf+/*7200*/blksz, size, 1, fdmp);
    // close readback dump file
	fclose(fdmp);
 #endif
 	if (2<system_upgrade || (4== argc && !strcasecmp(argv[3], upgrade_fwm_path)))
		 deinit_device(1) ; // finished regardless if there's error
	else {
#ifndef NON_NIOS // no need anymore
		if (2 == system_upgrade && first_file)
			goto next_file;
	else
#endif
		deinit_device(0) ; // normal shutdown
  	}
  	return 0;
  }
#endif
#ifdef UART_COMM // test atmel encapsulation of asic host
  #if false  // going thru serial uart now...
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
  #else  //UART_COMM
	puts("send uart cmd to init video sub-system");
			HANDSHAKE_UART("AT1", 3, init_again_v)
	if (1==do_exit)
		short_sleep(20); // allowed 30 sec for video sub-system comes up...
	puts("send uart cmd to start video sub-system");
			HANDSHAKE_UART("AT2", 3, start_again_v)
	if (1==do_exit)
		short_sleep(10); // allowed 10 sec for video sub-system streams up...
  #endif
printf("line # = %d\n", __LINE__);
	udpin_init();
#else
printf("line # = %d\n", __LINE__);
	init_rf2072();
printf("line # = %d\n", __LINE__);
	udpin_init();

	error=it9517_initialize (Bus_I2C,SERIAL_TS_INPUT);
	if(error)goto exit;
	//error= it9517_loadIQ_calibration_table (const char*file_name);
	//if(error)goto exit;
	error = it9517_reset_pidfilter();
	if(error)goto exit;
	error= it9517_control_pidfilter(0,0);
	if(error)goto exit;
	//	channel_Modulation.frequency=809000;
	//	channel_Modulation.bandwidth=6000;
	channel_Modulation.constellation=Constellation_QPSK;
	channel_Modulation.highCodeRate=CodeRate_1_OVER_2;
	channel_Modulation.interval=Interval_1_OVER_32;
	channel_Modulation.transmissionMode=TransmissionMode_2K;
	error=it9517_set_channel_modulation( channel_Modulation,2);
	if(error)goto exit;
	//error=it9517_acquire_channel(/*809000*//*720000*/750000,6000); //avoid conflict with wifi, liyenho
	error=it9517_acquire_channel(/*809000*/720000,6000);
	if(error)goto exit;
	//error=it9517_get_output_gain();
	//if(error)goto exit;
	//error=it9517_get_output_gain_range(809000,6000);
	//if(error)goto exit;
	error=it9517_adjust_output_gain(0/*-30*/);
	//error=it9517_adjust_output_gain(0);
	if(error)goto exit;
	//	error = it9517_reset_pidfilter();
	//	if(error)goto exit;
	//	error= it9517_control_pidfilter(0,1);
	//	if(error)goto exit;
	//error=it9517_add_pidfilter(0, 0x100);
	//if(error)goto exit;
	//	error=it9517_pcr_restamp(PcrModeDisable,1);
	//	if(error)goto exit;
	gettimeofday(&end1,NULL);
	error=it9517_enable_transmission_mode(1);
	if(error)goto exit;
	 //gettimeofday(&end1,NULL);
#endif
	int32_t  msg[80]; // access buffer
	//uint16_t *conv= (uint16_t*)acs->data;

	stream_on = true; // slow down ctrl xfer access at real time, liyenho
	dev_access *acs = (dev_access*)msg;
	acs->access = TS_VID_ACTIVE;
	acs->dcnt = 0; // no param
	acs->addr = 0x0; // by wire not addr
	pthread_mutex_lock(&mux);
	libusb_control_transfer(devh,CTRL_OUT,USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs), 0);
	pthread_mutex_unlock(&mux);
	short_sleep(0.1);
	pthread_mutex_lock(&mux);
	libusb_control_transfer(devh, CTRL_OUT, USB_RQ, 0x07, 0, NULL, 0, 0);
	pthread_mutex_unlock(&mux);
	gettimeofday(&end2,NULL);
	diff= 1000000*(end1.tv_sec-start.tv_sec)+end1.tv_usec-start.tv_usec;
	diff1= 1000000*(end2.tv_sec-end1.tv_sec)+end2.tv_usec-end1.tv_usec;
	printf("diff=%ld,diff1=%ld,diff+diff1=%ld\n",diff,diff1,diff+diff1);

	ready_wait_for_mloop = true;

	if(0>receive(2549/*usec*/, audbuf, UDP_PACKET_MAX))
		perror_exit("invalid socket read, bailed out...", -6);

	tag += 1;
	while (ITERS>=tag) {
		if (1 != do_exit) break; // we can't fail here......
		if (0>stream_block(audbuf,FRAME_SIZE_A)) {

			continue;
		}

		else{
			static int tpbprtcnt=0;
			tpbprtcnt++;
			if(tpbprtcnt>/*100*/28) {
				// it is approximately 1 sec after received up 167 TS blocks @ 2.5 mb/s
				// so it is approximately 1/6 sec after received up 28 TS blocks @ 2.5 mb/s
  #if false
  			gettimeofday(&start,NULL);
				fsm_time_ant_sw();  // it took 93-94 ms, way too long for smooth TS block xfer...
			gettimeofday(&end1,NULL);
			diff= 1000*((int)end1.tv_sec-
												(int)start.tv_sec)+
							0.001*((int)end1.tv_usec-(int)start.tv_usec);
			printf("time taken = %d ms\n", diff);
  #endif
				tpbprtcnt=0;
			}
			static int fcnt = 0;
			if (100 == ++fcnt) {
						printf("send a transport packet block,%d\n",tag);
					fcnt = 0;
			}
		}


		if (ITERS==tag)
			break;	// done

		if (ITERS-1==tag)
			r = FILE_LEN - (tag+1)*FRAME_SIZE_A;
		else

			r = FRAME_SIZE_A;

	 if(0>receive(2549, audbuf, UDP_PACKET_MAX))
	  	perror_exit("invalid socket read, bailed out...", -6);

     tag += 1;
		if (ITERS==tag)
			tag = 0; // make it endless, liyenho
	}
     // return 0;
 exit:
	if (error)  printf("error=%x,%d\n",error,__LINE__);
	deinit_device(0);
	return error;

   }

void fsm_time_ant_sw() {
	static int tm_tick =0;
	tm_tick += 1;
	switch(tm_tick) {
		case 1: it9517_adjust_output_gain(1);
						break;
		case 2: it9517_adjust_output_gain(2);
						break;
		case 3: it9517_adjust_output_gain(3);
						pthread_mutex_lock(&mux);
		 				libusb_control_transfer(devh,
		 													CTRL_OUT,
		 													USB_RQ,
															USB_VID_ANT_SWITCH,
															0x1,
															NULL,
															0,
															0);
						pthread_mutex_unlock(&mux);
						break ;
		case 4: it9517_adjust_output_gain(2);
						break;
		case 5: it9517_adjust_output_gain(1);
						break;
		case 6: it9517_adjust_output_gain(0);
						break;
		default: break;
	}
#ifdef TIME_ANT_SW
	if (18 == tm_tick) { // cycle every 3 sec
		tm_tick = 0;
	}
#else  // in real case
	if (6 == tm_tick) { // in a full sec
		tm_tick = 0;
	}
#endif
}




