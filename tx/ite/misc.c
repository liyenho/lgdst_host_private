#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <fcntl.h>  // for pipe creation
#include <stdlib.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <libusb.h>
#include <assert.h>

#include "usb_tx.h"
extern volatile uint8_t system_upgrade ;
extern volatile int do_exit ;
extern pthread_mutex_t mux;
#ifdef UART_COMM
  extern pthread_mutex_t mux_bulk;
#endif
extern struct libusb_device_handle *devh;
extern unsigned char vidbuf[FRAME_SIZE_V2*FRAME_BUFFS];
extern char upgrade_fwm_path[160];

void lgdst_upgrade_tx(int argc, char **argv)  // return -1 when failed, liyenho
{
 	uint32_t r, n, size, blksz, fw_info[3+1]; /*len = SMS_FW_HDR_LEN*/
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
		deinit_device(0); // normal shutdown
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
  }
}

#ifdef UART_COMM
 #ifndef NO_LOGGING_MODE
 	extern volatile uint8_t main_loop_on;
 	extern volatile bool logger_ready;
 #endif

 #define SHMKEY_LOG	7654

  #define ALL_STD_FDS   3
  //static int pipes[ALL_STD_FDS][2];
  typedef struct {
	  int pipes[ALL_STD_FDS][2];
	  bool active ;
  } IPC_LOG;
	IPC_LOG *ipc_mem;

  #define PARENT_WRITE_PIPE  0
  #define PARENT_READ_PIPE   1
  #define PARENT_ERR_PIPE   2

  #define READ_FD  0
  #define WRITE_FD 1

  //#define PARENT_READ_FD  ( pipes[PARENT_READ_PIPE][READ_FD]   )
    #define PARENT_READ_FD  ( ipc_mem->pipes[PARENT_READ_PIPE][READ_FD]   )
  //#define PARENT_WRITE_FD ( pipes[PARENT_WRITE_PIPE][WRITE_FD] )
    #define PARENT_WRITE_FD ( ipc_mem->pipes[PARENT_WRITE_PIPE][WRITE_FD] )
  //#define PARENT_ERR_FD ( pipes[PARENT_ERR_PIPE][READ_FD] )
    #define PARENT_ERR_FD ( ipc_mem->pipes[PARENT_ERR_PIPE][READ_FD] )

  //#define CHILD_READ_FD   ( pipes[PARENT_WRITE_PIPE][READ_FD]  )
    #define CHILD_READ_FD   ( ipc_mem->pipes[PARENT_WRITE_PIPE][READ_FD]  )
  //#define CHILD_WRITE_FD  ( pipes[PARENT_READ_PIPE][WRITE_FD]  )
    #define CHILD_WRITE_FD  ( ipc_mem->pipes[PARENT_READ_PIPE][WRITE_FD]  )
  //#define CHILD_ERR_FD  ( pipes[PARENT_ERR_PIPE][WRITE_FD]  )
    #define CHILD_ERR_FD  ( ipc_mem->pipes[PARENT_ERR_PIPE][WRITE_FD]  )

void *logging_poll(void *arg)
{  //recv module log mesgs
#ifndef NO_LOGGING_MODE
  //pid_t pid = 0;	// for both myself and client
#endif
  char msg_buf[MAX_MSG_LEN];
  char *pe= msg_buf+MAX_MSG_LEN, *pr= msg_buf;
  int i = 0, r, received;
#ifndef NO_LOGGING_MODE
 #if false
	int shmid_Logger = -1;
	if (0>(shmid_Logger= shmget(SHMKEY_LOG,sizeof(IPC_LOG),IPC_CREAT|0666))) {
		perror("failed to create IPC memory for debugger process, bailed out...");
		exit (2);
	}
	if (0>(ipc_mem=(IPC_LOG*)shmat(shmid_Logger, NULL, 0))) {
		perror("get shmLogger shmem failed");
		exit (3);
	}
	memset(ipc_mem, 0, sizeof(IPC_LOG));

	// start up pipe to comm with debugger client
  	pipe(ipc_mem->pipes[PARENT_WRITE_PIPE]);
	printf("parent write pipe fd = %d\n", PARENT_WRITE_FD);
	ipc_mem->active = true; // shm signalling flag to dbg client
  	/*if (0 == (pid=fork())) {	// logger child
//		dup2(CHILD_READ_FD, STDIN_FILENO);
		// not used any more on child
//		close(CHILD_READ_FD);
		close(PARENT_WRITE_FD);
		// start up debugger client
		static char* argv[2] = {"./atm_debugger", NULL};
		execve("./atm_debugger", argv, NULL);
		// should never get here unless error
		perror("atm_debugger launch failed...");
		exit(1);
	}
	if (-1 == pid) {
		perror("atm_debugger failed to launch, bailed out...");
		return (void*)-1;
	}*/
 #endif
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
	// signal main thread to proceed...
  logger_ready = true;
#endif
	char *pb, *pb0;
	pb = msg_buf + MAX_MSG_LEN;
	while (1==do_exit) {
		//pthread_mutex_lock(&mux_bulk);
		r=libusb_bulk_transfer(devh,
															EP_LOG,
															pr, MAX_MSG_LEN-i,
															&received,
															2);
		//pthread_mutex_unlock(&mux_bulk);
		if (0==r && 0!=received) {
			pr += received;
			i += received;
		}
		if (r<0 || MAX_MSG_LEN>i) {
			usleep(1000);
			continue ;
		}
#ifndef NO_LOGGING_MODE
		for (r = 0; r<MAX_MSG_LEN; r++)
      		fputc(msg_buf[r], stderr);
 #if false
		// release legging message to client
		write (PARENT_WRITE_FD,
					msg_buf, MAX_MSG_LEN);
 #endif
#endif
		// reset ptr/ind
		pr = msg_buf;
		i = 0;
	}
#ifndef NO_LOGGING_MODE
 #if false
	// close pipe resource
	close(PARENT_WRITE_FD);
	if (0 < shmid_Logger ) {
		shmdt(shmid_Logger );
		if (shmctl(shmid_Logger, IPC_RMID, NULL))
			perror("Error deleting shmid_Logger shared memory segment");
	}
 #endif
#endif
}
#endif
