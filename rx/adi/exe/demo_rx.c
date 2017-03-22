#include "lgdst_rx.h"
 #include <stdio.h>
 #include <string.h>
 #include <signal.h>
 #include <pthread.h>
 #include <time.h>
 #include <sys/time.h>
 #include <unistd.h>

  extern int time_diff(struct timeval *time1,
  								struct timeval *time2,
                                struct timeval *diffTime);
  extern int get_time(struct timeval *time);

  static pthread_t thread_lgdst_rx = 0,
  									thread_snd_rx= 0,
									thread_rec_rx = 0,
									thread_ts_rx = 0;
  static int do_exit_m = 0;

  static void *lgdst_rx(void *arg) {
		while (1 != do_exit_m) {
		  char cmdline[160], *pl = cmdline ;
		  size_t len = sizeof(cmdline);
		  if (-1 != getline(&pl, &len, stdin)) {
			  char *argv[20+1] ; // max 20 fields on cmdline
			  int argc = 0;
			  do {
			  		argv[argc++] = strtok(pl, " ");
			  		pl = NULL;
		  		} while (20>=argc && argv[argc-1]) ;
		  		if (1==argc) {
			  		puts("empty command line received...");
			  		continue;
		  		}
		  		if (20<argc) {
			  		puts("too many fields in command line...");
			  		continue;
		  		}
		  		if (!strcmp(argv[0], "lgdstv")) {
			  		strcpy(argv[0], "lgdst");
		  			lgdst_access_rx_vid(argc, argv);
	  			}
		  		else if (!strcmp(argv[0], "lgdstc")) {
			  		strcpy(argv[0], "lgdst");
		  			lgdst_access_rx_ctl(argc, argv);
	  			}
		  		else // invalid lgdst cmd
		  			puts("invalid lgdst command, only lgdstv or lgdstc");
		  }
	  }
	  return ;
	}
  #define FRAME_SIZE_A	(188*10)
  static void *vid_recv_rx(void *arg) {
	#if defined(PES_HDR_PROT) ||defined(PES_FRM_PROT) ||defined(PES_FRM_PROT1)
		uint8_t tsbuf[160*FRAME_SIZE_A]; // in case of big I frame
		uint32_t size = 160*FRAME_SIZE_A;
	#else
		uint8_t tsbuf[FRAME_SIZE_A];
	#endif
		//FILE *file_ts = fopen("movebox118440-rec.ts", "wb");
		FILE *file_ts = fopen("cap160910_pid0x100-rec.ts", "wb");
		if (!file_ts) {
			puts("failed to open TS dump file, bail out...");
			return ;
		}
		while (1 != do_exit_m) {
	#if defined(PES_HDR_PROT) ||defined(PES_FRM_PROT) ||defined(PES_FRM_PROT1)
			size = 160*FRAME_SIZE_A;
			lgdst_ts_rx(tsbuf, &size);
			fwrite(tsbuf, size, 1, file_ts);
	#else
			lgdst_ts_rx(tsbuf);
			fwrite(tsbuf, FRAME_SIZE_A, 1, file_ts);
	#endif
			//usleep(3333); // simulate 4.5 mb/s TS data rate
		}
		fclose(file_ts);
		return ;
	}

	#define RADIO_USR_TX_LEN		30 // ctl/sts radio payload byte length
	#define RADIO_USR_RX_LEN		30 // ctl/sts radio payload byte length
	#define RADIO_INFO_LEN  			4 // gives usb pipe info
  static void *ctrl_send_rx(void *arg) {
		uint8_t *pb, tpacket[RADIO_USR_TX_LEN] ;
		int n, nn= 0;
		while (1 != do_exit_m) {
			lgdst_ctl_snd_rx(tpacket);
#if false  // one can enable if he wants to exam the ctrl data link
			pb = tpacket;
			printf("sent ctrl msg\n\t");
			for (n=0; n<RADIO_USR_TX_LEN; n++) {
				*pb++ = (uint8_t)nn++;
				printf("0x%02x, ", *(pb-1));
			}
			puts("");
#endif
			usleep(24000);	// simulate 10 kb/s rec CTRL data rate
		}
		return ;
	}
  static void *ctrl_recv_rx(void *arg) {
		uint8_t *pb, rpacket[RADIO_USR_RX_LEN+RADIO_INFO_LEN] ;
		int n, nn;
		while (1 != do_exit_m) {
			lgdst_ctl_rec_rx(rpacket);
#if false // one can enable if he wants to exam the ctrl data link
			pb = rpacket;
			printf("received ctrl msg\n\t");
			for (n=0; n<RADIO_USR_RX_LEN; n++) {
				printf("0x%02x, ", *pb++);
			}
			printf("\nreceived ctrl sts\n\t");
			for (nn=0; nn<RADIO_INFO_LEN; nn++) {
				printf("0x%02x, ", *pb++);
			}
			puts("");
#endif
			usleep(48000);	// simulate 5 kb/s rec CTRL data rate
		}
		return ;
	}

static void sigint_handler(int signum)
{
   do_exit_m = 1; // this shall terminate threads
   sigint_handler_rx(signum);
}

int main(int argc, char **argv) {
  int r, res ;
   if (SIG_ERR == signal(SIGINT, sigint_handler)) {
     perror("FAIL: assigning signal handler");
   }
 #if (/*1*/0)
	res = lgdst_init_rx();
 #else
	res = lgdst_init_rx(argc, argv);
 #endif
	if (0>res) {  // failed to init, bail out
		lgdst_deinit_rx(res);
	}
	else if (res) { // system upgrade or atmel reboot
		res = (int)lgdst_upgrade_rx(argc, argv);
		lgdst_deinit_rx(res);
	}
	else {  // normal operation
		r = pthread_create(&thread_rec_rx, NULL, ctrl_recv_rx, NULL);
		if (0 != r) {
			perror("ctrl recv thread creation error");
			goto end;
		}
		r = pthread_create(&thread_snd_rx, NULL, ctrl_send_rx, NULL);
		if (0 != r) {
			perror("ctrl send thread creation error");
			goto end;
		}
		r = pthread_create(&thread_ts_rx, NULL, vid_recv_rx, NULL);
		if (0 != r) {
			perror("video ts recv thread creation error");
			goto end;
		}
		r = pthread_create(&thread_lgdst_rx, NULL, lgdst_rx, NULL);
		if (0 != r) {
			perror("lgdst thread creation error");
			goto end;
		}
		struct timeval tstart,tend,tdelta;
		const struct timeval loop= {10, 0}; // 10 sec run time
		get_time(&tstart);
		do {
			get_time(&tend);
			time_diff(&tend, &tstart, &tdelta);
		} while(1!=do_exit_m && compare_time(&tdelta,&loop)<0);
end:
		// terminate user threads
		do_exit_m = 1;
		sigint_handler_rx(0);
		pthread_join(thread_rec_rx, NULL);
		pthread_join(thread_snd_rx, NULL);
		pthread_join(thread_ts_rx, NULL);
		pthread_join(thread_lgdst_rx, NULL);
		lgdst_deinit_rx(res);
		// prompt user at the end
		puts("All user threads returned, exit main()...");
	}
	return 0;
}

