#include "lgdst_rx.h"
 #include <stdio.h>
 #include <string.h>
 #include <signal.h>
 #include <pthread.h>
 #include <time.h>
 #include <sys/time.h>
 #include <unistd.h>
 #include <sys/socket.h>
#include <arpa/inet.h>
#include "timer.h"

#define UDPOUT_ADDR 							"127.0.0.1"
#define UDPOUT_PORT  							5558
#define FRAME_SIZE_A							(188*10)
#define RADIO_USR_TX_LEN						30 // ctl/sts radio payload byte length
#define RADIO_USR_RX_LEN						30 // ctl/sts radio payload byte length
#define RADIO_INFO_LEN  						4 // gives usb pipe info



  static pthread_t thread_lgdst_rx = 0,
   				   thread_snd_rx= 0,
					thread_rec_rx = 0,
					thread_ts_rx = 0;
  static volatile int do_exit_m = 0;
  static int udpout_len;
  static int udpout_socket;
  static fd_set udpin_fd;
  struct sockaddr_in udpout;

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
		  		if (3>=argc) {
			  		puts("empty command line received...");
			  		if (1==do_exit_m) break;
			  		continue;
		  		}
		  		if (20<argc) {
			  		puts("too many fields in command line...");
			  		if (1==do_exit_m) break;
			  		continue;
		  		}
		  		lgdst_access_rx(argc, argv, NULL);
		  }
	  }
	  return ;
	}
static int udpout_init(char* udpaddr)
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

  static void *vid_recv_rx(void *arg) {
	  const struct timeval loop= {3, 0}; // in every three secs
	  struct timeval tstart,tend,tdelta;
		uint8_t tsbuf[FRAME_SIZE_A];
		udpout_init(UDPOUT_ADDR) ;
		get_time(&tstart);
		while (1 != do_exit_m) {
			lgdst_ts_rx(tsbuf);
      {
        int r = FRAME_SIZE_A;
	      int frag, sentsize;
        unsigned char *pb = tsbuf;
				for (frag=0; frag<5; frag++) {
					sentsize=sendto(udpout_socket, pb, r/5,0,(struct sockaddr *)&udpout,
	                  udpout_len);
	        if (sentsize < 0) printf("send pack ERorr\n");
	      		pb += r/5;
   		  }
      }
			get_time(&tend);
			time_diff(&tend, &tstart, &tdelta);
			if (compare_time(&tdelta,&loop)>0) {
				lgdst_vid_stats_rx();
				tstart = tend;
			}
		}
		return ;
	}


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
   puts("\nPlease hit <return> to terminate execution");
}

void send_pair_id_cmd(void)
{
int argctmp = 14;
char *sztmp[] = { "lgdst","0","rx","pair-id","00","00","00","00","00","00","00","00","00","01"
};

	lgdst_access_rx(argctmp, sztmp, NULL);
}




////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
  int r, res ;
   if (SIG_ERR == signal(SIGINT, sigint_handler)) {
     perror("FAIL: assigning signal handler");
   }

	res = lgdst_init_rx(argc, argv);
	if (res > 0 && res != 1) {  // failed to init, bail out
		lgdst_deinit_rx(res);
	}
	else if (1 == res) { // system upgrade or atmel reboot
		res = (int)lgdst_upgrade_rx(argc, argv);
		lgdst_deinit_rx(res);
	}
	else {  // normal operation
		printf("Creating threads\n");
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

    //user lgdst commands here
		puts("Finished starting threads");
		sleep(5);
		puts("Starting pair-id cmd");
		send_pair_id_cmd();

		struct timeval tstart,tend,tdelta;
		const struct timeval lfoop= {600, 0}; // 10 min run time
		get_time(&tstart);
		do {
			get_time(&tend);
			time_diff(&tend, &tstart, &tdelta);
		} while(1!=do_exit_m); // && compare_time(&tdelta,&loop)<0);
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


