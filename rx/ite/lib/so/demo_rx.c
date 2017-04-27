#include "lgdst_rx.h"
 #include <stdio.h>
 #include <string.h>
 #include <signal.h>
 #include <pthread.h>
 #include <unistd.h>
 #include <sys/socket.h>
#include <arpa/inet.h>
 #include <time.h>
 #include <sys/time.h>
 #include "timer.h"

#define UDPOUT_ADDR 							"127.0.0.1"
#define UDPOUT_PORT  							5558
#define FRAME_SIZE_A							(188*10)
#define RADIO_USR_TX_LEN						30 // ctl/sts radio payload byte length
#define RADIO_USR_RX_LEN						30 // ctl/sts radio payload byte length
#define RADIO_INFO_LEN  						4 // gives usb pipe info
#ifdef VIDEO_STATS
 #include <avformat.h>
 #include <avcodec.h>
 /* typedefs */
	#define REGISTER_PROTOCOL(X, x)                                         \
    {                                                                   \
        extern URLProtocol ff_##x##_protocol;                           \
        if (CONFIG_##X##_PROTOCOL)                                      \
            ffurl_register_protocol(&ff_##x##_protocol);                \
    }
	#define REGISTER_DEMUXER(X, x)                                          \
    {                                                                   \
        extern AVInputFormat ff_##x##_demuxer;                          \
        if (CONFIG_##X##_DEMUXER)                                       \
            av_register_input_format(&ff_##x##_demuxer);                \
    }
	#define REGISTER_DECODER(X, x)                                          \
    {                                                                   \
        extern AVCodec ff_##x##_decoder;                                \
        if (CONFIG_##X##_DECODER)                                       \
            avcodec_register(&ff_##x##_decoder);                        \
    }
  #define STRINGFY1(def) #def
  #define STRINGFY(def) STRINGFY1(def)
  #define UDPOUT_PORT_FF 	5557
#if UDPOUT_PORT_FF == UDPOUT_PORT
	error("duplicated UDP port for FFmpeg monitor...")
#endif
  #define VF_REPO_DUR		90	// approximately 3 sec
  static AVFormatContext *ic;
  static AVCodecContext *dec;
#ifdef _AUDIO_DECODE_
  static AVCodecContext *dec_aud;
#endif
	int st_index[AVMEDIA_TYPE_NB];
	static bool stream_on = false;
#endif
  static pthread_t thread_lgdst_rx = 0,
   				   thread_snd_rx= 0,
					thread_rec_rx = 0,
					thread_ts_rx = 0
#ifdef VIDEO_STATS
					, thread_vid_rx = 0
#endif
					;
  static uint8_t tsbuf[FRAME_SIZE_A];
  static volatile int do_exit_m = 0;
  static int udpout_len;
  static int udpout_socket;
  //static fd_set udpin_fd;
  struct sockaddr_in udpout;
#ifdef VIDEO_STATS
   static int udpout_socket_ff;
	struct sockaddr_in udpout_ff;
#endif
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
#ifdef VIDEO_STATS
	memset(&udpout_ff,0,sizeof(udpout_ff));
	udpout_ff.sin_family = AF_INET;
	udpout_ff.sin_port = htons(UDPOUT_PORT-1);
	udpout_ff.sin_addr.s_addr = inet_addr(udpaddr);
	udpout_socket_ff = socket(AF_INET, SOCK_DGRAM,0);
	if(udpout_socket_ff <0)
	{
		printf("Fail to open udpout_ff socket.\n");
		return -1;
	}
#endif
	return 0;
}

#ifdef VIDEO_STATS
  static int decode_interrupt_cb(void *ctx) {
	  return (do_exit_m ==1) ;
  }

  static void *vid_frmc_rx(void *arg) {
	  const struct timeval loop= {3, 0}; // in every three secs
	  struct timeval tstart,tend,tdelta;
//	printf("line # = %d\n", __LINE__);
	    /* allocate the output media context */
	    ic = avformat_alloc_context();
//	printf("line # = %d\n", __LINE__);
	    if (!ic) {
		    puts("failed to init ffmpeg util, bailed out");
		    exit(-1);
	    }
//	printf("line # = %d\n", __LINE__);
	     ic->ctx_flags &= ~AVFMTCTX_NOHEADER;
	    // register necessary access protocols
	    //REGISTER_PROTOCOL (UDP, udp);
	    // register necessary transport formats
	    //REGISTER_DEMUXER (MPEGTS, mpegts);
	     // register H264 decoder wrapper
        // REGISTER_DECODER (H264, h264);
       // register faad2 decoder
       // REGISTER_DECODER (AAC, aac);
//	printf("line # = %d\n", __LINE__);
	    av_register_all();
//	printf("line # = %d\n", __LINE__);
	    avformat_network_init();
		  // ffmpeg util specific
//	printf("line # = %d\n", __LINE__);
	     AVInputFormat *fmt=av_find_input_format("mpegts");
	    if (!fmt) {
	        puts("Unknown input format: mpegts, bail out");
	        goto failed;
	    }
//	printf("line # = %d\n", __LINE__);
	    ic = avformat_alloc_context();
	    if (!ic) {
	        puts("Could not allocate context, bail out");
	        goto failed;
	    }
//	printf("line # = %d\n", __LINE__);
	    ic->interrupt_callback.callback = decode_interrupt_cb;
//	printf("line # = %d\n", __LINE__);
		while (!stream_on) short_sleep(0.1); // wait for video ts stre
//	printf("line # = %d\n", __LINE__);
	printf("%s\n","udp://@"
	    								UDPOUT_ADDR
	    								":"
	    								STRINGFY(UDPOUT_PORT_FF)
	    								"&buffer_size=200000");
	    int r = avformat_open_input(&ic, "udp://@"
	    									UDPOUT_ADDR
	    									":"
	    									STRINGFY(UDPOUT_PORT_FF)
	    									"&buffer_size=200000",
	    									fmt,
	    									NULL);
	    if (r < 0) {
	        printf("Could not open input stream, error = %d\n", r);
	        goto failed;
	    }
//	printf("line # = %d\n", __LINE__);
		r = avformat_find_stream_info(ic, NULL);
	    if (r < 0) {
	        printf("Could not find stream info, error = %d\n", r);
	        goto failed;
	    }
//	printf("line # = %d\n", __LINE__);
		if (ic->pb)
	   	ic->pb->eof_reached = 0;
//	printf("line # = %d\n", __LINE__);
	   st_index[AVMEDIA_TYPE_VIDEO] =
	         av_find_best_stream(ic, AVMEDIA_TYPE_VIDEO,
	                           0/*signle stream expected*/, -1, NULL, 0);
	   AVCodecContext *avctx = 0;
		AVCodec *codec= 0;
		avctx = ic->streams[st_index[AVMEDIA_TYPE_VIDEO]]->codec;
//	printf("line # = %d\n", __LINE__);
		codec = avcodec_find_decoder(avctx->codec_id);
//	printf("line # = %d\n", __LINE__);
		avctx->codec_id = codec->id;
//	printf("line # = %d\n", __LINE__);
		if ((r = avcodec_open2(avctx, codec, NULL)) < 0) {
	        printf("Could not open codec, error = %d\n", r);
	        goto failed;
		}
		ic->streams[st_index[AVMEDIA_TYPE_VIDEO]]->discard = AVDISCARD_DEFAULT;
  		AVPacket pkt1={0},*pkt= &pkt1 ;
//	printf("line # = %d\n", __LINE__);
    	AVFrame *frm = 0;
    	frm = av_frame_alloc();
//	printf("line # = %d\n", __LINE__);
    	int got_frame =0, fper= 0;
    	uint32_t vfcnt =1;
		get_time(&tstart);
		while (1 != do_exit_m && !av_read_frame(ic, pkt) &&pkt->size) {
//	printf("line # = %d\n", __LINE__);
         if (0>avcodec_decode_video2(avctx, frm, &got_frame, pkt))
	         {  // decoding errors or reach EOF
	            puts("H264 decoder failed...");
	            /*break ;*/
	         }
	      else if (got_frame) {
		      if (VF_REPO_DUR < fper++) {
		      	printf("...... %u frames decoded ......\n", vfcnt);
		      	fper = 0;
	      	}
	      	vfcnt = vfcnt + 1;
	      }
			get_time(&tend);
			time_diff(&tend, &tstart, &tdelta);
			if (compare_time(&tdelta,&loop)>0) {
				lgdst_vid_stats_rx();
				tstart = tend;
			}
		}
failed:
//	printf("line # = %d\n", __LINE__);
		avcodec_close(avctx);
//	printf("line # = %d\n", __LINE__);
  		avcodec_free_frame(&frm);
//	printf("line # = %d\n", __LINE__);
		av_free_packet(pkt);
//	printf("line # = %d\n", __LINE__);
		return;
  }
#endif
  static void *vid_recv_rx(void *arg) {
		udpout_init(UDPOUT_ADDR) ;
      int r = FRAME_SIZE_A;
		while (1 != do_exit_m) {
			lgdst_ts_rx(tsbuf);
      {
	      int frag, sentsize;
        unsigned char *pb = tsbuf;
				for (frag=0; frag<5; frag++) {
					sentsize=sendto(udpout_socket, pb, r/5,0,(struct sockaddr *)&udpout,
	                  udpout_len);
#ifdef VIDEO_STATS
					sentsize=sendto(udpout_socket_ff, pb, r/5,0,(struct sockaddr *)&udpout_ff,
	                  udpout_len);
#endif
	        if (sentsize < 0) printf("send pack ERorr\n");
	      		pb += r/5;
   		  }
      }
#ifdef VIDEO_STATS
		stream_on = true;
#endif
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
  int r, n, res ;
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
#ifdef VIDEO_STATS
		r = pthread_create(&thread_vid_rx, NULL, vid_frmc_rx, NULL);
		if (0 != r) {
			perror("video frame counting thread creation error");
			goto end;
		}
#endif
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
#ifdef VIDEO_STATS
//	printf("line # = %d\n", __LINE__);
		pthread_join(thread_vid_rx, NULL);
      // shutdown decoders
 #ifdef _AUDIO_DECODE_
 		avcodec_close(dec_aud);
 #endif
//	printf("line # = %d\n", __LINE__);
      avcodec_close(dec);
      // free all resources prior to shutdown
//	printf("line # = %d\n", __LINE__);
		avformat_close_input(&ic);
//	printf("line # = %d\n", __LINE__);
      avformat_network_deinit();
#endif
//	printf("line # = %d\n", __LINE__);
		pthread_join(thread_lgdst_rx, NULL);
		lgdst_deinit_rx(res);
		// prompt user at the end
		puts("All user threads returned, exit main()...");
	}
	return 0;
}


