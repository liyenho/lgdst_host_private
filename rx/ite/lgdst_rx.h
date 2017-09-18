#ifdef LGDST_RX_H
#else
#define LGDST_RX_H

#include <stdint.h>
#include <stdbool.h>
void sigint_handler_rx(int signum);
#if defined(PES_HDR_PROT) ||defined(PES_FRM_PROT) ||defined(PES_FRM_PROT1)
  void lgdst_ts_rx(uint8_t *tsbuf0, uint32_t *size);
#else
  void lgdst_ts_rx(uint8_t *tsbuf0, uint32_t *blks);
#endif
void lgdst_vid_stats_rx(long *, double *);  // poll video channel state, as well for sensitivity measure
void lgdst_ctl_rec_rx(unsigned char *rpacket);
void lgdst_ctl_snd_rx(unsigned char *tpacket);
int lgdst_access_rx(int argc,char **argv, void **ret);
#if (/*1*/0)
  int lgdst_init_rx();
#else
  int lgdst_init_rx(int argc, char **argv);
  int lgdst_init_vid_rx();
#endif
void lgdst_deinit_rx(int rcode);
/* internal function, do not invoke directly but using command line */
//bool lgdst_upgrade_rx(int argc, char **argv);  // return -1 when failed, liyenho

 #include "platform_it9137.h"
 //#define VIDEO_DUAL_BUFFER
 #ifdef VIDEO_DUAL_BUFFER
  /*376*8/(1500*10^-6) approximate 2 mb/s,
  	to be 1333.333 ts pkts, take 251920=1340*188 to accommodate 1880 blk */
 	#define ONE_SEC_WORTHY	251920  // liyenho
	#define SEQ_SCH_TOL					100
 #endif
#endif  //LGDST_RX_H
