#ifdef LGDST_RX_H
#else
#define LGDST_RX_H

#include <stdint.h>
#include <stdbool.h>
void sigint_handler_rx(int signum);
#if defined(PES_HDR_PROT) ||defined(PES_FRM_PROT) ||defined(PES_FRM_PROT1)
  void lgdst_ts_rx(uint8_t *tsbuf0, uint32_t *size);
#else
  void lgdst_ts_rx(uint8_t *tsbuf0);
#endif
void lgdst_ctl_rec_rx(unsigned char *rpacket);
void lgdst_ctl_snd_rx(unsigned char *tpacket);
int lgdst_access_rx_ctl(int argc,char **argv, void **ret);
int lgdst_access_rx_vid(int argc,char **argv, void **ret);
#if (/*1*/0)
  int lgdst_init_rx();
#else
  int lgdst_init_rx(int argc, char **argv);
#endif
void lgdst_deinit_rx(int rcode);
/* internal function, do not invoke directly but using command line */
bool lgdst_upgrade_rx(int argc, char **argv);  // return -1 when failed, liyenho

#endif  //LGDST_RX_H
