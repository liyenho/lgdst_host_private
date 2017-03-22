#include <stdint.h>
#include <stdbool.h>
void sigint_handler_tx(int signum);
void lgdst_ts_tx(uint8_t *tsbuf0);
void lgdst_ctl_rec_tx(unsigned char *rpacket);
void lgdst_ctl_snd_tx(unsigned char *tpacket);
#if (/*1*/0)
  int lgdst_init_tx();
#else
  int lgdst_init_tx(int argc, char **argv);
#endif
void lgdst_deinit_tx(int rcode);
/* internal function, do not invoke directly but using command line */
bool lgdst_upgrade_tx(int argc, char **argv);  // return -1 when failed, liyenho

