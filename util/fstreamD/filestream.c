#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>
#include <linux/soundcard.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <math.h>
#ifdef SLOW
  #include <assert.h>
#endif

#define UDP_LOCAL_ADDR "127.0.0.1" //"192.168.3.16" //
#define UDP_LOCAL_PORT  5553
#define UDP_PACKET_MAX  1880
//#define UDPOUT_FILE_TS   "cap160910_pid0x100.ts"

int udpout_len;
int udpout_socket;
struct timeval udpin_tv= {0};
struct sockaddr_in  udplocalout;

//int udpout_init(void);
void sleepusec(unsigned int sleepTime)
{
    struct timeval timeout;

    //timeout.tv_sec = 0;
    //timeout.tv_usec = (sleepTime % 1000000) * 1000000;
     timeout.tv_sec = sleepTime / 1000000;
    timeout.tv_usec = (sleepTime % 1000000);

    select(0, NULL, NULL, NULL, &timeout);
}

int udplocal_init(void)
{

  memset(&udplocalout,0,sizeof(udplocalout));
  udplocalout.sin_family = AF_INET;
  udplocalout.sin_port = htons(UDP_LOCAL_PORT);
  //udpout.sin_port = htons(shmem_videoIn->udp_port);
  udplocalout.sin_addr.s_addr = inet_addr(UDP_LOCAL_ADDR);
  udpout_len = sizeof(udplocalout);
  udpout_socket = socket(AF_INET, SOCK_DGRAM,0);
  if(udpout_socket <0)
  { printf("Fail to open udpout socket\n"); return -1; }
  return 0;
}
#ifdef SLOW
static int parse_pcr(long long *ppcr_high, int *ppcr_low, const unsigned char *packet)
{
    int afc, len, flags;
    const unsigned char *p;
    unsigned int v;

    afc = (packet[3] >> 4) & 3;
    if (afc <= 1)
        return -5;
    p   = packet + 4;
    len = p[0];
    p++;
    if (len == 0)
        return -5;
    flags = *p++;
    len--;
    if (!(flags & 0x10))
        return -5;
    if (len < 6)
        return -5;
    *((unsigned char*)&v+3) = *(p+0);
    *((unsigned char*)&v+2) = *(p+1);
    *((unsigned char*)&v+1) = *(p+2);
    *((unsigned char*)&v+0) = *(p+3);
    *ppcr_high = ((long long) v << 1) | (p[4] >> 7);
    *ppcr_low  = ((int)(p[4] & 1) << 8) | p[5];
    return 0;
}

static long long parse_pes_pts(const unsigned char *buf) {
	unsigned short v1, v2;
	#define RB16(t,p) *((unsigned char*)&t+1) = *(p+0); \
    										*((unsigned char*)&t+0) = *(p+1);
	RB16(v1, buf+1)	// value in v1
	RB16(v2, buf+3)	// value in v2

    return ((long long)(*buf & 0x0e) << 29) |
            ((long long)(v1 >> 1) << 15) | ((long long)v2 >> 1);
	#undef RB16
}
#endif
int main(void){
  int sendsize,sentsize;
  int sendpacketsize;
  static unsigned int cntfile = 0;
  char txbuff[UDP_PACKET_MAX];
  udplocal_init();
  FILE *udpoutfile = NULL;
  udpoutfile = fopen(UDPOUT_FILE_TS, "rb");
  sendpacketsize = 376;
  int i;
  #ifdef SLOW
  	int get_pcr_inc= 2, get_pts_inc= 2;
  	unsigned pkts=1, pktcnts[2], pcr_l, err;
  	long long pcrs[2]={0}, ptss[2];
  	long long pcr_h, tmp_l;
  #endif
  while(1){
  #ifndef SLOW
    sleepusec(1200); // produce 2.5 mb/s ts
  #else
    sleepusec(1500/*1800*/); // produce 2.0 mb/s ts
  #endif
    sendsize = fread(txbuff,1,sendpacketsize,udpoutfile);
   //printf("READ: sentsize = %d. \n",sendsize);
  #ifdef SLOW
  	// restamp pcr accordingly...
  	assert(0==(sendpacketsize % 188));
  	unsigned char *tspkt = txbuff, adpf;
  	unsigned  pid;
  	for (i=0; i<sendsize/188; tspkt+=188, i++) {
	  	pid =( ((unsigned)(tspkt[1]&0x1f))<<8) + tspkt[2];
	  	if ( PID_VID== pid) {
		#define calc_tstp_del(v,c,t) \
							tmp_l = (v<c)?v+t:v; \
							tmp_l -= c;
		  	adpf = tspkt[3]&0x30;
		  	if ((0x20&adpf) && (0x10&*(tspkt+5))) { //pcr existed
				static float pcr_incr;  // pcr step per packet
			  	if (get_pcr_inc) {
					err = parse_pcr(&pcr_h, &pcr_l, tspkt);
				  	pcrs[2-get_pcr_inc] = pcr_h * 300 + pcr_l;
				  	pktcnts[2-get_pcr_inc] = pkts;
				  	get_pcr_inc -= (err)?0 :1;
				  	if (!get_pcr_inc) {
						goto pcr_next;
					}
			  	}
			  	else {
				  	long long pcr_slow;
		pcr_next:			// # wrap detection
					parse_pcr(&pcr_h, &pcr_l, tspkt);
					pcr_slow = pcr_h * 300 + pcr_l;
					calc_tstp_del(pcr_slow, pcrs[0], 0x40000000000)
					 pcr_slow = (long long)(tmp_l*15/12) + pcrs[0];
					 pcr_l = pcr_slow % 300;
					 pcr_h = pcr_slow / 300;
					 unsigned tmp = (pcr_h>>1);
					*(tspkt+9) = *((unsigned char*)&tmp+0);
					*(tspkt+8) = *((unsigned char*)&tmp+1);
					*(tspkt+7) = *((unsigned char*)&tmp+2);
					*(tspkt+6) = *((unsigned char*)&tmp+3);
					*(tspkt+10) &= 0x7e;
						 *(tspkt+10) |= ((0x1&pcr_h)<<7) | ((0x100&pcr_l)>>8);
					*(tspkt+11) = (unsigned char)pcr_l;
			  	}
		  	}
		  	if (0x40&*(tspkt+1)) { // pts existed
			  	assert(0x30==(0x30&adpf));
			  	int index = (5) + tspkt[4];
			  	adpf = tspkt[index+7];  // pes hdr flags
			  	pcr_h = (pcrs[0]+150)/300;
			  	index = index + 9 ; //pos of pts in pes payload
			  	switch(get_pts_inc) {
				  	case 2: ptss[2-get_pts_inc] = parse_pes_pts(tspkt+index); //pts
				  					if (pktcnts[0] != pkts) {
					  					assert(0 != pcrs[0]);
		#define assign_pts(p,v,f) \
								tmp_l = (v); \
								*(p) = (unsigned char)((tmp_l>>30)<<1)|((f)<<4)|1; \
								*(p+1) = (unsigned char)(tmp_l>>22); \
								*(p+2) = (unsigned char)((tmp_l>>15)<<1)|1; \
								*(p+3) = (unsigned char)(tmp_l>>7); \
								*(p+4) = (unsigned char)(tmp_l<<1)|1;
										calc_tstp_del(ptss[0], pcr_h, 0x200000000)
					  					assign_pts(tspkt+index,
					  											(tmp_l*15/12)+pcr_h,
					  											(0xc0==(0xc0&adpf))?0x3:0x2)
				  						get_pts_inc -= 1;
				  					}
				  					break;
				  	case 1:	if (0x40&adpf) { // dts
				  						ptss[2-get_pts_inc] = parse_pes_pts(tspkt+index+5);
					  					if (pktcnts[0] != pkts) {
						  					assert(0 != pcrs[0]);
						  					calc_tstp_del(ptss[1], pcr_h, 0x200000000)
						  					assign_pts(tspkt+index+5,
						  											(tmp_l*15/12)+pcr_h,
						  											0x1)
				  							get_pts_inc -= 1;
					  					}
			  						}
				  					break;
				  	default: break;
			  	}
			  	if (2>get_pts_inc) {
				  	long long pts_slow;
					ptss[0]=parse_pes_pts(tspkt+index); //pts
					calc_tstp_del(ptss[0], pcr_h, 0x200000000)
					 pts_slow = (tmp_l*15/12)+pcr_h;
//printf("pts : %lld\n", pts_slow);
					 assign_pts(tspkt+index,
					  						pts_slow,
					  						(0xc0==0xc0&adpf)?0x3:0x2)
					 if (!get_pts_inc && (0x40&adpf)) {
						ptss[1]=parse_pes_pts(tspkt+index+5); //pts
						calc_tstp_del(ptss[1], pcr_h, 0x200000000)
						 pts_slow = (tmp_l*15/12)+pcr_h;
//printf("dts : %lld\n", pts_slow);
						 assign_pts(tspkt+index+5,
						  						pts_slow,
						  						0x1)
					 }
			  	}
		#undef assign_pts
		#undef calc_tstp_del
		  	}
	  	}
	  	pkts = pkts+1; // track # of ts pkt
  	}
  #endif
    if(sendsize != sendpacketsize){
    // reached EOF, reset file
      fclose(udpoutfile);
      udpoutfile = fopen(UDPOUT_FILE_TS,"rb");
      printf("READ: UDPOUT_FILE_TS = %s. count = %d\n",UDPOUT_FILE_TS, cntfile++);
    }

    sentsize=sendto(udpout_socket, txbuff,sendsize,0,(struct sockaddr *)&udplocalout,
                  udpout_len);
  //sndcnt++;
  //if(sndcnt%500 == 0)
    //ERRC "Sendin IP (%d)\n", sndcnt/500 ERRD
  //ERRC "packet sent(Size=%d)\n",sentsize ERRD
    //printf("SENT: sentsize = %d. \n",sentsize);
    if(sentsize < 0)
      printf("UDP send FAIL: sentsize < 1. \n");
   }//end wile
   fclose(udpoutfile);
   close(udpout_socket);
   printf("i = %d done!!!!\n",i);
  return 1;
}//end main
