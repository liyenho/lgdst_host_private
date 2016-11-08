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



#define UDP_LOCAL_ADDR "127.0.0.1" //"192.168.3.16" //
#define UDP_LOCAL_PORT  5553
#define UDP_PACKET_MAX  1880
#define UDPOUT_FILE_TS   "cap160910_pid0x100.ts"

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
  while(1){
  
    sleepusec(1200);
    sendsize = fread(txbuff,1,sendpacketsize,udpoutfile);
   //printf("READ: sentsize = %d. \n",sendsize);

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
