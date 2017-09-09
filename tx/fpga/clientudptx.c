#include <stdio.h>      /* for printf() and fprintf() */
#include <sys/socket.h> /* for socket(), connect(), send(), and recv() */
#include <arpa/inet.h>  /* for sockaddr_in and inet_addr() */
#include <stdlib.h>     /* for atoi() and exit() */
#include <string.h>     /* for memset() */
#include <unistd.h>     /* for close() */
#include <signal.h>

#define DEBUG 1
#define PACKET_LENGTH	15
#define RCVBUFSIZE PACKET_LENGTH   /* Size of receive buffer */
#define SNDBUFSIZE PACKET_LENGTH   /* Size of receive buffer */
unsigned int do_exit = 0;

static void sigint_handler(int signum)
{
   do_exit = 1; // this shall terminate poll thread
}

static void DieWithError(char *errorMessage)  /* Error handling function */
{
    perror(errorMessage);
    exit(1);
}

int main(int argc, char *argv[])
{
  // echo count data.  default no data case: send count every 2 seconds
  // rcv/snd definition: ctrlrcv= rcv :from simulated source and send to usb_tx
  //                     ctrlsnd= snd :from usb_tx and send to simulated destination
  int ctrlrcv_socket, ctrlsnd_socket,err,rcvsize,echocnt=0;  // for udp:
  struct sockaddr_in ctrlrcv, ctrlsnd;  //for udp
  fd_set ctrlsnd_fd;
  struct timeval ctrlsnd_tv;
  int ctrlsnd_len;
  unsigned short ctrlsnd_pt, ctrlrcv_pt;
  char *servIP;                    /* Server IP address (dotted quad) */
  static char txbuffer[RCVBUFSIZE] ={0};     /* Buffer for echo string */
  static char rxbuffer[SNDBUFSIZE] ={0};
  int i,j;
  int r,s;
  char rcvfifoempty=0;

  if (argc != 3)    /* Test for correct number of arguments */
  {
    fprintf(stderr, "Usage: %s <Server IP> < tx_Port (in)>\n",
               argv[0]);
    exit(1);
  }

  servIP = argv[1];        /* First arg: server IP address (dotted quad) */
  ctrlrcv_pt = atoi(argv[2]); /* Use given port, if any */
  ctrlsnd_pt = ctrlrcv_pt+1;
  printf("target ip=%s portTx=%d portRx=%d\n",servIP, ctrlrcv_pt, ctrlsnd_pt);

  /* Construct the server address structure */
  r = socket(AF_INET,SOCK_DGRAM,0);
  s = socket(AF_INET,SOCK_DGRAM,0);

  if((r==-1)||(s==-1))
    DieWithError("socket() failed");

  ctrlsnd.sin_family = AF_INET;
  ctrlsnd.sin_addr.s_addr = htonl(INADDR_ANY);
  ctrlsnd.sin_port = htons(ctrlsnd_pt); //ctrl_pt+1, for output
  ctrlsnd_socket = r;
  ctrlsnd_len = sizeof(ctrlsnd);
  if (bind(r, (struct sockaddr *) &ctrlsnd, sizeof(ctrlsnd)) < 0)
		DieWithError("ctrlsnd: bind() failed");
  ctrlrcv.sin_family = AF_INET;
  ctrlrcv.sin_addr.s_addr = inet_addr(servIP); //no bind() required
  ctrlrcv.sin_port = htons(ctrlrcv_pt); //ctrl_pt+1, for socket output port
  ctrlrcv_socket = s;

  if (SIG_ERR == signal(SIGINT, sigint_handler)) {
     perror("FAIL: assigning signal handler");
  }

  while (do_exit ==0) {
  /****************************************************************/
  /* RECEIVE Data Processing **************************************/
  /****************************************************************/
  rcvfifoempty=0;
  while(!rcvfifoempty)
  {
  ctrlsnd_tv.tv_sec = 0; //default send message
  ctrlsnd_tv.tv_usec = 0;
  FD_ZERO(&ctrlsnd_fd);
  FD_SET(ctrlsnd_socket,&ctrlsnd_fd);
  err = select(ctrlsnd_socket+1,&ctrlsnd_fd,0,0,&ctrlsnd_tv);
  if(err<0) {
    printf("Warning: ctrlsnd: select failed\n");
    return err;
  }
  rcvsize = 0;
  if (FD_ISSET(ctrlsnd_socket,&ctrlsnd_fd)) {
	  rcvsize= recvfrom(ctrlsnd_socket ,rxbuffer, sizeof(rxbuffer), 0,
         (struct sockaddr *)&ctrlsnd, &ctrlsnd_len);
  printf("                                    Received %d bytes:", rcvsize);
  for(i=0;i<rcvsize;i++)
    printf("%x ",(rxbuffer[i] & 0xff));
  printf("\n");
  if(DEBUG)
  {
    // reciver internal dataloss flag
    static unsigned char val_exp;
    unsigned char        val_cur;
    static unsigned char cnt_exp;
    unsigned char        cnt_cur;
    static int ilcnt=0;

    val_cur = rxbuffer[5];
    cnt_cur = rxbuffer[3];
    val_exp = (unsigned char)(val_exp+1);
    														// cnt_cur: 3rd byte of echocnt in ctrl_poll_send() from usb_tx./rx module,
    														// it doesn't truly meant anything like 'internal', just a counter mismatch
    														// not yet reach more than 65536!;-) non-sense on the prompt message;-( liyenho
    if((val_exp != val_cur)&&(cnt_cur == cnt_exp)) {
      ilcnt++;
      printf("Rec internal dataloss = %d. !!!!!!!!!!!!!!!!!!!!!!!!!!!!\n",ilcnt);
    }
    cnt_exp = cnt_cur;
    val_exp = val_cur;
  }
  }
  else rcvfifoempty=1;
  }

  /****************************************************************/
  /* SEND Data Processing **************************************/
  /****************************************************************/
  /* Send the string to the server */
  /* lossy connection: gated by pipe conditions */
  /* data sent only when a packet was recieved (controlled within usb_test.c */
	  txbuffer[0] = 0xb5;
	  txbuffer[1] = 0xb5;
	  txbuffer[2] = ((unsigned char *)&echocnt)[3];
	  txbuffer[3] = ((unsigned char *)&echocnt)[2];
	  txbuffer[4] = ((unsigned char *)&echocnt)[1];
	  txbuffer[5] = ((unsigned char *)&echocnt)[0];
     echocnt++;

  if (sendto(ctrlrcv_socket, txbuffer, sizeof(txbuffer), 0,
   	 		(struct sockaddr *)&ctrlrcv, sizeof(ctrlrcv)) != SNDBUFSIZE)
    DieWithError("send() sent a different number of bytes than expected");

  printf("Drone: Sent: ");
  for(i=0;i<SNDBUFSIZE;i++) printf("%x ", (txbuffer[i])&0xff);
  printf("\n");
  usleep(500000);
  } //while(do_exit)
  close(ctrlsnd_socket);
  close(ctrlrcv_socket);
}

