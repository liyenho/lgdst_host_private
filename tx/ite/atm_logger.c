#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <signal.h>
#include <fcntl.h>  // for pipe creation
#include <stdlib.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <assert.h>

  #define SHMKEY_LOG	7654
  typedef int bool;

  #define ALL_STD_FDS   3
  //static int pipes[ALL_STD_FDS][2];
  typedef struct {
	  int pipes[ALL_STD_FDS][2];
	  bool active ;
  } IPC_LOG;
	IPC_LOG *ipc_mem = NULL;

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

int do_exit = 0; // termination if do_exit = 1

void sigint_handler(int signum)
{
	puts("exiting atm_debugger...");
	do_exit = 1;
}

void main(int argc, char **argv)
{
	if (SIG_ERR == signal(SIGINT, sigint_handler)) {
		perror("FAIL: assigning signal handler");
		exit (1);
	}
	int shmid_Logger = -1;
	while (!do_exit && 0>=shmid_Logger) {
		puts("Getting IPC memory...");
	  	shmid_Logger = shmget(SHMKEY_LOG, sizeof(IPC_LOG), 0666);
  }
  if (0<shmid_Logger) {
	  puts("Got IPC memory...");
	  	if (0>(ipc_mem=(IPC_LOG*)shmat(shmid_Logger, NULL, 0))) {
			perror("get IPC_LOG shmem failed");
		  	exit (2);
		}
		while (!ipc_mem->active) ; // wait for dbg server to come up
	}
	// began debugger specific init...
	struct timeval time_out= { .tv_sec = 0, // 10 msec timeout
															.tv_usec = 10000};
   char buffer[48], *pb0, *pb;
   int tcount= sizeof(buffer), count;
   fd_set rfds;
   pb0 = buffer;
	if (ipc_mem)
		printf("entering atm_debugger, client read pipe fd = %d\n",CHILD_READ_FD);
   while (!do_exit) {
	    FD_ZERO(&rfds);
	    FD_SET(CHILD_READ_FD, &rfds);
      while(!do_exit && 0>select(CHILD_READ_FD+1, &rfds, NULL, NULL, &time_out));
      if (FD_ISSET(CHILD_READ_FD, &rfds)) {
        count = read(CHILD_READ_FD, pb0, tcount);
         if (count>0) {
	         pb = pb0 + count;
	         pb0 = buffer;
	         for (; pb0<pb; pb0++)
	         	fputc(*pb0, stderr);
	         pb0 = buffer;
         }
      }
   }
	if (0<shmid_Logger) {
	   if (CHILD_READ_FD) {
	   	close(CHILD_READ_FD);
   	}
	  	shmdt(shmid_Logger);
  	}
}
