#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <unistd.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>

#define ERRA     fprintf(stderr,
#define ERRB     );

#define  WAKE_PERIOD_SEC     0         // 1= 1sec
#define  WAKE_PERIOD_USEC   31500      // 1000=1ms (on Monolith PC),

static void wakeup_handler(int sig);
struct timeval tstart,tend,tcurr,tdelta;
struct timeval tdeltamin_w,tdeltamax_w,tprev_w;
struct timeval tdeltamin_e,tdeltamax_e,tprev_e;
int tdeltamincnt_w,tdeltamaxcnt_w;
int tdeltamincnt_e,tdeltamaxcnt_e;
int proc_cnt,handle_cnt;

int timer_init(void)
{
  struct sched_param schedparam;
  pthread_t threadid;
  int routinereturn;
  struct itimerval oldtime, newtime;

   /* set time statistics */
  get_time(&tstart);
  //print_time(tstart);
  tprev_w.tv_sec = 0;
  tprev_w.tv_usec = 0;
  tdeltamin_w.tv_sec = 0;
  tdeltamin_w.tv_usec = 999999;
  tdeltamax_w.tv_sec = 0;
  tdeltamax_w.tv_usec = 0;
  tdeltamincnt_w=0; tdeltamaxcnt_w=0;
  tprev_e.tv_sec = 0;
  tprev_e.tv_usec = 0;
  tdeltamin_e.tv_sec = 0;
  tdeltamin_e.tv_usec = 999999;
  tdeltamax_e.tv_sec = 0;
  tdeltamax_e.tv_usec = 0;
  tdeltamincnt_e=0; tdeltamaxcnt_e=0;

  /* config wakeup timer priority */
  threadid = pthread_self();
  schedparam.sched_priority = 127; /* highest sch pri */
  routinereturn = pthread_setschedparam(threadid,SCHED_FIFO,&schedparam);

  /* initialize wakeup timer period */
  newtime.it_interval.tv_sec = 0;
  newtime.it_interval.tv_usec = 0;
  newtime.it_value.tv_sec = 0;
  newtime.it_value.tv_usec = 0;
  if(setitimer(ITIMER_REAL, &newtime,&oldtime)<0)
  { ERRA"itimer init failed...\n"ERRB  return -1; }

  return 0;
}

int timer_kickoff(int wakeperiod)
{
  struct itimerval oldtime, newtime;
    /* arm wakeup timer */
  signal(SIGALRM, wakeup_handler);
  newtime.it_interval.tv_sec = /*WAKE_PERIOD_SEC*/wakeperiod/1000000;
  newtime.it_interval.tv_usec = wakeperiod%1000000;
  newtime.it_value.tv_sec = /*WAKE_PERIOD_SEC*/wakeperiod/1000000;
  newtime.it_value.tv_usec = wakeperiod%1000000;
  if(setitimer(ITIMER_REAL, &newtime,&oldtime)<0)
  { ERRA"itimer arm failed...\n"ERRB  return -1; }
  return 0;
}

int timer_term(void)
{
  struct itimerval oldtime, newtime;
  newtime.it_interval.tv_sec = 0;
  newtime.it_interval.tv_usec = 0;
  newtime.it_value.tv_sec = 0;
  newtime.it_value.tv_usec = 0;
  if(setitimer(ITIMER_REAL, &newtime,&oldtime)<0)
  {  ERRA"itimer terminate failed...\n"ERRB return -1; }
  get_time(&tend);
  //ERRA"endtime: "ERRB print_time(tend);
  ERRA"STATUS:  deltamin wakeHandle: cnt=%d ",tdeltamincnt_w ERRB
  	print_time(tdeltamin_w);
  ERRA"STATUS:  deltamax wakeHandle: cnt=%d ",tdeltamaxcnt_w ERRB
  	print_time(tdeltamax_w);
  //ERRA"proc_cnt=%d, handle_cnt=%d\n",proc_cnt,handle_cnt ERRB

  return 0;
}

static void wakeup_handler(int sig)
{
  // monitor response time
  get_time(&tcurr);
  time_diff(&tcurr,&tstart,&tdelta);
  if((tprev_w.tv_usec != 0)||(tprev_w.tv_sec!=0))
  {
    time_diff(&tcurr,&tprev_w,&tdelta);
    //print_time(tdelta);
	if(compare_time(&tdelta,&tdeltamin_w)<0){
      copy_time(&tdelta,&tdeltamin_w);
      tdeltamincnt_w = handle_cnt;            }
  }
  copy_time(&tcurr,&tprev_w);
  handle_cnt++;
}

int get_time(struct timeval *time)
{
  /* this routine will return absolute time */
  /* delcarations */
  clockid_t ckid = CLOCK_MONOTONIC;
  struct timespec tm;

  clock_gettime(ckid, &tm);
  time->tv_sec = tm.tv_sec;
  time->tv_usec = (suseconds_t)(tm.tv_nsec/1000);
  return 0;
}

void print_time(struct timeval ttime)
{
  ERRA"time: s=%u us=%u\n",
    (unsigned int)ttime.tv_sec,(unsigned int)ttime.tv_usec ERRB
}

int time_diff(struct timeval *time1, struct timeval *time2,
                                                struct timeval *diffTime)
{
  /* This routine will put the result of time1 - time2 into diffTime */
  /* It always assumes that time1 > time2, and generate a positive difference */
  /* It will not generate signed numbers */
  /* Current implementation assumes no roller. Tests suggests that Linux */
  /* assumes an absolute starting time of Dec 31, 1969.  The tv_sec will then */
  /* hold a value of 1073342280 for Jan 5, 2004                         */
  long int carryOver;
  int returnVal=0;

  /* process usec parameter */
  if(time1->tv_usec >= time2->tv_usec)
  {
    diffTime->tv_usec = time1->tv_usec - time2->tv_usec;
    carryOver = 0;
  }
  else
  {
    diffTime->tv_usec = 1000000;
    diffTime->tv_usec -= time2->tv_usec;
    diffTime->tv_usec += time1->tv_usec;
    carryOver = 1;
  }

  /* process sec parameter */
  if((time1->tv_sec - carryOver)< time2->tv_sec)
  {
    /* time1< time2 detected, fail condition */
    diffTime->tv_sec = 0;
    returnVal = -1;
  }
  else
  {
    diffTime->tv_sec = time1->tv_sec - time2->tv_sec - carryOver;
  }
  return(returnVal);
}

int copy_time(struct timeval * timeSource, struct timeval *timeDist)
{
  /* copies values in timeSource to timeDist */
  timeDist->tv_usec = timeSource->tv_usec;
  timeDist->tv_sec = timeSource->tv_sec;
  return(0);
}

int compare_time(struct timeval *time1, struct timeval *time2)
{
  /* returns 1 if positive or the same, -1 for negative */
  /* assumes only positive numbers, even though struct timeval can hold neg */
  long int carryOver;

  if(time1->tv_usec < time2->tv_usec)
    carryOver = 1;
  else
    carryOver = 0;
  if((time1->tv_sec - carryOver) < time2->tv_sec)
    return(-1);
  else
    return(1);
}
