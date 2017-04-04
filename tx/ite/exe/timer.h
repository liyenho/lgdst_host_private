#ifndef  __ASIC_TIMER_H__
#define  __ASIC_TIMER_H__
int timer_init(void);


int timer_kickoff(int wakeperiod);


int timer_term(void);


static void wakeup_handler(int sig);


int get_time(struct timeval *time);


void print_time(struct timeval ttime);

int time_diff(struct timeval *time1, struct timeval *time2, struct timeval *diffTime);


int copy_time(struct timeval * timeSource, struct timeval *timeDist);

int compare_time(struct timeval *time1, struct timeval *time2);
#endif




