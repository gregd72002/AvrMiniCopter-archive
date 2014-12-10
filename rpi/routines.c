#include "routines.h"

void packi16(unsigned char *buf, unsigned int i)
{
    *buf++ = i>>8; *buf++ = i;
}

unsigned int unpacki16(unsigned char *buf)
{
    return (buf[0]<<8) | buf[1];
}

void mssleep(unsigned int ms) {
  struct timeval timeout;

  timeout.tv_sec = ms/1000;
  timeout.tv_usec = (ms % 1000)*1000L;
   
   if (select(0,NULL,NULL,NULL,&timeout)<0 )
   {
      printf("Select sleep system call failed \n");
   }
}
void mssleep1(unsigned int ms) {
  struct timespec tim, tim2;
   tim.tv_sec = ms/1000;
   tim.tv_nsec = 1000000L * (ms % 1000);
   
   if(nanosleep(&tim , &tim2) < 0 )
   {
      printf("Nano sleep system call failed \n");
   }
}

struct timespec *TimeSpecDiff(struct timespec *ts1, struct timespec *ts2)
{
        static struct timespec ts;
        ts.tv_sec = ts1->tv_sec - ts2->tv_sec;
        ts.tv_nsec = ts1->tv_nsec - ts2->tv_nsec;
        if (ts.tv_nsec < 0) {
                ts.tv_sec--;
                ts.tv_nsec += 1000000000;
        }
        return &ts;
}

