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
  struct timespec tim, tim2;
   tim.tv_sec = ms/1000;
   tim.tv_nsec = 1000000L * (ms % 1000);
   if(nanosleep(&tim , &tim2) < 0 )
   {
      printf("Nano sleep system call failed \n");
   }
}
