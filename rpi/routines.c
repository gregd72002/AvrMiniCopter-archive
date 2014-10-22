#include "routines.h"

void mssleep(unsigned int ms) {
  struct timespec tim, tim2;
   tim.tv_sec = ms/1000;
   tim.tv_nsec = 1000000L * (ms % 1000);
   if(nanosleep(&tim , &tim2) < 0 )
   {
      printf("Nano sleep system call failed \n");
   }
}

