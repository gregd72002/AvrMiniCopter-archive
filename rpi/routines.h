#ifndef ROUTINES_H
#define ROUTINES_H

#include <time.h>
#include <stdlib.h>
#include <stdio.h>

/*
** packi16() -- store a 16-bit int into a char buffer (like htons())
*/ 
void packi16(unsigned char *buf, unsigned int i);

/*
** unpacki16() -- unpack a 16-bit int from a char buffer (like ntohs())
*/ 
unsigned int unpacki16(unsigned char *buf);

void mssleep(unsigned int ms); 

struct timespec *TimeSpecDiff(struct timespec *ts1, struct timespec *ts2);

#endif
