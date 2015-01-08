#ifndef ROUTINES_H
#define ROUTINES_H

#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

/*
** packi16() -- store a 16-bit int into a char buffer (like htons())
*/ 
void packi16(unsigned char *buf, unsigned int i);

/*
** unpacki16() -- unpack a 16-bit int from a char buffer (like ntohs())
*/ 
unsigned int unpacki16(unsigned char *buf);


#define packu16(b,i) (packi16(b,i))

unsigned int unpacku16(unsigned char *buf);

void mssleep(unsigned int ms); 

struct timespec *TimeSpecDiff(struct timespec *ts1, struct timespec *ts2);



#define AVR_MSG_SIZE 3
struct avr_msg {
        uint8_t t; //type
        int16_t v; //value
};

#define LOCAL_MSG_SIZE 4
struct local_msg {
        uint8_t c; //control
        uint8_t t; //type
        int16_t v; //value
};

void pack_lm(unsigned char *buf,struct local_msg *m); 
void unpack_lm(unsigned char *buf,struct local_msg *m);

void local2avr(struct local_msg *lm, struct avr_msg *am);
void avr2local(struct avr_msg *am, struct local_msg *lm);

#endif
