#ifndef FLIGHTLOG_H
#define FLIGHTLOG_H

#include <stdarg.h>

//#define MAX_LOG 128000
#define MAX_LOG 24000
#define MAX_VALS 24 

struct s_flog {
	float v[MAX_VALS];
};

void flog_init(const char *path);

int flog_getmode();

int flog_push(int n, ...);

int flog_save();

void flog_cfg_save();

void flog_reset();

void flog_loop();

void flog_process_msg(struct avr_msg *m);

void flog_process_avrmsg(struct avr_msg *m);


#endif

