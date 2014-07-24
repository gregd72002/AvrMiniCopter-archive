#include "flightlog.h"
#include <string.h>
#include <stdio.h>
#include "config.h"

struct s_flog flog[MAX_LOG];

static FILE *f;
static int buf_overrun;
static int m_val;
static char p[128];
static const char *_path;
static struct s_flog *ptr;

int flog_reset() {
	sprintf(p,"%s/flight-%i.t%i.log\0",_path,config.log_seq,config.log_t);
	config.log_seq++;
	ptr = flog;
	buf_overrun = 0;
	m_val = 0;
/*
	f = fopen(p,"w");
	if (f == NULL) {
		printf("Error creating flight log file!\n");
                return -1;
	}
	fflush(NULL);
	fclose(f);
*/
	return 0;
}

int flog_open(const char *path) {
	_path = path;
	return flog_reset();
}

int flog_push(int n, ...) {
	va_list ap;
	va_start(ap,n);
	
	if (n>=MAX_VALS) n = MAX_VALS;
	m_val = m_val<n?n:m_val;

	for (int i=0;i<n;i++) {
		ptr->v[i]  = va_arg(ap,double);
	}

	va_end(ap);

	ptr++;

	if (ptr == &flog[MAX_LOG]) { 
		ptr = flog; //buffer overrun -> start from the beginning
		buf_overrun = 0;
	}

	return 0;
}

int flog_save() {
	struct s_flog *i;
	int j;
	if ((ptr==flog) && !buf_overrun) return 0; //nothing to write
	f = fopen(p,"w");
	if (f == NULL) {
		printf("Error opening flight log file!\n");
                return -1;
	}
	
	if (buf_overrun) { //save the end of buffer before saving the beginning
		for (i=ptr;i!=&flog[MAX_LOG];i++) {
			for (j=0;j<m_val;j++)
				fprintf(f,"%2.4f\t",i->v[j]);
			fprintf(f,"\n");
		}
	}

	for (i=flog;i!=ptr;i++) {
		for (j=0;j<m_val;j++)
			fprintf(f,"%2.4f\t",i->v[j]);
		fprintf(f,"\n");
	}

	ptr = flog;
	buf_overrun = 0;
	
	fflush(NULL);

	fclose(f);

	flog_reset(); //prepare next log file

	return 0;
}
	


