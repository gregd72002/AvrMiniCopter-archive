#include "flightlog.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "routines.h"

struct s_flog flog[MAX_LOG];
static struct s_flog *ptr;
static int throttle = 0;
static unsigned long flight_time;

static char c_log_f[128];
static int log[256];

static int buf_overrun;
static int m_val;
static int log_mode = 0, log_count;
static const char *_path;

void flog_cfg_save();
void flog_reset();

void flog_init(const char *path) {
	_path = path;
	char p[128];
	sprintf(p,"%sflog.config%c",path,'\0');
	int state = 0;
	FILE *f = fopen(p,"r");
	if (f == NULL) state = 1;

	if (state == 0) {
		if (fscanf(f,"%i\t%i\n",&log_mode,&log_count)!=2) state = 1;
	}

	if (state) {
		printf("FLOG config error. New one will be created.\n");
		if (f) fclose(f);
		log_mode = 4;
		log_count = 0;
		flog_cfg_save();
	}

	flog_reset();
}

void flog_cfg_save() {
	char p[128];
	sprintf(p,"%sflog.config%c",_path,'\0');
	if (verbose) printf("AVRSPI: Saving flog cfg: %s\n",p);
	FILE *f = fopen(p,"w");
	if (f == NULL) {
		perror("Error saving FLOG CFG!\n");
		return;
	}
	fprintf(f,"%i\t%i\n",log_mode,log_count);
	fclose(f);
	fflush(NULL);
}

int flog_getmode() {
	return log_mode;
}

void flog_reset() {
	sprintf(c_log_f,"/rpicopter/log/flight-%05d.t%i.log%c",log_count,log_mode,'\0');
	if (verbose) printf("AVRSPI: Next log file: %s\n",c_log_f);
	ptr = flog;
	buf_overrun = 0;
	m_val = 0;
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


void log_altitude() {
	flog_push(4, 
			(float)flight_time
			,log[18]/1.f,log[19]/1.f,log[20]/1.f
		 );
}

void log_altitude_print() {
	printf("T: %li\ttarget_alt: %i\talt: %i\tvz: %i\tp_accel: %i\n",
			flight_time,log[18],log[19],log[20],log[21]);
}

void log100_print() {
	printf("T: %li\tvz: %i\tpos_err: %i\taccel_err: %i\tpid_alt: %i\tpid_vz: %i\tpid_accel: %i\n",
			flight_time,log[100],log[101],log[102],log[103],log[104],log[105]);
}

void log_quat() {
	flog_push(9, 
			(float)flight_time
			,log[4]/100.0f,log[5]/100.0f,log[6]/100.0f,log[7]/100.0f
			,log[8]/1.f,log[9]/1.f,log[10]/1.f
			,log[11]/1.f
		 );
}

void log_quat_print() {
	printf("T: %li\tfl: %i\tbl: %i\tfr: %i\tbr: %i\tqy: %f\tqp: %f\tqr: %f\tyt: %f\ty: %f\n",
			flight_time,log[8],log[9],log[10],log[11],
			log[4]/100.0f,log[5]/100.0f,log[6]/100.0f,log[7]/100.0f,(log[7]-log[4])/100.0f
	      );
	printf("T: %li\tfl: %i\tbl: %i\tfr: %i\tbr: %i\n",
			flight_time,log[10],log[11],log[12],log[13]);
}
void log_gyro() { //gyro & quat
	flog_push(8, 
			(float)flight_time
			,log[1]/100.0f,log[2]/100.0f,log[3]/100.0f
			,log[8]/1.f,log[9]/1.f,log[10]/1.f
			,log[11]/1.f
		 );
}

void log_gyro_print() {
	printf("T: %li\tgy: %2.2f\tgp: %2.2f\tgr: %2.2f\tfl: %i\tbl: %i\tfr: %i\tbr: %i\n",
			flight_time,log[1]/100.0f,log[2]/100.0f,log[3]/100.0f,log[8],log[9],log[10],log[11]);
}

void log_accel() {
	flog_push(7, 
			(float)flight_time
			,log[12]/1000.0f,log[13]/1000.0f,log[14]/1000.0f
			,log[15]/1000.0f,log[16]/1000.0f,log[17]/1000.0f
		 );
}

void log_accel_print() {
	printf("T: %li\tax: %2.3f\t\ay: %2.3f\t\az: %2.3f\tbx: %2.3f\tby: %2.3f\tbz: %2.3f\n",
			flight_time,log[12]/1000.0f,log[13]/1000.0f,log[14]/1000.0f,log[15]/1000.0f,log[16]/1000.0f,log[17]/1000.0f);
}


void flog_loop() {
	static unsigned long prev_ts = 0;
	flight_time += 25; //flog_loop is executed every 25ms

	if (flight_time - prev_ts < 50) return; //but our log resolution will be 50ms
	prev_ts = flight_time;

	switch(log_mode) {
		case 0: return; break;
		case 1: 
			log_accel(); 
			if (verbose==2) log_accel_print();
			break;
		case 2: 
			log_gyro(); 
			if (verbose==2) log_gyro_print();
			break;
		case 3: 
			log_quat(); 
			if (verbose==2) log_quat_print();
			break;
		case 4:
			log_altitude();
			if (verbose==2) log_altitude_print();
			break;
		case 100:
			if (verbose==2) log100_print();
			break;
		default: break;
	}
}

void flog_process_msg(struct avr_msg *m) {
	switch (m->t) {
		case 2: log_mode = m->v; flog_reset(); break;
		case 13: throttle = m->v; break;
	}
}

void flog_process_avrmsg(struct avr_msg *m) {
	log[m->t] = m->v;
}

int flog_save() {
	struct s_flog *i;
	int j;
	if ((ptr==flog) && !buf_overrun) return 0; //nothing to write
	FILE *f = fopen(c_log_f,"w");
	if (f == NULL) {
		printf("Error saving flight log file!\n");
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
	if (verbose) printf("AVRSPI: Flight log saved.\n");

	flog_reset(); //prepare next log file
	log_count++;

	flog_cfg_save();


	return 0;
}

