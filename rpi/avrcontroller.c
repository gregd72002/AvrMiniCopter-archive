#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <stdlib.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <getopt.h>

#include "config.h"
#include "ps3controller.h"
#include "flightlog.h"
#include "bmpsensor/interface.h"
#include "mpu.h"

#include "routines.h"
#include "msg.h"

char sock_path[256] = "/tmp/avrspi.socket";

int ret;
int err = 0;
int stop = 0;
int bs_err = 0;

int avr_s[256];

unsigned long flight_time = 0;

int trim[3] = {0,0,0};//in degrees * 1000
int mode = 0;
int rec_setting = 0;

int alt_hold = 0;
int throttle_hold = 0;
int throttle_target = 0;

int sock;
struct sockaddr_un address;

int verbose = 0;

int sendMsg(int t, int v) {
	static unsigned char buf[3];
	buf[0] = t;
	packi16(buf+1,v);
	if (write(sock, buf, 3) < 0) {
		perror("writing");
		return -1;
	}
	return 0;
}

void recvMsg() {
	static int i=0,ret=0;
	static unsigned char buf[3];
	static struct s_msg m;
	do {
		ret = read(sock,buf+i,3-i);
		if (ret>0) {
			i+=ret;
			if (i==3) {
				m.t = buf[0];
				m.v = unpacki16(buf+1);
				avr_s[m.t] = m.v;
				if (verbose) printf("Received t: %u v: %i\n",m.t,m.v);
				i = 0;
			}
		}
	} while (ret>0);
}

void sendTrims() {
	for (int i=0;i<3;i++)
		sendMsg(20+i,trim[i]);
}



void sendConfig() {

	int config_count = 56;
	sendMsg(1,config_count);

	sendMsg(2,config.log_t);

	sendMsg(3,mode);

	int gyro_orientation = inv_orientation_matrix_to_scalar(config.gyro_orient);
	sendMsg(4,gyro_orientation);

	sendMsg(9,config.mpu_addr);

	sendMsg(17,config.rec_t[0]);
	sendMsg(18,config.rec_t[2]);
	//6
	for (int i=0;i<4;i++) 
		sendMsg(5+i,config.motor_pin[i]);
	//10
	//PIDS
	for (int i=0;i<3;i++) 
		for (int j=0;j<5;j++) {
			sendMsg(100+i*10+j,config.r_pid[i][j]);
			sendMsg(200+i*10+j,config.s_pid[i][j]);
		}
	//40
	for (int i=0;i<5;i++) { 
		sendMsg(70+i,config.accel_pid[i]);
		sendMsg(80+i,config.alt_pid[i]);
		sendMsg(90+i,config.vz_pid[i]);
	}
	//55
	sendMsg(130,config.a_pid[0]);
	//56
	//END PIDS
	int config_done = 1;
	sendMsg(255,config_done);
	mssleep(1000);

}

void checkPIDs() {
	for (int i=0;i<3;i++) {
		if (config.r_pid[i][2]<0) config.r_pid[i][2]=0;
		if (config.r_pid[i][3]<0) config.r_pid[i][3]=0;
		if (config.r_pid[i][4]<0) config.r_pid[i][4]=0;
		if (config.s_pid[i][2]<0) config.s_pid[i][2]=0;
		if (config.s_pid[i][3]<0) config.s_pid[i][3]=0;
		if (config.s_pid[i][4]<0) config.s_pid[i][4]=0;
	}

}

void do_adjustments() {
	if (rec.aux<0) return;

	static int adj3 = 1; //for trim
	static int adj4 = 100; //for altitude (mm)

	static char str[128];

	switch (rec.aux) {
		case 8: //L2
			memset(str, '\0', 128);
			sprintf(str, "/usr/local/bin/vidsnap.sh %05d ", config.cam_seq++);
			ret=system(str);
			break;
		case 10: //L1
			//take picture
			memset(str, '\0', 128);
			sprintf(str, "/usr/local/bin/picsnap.sh %05d ", config.cam_seq++);
			ret=system(str);
			break;
		case 11: //R1
			if (alt_hold) {
				sendMsg(16,adj4);
			}
			break;
		case 9: //R2
			if (alt_hold) {
				sendMsg(16,-adj4);
			}
			break;
		case 0:
			if (rec.yprt[3]<config.rec_t[2]) {flog_save(); config_save(); sync(); fflush(NULL);}
			break;
		case 3: 
			if (rec.yprt[3]<config.rec_t[2]) {
				stop=1;
				/*
				   reset_avr();
				   ret=config_open("/var/local/rpicopter.config");
				   mode = 0;
				   alt_hold = 0;
				   throttle_hold = 0;
				   sendConfig();
				   sendTrims();
				   bs_reset();
				 */
			}
			break;
		case 12:
			if (rec_setting) rec_setting = 0;
			else rec_setting = 1;
			rec_setSetting(rec_setting);
			break;
		case 1:
			alt_hold = 0;
			throttle_hold = 0;
			sendMsg(15,alt_hold);
			break;
		case 13:
			/*
			   if (throttle_hold) throttle_hold=0;
			   else throttle_hold = 1;
			   throttle_target = rec.yprt[3];
			 */
			break;
		case 14:
			if (alt_hold) alt_hold=0;
			else if (!bs_err) alt_hold = 1;
			sendMsg(15,alt_hold);
			break;
		case 4:
			trim[1]+=adj3;
			sendMsg(21,trim[1]);
			break;
		case 6:
			trim[1]-=adj3;
			sendMsg(21,trim[1]);
			break;
		case 7:
			trim[2]+=adj3;
			sendMsg(22,trim[2]);
			break;
		case 5:
			trim[2]-=adj3;
			sendMsg(22,trim[2]);
			break;
		case 16: //mode change
			//if (rec.yprt[3]<config.rec_t[2] && !alt_hold) {
			{
				mode++;
				if (mode==2) mode=0;
				int t = 1;
				sendMsg(1,t);
				sendMsg(3,mode);
			}
			//}
			break;
		default:
			printf("Unknown command %i\n",rec.aux);
	}
	if (rec.aux!=-1) printf("Button: %i\n",rec.aux);
	rec.aux=-1; //reset receiver command

} 

void reset_avr() {
	sendMsg(255,255);
	mssleep(1000);
}

void catch_signal(int sig)
{
	printf("signal: %i\n",sig);
	stop = 1;
}

long dt_ms = 0;
static struct timespec ts,t1,t2,*dt;

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

void log() {
	flog_push(6, 
			(float)t2.tv_sec-ts.tv_sec
			,bs.alt,bs.t
			,avr_s[10]/10.0f,avr_s[11]/100.0f,avr_s[12]/100.0f
		 );
#ifdef DEBUG
	//printf("T: %li\tA: %2.1f\tT: %2.1f\t\talt: %2.1f\t\tvz: %2.2f\t\the: %2.1f\n",t2.tv_sec-ts.tv_sec,bs.alt,bs.t,avr_s[10]/10.0f,avr_s[11]/100.0f,avr_s[12]/100.0f);
	printf("T: %li\tdp: %2.2f\tdr: %2.2f\t\tdip: %2.1f\tdir: %2.2f\n",
			t2.tv_sec-ts.tv_sec
			,avr_s[0x20]/100.0f,avr_s[0x21]/100.0f,avr_s[0x22]/100.0f,avr_s[0x23]/100.0f
	      );
#endif
}

void log5() {
	flog_push(5, 
			(float)t2.tv_sec-ts.tv_sec
			,(float)flight_time
			,avr_s[30]/1.f,avr_s[31]/1.f,avr_s[32]/1.f
		 );
}

void log5_print() {
	printf("T: %li\talt: %i\tv_est: %i\th_est: %i\n",
			flight_time,avr_s[30],avr_s[31],avr_s[32]);
}

void log3() {
	flog_push(6, 
			(float)t2.tv_sec-ts.tv_sec
			,(float)flight_time
			,avr_s[10]/1.f,avr_s[11]/1.f,avr_s[12]/1.f
			,avr_s[13]/1.f
		 );
}

void log3_print() {
	printf("T: %li\tfl: %i\tbl: %i\tfr: %i\tbr: %i\n",
			flight_time,avr_s[10],avr_s[11],avr_s[12],avr_s[13]);
}
void log2() { //gyro & quat
	flog_push(9, 
			(float)t2.tv_sec-ts.tv_sec
			,(float)flight_time
			,avr_s[1]/100.0f,avr_s[2]/100.0f,avr_s[3]/100.0f
			,avr_s[5]/100.0f,avr_s[6]/100.0f,avr_s[7]/100.0f,avr_s[8]/100.0f
		 );
}

void log2_print() {
	printf("T: %li\tgy: %2.2f\tgp: %2.2f\tgr: %2.2f\tqy: %2.2f\tqp: %2.2f\tqr: %2.2f\tyt: %2.2f\n",
			flight_time,avr_s[1]/100.0f,avr_s[2]/100.0f,avr_s[3]/100.0f,avr_s[5]/100.0f,avr_s[6]/100.0f,avr_s[7]/100.0f,avr_s[8]/100.0f);
}

void log1() {
	flog_push(8, 
			(float)t2.tv_sec-ts.tv_sec
			,(float)flight_time
			,avr_s[20]/1000.0f,avr_s[21]/1000.0f,avr_s[22]/1000.0f
			,avr_s[25]/1000.0f,avr_s[26]/1000.0f,avr_s[27]/1000.0f
		 );
}

void log1_print() {
	printf("T: %li\tax: %2.3f\t\ay: %2.3f\t\az: %2.3f\tbx: %2.3f\tby: %2.3f\tbz: %2.3f\n",
			flight_time,avr_s[20]/1000.0f,avr_s[21]/1000.0f,avr_s[22]/1000.0f,avr_s[25]/1000.0f,avr_s[26]/1000.0f,avr_s[27]/1000.0f);
}

void log4() {
	flog_push(11, 
			(float)t2.tv_sec-ts.tv_sec
			,(float)flight_time
			,avr_s[10]/1.f,avr_s[11]/1.f,avr_s[12]/1.f,avr_s[13]/1.f
			,avr_s[5]/100.0f,avr_s[6]/100.0f,avr_s[7]/100.0f,avr_s[8]/100.0f,(avr_s[8]-avr_s[5])/100.0f
		 );
}

void log4_print() {
	printf("T: %li\tfl: %i\tbl: %i\tfr: %i\tbr: %i\tqy: %f\tqp: %f\tqr: %f\tyt: %f\ty: %f\n",
			flight_time,avr_s[10],avr_s[11],avr_s[12],avr_s[13],
			avr_s[5]/100.0f,avr_s[6]/100.0f,avr_s[7]/100.0f,avr_s[8]/100.0f,(avr_s[8]-avr_s[5])/100.0f
	      );
}


#define MAX_SAFE_STACK (MAX_LOG*MAX_VALS + 64*1024)
void stack_prefault(void) {

	unsigned char dummy[MAX_SAFE_STACK];

	memset(dummy, 0, MAX_SAFE_STACK);
	return;
}

unsigned long c = 0,k = 0;

//#define NOCONTROLLER 1
void loop() {
	int prev_status = avr_s[255];
	int delay = 500;
	sendMsg(255,0); //query status
	recvMsg();
	mssleep(250);
	while (avr_s[255]!=5 && !stop) {
		sendMsg(255,0);
		recvMsg();
		mssleep(delay);
		if ((prev_status != avr_s[255]) && avr_s[255]!=-1) {
			printf("AVR initiation: %i/5\n",avr_s[255]);	
			prev_status = avr_s[255];
		}
		switch (avr_s[255]) {
			case -1: break;
			case 0: break;
			case 1: avr_s[255]=-1; sendConfig(); delay=250; break;
			case 2: break;
			case 3: break;
			case 4: delay=100; break;
			case 5: break;
			case 255: avr_s[255]=-1; reset_avr(); delay=500; break; 
			default: printf("Unknown AVR status %i\n",avr_s[255]); break;
		} 
	}
	bs_reset();
	clock_gettime(CLOCK_REALTIME,&t2);                                           
	ts = t1 = t2;

	while (1 && !err && !stop) {
#ifndef NOCONTROLLER
		ret = rec_update(); 
		// 0 - no update but read ok
		// 1 - update
		if (ret < 0) {
			err = 1;
			printf("Receiver reading error: [%s]\n",strerror(ret));
			return;
		}
#else
		ret = 0;
		rec.aux = -1;
		rec.yprt[0] = 0;
		rec.yprt[1] = 0;
		rec.yprt[2] = 0;
		rec.yprt[2] = 1000;
#endif

		if (alt_hold && abs(rec.yprt[3]) > (config.rec_t[1]-50)) {
			alt_hold = 0;
			throttle_hold = 0;
			sendMsg(15,alt_hold);
		}



		clock_gettime(CLOCK_REALTIME,&t2);                                           
		dt = TimeSpecDiff(&t2,&t1);
		dt_ms = dt->tv_sec*1000 + dt->tv_nsec/1000000;

		//if (ret == 0 && dt_ms<20) continue;
		if (dt_ms<15) continue; //4 packets normally take 50ms anyway, so it should not stop in here
		t1 = t2;

		if (alt_hold || rec.yprt[3]>config.rec_t[2])
			flight_time += dt_ms; 

		c++;
		if (!bs_err) {
			static float _a = bs.alt; 
			bs_err = bs_update(c*dt_ms);
			if (bs_err) {
				bs.alt = _a;
				printf("Disabling barometer. Fixing altitude at %2.1f\n",bs.alt);
			}
		} 

		switch(config.log_t) {
			case 0: break;
			case 1: 
				log1(); 
#ifdef DEBUG
				log1_print();
#endif
				break;
			case 2: 
				log2(); 
#ifdef DEBUG
				log2_print();
#endif
				break;
			case 3: 
				log3(); 
#ifdef DEBUG
				log3_print();
#endif
				break;
			case 4:
				log4();
#ifdef DEBUG
				log4_print();
#endif
				break;
			case 5:
				log5();
				log5_print();
				break;
			case 99:
				printf("SPI CRC errors: %i\n",avr_s[254]);
				break;
			case 100:
				printf("Accel pid: %i\n",avr_s[35]);
				break;
			default: break;
		}

		do_adjustments();

		if (throttle_hold) {
			rec.yprt[3] = throttle_target;
		}

		for (int i=0;i<4;i++) {
			sendMsg(10+i,rec.yprt[i]);
		}
		sendMsg(14,bs.alt*100); //in cm
		recvMsg();
	}
}

void print_usage() {
	printf("-v - verbose mode\n");
	printf("-s [path] - path to socket to use\n");
}

int main(int argc, char **argv) {

	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);

        int option;
        verbose = 0;
        while ((option = getopt(argc, argv,"vs:")) != -1) {
                switch (option)  {
                        case 'v': verbose=1; break;
                        case 's': strcpy(sock_path,optarg); break;
                        default:
                                print_usage();
                                return -1;
                }
        }

	struct sched_param param;


	param.sched_priority = 49;
	if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
		perror("sched_setscheduler failed");
		exit(-1);
	}

	if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
		perror("mlockall failed");
		exit(-2);
	}

	stack_prefault();

	for (int i=0;i<256;i++)
		avr_s[i] = 0;

	reset_avr();


	/* Create socket on which to send. */
	sock = socket(AF_UNIX, SOCK_STREAM, 0);
	if (sock < 0) {
		perror("opening socket");
		exit(1);
	}
	/* Construct name of socket to send to. */
	address.sun_family = AF_UNIX;
	strcpy(address.sun_path, sock_path);

        if (connect(sock, (struct sockaddr *) &address, sizeof(struct sockaddr_un)) < 0) {
                close(sock);
                perror("connecting socket");
                exit(1);
        }



	ret=config_open("/var/local/rpicopter.config");
	if (ret<0) {
		printf("Failed to initiate config! [%s]\n", strerror(ret));	
		return -1;
	}

	ret=flog_open("/rpicopter");                                                
	if (ret<0) {                                                                 
		printf("Failed to initiate log! [%s]\n", strerror(err));         
		return -1;                                                               
	}   

#ifndef NOCONTROLLER
	ret=rec_open();
	if (ret<0) {
		printf("Failed to initiate receiver! [%s]\n", strerror(ret));	
		return -1;
	}
#endif

	ret=bs_open();
	if (ret<0) {
		printf("Failed to initiate pressure sensor! [%s]\n", strerror(err));	
		return -1;
	}

	mssleep(100);
	printf("int size: %lu\nlong size: %lu\nfloat size: %lu\nyprt size: %lu\n",sizeof(int),sizeof(long),sizeof(float),sizeof(rec.yprt));


	printf("Starting main loop...\n");

	loop();
	printf("Closing.\n");
	return 0;
}
