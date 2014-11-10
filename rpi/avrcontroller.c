#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdlib.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <getopt.h>

#include "config.h"
#include "ps3controller.h"
#include "flightlog.h"
#include "mpu.h"

#include "routines.h"
#include "msg.h"


int ret;
int err = 0;
int stop = 0;

int avr_s[256];

unsigned long flight_time = 0;

int trim[3] = {0,0,0};//in degrees * 1000
int mode = 0;
int rec_setting = 0;

int alt_hold = 0;
int throttle_hold = 0;
int throttle_target = 0;

int sock = 0;
char sock_path[256] = "127.0.0.1";
int portno = 1030;
struct sockaddr_in address;
struct hostent *server;

int verbose = 0;
int nocontroller = 0;

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

void recvMsgs() {
	static int sel=0,i=0,ret=0;
	static unsigned char buf[4];
	static struct s_msg m;

	static fd_set fds;
	static struct timeval timeout;

	do {
		FD_ZERO(&fds);
		FD_SET(sock,&fds);
		timeout.tv_sec = 0;
		timeout.tv_usec = 0;
		sel = select( sock + 1 , &fds , NULL , NULL , &timeout);
		if ((sel<0) && (errno!=EINTR)) {
			perror("select");
			stop=1;
		}
		else if (sel && !stop && FD_ISSET(sock, &fds)) {
			ret = read(sock,buf+i,4-i);
			if (ret<0) {
				perror("reading");
				stop = 1;
			}
			else {
				i+=ret;
				if (i==4) {
					if (buf[0] == 1) {
						if (verbose) printf("Disconnect request.\n");
						stop = 1;	
					} else {
						m.t = buf[1];
						m.v = unpacki16(buf+2);
						avr_s[m.t] = m.v;
						i = 0;
					}
				}
			}
		}
	} while (!stop && sel && ret>0); //no error happened; select picked up socket state change; read got some data back
}

void sendTrims() {
	for (int i=0;i<3;i++)
		sendMsg(20+i,trim[i]);
}



void sendConfig() {
	sendMsg(2,config.log_t);

	sendMsg(3,mode);

	int gyro_orientation = inv_orientation_matrix_to_scalar(config.gyro_orient);
	sendMsg(4,gyro_orientation);

	sendMsg(9,config.mpu_addr);

	sendMsg(17,config.rec_t[0]);
	sendMsg(18,config.rec_t[2]);

	for (int i=0;i<4;i++) 
		sendMsg(5+i,config.motor_pin[i]);

	sendMsg(69,config.baro_f);
	//PIDS
	for (int i=0;i<3;i++) 
		for (int j=0;j<5;j++) {
			sendMsg(100+i*10+j,config.r_pid[i][j]);
			sendMsg(200+i*10+j,config.s_pid[i][j]);
		}

	for (int i=0;i<5;i++) { 
		sendMsg(70+i,config.accel_pid[i]);
		sendMsg(80+i,config.alt_pid[i]);
		sendMsg(90+i,config.vz_pid[i]);
	}

	sendMsg(130,config.a_pid[0]);

	int config_done = 1;
	sendMsg(255,config_done);
	mssleep(1000);

}

void reset_avr() {
	sendMsg(255,255);
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
	static int adj4 = 25; //for altitude (mm)

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
			stop=1;
			reset_avr();
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
			else alt_hold = 1;
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

void catch_signal(int sig)
{
	printf("signal: %i\n",sig);
	stop = 1;
}

long dt_ms = 0;
static struct timespec ts,t1,t2,*dt;

void log5() {
	flog_push(5, 
			(float)t2.tv_sec-ts.tv_sec
			,(float)flight_time
			,avr_s[30]/1.f,avr_s[31]/1.f,avr_s[32]/1.f
		 );
}

void log5_print() {
	printf("T: %li\ttarget_alt: %i\talt: %i\tvz: %i\taccel_err: %2.2f\n",
			flight_time,avr_s[30],avr_s[31],avr_s[32],avr_s[33]/100.f);
}

void log100_print() {
	printf("T: %li\tvz: %i\tpos_err: %i\taccel_err: %i\tpid_alt: %i\tpid_vz: %i\tpid_accel: %i\n",
			flight_time,avr_s[100],avr_s[101],avr_s[102],avr_s[103],avr_s[104],avr_s[105]);
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


unsigned long k = 0;

void init() {
	//feeds all config and waits for calibration
	int prev_status = avr_s[255];
	avr_s[255] = 0;
	if (verbose) printf("Initializing RPiCopter...\n");
	while (avr_s[255]!=5 && !stop) {
		sendMsg(255,0); //query status
		sendMsg(255,1); //crc status
		mssleep(350);
		recvMsgs();
		if (prev_status!=avr_s[255]) {
			if (verbose) {
				if (avr_s[255]==-1) printf("Waiting for AVR status change.\n");
				else printf("AVR Status: %i\n",avr_s[255]);
			}
			prev_status = avr_s[255];
		}

		if (avr_s[254]!=0) { //AVR reported crc error when receiving data
			reset_avr();
			avr_s[254] = 0;
			avr_s[255] = 0;
		}
		else switch (avr_s[255]) {
			case -1: break;
			case 0: reset_avr(); break; //AVR should boot into status 1 so 0 means something wrong
			case 1: sendConfig(); mssleep(1000); avr_s[255] = -1; sendMsg(255,2); break;
			case 2: break; //AVR should arm motors and set status to 3
			case 3: break; //AVR is initializing MPU 
			case 4: break; //AVR is calibration gyro
			case 5: break;
			case 255: printf("Gyro calibration failed!\n"); reset_avr(); avr_s[255] = 0; break; //calibration failed
			default: printf("Unknown AVR status %i\n",avr_s[255]); break;
		}
	}
}

void loop() {
	clock_gettime(CLOCK_REALTIME,&t2);                                           
	ts = t1 = t2;
	if (verbose) printf("Starting main loop...\n");
	while (1 && !err && !stop) {
		if (!nocontroller) {
			ret = rec_update(); 
			// 0 - no update but read ok
			// 1 - update
			if (ret < 0) {
				err = 1;
				printf("Receiver reading error: [%s]\n",strerror(ret));
				return;
			}
		} else {
			ret = 0;
			rec.aux = -1;
			rec.yprt[0] = 0;
			rec.yprt[1] = 0;
			rec.yprt[2] = 0;
			rec.yprt[2] = 1000;
		}

		if (alt_hold && abs(rec.yprt[3]) > (config.rec_t[1]-50)) {
			alt_hold = 0;
			throttle_hold = 0;
			sendMsg(15,alt_hold);
		}



		clock_gettime(CLOCK_REALTIME,&t2);                                           
		dt = TimeSpecDiff(&t2,&t1);
		dt_ms = dt->tv_sec*1000 + dt->tv_nsec/1000000;

		//if (ret == 0 && dt_ms<20) continue;
		if (dt_ms<50) {
			mssleep(50-dt_ms);
			continue; //do not flood AVR with data - will cause only problems; each loop sends 4 msg; 50ms should be enough for AVR to consume them
		}
		t1 = t2;

		if (alt_hold || rec.yprt[3]>config.rec_t[2])
			flight_time += dt_ms; 

		switch(config.log_t) {
			case 0: break;
			case 1: 
				log1(); 
				if (verbose==2) log1_print();
				break;
			case 2: 
				log2(); 
				if (verbose==2) log2_print();
				break;
			case 3: 
				log3(); 
				if (verbose==2) log3_print();
				break;
			case 4:
				log4();
				if (verbose==2) log4_print();
				break;
			case 5:
				log5();
				if (verbose==2) log5_print();
				break;
			case 100:
				if (verbose==2) log100_print();
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
		recvMsgs();
	}
}

void print_usage() {
	printf("-v [level] - verbose mode\n");
	printf("-a [addr] - address to connect to (defaults to 127.0.0.1)\n");
	printf("-p [port] - port to connect to (default to 1030)\n");
	printf("-f - do not initialize joystic\n");
}

int main(int argc, char **argv) {

	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);

	int option;
	verbose = 0;
	while ((option = getopt(argc, argv,"v:a:p:f")) != -1) {
		switch (option)  {
			case 'v': verbose=atoi(optarg); break;
			case 'f': nocontroller = 1; break;
			case 'a': strcpy(sock_path,optarg); break;
			case 'p': portno = atoi(optarg); break;
			default:
				  print_usage();
				  return -1;
		}
	}

	for (int i=0;i<256;i++)
		avr_s[i] = 0;

	if (verbose) printf("Opening socket...\n");

	/* Create socket on which to send. */
	sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock < 0) {
		perror("opening socket");
		exit(1);
	}
	server = gethostbyname(sock_path);
	if (server == NULL) {
		fprintf(stderr,"ERROR, no such host\n");
		return -1;
	}
	bzero((char *) &address, sizeof(address));
	address.sin_family = AF_INET;
	bcopy((char *)server->h_addr,
			(char *)&address.sin_addr.s_addr,
			server->h_length);
	address.sin_port = htons(portno);

	if (connect(sock, (struct sockaddr *) &address, sizeof(struct sockaddr_in)) < 0) {
		close(sock);
		perror("connecting socket");
		exit(1);
	}

	/* set non-blocking
	   ret = fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0) | O_NONBLOCK);
	   if (ret == -1){
	   perror("calling fcntl");
	   return -1;
	   }
	 */
	if (verbose) printf("Connected to avrspi\n");
	reset_avr();



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

	if (!nocontroller) {
		ret=rec_open();
		if (ret<0) {
			printf("Failed to initiate receiver! [%s]\n", strerror(ret));	
			return -1;
		}
	}

	mssleep(100);
	if (verbose) printf("int size: %lu\nlong size: %lu\nfloat size: %lu\nyprt size: %lu\n",sizeof(int),sizeof(long),sizeof(float),sizeof(rec.yprt));



	init();
	loop();
	close(sock);
	if (verbose) printf("Closing.\n");
	return 0;
}
