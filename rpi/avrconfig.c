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
#include "mpu.h"

#include "routines.h"
#include "msg.h"

#define MSG_SIZE 4
int ret;
int err = 0;
int stop = 0;

int avrstatus;
int avrcrcerrors;

int sock = 0;
char sock_path[256] = "127.0.0.1";
int portno = 1030;
struct sockaddr_in address;
struct hostent *server;

int verbose = 0;
int background = 0;
int delay = 1000;

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

void processMsg(unsigned char *buf) {
	static struct s_msg m;
	if (buf[0] == 1) {
		if (verbose) printf("AVRCONFIG: Disconnect request.\n");
		stop = 1;	
	} else {
		m.t = buf[1];
		m.v = unpacki16(buf+2);
		switch (m.t) {
			case 255: avrstatus = m.v;
			case 254: avrcrcerrors = m.v;
		}
	}
}

void recvMsgs() {
	static int sel=0,i=0,ret=0,j;
	static unsigned char buf[64];

	static fd_set fds;
	static struct timeval timeout;

	do {
		FD_ZERO(&fds);
		FD_SET(sock,&fds);
		timeout.tv_sec = 0;
		timeout.tv_usec = delay*1000L;
		sel = select( sock + 1 , &fds , NULL , NULL , &timeout);
		if ((sel<0) && (errno!=EINTR)) {
			perror("select");
			stop=1;
		}
		else if (sel && !stop && FD_ISSET(sock, &fds)) {
			ret = read(sock,buf+i,64-i);
			if (ret<0) {
				perror("reading");
				stop = 1;
			}
			else {
				i+=ret;
				int msg_no = ret / MSG_SIZE;
				int reminder = ret % MSG_SIZE;
				for (j=0;j<msg_no;j++)
					processMsg(buf+j*MSG_SIZE);
				for (j=0;j<reminder;j++)
					buf[j] = buf[msg_no*MSG_SIZE+j];
				i = reminder;
			}
		}
	} while (!stop && sel && ret>0); //no error happened; select picked up socket state change; read got some data back
}

void sendConfig() {
	sendMsg(3,0); //initial mode 

	int gyro_orientation = inv_orientation_matrix_to_scalar(config.gyro_orient);
	sendMsg(4,gyro_orientation);

	sendMsg(9,config.mpu_addr);

	sendMsg(17,config.throttle_min);
	sendMsg(18,config.throttle_inflight);

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
	//ensure AVR is properly rebooted
	while (avrstatus!=1 && !stop) { //once rebooted AVR will report status = 1;
		avrstatus = -1;
		avrcrcerrors = 0;
		sendMsg(255,255);
		mssleep(1500);
		recvMsgs();
	}
}

void catch_signal(int sig)
{
	printf("AVRCONFIG: signal: %i\n",sig);
	stop = 1;
}

void loop() {
	//feeds all config and waits for calibration
	int prev_status;

	while (!stop) {
		recvMsgs();
		if (avrstatus!=5) {
			reset_avr();
			prev_status = avrstatus;
		}
		while (avrstatus!=5 && !stop) {
			sendMsg(255,0); //query status
			sendMsg(255,1); //crc status
			recvMsgs();
			if (prev_status!=avrstatus) {
				if (verbose) {
					if (avrstatus==-1) printf("AVRCONFIG: Waiting for AVR status change.\n");
					else printf("AVRCONFIG: AVR Status: %i\n",avrstatus);
				}
				prev_status = avrstatus;
			}

			if (avrcrcerrors!=0) { //AVR reported crc error when receiving data
				printf("AVRCONFIG: AVR reports CRC errors %i\n",avrcrcerrors);
				reset_avr();
			}

			else switch (avrstatus) {
				case -1: break;
				case 0: reset_avr(); break; //AVR should boot into status 1 so 0 means something wrong
				case 1: sendConfig(); mssleep(1000); avrstatus = -1; sendMsg(255,2); break;
				case 2: break; //AVR should arm motors and set status to 3
				case 3: break; //AVR is initializing MPU 
				case 4: break; //AVR is calibration gyro
				case 5: printf("Initialization OK.\n"); break;
				case 255: printf("AVRCONFIG: Gyro calibration failed!\n"); reset_avr(); avrstatus = 0; break; //calibration failed
				default: printf("AVRCONFIG: Unknown AVR status %i\n",avrstatus); break;
			}

			if (verbose && avrstatus==5) printf("AVRCONFIG: Initialization OK\n");
		}
	}
}

void print_usage() {
	printf("-v [level] - verbose mode\n");
	printf("-a [addr] - address to connect to (defaults to 127.0.0.1)\n");
	printf("-p [port] - port to connect to (default to 1030)\n");
}

int main(int argc, char **argv) {

	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);

	int option;
	verbose = 1;
	while ((option = getopt(argc, argv,"v:a:p:b")) != -1) {
		switch (option)  {
			case 'b': background = 1; verbose = 0; break;
			case 'v': verbose=atoi(optarg); break;
			case 'a': strcpy(sock_path,optarg); break;
			case 'p': portno = atoi(optarg); break;
			default:
				  print_usage();
				  return -1;
		}
	}

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

	if (verbose) printf("Connected to avrspi\n");

	ret=config_open("/var/local/rpicopter.config");
	if (ret<0) {
		printf("Failed to initiate config! [%s]\n", strerror(ret));	
		return -1;
	}

	if (background) {
		if (daemon(0,1) < 0) {
			perror("AVRCONFIG: daemon");
			return -1;
		}
		if (verbose) printf("AVRCONFIG: Running in the background\n");
	}

	reset_avr();
	loop();

	close(sock);

	if (verbose) printf("Closing.\n");
	return 0;
}
