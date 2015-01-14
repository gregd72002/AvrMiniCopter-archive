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

#include "ps3config.h"
#include "ps3dev.h"

#include "routines.h"

#define CFG_PATH "/etc/avrminicopter/"

struct ps3_config ps3config;
int verbose; 

struct s_rec js[2];

int ret;
int err = 0;
int stop = 0;

int avr_s[256];

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

int nocontroller = 0;
int cam_seq = 0;

int flight_threshold;

int sendMsg(int t, int v) {
        static unsigned char buf[4];
        static struct local_msg m;
        m.c = 0;
        m.t = t;
        m.v = v;
        pack_lm(buf,&m);
	if (write(sock, buf, 4) < 0) {
		perror("writing");
		return -1;
	}
/*
        ret = sendto(sock,buf,LOCAL_MSG_SIZE,0,(struct sockaddr *)&address,sizeof(address));
        if (ret<=0) {
                perror("AVRBARO: writing");
        }
*/
        return 0;
}


void recvMsgs() {
	static int sel=0,i=0,ret=0;
	static unsigned char buf[4];
	static struct avr_msg m;

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

void reset_avr() {
	//ensure AVR is properly rebooted
	while (avr_s[255]!=1) { //once rebooted AVR will report status = 1;
		avr_s[255] = -1;
		sendMsg(255,255);
		mssleep(1500);
		recvMsgs();
	}
}

void do_adjustments_secondary(struct s_rec *js) {
	static int adj4 = 25; //for altitude (cm)
	if (js->aux<0) return;
	switch (js->aux) {
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
		case 12:
			if (rec_setting) rec_setting = 0;
			else rec_setting = 1;
			rec_config(js,ps3config.rec_ypr[rec_setting],ps3config.throttle);
			break;
	}

	js->aux = -1;
}

void do_adjustments(struct s_rec *js) {
	if (js->aux<0) return;

	static int adj3 = 1; //for trim
	static int adj4 = 25; //for altitude (cm)

	static char str[128];

	switch (js->aux) {
		case 8: //L2
			memset(str, '\0', 128);
			sprintf(str, "/usr/local/bin/vidsnap.sh %05d ", cam_seq++);
			ret=system(str);
			break;
		case 10: //L1
			memset(str, '\0', 128);
			sprintf(str, "/usr/local/bin/picsnap.sh %05d ", cam_seq++);
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
			if (js[0].yprt[3]<flight_threshold) sendMsg(0,4); 
			break;
		case 3: 
			stop=1;
			break;
		case 12:
			if (rec_setting) rec_setting = 0;
			else rec_setting = 1;
			rec_config(js,ps3config.rec_ypr[rec_setting],ps3config.throttle);
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
			   throttle_target = js->yprt[3];
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
			mode++;
			if (mode==2) mode=0;
			sendMsg(3,mode);
			break;
		default:
			printf("Unknown command %i\n",js->aux);
	}
	if ((verbose) && (js->aux!=-1)) printf("Button: %i\n",js->aux);
	js->aux=-1; //reset receiver command

} 

void catch_signal(int sig)
{
	printf("signal: %i\n",sig);
	stop = 1;
}

long dt_ms = 0;
static struct timespec ts,t1,t2,*dt;

unsigned long k = 0;

void loop() {
	clock_gettime(CLOCK_REALTIME,&t2);                                           
	ts = t1 = t2;
	if (verbose) printf("Starting main loop...\n");
	int yprt[4] = {0,0,0,0};
	while (1 && !err && !stop) {
		if (!nocontroller) {
			ret = rec_update(&js[0]); 
			// 0 - no update but read ok
			// 1 - update
			if (ret < 0) {
				err = 1;
				printf("Receiver reading error: [%s]\n",strerror(ret));
				return;
			}
			do_adjustments(&js[0]);
			memcpy(yprt,js[0].yprt,sizeof(int)*4);

			if (js[1].fd) {
				ret = rec_update(&js[1]);
				if (ret<0) {
					printf("Secondary receiver error: [%s]\n",strerror(ret));
					js[1].fd = 0;
				}
				do_adjustments_secondary(&js[1]);
				//js0 is the master control so only use js1 if js0 has no value
				if (abs(yprt[0]) < 5) yprt[0] = js[1].yprt[0]; 
				if (yprt[1] == 0) yprt[1] = js[1].yprt[1];
				if (yprt[2] == 0) yprt[2] = js[1].yprt[2];
			}
		} else {
			ret = 0;
			js[0].aux = -1;
			yprt[0] = js[0].yprt[0] = 0;
			yprt[1] = js[0].yprt[1] = 0;
			yprt[2] = js[0].yprt[2] = 0;
			yprt[2] = js[0].yprt[3] = 1000;
		}

		if (alt_hold && (yprt[3] > (ps3config.throttle[1]-50) || yprt[3] < ps3config.throttle[0]-50)) {
			alt_hold = 0;
			throttle_hold = 0;
			sendMsg(15,alt_hold);
		}



		clock_gettime(CLOCK_REALTIME,&t2);                                           
		dt = TimeSpecDiff(&t2,&t1);
		dt_ms = dt->tv_sec*1000 + dt->tv_nsec/1000000;

		if (dt_ms<50) {
			mssleep(50-dt_ms);
			continue; //do not flood AVR with data - will cause only problems; each loop sends 4 msg; 50ms should be enough for AVR to consume them
		}
		t1 = t2;

		if (throttle_hold) {
			yprt[3] = throttle_target;
		}

		sendMsg(10,yprt[0]+trim[0]);
		sendMsg(11,yprt[1]+trim[1]);
		sendMsg(12,yprt[2]+trim[2]);
		sendMsg(13,yprt[3]);
		recvMsgs();
	}

	sendMsg(13,ps3config.throttle[0]);
	sendMsg(15,0);
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



	ret=ps3config_open(&ps3config,CFG_PATH);
	if (ret<0) {
		printf("Failed to initiate config! [%s]\n", strerror(ret));	
		return -1;
	}
	flight_threshold = ps3config.throttle[0]+50;

	if (!nocontroller) {
		ret=rec_open("/dev/input/js0",&js[0]);
		if (ret<0) {
			printf("Failed to initiate receiver! [%s]\n", strerror(ret));	
			return -1;
		}
		rec_config(&js[0],ps3config.rec_ypr[0],ps3config.throttle);

		ret=rec_open("/dev/input/js1",&js[1]);
		if (ret<0) {
			printf("Using single receiver\n");	
		} else {
			printf("Using two receivers\n");	
			rec_config(&js[1],ps3config.rec_ypr[0],ps3config.throttle);
		}
	}

	loop();
	close(sock);
	if (!nocontroller) {
		if (js[0].fd) rec_close(&js[0]);
		if (js[1].fd) rec_close(&js[1]);
	}
	if (verbose) printf("Closing.\n");
	return 0;
}
