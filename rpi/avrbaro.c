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

#include "bmpsensor/bmp180.h"

#include "routines.h"
#include "msg.h"


int ret;
int background = 0;
int stop = 0;
int bs_err = 0;
int initialized = 0;

int connected = 0;
int sock = 0;
char sock_path[256] = "127.0.0.1";
int portno = 1030;
struct sockaddr_in address;
struct hostent *server;

int verbose = 0;
int delay;

void reset() {
	connected = 0;
	close(sock);
	delay = 2500;
}

int sendMsg(int t, int v) {
	static unsigned char buf[3];
	buf[0] = t;
	packi16(buf+1,v);
	if (write(sock, buf, 3) < 0) {
		perror("writing");
		reset();
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
			reset();
		}
		else if (sel && !stop && FD_ISSET(sock, &fds)) {
			ret = read(sock,buf+i,4-i);
			if (ret<0) {
				perror("reading");
				reset();
			}
			else {
				i+=ret;
				if (i==4) {
					i = 0;
					if (buf[0] == 1) {
						if (verbose) printf("Disconnect request.\n");
						reset();
					} else {
						m.t = buf[1];
						m.v = unpacki16(buf+2);
						if (verbose==3) printf("Received msg: %u %i\n",m.t,m.v);
						if (m.t==255 && m.v==1) bs.p0=bs.p;
					}
				}
			}
		}
	} while (!stop && connected && sel && ret>0); //no error happened; select picked up socket state change; read got some data back
}

void catch_signal(int sig)
{
	printf("signal: %i\n",sig);
	stop = 1;
}

long dt_ms = 0;
static struct timespec ts,t1,t2,*dt;

unsigned long c = 0,k = 0;

void avrconnect() {
	/* Create socket on which to send. */
	sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock < 0) {
		perror("opening socket");
		return;
	}
	server = gethostbyname(sock_path);
	if (server == NULL) {
		fprintf(stderr,"ERROR, no such host\n");
		return;
	}
	bzero((char *) &address, sizeof(address));
	address.sin_family = AF_INET;
	bcopy((char *)server->h_addr,
			(char *)&address.sin_addr.s_addr,
			server->h_length);
	address.sin_port = htons(portno);

	if (connect(sock, (struct sockaddr *) &address, sizeof(struct sockaddr_in)) < 0) {
		close(sock);
		if (verbose) perror("connecting socket");
		return;
	}

	/* set non-blocking
	   ret = fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0) | O_NONBLOCK);
	   if (ret == -1){
	   perror("calling fcntl");
	   return -1;
	   }
	 */
	if (verbose) printf("Connected to avrspi\n");
	connected = 1;

}

void init() {
	ret=bs_open();
	if (ret<0) {
		printf("Failed to initiate pressure sensor! [%s]\n", strerror(ret));
		return;
	}

	if (verbose>0) printf("Initialization successful!\n");
	initialized = 1;

	mssleep(100);
}

void loop() {
	clock_gettime(CLOCK_REALTIME,&t2);                                           
	ts = t1 = t2;
	delay = 2500;
	if (verbose) printf("Starting main loop...\n");
	while (!stop) {
		clock_gettime(CLOCK_REALTIME,&t2);                                           
		dt = TimeSpecDiff(&t2,&t1);
		dt_ms = dt->tv_sec*1000 + dt->tv_nsec/1000000;

		//if (ret == 0 && dt_ms<20) continue;
		if (dt_ms<delay) {
			mssleep(delay-dt_ms);
			continue; //do not flood AVR with data - will cause only problems; each loop sends 4 msg; 50ms should be enough for AVR to consume them
		}
		t1 = t2;
		if (!initialized) {
			init();
		}
		else if (!connected) {
			avrconnect();	
			if (connected) { delay = 50; c = 0; }
		} else { //initialized && connected = connected
			c++;

			bs_err = bs_update(c*dt_ms);
			if (bs_err>=0) {
				if (verbose==2) {
					printf("T: %2.2f\tAlt: %2.1f\tP: %2.2f\n",bs.t,bs.alt*100.f,bs.p);
				}
				if (bs_err == 1) //pressure updated
					sendMsg(14,bs.alt*100.f); //in cm
				else if (bs_err == 2) c = 0; //end of cycle
			} else 
				if (verbose) printf("Barometer reading error!\n");

			recvMsgs();
		}
	}
}

void print_usage() {
	printf("-b - run in the background\n");
	printf("-v [level] - verbose mode\n");
	printf("-a [addr] - address to connect to (defaults to 127.0.0.1)\n");
	printf("-p [port] - port to connect to (default to 1030)\n");
}

int main(int argc, char **argv) {

	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);

	int option;
	verbose = 0;
	while ((option = getopt(argc, argv,"bv:a:p:")) != -1) {
		switch (option)  {
			case 'b': background = 1; break;
			case 'v': verbose=atoi(optarg); break;
			case 'a': strcpy(sock_path,optarg); break;
			case 'p': portno = atoi(optarg); break;
			default:
				  print_usage();
				  return -1;
		}
	}

        if (background) {
                if (daemon(0,1) < 0) {
                        perror("daemon");
                        return -1;
                }
                if (verbose) printf("Running in the background\n");
        }

	loop();
	if (verbose) printf("Closing.\n");
	close(sock);

	return 0;
}

