#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/time.h>
#include "routines.h"
#include "spidev.h"
#include "gpio.h"
#include <getopt.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <strings.h>

#include <stdio.h>

#define MAX_CLIENTS 5
#define MSG_SIZE 3

int verbose = 1;
int spi = 1;
int echo = 0;
int background = 0;
int stop = 0;

struct timespec time_now,last_msg;
struct timespec *dt;

void catch_signal(int sig)
{
	if (verbose) printf("Signal: %i\n",sig);
	stop = 1;
}

void process_msg(unsigned char *b) {
	struct s_msg m;
	m.t = b[0];
	m.v = unpacki16(b+1);
	if (echo) 
		spi_buf[spi_buf_c++] = m;

	if (m.t == 255 && m.v == 255) {
		if (verbose) printf("Reset AVR\n");
		linuxgpio_initpin(25);
		linuxgpio_highpulsepin(25,500);
		linuxgpio_close();
		//mssleep(1500);
	} else {
		if (verbose) printf("Forwarding to AVR t: %u v: %i\n",m.t,m.v);
		if (spi) spi_sendIntPacket(m.t,&m.v);
		clock_gettime(CLOCK_REALTIME, &last_msg);	
	}
}

void print_usage() {
	printf("-d run in background\n");
	printf("-e run echo mode (useful for debugging)\n");
	printf("-f do not do SPI (useful for debugging)\n");
	printf("-p [port] port to listen on (defaults to 1030)\n");
}

int main(int argc, char **argv)
{
	int sock[MAX_CLIENTS+1], max_fd;
	int i,ret;
	int portno = 1030;
	struct sockaddr_in address;
	unsigned char buf[MAX_CLIENTS][MSG_SIZE];
	unsigned short buf_c[MAX_CLIENTS];
	unsigned char bufout[4];
	struct timeval timeout;
	fd_set readfds;
	struct s_msg dummy_msg = {t: 255, v: 254};
	long dt_ms = 0;

	int option;
	verbose = 1;
	background = 0;
	echo = 0;
	while ((option = getopt(argc, argv,"dep:f")) != -1) {
		switch (option)  {
			case 'd': background = 1; verbose=0; break;
			case 'p': portno = atoi(optarg);  break;
			case 'f': spi=0; break;
			case 'e': echo=1; break;
			default:
				  print_usage();
				  return -1;
		}
	}

	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);

	for (i=0; i<MAX_CLIENTS+1; i++) { 
		sock[i] = 0;
		if (i>0) {
			buf_c[i-1] = 0;
		}
	}

	sock[0] = socket(AF_INET, SOCK_STREAM, 0);
	if (sock[0] < 0) {
		perror("opening socket");
		exit(1);
	}


	/* Create name. */
	bzero((char *) &address, sizeof(address));
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons(portno);

	if (bind(sock[0], (struct sockaddr *) &address, sizeof(struct sockaddr_in))) {
		perror("binding stream socket");
		exit(1);
	}
	printf("Socket created on port %i\n", portno);

	if (listen(sock[0],3) < 0) {
		perror("listen");
		stop=1;
	}

	if (background) {
		if (daemon(0,1) < 0) { 
			perror("daemon");
			return -1;
		}
		if (verbose) printf("Running in the background\n");
	}


	if (spi) {
		ret = spi_init();
		if (ret < 0) {
			printf("Error initiating SPI! %i\n",ret);
			stop = 1;
		}
	}

	clock_gettime(CLOCK_REALTIME, &last_msg);

	if (verbose) printf("Starting main loop\n");
	while (!stop) {
		FD_ZERO(&readfds);
		max_fd = 0;
		for (i=0;i<MAX_CLIENTS+1;i++) {
			if (sock[i]<=0) continue;
			FD_SET(sock[i], &readfds);
			max_fd = sock[i]>max_fd?sock[i]:max_fd;
		}

		timeout.tv_sec = 0;
		timeout.tv_usec = 100*1000L; //100ms
		int sel = select( max_fd + 1 , &readfds , NULL , NULL , &timeout);
		if ((sel<0) && (errno!=EINTR)) {
			perror("select");
			stop=1;
		}
		//If something happened on the master socket , then its an incoming connection
		if (!stop && FD_ISSET(sock[0], &readfds)) {
			int t = accept(sock[0], 0, 0);
			if (t<0) {
				perror("accept");
				continue;
			}
			for (i=0;i<MAX_CLIENTS;i++)
				if (sock[i+1] == 0) {
					if (verbose) printf("Incoming client: %i\n",i);
					sock[i+1] = t;
					buf_c[i+1] = 0;
					break;
				}
			if (i==MAX_CLIENTS) {
				printf("AVRSPI: No space in connection pool! Disconnecting client.\n");
				close(t);
			}
		} 
		for (i=0;(i<MAX_CLIENTS) && (!stop);i++) {
			if (FD_ISSET(sock[i+1], &readfds)) {
				ret = read(sock[i+1] , buf[i]+buf_c[i], MSG_SIZE - buf_c[i]); 
				if (ret < 0) {
					perror("Reading error");
					close(sock[i+1]);
					sock[i+1] = 0;
				}
				else if (ret == 0) {	//client disconnected
					if (verbose) printf("Client %i disconnected.\n",i);
					close(sock[i+1]);
					sock[i+1] = 0;
				} else { //pending message in buffer - forward to SPI
					buf_c[i] += ret;
					if (verbose) printf("Received: %i bytes\n",ret);
					if (buf_c[i] == MSG_SIZE) {
						process_msg(buf[i]);
						buf_c[i] = 0;
					}
				}
			}
		}
		//send out any available message to clients
		for (int j=0;j<spi_buf_c;j++) {
			if (verbose) printf("To clients: t: %u v: %i\n",spi_buf[j].t,spi_buf[j].v);
			bufout[0] = 0;
			bufout[1] = spi_buf[j].t;
			packi16(bufout+2,spi_buf[j].v);
			for (int k=0;k<MAX_CLIENTS;k++) {
				if (sock[k+1]!=0) { 
					ret = send(sock[k+1], bufout, 4, MSG_NOSIGNAL );
					if (ret == -1) {
						if (verbose) printf("Lost connection to client %i.\n",k);
						close(sock[k+1]);
						sock[k+1] = 0;
					}
				}
			}
		}
		spi_buf_c = 0;

		//ping if needed
		clock_gettime(CLOCK_REALTIME, &time_now);
		dt = TimeSpecDiff(&time_now,&last_msg);
		dt_ms = dt->tv_sec*1000 + dt->tv_nsec/1000000;
		if (dt_ms>50) {
			if (spi) spi_sendIntPacket(dummy_msg.t,&dummy_msg.v);
			clock_gettime(CLOCK_REALTIME, &last_msg);	
		}
	}

	if (echo) spi_close();

	bufout[0] = 1; //disconnect msg
	for (int k=0;k<MAX_CLIENTS;k++) {
		if (sock[k+1]!=0) 
			send(sock[k+1], bufout, 4, MSG_NOSIGNAL );
	}

	mssleep(1000);

	if (verbose) {
		printf("closing\n");
		fflush(NULL);
	}

	for (i=0;i<MAX_CLIENTS+1;i++)
		if (sock[i]!=0) close(sock[i]);

}

