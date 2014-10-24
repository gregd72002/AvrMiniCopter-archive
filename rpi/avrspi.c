#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/time.h>
#include "routines.h"
#include "spidev.h"
#include "gpio.h"
#include <getopt.h>
#include <sys/file.h>
#include <sys/stat.h>

/*
 * In the included file <sys/un.h> a sockaddr_un is defined as follows
 * struct sockaddr_un {
 *  short   sun_family;
 *  char    sun_path[108];
 * };
 */


#include <stdio.h>

#define MAX_CLIENTS 5
#define MSG_SIZE 3

int verbose = 1;
int echo = 0;
int background = 0;
int stop = 0;

void catch_signal(int sig)
{
	if (verbose) printf("Signal: %i\n",sig);
	stop = 1;
}

void process_msg(unsigned char *b) {
	struct s_msg m;
	m.t = b[0];
	m.v = unpacki16(b+1);
	if (echo) {
		spi_buf[spi_buf_c++] = m;
		return;
	}
	if (m.t == 255 && m.v == 255) {
		if (verbose) printf("Reset AVR\n");
		linuxgpio_initpin(25);
		linuxgpio_highpulsepin(25,500);
		linuxgpio_close();
		//mssleep(1500);
	} else {
		if (verbose) printf("Forwarding to AVR t: %u v: %i\n",m.t,m.v);
		//spi_sendMsg(&m);
	}
}

void print_usage() {
	printf("-d run in background\n");
	printf("-e run echo mode (useful for debugging)\n");
	printf("-s [path] path to socket to create\n");
}

int main(int argc, char **argv)
{
	int sock[MAX_CLIENTS+1], max_fd;
	int i,ret;
	struct sockaddr_un address;
	unsigned char buf[MAX_CLIENTS][MSG_SIZE];
	unsigned short buf_c[MAX_CLIENTS];
	unsigned char bufout[3];
	struct timeval timeout;
	fd_set readfds;
	char sock_path[256] = "/tmp/avrspi.socket";


	int option;
	verbose = 1;
	background = 0;
	echo = 0;
	while ((option = getopt(argc, argv,"des:")) != -1) {
		switch (option)  {
			case 'd': background = 1; verbose=0; break;
			case 's': strcpy(sock_path,optarg); break;
			case 'e': echo=1; break;
			default:
				  print_usage();
				  return -1;
		}
	}

	struct stat st;
	ret = stat(sock_path, &st);
	if (ret == 0) {
		printf("Another instance seems to be running. Closing.\n");
		return -1;
	}


	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);

	for (i=0; i<MAX_CLIENTS+1; i++) { 
		sock[i] = 0;
		if (i>0) {
			buf_c[i-1] = 0;
		}
	}

	sock[0] = socket(AF_UNIX, SOCK_STREAM, 0);
	if (sock[0] < 0) {
		perror("opening socket");
		exit(1);
	}


	/* Create name. */
	address.sun_family = AF_UNIX;
	strcpy(address.sun_path, sock_path);

	if (bind(sock[0], (struct sockaddr *) &address, sizeof(struct sockaddr_un))) {
		perror("binding stream socket");
		exit(1);
	}
	printf("Socket created: %s\n", address.sun_path);

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
		timeout.tv_usec = 100*1000L;
		int sel = select( max_fd + 1 , &readfds , NULL , NULL , &timeout);
		if ((sel<0) && (errno!=EINTR)) {
			perror("select");
			stop=1;
		}
		//If something happened on the master socket , then its an incoming connection
		if (!stop && FD_ISSET(sock[0], &readfds)) {
			if (verbose) printf("Incoming client: ");
			int t = accept(sock[0], 0, 0);
			if (t<0) {
				perror("accept");
				continue;
			}
			for (i=0;i<MAX_CLIENTS;i++)
				if (sock[i+1] == 0) {
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
					sock[i] = 0;
				}
				else if (ret == 0) {	//client disconnected
					if (verbose) printf("Client %i disconnected.\n",i);
					close(sock[i+1]);
					sock[i+1] = 0;
				} else { //pending message in buffer
					buf_c[i] += ret;
					if (verbose) printf("Received: %i bytes\n",ret);
					if (buf_c[i] == MSG_SIZE) {
						process_msg(buf[i]);
						buf_c[i] = 0;
						//send out any available message to clients
						for (int j=0;j<spi_buf_c;j++) {
							if (verbose) printf("To clients: t: %u v: %i\n",spi_buf[j].t,spi_buf[j].v);
							bufout[0] = spi_buf[j].t;
							packi16(bufout+1,spi_buf[j].v);
							for (int k=0;k<MAX_CLIENTS;k++) {
								if (sock[k+1]!=0) 
									ret = send(sock[k+1], bufout, 3, MSG_NOSIGNAL );
							}
						}
						spi_buf_c = 0;
					}
				}
			} 
		}

	}

	if (verbose) {
		printf("closing\n");
		fflush(NULL);
	}

	for (i=0;i<MAX_CLIENTS+1;i++)
		if (sock[i]!=0) close(sock[i]);

	unlink(sock_path);
}

