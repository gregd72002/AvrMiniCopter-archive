#include <arpa/inet.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/time.h>
#include <getopt.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <strings.h>
#include <string.h>
#include "routines.h"

#include <stdio.h>

#define BUF_SIZE 1024 //receiving buffer
unsigned char buf[BUF_SIZE];
int portno = 1036;


int sock;
struct sockaddr_in tmpaddress;
socklen_t addrlen = sizeof(tmpaddress);

int verbose = 1;
int background = 0;
int stop = 0;


void print_usage() {
	printf("-d run in background\n");
	printf("-p [port] port to listen on (defaults to %i)\n",portno);
	printf("-v [level] debug level\n");
}

void catch_signal(int sig)
{
	if (verbose) printf("Signal: %i\n",sig);
	stop = 1;
}

void processMsg(struct sockaddr_in *client,unsigned char *buf, int len) {	
	unsigned char ip[4];
	int ret,tmp;

	if (len!=4) {
		printf("AVRPING: Incorrect msg size. Expected 4 bytes, got: %i\n",len);
		return;
	}

	memcpy(&tmp,buf,4);
	tmp = ntohl(tmp);

	if (verbose) printf("AVRPING: Responding to ping %i\n",tmp);


	tmp = htonl(tmp);
        memcpy(buf,&tmp,4);

        ret = sendto(sock,buf,4,0,(struct sockaddr *)client,addrlen);

        if (ret<0) {
                printf("AVRPING: Error sending ping response\n");
        }

}

int main(int argc, char **argv)
{
	int max_fd;
	int i,ret;
	struct sockaddr_in address;
	struct timeval timeout;
	fd_set readfds;

	int option;

	while ((option = getopt(argc, argv,"dp:v:")) != -1) {
		switch (option)  {
			case 'd': background = 1; verbose=0; break;
			case 'p': portno = atoi(optarg);  break;
			case 'v': verbose = atoi(optarg); break;
			default:
				  print_usage();
				  return -1;
		}
	}

	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);

	sock = 0;

	sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock < 0) {
		perror("AVRPING: opening socket");
		exit(1);
	}


	/* Create name. */
	bzero((char *) &address, sizeof(address));
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons(portno);

	if (bind(sock, (struct sockaddr *) &address, sizeof(struct sockaddr_in))) {
		perror("AVRPING: binding socket");
		exit(1);
	}
	if (verbose) printf("AVRPING: Socket created on port %i\n", portno);

	if (background) {
		if (daemon(0,1) < 0) { 
			perror("AVRPING: daemon");
			return -1;
		}
		if (verbose) printf("AVRPING: Running in the background\n");
	}


	if (verbose) printf("AVRPING: Starting main loop\n");
	while (!stop) {
		FD_ZERO(&readfds);
		max_fd = 0;
		FD_SET(sock, &readfds);
		max_fd = sock;

		timeout.tv_sec = 0;
		timeout.tv_usec = 1000*1000L; //every sec 
		int sel = select( max_fd + 1 , &readfds , NULL , NULL , &timeout);
		if ((sel<0) && (errno!=EINTR)) {
			perror("AVRPING: select");
			stop=1;
		}

		if (!stop && FD_ISSET(sock, &readfds)) {
			ret = recvfrom(sock, buf, BUF_SIZE, 0, (struct sockaddr *)&tmpaddress, &addrlen);
			processMsg(&tmpaddress,buf,ret);
		}
	}

	if (verbose) {
		printf("AVRPING: closing\n");
		fflush(NULL);
	}

	sleep(1);

	close(sock);
}

