#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <stdio.h>
#include <getopt.h>
#include "routines.h"
#include "msg.h"
#include <inttypes.h>

#ifndef SCNu8
#define SCNu8 "hhu"
#endif
#ifndef SCNi16
#define SCNi16 "hi"
#endif

#define MAX_CLIENTS 5

int stop = 0;
int verbose = 1;
int background = 0;

unsigned char obuf[4096];
unsigned char obuf_size = 0;
unsigned int max_obuf_size = 1536; //3 * 512 

void print_usage() {
	printf("-a [address] address to use (default to 127.0.0.1)\n");
	printf("-p [port] port to connect to (defaults to 1030)\n");
	printf("-l [port] port to listen on (defaults to 1031)\n");
	printf("-b for background operation\n");
	printf("-c [size] number of msg to store (defaults to 512)\n");
}

void catch_signal(int sig)
{
	printf("signal: %i\n",sig);
	stop = 1;
}

void store_msg(unsigned char *buf) {
	if (verbose) {
		struct s_msg m;
		m.t = buf[0];
		m.v = unpacki16(buf+1);
		printf("Storing t: %u v: %i\n",m.t,m.v);
	}

	if (max_obuf_size - obuf_size < 3) obuf_size = 0;
	memcpy(obuf+obuf_size,buf,3);
	obuf_size += 3;
}

void send_msgs(int sock) {
	int ret = write(sock,obuf,obuf_size);
	if (ret < 0) {
		perror("Sending data");
	} else {	
		obuf_size = 0;
	}
}

int main(int argc, char **argv)
{
	int ssock;
	int csock; 
	struct sockaddr_in caddress,saddress;
	struct hostent *server;
	int cportno = 1030;
	int sportno = 1031;
	fd_set fds;
	int max_fd;
	struct timeval timeout;
	unsigned char buf[4];
	int ret,c;
	char sock_path[32] = "127.0.0.1";


	int option;
	while ((option = getopt(argc, argv,"bc:p:a:l:")) != -1) {
		switch (option)  {
			case 'c': max_obuf_size = 3*atoi(optarg); break;
			case 'p': cportno = atoi(optarg); break;
			case 'l': sportno = atoi(optarg); break;
			case 'a': strcpy(sock_path,optarg); break;
			case 'b': verbose = 0; background = 1; break;
			default:
				  print_usage();
				  return -1;
		}
	}

	if (max_obuf_size>4096) max_obuf_size = 4096;

	/* Create socket to listen */
	ssock = socket(AF_INET, SOCK_STREAM, 0);
        if (ssock < 0) {
                perror("opening server socket");
                exit(1);
        }

        bzero((char *) &saddress, sizeof(saddress));
        saddress.sin_family = AF_INET;
        saddress.sin_addr.s_addr = INADDR_ANY;
        saddress.sin_port = htons(sportno);

        if (bind(ssock, (struct sockaddr *) &saddress, sizeof(struct sockaddr_in))) {
                perror("binding stream socket");
                exit(1);
        }

        if (listen(ssock,3) < 0) {
                perror("listen");
                stop=1;
        }

	/* Create socket to avr. */
	csock = socket(AF_INET, SOCK_STREAM, 0);
	if (csock < 0) {
		perror("opening client socket");
		exit(1);
	}
	server = gethostbyname(sock_path);
	if (server == NULL) {
		fprintf(stderr,"ERROR, no such host\n");
		return -1;
	}
	bzero((char *) &caddress, sizeof(caddress));
	caddress.sin_family = AF_INET;
	bcopy((char *)server->h_addr, 
			(char *)&caddress.sin_addr.s_addr,
			server->h_length);
	caddress.sin_port = htons(cportno);
	/* Construct name of socket to send to. */
	/* Send message. */

	if (connect(csock, (struct sockaddr *) &caddress, sizeof(struct sockaddr_in)) < 0) {
		close(csock);
		printf("%s\n",sock_path);
		perror("connecting socket");
		printf("Check if avrspi is running\n");
		exit(1);
	}

        if (background) {
                if (daemon(0,1) < 0) {
                        perror("daemon");
                        return -1;
                }
                if (verbose) printf("Running in the background\n");
		printf("%i\n",getpid());
        }

	c = 0;
	while(!stop) {
		FD_ZERO(&fds);
		FD_SET(csock,&fds);
		FD_SET(ssock,&fds);
		max_fd = ssock>csock?ssock:csock;
		timeout.tv_sec = 0;
		timeout.tv_usec = 100*1000L;
		int sel = select( max_fd + 1 , &fds , NULL , NULL , &timeout);
		if ((sel<0) && (errno!=EINTR)) {
			perror("select");
			stop=1;
		}
		if (!stop && FD_ISSET(csock, &fds)) {
			ret = read(csock , buf+c, 4-c);
			if (ret<=0) {
				perror("reading");
				stop = 1;
			} else {
				c += ret;
				if (c==4) {
                                        if (buf[0] == 1) {
                                                if (verbose) printf("Disconnect request.\n");
                                                stop = 1;
                                        } else {
						store_msg(buf+1);	
					}
					c = 0;
				}
			}
		}
		if (!stop && FD_ISSET(ssock,&fds)) {
                        int t = accept(ssock, 0, 0);
                        if (t<0) {
                                perror("accept");
                                continue;
                        }

                        if (verbose) printf("Client connected - sending data & disconnecting: %u\n",obuf_size);
			send_msgs(t);
			close(t);
		}
	}

	close(ssock);
	close(csock);
}

