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

int stop = 0;
void print_usage() {
	printf("Error. Use -t X for type and -v X for value\n");
	printf("type is of type uint8_t, value of type int16_t\n");
	printf("-a [address] address to use (default to 127.0.0.1)\n");
	printf("-p [port] port to use (defaults to 1030)\n");
	printf("-l do not disconnect, keep listening\n");
}

void catch_signal(int sig)
{
	printf("signal: %i\n",sig);
	stop = 1;
}

void print_msg(unsigned char *buf) {
	struct s_msg m;
	m.t = buf[0];
	m.v = unpacki16(buf+1);
	printf("Recieved t: %u v: %i\n",m.t,m.v);
}
void processMsg(unsigned char *buf) {
	if (buf[0] == 1) {
		printf("Disconnect request.\n");
		stop = 1;
	} else {
		print_msg(buf+1);	
	}
}


#define MSG_SIZE 4
#define MAX_BUF 64

int main(int argc, char **argv)
{
	int sock;
	struct sockaddr_in address;
	struct hostent *server;
	int portno = 1030;
	struct s_msg msg;
	unsigned char buf1[MAX_BUF]; //receiving buffer
	char buf2[256];
	int b1,b2,cmd;
	int c = 0;
	fd_set fds;
	int max_fd;
	struct timeval timeout;
	int listen = 0;
	int ret;
	unsigned char buf[3];
	char sock_path[32] = "127.0.0.1";
	msg.t = 0;
	msg.v = 0;


	int option;
	while ((option = getopt(argc, argv,"t:v:p:a:l")) != -1) {
		switch (option)  {
			case 't': c++; msg.t = atoi(optarg); break;
			case 'v': c++; msg.v = atoi(optarg); break;
			case 'l': listen=1; break;
			case 'a': strcpy(sock_path,optarg); break;
			case 'p': portno = atoi(optarg); break;
			default:
				  print_usage();
				  return -1;
		}
	}

	if ((c < 2) && (!listen)) {
		print_usage();
		return -1;
	}

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
	/* Construct name of socket to send to. */
	/* Send message. */

	if (connect(sock, (struct sockaddr *) &address, sizeof(struct sockaddr_in)) < 0) {
		close(sock);
		printf("%s\n",sock_path);
		perror("connecting socket");
		printf("Check if avrspi is running\n");
		exit(1);
	}
	if (c) {
		printf("Sending message t: %u v: %i\n",msg.t,msg.v);
		buf[0] = msg.t;
		packi16(buf+1,msg.v);
		if (write(sock, buf, 3) < 0)
			perror("writing");
	}

	if (!listen) {
		close(sock);
		return 0;
	}

	cmd = 0;
	b1 = 0;
	b2 = 0;
	while(!stop) {
		FD_ZERO(&fds);
		FD_SET(sock,&fds);
		FD_SET(STDIN_FILENO,&fds);
		max_fd = sock>STDIN_FILENO?sock:STDIN_FILENO;
		timeout.tv_sec = 0;
		timeout.tv_usec = 100*1000L;
		int sel = select( max_fd + 1 , &fds , NULL , NULL , &timeout);
		if ((sel<0) && (errno!=EINTR)) {
			perror("select");
			stop=1;
		}
		if (!stop && FD_ISSET(sock, &fds)) {
			ret = read(sock, buf1+b1,MAX_BUF-b1);
			if (ret<=0) {
				perror("reading");
				stop = 1;
			} else {
				b1+=ret;
				int msg_no = ret / MSG_SIZE;
				int reminder = ret % MSG_SIZE;
				int i;
				for (i=0;i<msg_no;i++)
					processMsg(buf1+i*MSG_SIZE);
				for (i=0;i<reminder;i++)
					buf1[i] = buf[msg_no*MSG_SIZE+i];
				b1 = reminder;
			}
		}
		if (!stop && FD_ISSET(STDIN_FILENO,&fds)) {
			do {
				ret = read(STDIN_FILENO, buf2+b2,1);
				if (ret>0) {
					if (buf2[b2]=='\n') cmd=1;
					b2++;
				}
			} while (!cmd && ret>0);
			if (cmd) {
				ret = sscanf(buf2,"%" SCNu8 "%" SCNi16 "\n",&msg.t,&msg.v);
				if ((ret == 2) && (b2>3)) {
					buf[0] = msg.t;
					packi16(buf+1,msg.v);
					printf("Sending t: %u v: %i\n",msg.t,msg.v);
					if (write(sock, buf, 3) < 0)
						perror("writing");
				} else printf("Error paring. Enter 2 arguments: type and value\n");
			}
			cmd = 0;
			b2 = 0;
		}
	}

	close(sock);
}

