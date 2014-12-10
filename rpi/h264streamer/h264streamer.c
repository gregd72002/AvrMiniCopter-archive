#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/time.h>
#include "routines.h"
#include <getopt.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <strings.h>
#include <string.h>

#include <stdio.h>

int verbose = 1;
int stop = 0;

#define MAX_CLIENTS 5
#define MAX_NAL_SIZE 65535

#define BUF_SIZE 1400


unsigned int client_timeout=10;

unsigned char SPS[MAX_NAL_SIZE];
unsigned int SPS_size = 0;
unsigned char PPS[MAX_NAL_SIZE];
unsigned int PPS_size = 0;
int frames_found = 0;

unsigned char buf[BUF_SIZE];
unsigned char recvbuf[1024];

struct sockaddr_in raddress[MAX_CLIENTS];
int clients[MAX_CLIENTS];

int sock;

struct sockaddr_in tmpaddress;
socklen_t addrlen = sizeof(tmpaddress);

void catch_signal(int sig)
{
	if (verbose) printf("Signal: %i\n",sig);
	stop = 1;
}

void print_usage() {
	printf("-p [port] port to listen on (defaults to 8888)\n");
	printf("-t [timeout] client timeout (defaults to 10 sec)\n");
	printf("-v [level] verbose level\n");
}

int ping_client(struct sockaddr_in *c) {
	int i;
	for (i=0;i<MAX_CLIENTS;i++)
		if (clients[i]) //client active
			if ((c->sin_port == raddress[i].sin_port) &&
					(c->sin_addr.s_addr == raddress[i].sin_addr.s_addr)) {
				clients[i] = client_timeout; 
				return i;
			}

	return -1;
}

int add_client(struct sockaddr_in *c) {
	if (verbose) printf("New client... ");
	int i;

	for (i=0;i<MAX_CLIENTS && clients[i]!=0;i++); //find a free spot

	if (i==MAX_CLIENTS) {
		if (verbose) printf("All connections taken!\n");
		return -1;
	} else {
		clients[i] = client_timeout;
		memcpy(&raddress[i],c,addrlen);
	}

	if (verbose) printf("OK: %i\n",i);
	return i;
}

void send_frames(int client) {
	int ret;
	ret = sendto(sock, SPS, SPS_size, 0, (struct sockaddr *)&raddress[client], addrlen);
	if (ret<0) {
		if (verbose) printf("Send to %i failed. Disconnecting.\n",client);
		clients[client] = 0;
	}

	ret = sendto(sock, PPS, PPS_size, 0, (struct sockaddr *)&raddress[client], addrlen);
	if (ret<0) {
		if (verbose) printf("Send to %i failed. Disconnecting.\n",client);
		clients[client] = 0;
	}
}

int find_NAL(unsigned char *buf, int size) {
	int i=0,j=0;
	char pattern[4] = {0x00,0x00,0x00,0x01};
	//printf("Pattern %i %i %i\n",pattern[0],pattern[1],pattern[2]);
	for (i=0;i<size;i++) {
		for (j=0;j<4 && buf[i+j]==pattern[j];j++); //check bytes in front
		if (j==4) {
			//printf("Result %i %i %i\n",buf[i],buf[i+1],buf[i+2]);
			return i;
		}
	}
	return -1;
}

void process_NAL(unsigned char *buf, int size) {
	int type = buf[4] & 0x1F;
	if (type==7) { //SPS
		if (verbose) printf("SPS found. Len: %i\n",size);
		memcpy(SPS,buf,size);
		SPS_size = size;
	}
	if (type==8) { //PPS
		if (verbose) printf("PPS found. Len: %i\n",size);
		memcpy(PPS,buf,size);
		PPS_size = size;
	}

	if (SPS_size && PPS_size) {
		frames_found = 1;
		if (verbose) printf("SPS and PPS ready.\n");
	}
}

void receive_SPS_PPS() {
	int shift = 0;
	int recvlen = 0;
	int start=0,ret,len=0;

	int mode = 0;
	unsigned char NAL[MAX_NAL_SIZE];
	int size = 0;
	
	int i;

	do {
		recvlen = read(STDIN_FILENO, buf+shift,BUF_SIZE-shift);
//		for (i=0;i<40;i++) {
//			printf("%i ",(buf+shift)[i]);
//		}
//		printf("\n");
		if (recvlen==0) continue;
		//if (verbose) printf("STDIN received %d bytes\n",recvlen);
		if (recvlen<0) {
			perror("Pipe broken?");
			stop = 1;
			return;
		} 

		if (mode=0) start = find_NAL(buf,recvlen);
		else start = 0;
		//printf("NAL finder start: %i\n",start);

		do {
			ret = find_NAL(buf+start+1,recvlen-start-1);
			len = ret + 1;
			//printf("NAL finder ret: %i\n",ret);
			if (ret>=0) {
				memcpy(NAL+size,buf+start,len); //end of NAL found - copy
				size += (len);
				process_NAL(NAL,size);

				size = 0;
				start = start+len; //we have not processed the entire buffer, the remaining buffer may contain another NAL, as such we need to ensure the remaining buffer is not overwritten by read operation
				mode = 0;
			}
			else {
				len = recvlen-start-3; //worst case - buf ends with 0x00, 0x00, 0x00 we need to prepend this to the next buffer
				memcpy(NAL+size,buf+start,len); //end of NAL not found - copy entire buffer	
				size += len; 
				buf[0] = buf[len];
				buf[1] = buf[len+1];
				buf[2] = buf[len+2];
				shift = 3;
				mode = 1;
			}
		} while (ret>=0 && !frames_found);

	} while (!frames_found);
}

int main(int argc, char **argv)
{
	int max_fd;
	fd_set fds;
	int i,ret;
	int portno = 8888;
	struct sockaddr_in address;
	struct timeval timeout;

	struct timespec time_now,time_prev;
	struct timespec *dt;
	long dt_ms = 0;

	int option;
	verbose = 1;
	while ((option = getopt(argc, argv,"p:t:v:")) != -1) {
		switch (option)  {
			case 'p': portno = atoi(optarg);  break;
			case 't': client_timeout = atoi(optarg);  break;
			case 'v': verbose = atoi(optarg);  break;
			default:
				  print_usage();
				  return -1;
		}
	}

	signal(SIGTERM, catch_signal);

	sock = 0;
	for (i=0;i<MAX_CLIENTS;i++)
		clients[i] = 0;


	sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock < 0) {
		perror("opening socket");
		exit(1);
	}

	/* Create name. */
	bzero((char *) &address, sizeof(address));
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons(portno);

	if (bind(sock, (struct sockaddr *) &address, sizeof(struct sockaddr_in))) {
		perror("binding stream socket");
		exit(1);
	}
	if (verbose) printf("Socket created on port %i\n", portno);

	clock_gettime(CLOCK_REALTIME, &time_now);
	clock_gettime(CLOCK_REALTIME, &time_prev);
	while (!stop) {
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

		//check for orphan connections
		clock_gettime(CLOCK_REALTIME, &time_now);
		dt = TimeSpecDiff(&time_now,&time_prev);
		dt_ms = dt->tv_sec*1000 + dt->tv_nsec/1000000;
		if (dt_ms>1000) {
			time_prev = time_now;
			for (i=0;i<MAX_CLIENTS;i++) 
				if (clients[i]) {
					clients[i]--;
					if (clients[i]==0) if (verbose) printf("Client %i timeout.\n",i);
				}
		}

		if (!stop && FD_ISSET(sock, &fds)) {
			ret = recvfrom(sock, recvbuf, 1024, 0, (struct sockaddr *)&tmpaddress, &addrlen);
			if (frames_found) { //dont forward to client if we have not SPS and PPS frames yet
				//check if we know this client
				i = ping_client(&tmpaddress);
				if (i<0) { //new client
					i=add_client(&tmpaddress);
					//forward SPS and PPS frames
					if (i>=0) send_frames(i);
				}
				if (i>=0) {
					if (verbose) printf("Client %i received %d bytes\n", i,ret);
					if (verbose==3) if (ret > 0) {
						buf[ret] = 0;
						printf("Client %i received message: %s\n", i,recvbuf);
					}
				}
			}
		}
		if (!stop && FD_ISSET(STDIN_FILENO,&fds)) {
			if (!frames_found) {
				receive_SPS_PPS();
				continue;
			}
			int recvlen = read(STDIN_FILENO, buf,BUF_SIZE);
			if (verbose==2) printf("STDIN received %d bytes\n",recvlen);
			if (recvlen<0) {
				perror("Pipe broken?");
				stop = 1;
			} else {
				//forward  
				for (i=0;i<MAX_CLIENTS;i++) {
					if (clients[i]) {
						ret = sendto(sock, buf, recvlen, 0, (struct sockaddr *)&raddress[i], addrlen);
						if (ret<0) {
							if (verbose) printf("Send to %i failed. Disconnecting.\n",i);
							clients[i] = 0;
						}
					}
				}
			}
		}

	}


	printf("Closing.\n");
	close(sock);
	return 0;
}

