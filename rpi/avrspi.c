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
#include <string.h>

#include <stdio.h>

#define MAX_TCP_CLIENTS 6
#define MAX_UDP_CLIENTS 6


//avrspi will keep SPI transfer active by sending MSG_RATE messages every MSG_PERIOD
#define MSG_RATE 6 
#define MSG_PERIOD 25 //ms

int msg_counter = 0;

#define OUT_TCP_MSG_SIZE 4
#define IN_TCP_MSG_SIZE 3

#define BUF_SIZE 64

int portno = 1030;
/* UDP */
#define CLIENT_TIMEOUT 10
struct sockaddr_in uaddress[MAX_UDP_CLIENTS];
int uclients[MAX_UDP_CLIENTS];
unsigned char ubuf[BUF_SIZE]; //input udp buffer 
struct sockaddr_in tmpaddress;
socklen_t addrlen = sizeof(tmpaddress);
int usock;
struct timespec udp_time_now,udp_time_prev,udp_last_msg;
/* UDP END */

/* TCP */
int sock[MAX_TCP_CLIENTS+1]; 
struct sockaddr_in address;
unsigned char buf[MAX_TCP_CLIENTS][BUF_SIZE]; //input tcp buffer for each client
unsigned short buf_c[MAX_TCP_CLIENTS];
/* TCP END */

int verbose = 1;
int spi = 1;
int echo = 0;
int background = 0;
int stop = 0;

struct timespec spi_time_now,spi_time_prev,spi_last_msg;
struct timespec *dt;

void catch_signal(int sig)
{
	if (verbose) printf("Signal: %i\n",sig);
	stop = 1;
}

int udp_check_client(struct sockaddr_in *c) {
	int i;
	for (i=0;i<MAX_UDP_CLIENTS;i++)
		if (uclients[i]) //client active
			if ((c->sin_port == uaddress[i].sin_port) &&
					(c->sin_addr.s_addr == uaddress[i].sin_addr.s_addr)) {
				uclients[i] = CLIENT_TIMEOUT;
				return i;
			}

	return -1;
}

int udp_add_client(struct sockaddr_in *c) {
	if (verbose) printf("New client... ");
	int i;

	for (i=0;i<MAX_UDP_CLIENTS && uclients[i]>0;i++); //find a free spot

	if (i==MAX_UDP_CLIENTS) {
		if (verbose) printf("All spots taken!\n");
		return -1;
	} else {
		uclients[i] = CLIENT_TIMEOUT;
		memcpy(&uaddress[i],c,addrlen);
	}

	if (verbose) printf("OK: %i\n",i);
	return i;
}

void ping_back(unsigned char *b, struct sockaddr_in *c) {
	static char buf[5];
	int tmp,ret;
	buf[0] = 1;
	memcpy(&tmp,b,4);
	tmp = ntohl(tmp);

	if (verbose) printf("AVRSPI: Responding to ping %i\n",tmp);


	tmp = htonl(tmp);
	memcpy(buf+1,&tmp,4);

	ret = sendto(usock,buf,5,0,(struct sockaddr *)c,addrlen);

	if (ret<0) {
		printf("AVRSPI: Error sending ping response\n");
	}
}

void process_msg(unsigned char *b,struct sockaddr_in *addr) {
	struct s_msg m;
	m.t = b[0];
	m.v = unpacki16(b+1);

	if (echo) 
		spi_buf[spi_buf_c++] = m;

	if (m.t == 0) {
		if (verbose) printf("Skipping msg t: %u v: %i\n",m.t,m.v);
	} else if (m.t == 255 && m.v == 255) { //REBOOT AVR
		if (verbose) printf("Reset AVR\n");
		linuxgpio_initpin(25);
		linuxgpio_highpulsepin(25,500);
		linuxgpio_close();
		//mssleep(1500);
	}
	else if (m.t == 255 && m.v == 256) {
		m.t = 253; m.v=spi_crc_err;
		spi_buf[spi_buf_c++] = m;
	}
	else {
		if (verbose) printf("Forwarding to AVR t: %u v: %i\n",m.t,m.v);
		if (spi) spi_sendIntPacket(m.t,&m.v);
		clock_gettime(CLOCK_REALTIME, &spi_last_msg);	
		msg_counter++;
	}
}

void print_usage() {
	printf("-d run in background\n");
	printf("-e run echo mode (useful for debugging)\n");
	printf("-f do not do SPI (useful for debugging)\n");
	printf("-p [port] port to listen on (defaults to %i)\n",portno);
}

int main(int argc, char **argv)
{
	/* SELECT */
	int max_fd;
	fd_set readfds;
	struct timeval timeout;
	/* END SELECT */
	int i,j,ret;
	unsigned char bufout[BUF_SIZE];
	struct s_msg dummy_msg = {t: 255, v: 254};
	long dt_ms = 0;

	clock_gettime(CLOCK_REALTIME, &spi_time_prev);
	clock_gettime(CLOCK_REALTIME, &spi_time_now);

	//we use this to measure UDP connection time each second
	clock_gettime(CLOCK_REALTIME, &udp_time_prev);
	clock_gettime(CLOCK_REALTIME, &udp_time_now);

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

	for (i=0; i<MAX_TCP_CLIENTS+1; i++) { 
		sock[i] = 0;
		if (i>0) {
			buf_c[i-1] = 0;
		}
	}

	sock[0] = socket(AF_INET, SOCK_STREAM, 0);
	if (sock[0] < 0) {
		perror("opening stream socket");
		exit(1);
	}

	usock = socket(AF_INET, SOCK_DGRAM, 0);
	if (usock < 0) {
		perror("opening datagram socket");
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
	if (bind(usock, (struct sockaddr *) &address, sizeof(struct sockaddr_in))) {
		perror("binding datagram socket");
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

	clock_gettime(CLOCK_REALTIME, &spi_last_msg);

	if (verbose) printf("Starting main loop\n");
	while (!stop) {
		FD_ZERO(&readfds);
		max_fd = 0;
		for (i=0;i<MAX_TCP_CLIENTS+1;i++) {
			if (sock[i]<=0) continue;
			FD_SET(sock[i], &readfds);
			max_fd = sock[i]>max_fd?sock[i]:max_fd;
		}
		FD_SET(usock, &readfds);
		max_fd = usock>max_fd?usock:max_fd;

		timeout.tv_sec = 0;
		timeout.tv_usec = MSG_PERIOD*1000L; //ms 
		int sel = select( max_fd + 1 , &readfds , NULL , NULL , &timeout);
		if ((sel<0) && (errno!=EINTR)) {
			perror("select");
			stop=1;
		}

		//check for orphan UDP connections
		clock_gettime(CLOCK_REALTIME, &udp_time_now);
		dt = TimeSpecDiff(&udp_time_now,&udp_time_prev);
		dt_ms = dt->tv_sec*1000 + dt->tv_nsec/1000000;
		if (dt_ms>1000) {
			udp_time_prev = udp_time_now;
			for (i=0;i<MAX_UDP_CLIENTS;i++)
				if (uclients[i]>0) {
					uclients[i]--;
					if (uclients[i]<=0) {
						if (verbose) printf("Client %i timeout.\n",i);
					}
				}
		}
		//check UDP
		if (!stop && FD_ISSET(usock, &readfds)) {
			ret = recvfrom(usock, ubuf, BUF_SIZE, 0, (struct sockaddr *)&tmpaddress, &addrlen);
			if (ret<=0) {
				printf("UDP recvfrom error? %i\n",ret);
			} else {
				if (verbose) printf("UDP received %d bytes\n", ret);
				int c = 0;
				while (c<ret) {
					unsigned char t = ubuf[c];
					if (t==0) { //message, but dont add client to the list
						process_msg(ubuf+c+1,&tmpaddress);
						c+=4;
					} else if (t==1) { //msg
						process_msg(ubuf+c+1,&tmpaddress);
						i = udp_check_client(&tmpaddress);
						if (i<0) { //new client
							i=udp_add_client(&tmpaddress);
						}
						c+=4;
					}
					else if (t==2) { //keep alive ping
						i = udp_check_client(&tmpaddress);
						if (i<0) { //new client
							i=udp_add_client(&tmpaddress);
						}
						ping_back(ubuf+c+1,&tmpaddress);
						c+=5;
					} else {
						printf("Unknown UDP message t: %i len: %i\n",t,ret);
					}
				}
			}
		}

		//If something happened on the master socket , then its an incoming connection
		if (!stop && FD_ISSET(sock[0], &readfds)) {
			int t = accept(sock[0], 0, 0);
			if (t<0) {
				perror("accept");
				continue;
			}
			for (i=0;i<MAX_TCP_CLIENTS;i++)
				if (sock[i+1] == 0) {
					if (verbose) printf("Incoming client: %i\n",i);
					sock[i+1] = t;
					buf_c[i+1] = 0;
					break;
				}
			if (i==MAX_TCP_CLIENTS) {
				printf("AVRSPI: No space in connection pool! Disconnecting client.\n");
				close(t);
			}
		} 
		for (i=0;(i<MAX_TCP_CLIENTS) && (!stop);i++) {
			if (FD_ISSET(sock[i+1], &readfds)) {
				ret = read(sock[i+1] , buf[i]+buf_c[i], BUF_SIZE - buf_c[i]); 
				if (ret < 0) {
					perror("Reading error");
					close(sock[i+1]);
					sock[i+1] = 0;
				}
				else if (ret == 0) {	//client disconnected
					if (verbose) printf("Client %i disconnected.\n",i);
					close(sock[i+1]);
					sock[i+1] = 0;
					buf_c[i] = 0;
				} else {
					buf_c[i] += ret;
					if (verbose) printf("Received: %i bytes. Buf size: %i\n",ret,buf_c[i]);
				}

				if (buf_c[i]>=IN_TCP_MSG_SIZE) {

					int msg_c = buf_c[i] / IN_TCP_MSG_SIZE; //number of messages in buffer
					int msg_r = buf_c[i] % IN_TCP_MSG_SIZE; //reminder

					for (j=0;j<msg_c;j++) 
						process_msg(buf[i] + (j*IN_TCP_MSG_SIZE),NULL);

					for (j=0;j<msg_r;j++)
						buf[i][j] = buf[i][j+msg_c*IN_TCP_MSG_SIZE];
					buf_c[i] = msg_r;
				}
			}
		}
		//send out any available messages to clients

		if (spi_buf_c*OUT_TCP_MSG_SIZE>BUF_SIZE) {
			printf("output buffer overflow (bufout)!");
			spi_buf_c = 0;
		}

		for (int j=0;j<spi_buf_c;j++) {
			if (verbose>=2) printf("To clients: t: %u v: %i\n",spi_buf[j].t,spi_buf[j].v);
			bufout[j*OUT_TCP_MSG_SIZE] = 0;
			bufout[j*OUT_TCP_MSG_SIZE+1] = spi_buf[j].t;
			packi16(bufout+j*OUT_TCP_MSG_SIZE+2,spi_buf[j].v);
		}
		if (spi_buf_c>0 && verbose) printf("To clients msgs: %i bytes: %i\n",spi_buf_c,spi_buf_c*OUT_TCP_MSG_SIZE);
		for (int k=0;k<MAX_TCP_CLIENTS;k++) {
			if (sock[k+1]!=0) { 
				ret = send(sock[k+1], bufout, spi_buf_c*OUT_TCP_MSG_SIZE, MSG_NOSIGNAL );
				if (ret == -1) {
					if (verbose) printf("Lost connection to client %i.\n",k);
					close(sock[k+1]);
					sock[k+1] = 0;
				}
			}
		}
		for (int k=0;k<MAX_UDP_CLIENTS;k++) {
			if (uclients[k]>0) { 
				ret = sendto(usock,bufout,OUT_TCP_MSG_SIZE,0,(struct sockaddr *)&uaddress[k],addrlen);
				if (ret<0) {
					printf("Error sending datagram packet\n");
					uclients[k] = 0;
				}
			}
		}

		spi_buf_c = 0;

		//ping avr if needed
		clock_gettime(CLOCK_REALTIME, &spi_time_now);
		dt = TimeSpecDiff(&spi_time_now,&spi_time_prev);
		dt_ms = dt->tv_sec*1000 + dt->tv_nsec/1000000;
		if (dt_ms>=MSG_PERIOD) {
			spi_time_prev = spi_time_now;
			if (spi) for (i=msg_counter;i<MSG_RATE;i++) spi_sendIntPacket(dummy_msg.t,&dummy_msg.v);
			msg_counter = 0;
		}
	}

	if (echo) spi_close();

	bufout[0] = 1; //disconnect msg
	for (int k=0;k<MAX_TCP_CLIENTS;k++) {
		if (sock[k+1]!=0) 
			send(sock[k+1], bufout, OUT_TCP_MSG_SIZE, MSG_NOSIGNAL );
	}

	mssleep(1000);

	if (verbose) {
		printf("closing\n");
		fflush(NULL);
	}

	for (i=0;i<MAX_TCP_CLIENTS+1;i++)
		if (sock[i]!=0) close(sock[i]);

	close(usock);

}

