#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <string.h>

#define DEVOUTFILE "/dev/avroutdev"
#define DEVINFILE "/dev/avrindev"

static const char *device = "/dev/spidev0.0";
static uint8_t mode = 0;
static uint8_t bits = 8;
static uint32_t speed = 1000000;
static uint32_t mspeed = 1000000;
static uint16_t delay = 100;
static int ret;
static int spifd;

typedef unsigned char byte;

int fdin, fdout;
int stop = 0;

byte CRC8(const byte *data, byte len) {
  byte crc = 0x00;
  while (len--) {
    byte extract = *data++;
    for (byte tempI = 8; tempI; tempI--) {
      byte sum = (crc ^ extract) & 0x01;
      crc >>= 1;
      if (sum) {
        crc ^= 0x8C;
      }
      extract >>= 1;
    }
  }
  return crc;
}

void _spi_addByte(uint8_t b) {
    static uint8_t buf[4];
    static int p = 0;

    buf[p++] = b;

    if (p==4) {
        int16_t v = 0;
        //v = buf[2] << 8 | buf [1];
        if (CRC8(buf,3) == buf[3]) {
		write(fdin,buf,3);
        } else printf("CRC failed\n");
        p = 0;
    }
}

int spi_writeBytes(uint8_t *data, unsigned int len) {
    static int np =0;
    uint8_t dummy[4];
    data[len] = CRC8((unsigned char*)(data),len);
    len++;
    struct spi_ioc_transfer tr[len];
    for (int i=0;i<len;i++) {
        tr[i].tx_buf = (unsigned long)(data+i);
        //tr[i].rx_buf = (unsigned long)(buf.b+i);
        tr[i].rx_buf = (unsigned long)(dummy+i);
        tr[i].len = sizeof(*(data+i));
        tr[i].speed_hz = speed;
        tr[i].delay_usecs = delay;
        tr[i].bits_per_word = bits;
        tr[i].cs_change = 0;
    };

    ret = ioctl(spifd, SPI_IOC_MESSAGE(len), &tr);
    if (ret<0) {
        perror("Error transmitting spi data \n");
    }

    for (int i=0;i<len;i++) {
        if (!np && dummy[i]!=0) np++;
        else if (np) np++;
        if (np) _spi_addByte(dummy[i]);
        if (np==4) np = 0;
    }

    usleep(5000);

    return ret;
}

int spi_close() {
    return close(spifd);
}

int spi_init() {

    spifd = open(device, O_RDWR);
    if (spifd < 0)
        return -1;

    /*
     * spi mode
     */
    ret = ioctl(spifd, SPI_IOC_WR_MODE, &mode);
    if (ret == -1)
        return -2;
    ret = ioctl(spifd, SPI_IOC_RD_MODE, &mode);
    if (ret == -1)
        return -20;

    /*
     * bits per word
     */
    ret = ioctl(spifd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret == -1)
        return -3;
    ret = ioctl(spifd, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if (ret == -1)
        return -30;
    /*
     * max speed hz
     */
    ret = ioctl(spifd, SPI_IOC_WR_MAX_SPEED_HZ, &mspeed);
    if (ret == -1)
        return -4;
    ret = ioctl(spifd, SPI_IOC_RD_MAX_SPEED_HZ, &mspeed);
    if (ret == -1)
        return -4;

    return 0;
}

void catch_signal(int sig)
{
	spi_close();
	close(fdout);
	close(fdin);
	unlink(DEVOUTFILE);
	unlink(DEVINFILE);
	stop = 1;
}

int main(int argc, char **argv) {
	int n,ret,daemonize=0;
	int t = -1, v;

	    signal(SIGTERM, catch_signal);
	    signal(SIGINT, catch_signal);

int option;
while ((option = getopt(argc, argv,"dt:v:")) != -1) {
	switch (option)  {
		case 'd': daemonize = 1; break;
		case 't': t = atoi(optarg); break;
		case 'v': v = atoi(optarg); break;
		default: 
			printf("Error. Use -d to daemonize, -t X for type and -v X for value\n");
			return -1;
	}
}

	ret = spi_init();
	if (ret!=0) {
		printf("Error initiating SPI!\n");
		return -1;
	}

	if (t!=-1) {
		static unsigned char b[4];
		    b[0] = t;
		    int16_t c = v;
		    memcpy(b+1,&c,2);
		printf("Sending packet t: %i v: %i ... ",b[0],c);
			spi_writeBytes(b,3);
			spi_close();
		printf("Done.\n");
			return 0;
	}

	unlink(DEVOUTFILE);
	unlink(DEVINFILE);
	if (mkfifo(DEVOUTFILE, 0666) < 0) {
		printf("avrspi: Failed to create %s: %m\n", DEVOUTFILE);
		return -1;
	}
	if (chmod(DEVOUTFILE, 0666) < 0) {
		printf("avrspi: Failed to set permissions on %s: %m\n", DEVOUTFILE);
		return -1;
	}
	if (mkfifo(DEVINFILE, 0666) < 0) {
		printf("avrspi: Failed to create %s: %m\n", DEVINFILE);
		return -1;
	}
	if (chmod(DEVINFILE, 0666) < 0) {
		printf("avrspi: Failed to set permissions on %s: %m\n", DEVINFILE);
		return -1;
	}

	if (daemonize && daemon(0,1) < 0) {
		printf("avrspi: Failed to daemonize process: %m\n");
		return -1;
	}

	fd_set ifds;
	unsigned char buf[4];
	if ((fdout = open(DEVOUTFILE, O_RDWR|O_NONBLOCK)) == -1) {
		printf("avrspi: Failed to open %s: %m\n", DEVOUTFILE);
		return -1;
	}
	if ((fdin = open(DEVINFILE, O_RDWR|O_NONBLOCK)) == -1) {
		printf("avrspi: Failed to open %s: %m\n", DEVINFILE);
		return -1;
	}
	
	while (!stop) {
		FD_ZERO(&ifds);
		FD_SET(fdout, &ifds);
		if ((n = select(fdout+1, &ifds, NULL, NULL, NULL)) != 1)
			continue;
		ret = read(fdout,buf,3);
		//if (ret==-1) continue;
		if (ret!=3) {
			printf("Error receiving bytes %i!\n",ret);
			continue;
		}

		spi_writeBytes(buf,3);
		
	}

	return 0;
}

