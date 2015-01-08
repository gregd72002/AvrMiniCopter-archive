/*
 * SPI testing utility (using spidev driver)
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include "spidev.h"
#include "crc8.h"
#include "routines.h"
#include <string.h>

static const char *device = "/dev/spidev0.0";
static uint8_t mode = 0;
static uint8_t bits = 8;
static uint32_t speed = 2000000;
static uint32_t mspeed = 2000000;
//static uint16_t delay = 25;
static uint16_t delay = 100; //without delay RPi reads corrupted data
//static uint16_t delay = 0;

static int ret;
static int fd;


struct avr_msg spi_buf[SPI_BUF_SIZE]; //reader buf
int spi_buf_c = 0;

int spi_crc_err = 0;

void _spi_addByte(uint8_t b) {
    static uint8_t buf[4];
    static int p = 0;

    buf[p++] = b;

    if (p==4) {
        int16_t v = 0;
        v = buf[2] << 8 | buf [1]; 
        //uint8_t c = CRC8(buf,3);
        if (CRC8(buf,3) == buf[3]) {
	    if (spi_buf_c == SPI_BUF_SIZE) {
		printf("SPI input buffer full. Resetting.");
		spi_buf_c = 0;
	    }
	    spi_buf[spi_buf_c].t = buf[0];
	    spi_buf[spi_buf_c].v = v;
	    spi_buf_c++;
	    p = 0;
        } else {
		spi_crc_err++;
		//printf("Received CRC failed %i msg %i %i %i\n",spi_crc_err,buf[0],v,buf[3]);
		buf[0]=buf[1];
		buf[1]=buf[2];
		buf[2]=buf[3];
		p--;
	}
    }
}

int spi_writeBytes(uint8_t *data, unsigned int len) {
    static int np =0;
    uint8_t dummy[4];
    data[len] = CRC8((unsigned char*)(data),len);
    len++;
    struct spi_ioc_transfer tr[len];
    for (unsigned int i=0;i<len;i++) {
        tr[i].tx_buf = (unsigned long)(data+i);
        //tr[i].rx_buf = (unsigned long)(buf.b+i);
        tr[i].rx_buf = (unsigned long)(dummy+i);
        tr[i].len = sizeof(*(data+i));
        tr[i].speed_hz = speed;
        tr[i].delay_usecs = delay;
        tr[i].bits_per_word = bits;
        tr[i].cs_change = 0;
    };

    ret = ioctl(fd, SPI_IOC_MESSAGE(len), &tr);
    if (ret<0) {
        perror("Error transmitting spi data \n");
    }

    for (unsigned int i=0;i<len;i++) {
        if (!np && dummy[i]!=0) np++;
        else if (np) np++;
        if (np) _spi_addByte(dummy[i]);
        if (np==4) np = 0;
    }

    mssleep(5); //without delay AVR reads corrupted messages
    //usleep(5000); //without it AVR reads corrupted messages

    return ret;
}

void spi_sendMsg(struct avr_msg *m) {
    spi_sendIntPacket(m->t,&m->v);
}

void spi_sendIntPacket(uint8_t t, int16_t *v) {
    static unsigned char b[4];
    b[0] = t;
    memcpy(b+1,v,2);

    spi_writeBytes(b,3);
}

int spi_close() {
    return close(fd);
}

int spi_init() {
    spi_buf_c = 0;

    fd = open(device, O_RDWR);
    if (fd < 0)
        return -1;

    /*
     * spi mode
     */
    ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
    if (ret == -1)
        return -2;
    ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
    if (ret == -1)
        return -20;

    /*
     * bits per word
     */
    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret == -1)
        return -3;
    ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if (ret == -1)
        return -30;
    /*
     * max speed hz
     */
    ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &mspeed);
    if (ret == -1)
        return -4;
    ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &mspeed);
    if (ret == -1)
        return -4;

    return 0;
}

