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
#include <string.h>

static const char *device = "/dev/spidev0.0";
static uint8_t mode = 0;
static uint8_t bits = 8;
static uint32_t speed = 1000000;
static uint32_t mspeed = 1000000;
static uint16_t delay = 100;

static int ret;
static int fd;

union s_packet {
    uint8_t b[4];
    struct {
        uint8_t t;
        int16_t i;
        uint8_t c;
    };
} ibuf[64];

unsigned int ibuf_size = 0;

int spi_transferPacket(union s_packet *packet) {
    union s_packet buf;
    
    buf.b[0] = 0;
    packet->c = CRC8((unsigned char*)(packet->b),3);
    struct spi_ioc_transfer tr[4];
    for (int i=0;i<4;i++) {
        tr[i].tx_buf = (unsigned long)(packet->b+i);
        tr[i].rx_buf = (unsigned long)(buf.b+i);
        tr[i].len = 1; //sizeof(*(packet->b+i));
        tr[i].speed_hz = speed;
        tr[i].delay_usecs = delay;
        tr[i].bits_per_word = bits;
        tr[i].cs_change = 0;
    };

    ret = ioctl(fd, SPI_IOC_MESSAGE(4), &tr);
    if (ret<0) {
        perror("Error transmitting spi data \n");
    }

    //transfer correct obuf packets into a seperate buffer to be read by getPacket..
    if (buf.t != 0) {
        int v = 0;
        unsigned char b = buf.b[2];
        buf.b[2] = buf.b[1];
        buf.b[1] = b;
        v = buf.i; 
        int8_t c = CRC8(buf.b,3);
        //v = (*(obuf+2)) << 8 | (*(obuf+1));
        printf("Received t: %i, v: %i, gCRC: %i, mCRC: %i, OK=%i\n",buf.t,v,buf.c,c,buf.c==c);
    }

    usleep(5000);

//    fflush(NULL);
//    sync();

    return ret;
}

int spi_writeBytes(uint8_t *data, unsigned int len) {
    union s_packet buf;
    buf.b[0] = 0;
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

    ret = ioctl(fd, SPI_IOC_MESSAGE(len), &tr);
    if (ret<0) {
        perror("Error transmitting spi data \n");
    }

    if (dummy[0] != 0) {
        int v = 0;
        v = dummy[2] << 8 | dummy[1]; 
        int8_t c = CRC8(dummy,3);
        //v = (*(obuf+2)) << 8 | (*(obuf+1));
        printf("Received t: %i, v: %i, gCRC: %i, mCRC: %i, OK=%i\n",dummy[0],v,dummy[3],c,dummy[3]==c);
    }
    /*
    if (buf.t != 0) {
        int v = 0;
        v = buf.i; 
        int8_t c = CRC8(buf.b,3);
        //v = (*(obuf+2)) << 8 | (*(obuf+1));
        printf("Received t: %i, v: %i, gCRC: %i, mCRC: %i, OK=%i\n",buf.t,v,buf.c,c,buf.c==c);
    }*/

    usleep(5000);

//    fflush(NULL);
//    sync();

    return ret;
}

void spi_sendIntPacket(uint8_t t, int *v) {
    static unsigned char b[4];
    b[0] = t;
    int16_t c = *v;
    memcpy(b+1,&c,2);

    spi_writeBytes(b,3);
}


int spi_writeBytes1(uint8_t *data, unsigned int len) {
    data[len] = CRC8((unsigned char*)(data),len);
    len++;
    struct spi_ioc_transfer tr[1];
    tr[0].tx_buf = (unsigned long)(data);
    tr[0].rx_buf = (unsigned long)(data);
    tr[0].len = len; 
    tr[0].speed_hz = speed;
    tr[0].delay_usecs = delay;
    tr[0].bits_per_word = bits;
    tr[0].cs_change = NULL;

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret<0) {
        perror("Error transmitting spi data \n");
    }

    return ret;
}


int spi_close() {
    return close(fd);
}

int spi_init() {

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

