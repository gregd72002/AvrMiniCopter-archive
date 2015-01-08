#ifndef _SPIDEV_H_
#define _SPIDEV_H_

#include <stdint.h>
#include "routines.h"

#define SPI_BUF_SIZE 16

extern struct avr_msg spi_buf[SPI_BUF_SIZE];

extern int spi_buf_c;
extern int spi_crc_err;

void spi_sendMsg(struct avr_msg *m);
void spi_sendIntPacket(uint8_t t, int16_t *v);

int spi_writeBytes(uint8_t *data, unsigned int len);

int spi_close();

int spi_init();

#endif
