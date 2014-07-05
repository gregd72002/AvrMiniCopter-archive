#ifndef SPIDEV_H
#define SPIDEV_H

#include <stdint.h>

extern int spi_v[256];

void spi_sendIntPacket(uint8_t t, int *v);

int spi_writeBytes(uint8_t *data, unsigned int len); 

int spi_close();

int spi_init(); 

#endif
