#ifndef SPIDEV_H
#define SPIDEV_H

#include <SPI.h>
#include "crc8.h"

volatile byte SPI_ptr = 0;                                                       
volatile byte SPI_size = 0;                                                      
unsigned char SPI_buf[128];                                                       

int SPI_resetBuf() {
    SPI_ptr = 0;
    SPI_size = 0;
    return 0;
}

int SPI_getByte(byte *b) {                                                      
    if (!SPI_size) return -1;                                                    

    if (SPI_ptr<SPI_size)                                                        
        (*b) = SPI_buf[sizeof SPI_buf - SPI_size-- + SPI_ptr];                       
    else (*b) = SPI_buf[SPI_ptr - SPI_size--];                                   

    return 0;                                                                    
}                                                                                

int SPI_getPacket(byte *b) {
    byte c;
    int j = SPI_size;

    if (j<4) return -1; //type, val(2), crc

    for (int i=0;i<3;i++)
        SPI_getByte(b+i);

    SPI_getByte(&c);

    if (CRC8(b,3)!=c) {
        static unsigned int n = 0;
        SPI_resetBuf();
#ifdef DEBUG
        Serial.print("SPI packet corrupted!! t: "); Serial.print(b[0],DEC); Serial.print(" v: "); Serial.print(*((int*)(b+1)),DEC); Serial.println(n++);
#endif
        return -1;
    }

    return 0;
}

int16_t SPI_getInt16(int *i) {
    if (SPI_size<2) return -1;                                                    
    byte b1,b2;
    SPI_getByte(&b1);
    SPI_getByte(&b2);
    return (int16_t)(b2<<8 | b1);
}

ISR (SPI_STC_vect) {                                                             
    byte c = SPDR;                                                               
    if (SPI_ptr >= sizeof SPI_buf)    
        SPI_ptr = 0;                                                             

    SPI_buf[SPI_ptr++] = c;                                                      

    if (++SPI_size > sizeof SPI_buf) SPI_size = sizeof SPI_buf;                          
} 
#endif

