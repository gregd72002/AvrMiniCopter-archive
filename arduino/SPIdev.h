#ifndef SPIDEV_H
#define SPIDEV_H

#include <SPI.h>
#include "crc8.h"

#define BUF_SIZE 32 //this is 16 msg (each msg has 4 bytes) 

volatile byte SPI_iptr = 0;                                                       
volatile byte SPI_isize = 0;                                                      
unsigned char SPI_ibuf[BUF_SIZE];                                                       
volatile byte SPI_optr = 0;                                                       
volatile byte SPI_osize = 0;                                                      
unsigned char SPI_obuf[BUF_SIZE];                                                       

int SPI_resetBuf() {
    SPI_iptr = 0;
    SPI_isize = 0;
    return 0;
}

int SPI_getByte(byte *b) {                                                      
    if (!SPI_isize) return -1;                                                    

    if (SPI_iptr<SPI_isize)                                                        
        (*b) = SPI_ibuf[BUF_SIZE - SPI_isize + SPI_iptr];                       
    else (*b) = SPI_ibuf[SPI_iptr - SPI_isize];                                   
    SPI_isize--;

    return 0;                                                                    
}                                                                                

void SPI_sendBytes(uint8_t *data, byte len) {
    if (SPI_osize+len>=BUF_SIZE) {
#ifdef DEBUG
        Serial.println("SPI out buf full!");
        //if you decide to wrap the buffer, will need to change firstLoad to load a new byte into reg
#endif
        return;
    } 
    data[len] = CRC8((byte*)data,len);
    len++;

    for (byte i=0;i<len;i++) {
        if (SPI_optr >= BUF_SIZE) SPI_optr = 0;
        SPI_obuf[SPI_optr++] = data[i];
        if (++SPI_osize > BUF_SIZE) SPI_osize = BUF_SIZE;                          
    }

}

int SPI_getPacket(byte *b) {
    //byte c;
    byte j = SPI_isize; //number of bytes in the buffer

    if (j<4) return -1; //type, val(2), crc

    for (byte i=0;i<4;i++)
        SPI_getByte(b+i);

    if (CRC8(b,3)!=b[3]) {
	crc_err++;
        SPI_resetBuf();
#ifdef DEBUG
        static unsigned int n = 0;
        Serial.print("SPI packet corrupted!!"); Serial.println(n++);
#endif
        return -1;
    }

    return 0;
}

int16_t SPI_getInt16(int *i) {
    if (SPI_isize<2) return -1;                                                    
    byte b1=0,b2=0;
    SPI_getByte(&b1);
    SPI_getByte(&b2);
    return (int16_t)(b2<<8 | b1);
}

ISR (SPI_STC_vect) {                                                             
    byte c = SPDR;   
    
//set output byte
    if (SPI_osize) {
        if (SPI_optr<SPI_osize)                                                        
            SPDR = SPI_obuf[BUF_SIZE - SPI_osize + SPI_optr];  
        else SPDR = SPI_obuf[SPI_optr - SPI_osize];                                   
	SPI_osize--;
    } else SPDR = 0;

//get input
    if (SPI_iptr >= BUF_SIZE)    
        SPI_iptr = 0;                                                             

    SPI_ibuf[SPI_iptr++] = c;                                                      

    if (++SPI_isize > BUF_SIZE) SPI_isize = BUF_SIZE;                          
} 
#endif

