/*
	SFE_BMP180.h
	Bosch BMP180 pressure sensor library for the Arduino microcontroller
	Mike Grusin, SparkFun Electronics

	Uses floating-point equations from the Weather Station Data Logger project
	http://wmrx00.sourceforge.net/
	http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf

	Forked from BMP085 library by M.Grusin

	version 1.0 2013/09/20 initial version
	
	Our example code uses the "beerware" license. You can do anything
	you like with this code. No really, anything. If you find it useful,
	buy me a (root) beer someday.
*/

#ifndef BMP180_h
#define BMP180_h

#include "interface.h"
struct s_bs bs {                                                                              
    .t = 1.0f,                                                                   
    .p = 1.0f,                                                                   
    .p0 = 1.0f,                                                                  
    .alt = 1.0f  //in meters                                                                
};

int bs_open();
int bs_update(unsigned long t_ms);
int bs_reset();
int bs_close();

static int bsWriteBytes(unsigned char *values, char length);
static int bsReadBytes(unsigned char *values, char length);
static int bsReadU(unsigned char address, unsigned short &value);
static int bsReadS(unsigned char address, short &value);
static float altitude(float P, float P0);
static int getPressure(float &P);
static int preparePressure(int oversampling);
static int getTemperature(float &T);
static int prepareTemperature(void);
#endif
