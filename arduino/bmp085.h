#ifndef BMP180_h
#define BMP180_h

#include "I2Cdev.h"

struct s_bs {
        float t;
        float p;
        float p0;
        float alt;
};

extern struct s_bs bs;

int bs_open();
int bs_update(unsigned long t_ms);
int bs_reset();
int bs_close();

static float altitude(float P, float P0);
static int getPressure(float &P);
static int preparePressure();
static int getTemperature(float &T);
static int prepareTemperature(void);
#endif
