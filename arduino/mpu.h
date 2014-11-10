#ifndef MPU_H
#define MPU_H

#include <Arduino.h>

struct s_mympu {
	float ypr[3];
	float gyro[3];
	float accel[3];
#ifdef MPU9150
    float comp[3];
#endif
    float gravity;
};

extern struct s_mympu mympu;

int8_t mympu_open(short addr,unsigned int rate,unsigned short orient);
int8_t mympu_update();
void mympu_reset_fifo();
#ifdef MPU9150
int8_t mympu_update_compass();
#endif

#endif

