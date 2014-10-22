#ifndef MPU_H
#define MPU_H

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

int mympu_open(short addr,unsigned int rate,unsigned short orient);
int mympu_update();
void mympu_reset_fifo();
#ifdef MPU9150
int mympu_update_compass();
#endif

#endif

