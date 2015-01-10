#ifndef AVRCONFIG_H
#define AVRCONFIG_H

#include "routines.h"

struct s_config {
    char path[128];
    int throttle_min, throttle_inflight;
    int s_pid[3][5]; //y,p,r + [max,imax,kp,ki,kd]
    int r_pid[3][5];
    int a_pid[1]; //acro_p
    int accel_pid[5]; //[max,imax,kp,ki,kd] 
    int baro_f;
    int alt_pid[5];
    int vz_pid[5];
    signed char gyro_orient[9];
    int motor_pin[4];
    int mpu_addr;
};

int config_open(struct s_config *c, const char *path);


#endif

