#ifndef QCONFIG_H
#define QCONFIG_H

struct s_config {
    int throttle_min, throttle_inflight;
    int s_pid[3][5]; //y,p,r + [max,imax,kp,ki,kd]
    int r_pid[3][5];
    int a_pid[1]; //acro_p
    int accel_pid[5]; //[max,imax,kp,ki,kd] 
    int baro_f;
    int alt_pid[5];
    int vz_pid[5];
    int mpu_inverted;
    int motor_pin[4];
    int mpu_addr;
};

extern struct s_config config;

int config_open(const char *path);


#endif

