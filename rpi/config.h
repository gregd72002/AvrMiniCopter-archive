#ifndef QCONFIG_H
#define QCONFIG_H

struct s_config {
    int log_seq,log_t;
    int rec_t[3]; //t min, t_max, t inflight
    int rec_ypr[2][3]; //y, p, r
    int s_pid[3][5]; //y,p,r + [min,max,kp,ki,kd]
    int r_pid[3][5];
    int alt_pid[5];
    int vz_pid[5];
    signed char gyro_orient[9];
    int motor_pin[4];
    int mpu_addr;
};

extern struct s_config config;

int config_open(const char *path);

int config_save();


#endif

