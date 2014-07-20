#ifndef QCONFIG_H
#define QCONFIG_H

struct s_config {
    int log_seq,log_t;
    int rec_t[2]; //t min, t max
    int rec_ypr[2][3]; //y, p, r
    int s_pid[3][5]; //y,p,r + [min,max,kp,ki,kd]
    int r_pid[3][5];
    int alt_pid[5];
    int vz_pid[5];
    signed char gyro_orient[9];
};

extern struct s_config config;

int config_open(const char *path);

int config_save();


#endif

