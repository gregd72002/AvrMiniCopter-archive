#ifndef QCONFIG_H
#define QCONFIG_H

struct s_config {
    int rec[5]; //y, p, r, t min, tmax
    int s_pid[3][5]; //y,p,r + [min,max,kp,ki,kd]
    int r_pid[3][5];
    int a_pid[5];
};

extern struct s_config config;

int config_open(const char *path);

int config_save();


#endif

