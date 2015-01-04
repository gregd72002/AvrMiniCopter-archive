#ifndef PS3CONFIG_H
#define PS3CONFIG_H

struct ps3_config {
    int log_seq,log_t,cam_seq;
    int throttle[2]; //t min, t_max
    int rec_ypr[2][3]; //y, p, r
};

extern struct ps3_config ps3config;

int ps3config_open(struct ps3_config *c,const char *path);

#endif

