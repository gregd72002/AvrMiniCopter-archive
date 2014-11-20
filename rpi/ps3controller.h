#ifndef PS3CONTROLLER_H
#define PS3CONTROLLER_H

struct s_rec {
    int yprt[4];
    int aux;

    int fd;
    int config[5]; //yaw max, pitch max, roll max, throttle min, throttle max
};

int rec_open(const char *path, struct s_rec *s);
int rec_config(struct s_rec *s, int *t, int *v);
int rec_update(struct s_rec *s);
int rec_close(struct s_rec *s);

#endif

