#ifndef PS3CONTROLLER_H
#define PS3CONTROLLER_H

struct s_rec {
    int yprt[4];
    int aux;
};

extern struct s_rec rec;

int rec_open();
int rec_update();
int rec_close();
void rec_setSetting(int);

#endif

