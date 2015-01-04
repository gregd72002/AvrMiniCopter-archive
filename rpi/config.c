#include "config.h"
#include <stdio.h>

struct s_config config;

static FILE *f;
static const char *p;

int config_open(const char *path) {
    int state = 0;
    p = path;
    f = fopen(p,"r");
    if (f == NULL) state = 1; 

    if (state ==0) {

    	if (fscanf(f,"%i\t%i\n",&config.throttle_min,&config.throttle_inflight)!=2) state = 1;

        for (int i=0;i<3 && !state;i++) {
            for (int j=0;j<5 && !state;j++)
                if (fscanf(f,"%i\t",&config.r_pid[i][j])!=1) state = 1;
            if (fscanf(f,"\n")<0) state=1;
        }

        for (int i=0;i<3 && !state;i++) {
            for (int j=0;j<5 && !state;j++)
                if (fscanf(f,"%i\t",&config.s_pid[i][j])!=1) state = 1;
            if (fscanf(f,"\n")<0) state=1;
        }

        for (int j=0;j<5 && !state;j++)
            if (fscanf(f,"%i\t",&config.alt_pid[j])!=1) state = 1;
        if (fscanf(f,"\n")<0) state=1;
        for (int j=0;j<5 && !state;j++)
            if (fscanf(f,"%i\t",&config.vz_pid[j])!=1) state = 1;
        if (fscanf(f,"\n")<0) state=1;
        for (int j=0;j<5 && !state;j++)
            if (fscanf(f,"%i\t",&config.accel_pid[j])!=1) state = 1;
        if (fscanf(f,"\n")<0) state=1;

        if (fscanf(f,"%i\t%i",&config.a_pid[0],&config.baro_f)!=2) state = 1;
        if (fscanf(f,"\n")<0) state=1;

        for (int i=0;i<9 && !state;i++)
            if (fscanf(f,"%i\t",&config.gyro_orient[i])!=1) state = 1;
        if (fscanf(f,"\n")<0) state=1;

        for (int i=0;i<4 && !state;i++)
            if (fscanf(f,"%i\t",&config.motor_pin[i])!=1) state = 1;
        if (fscanf(f,"\n")<0) state=1;

        if (fscanf(f,"%i\t",&config.mpu_addr)!=1) state = 1;

        fclose(f);

        fflush(NULL);
    }

    //y,p,r + [min,max,kp,ki,kd]
    if (state) {
        printf("No config file or config syntax issue!\n");
	return -1;
    }
}

