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

        for (int i=0;i<5 && !state;i++) 
            if (fscanf(f,"%i\t",&config.rec[i])<0) state = 1;

        fscanf(f,"\n");

        for (int i=0;i<3 && !state;i++) {
            for (int j=0;j<5 && !state;j++)
                if (fscanf(f,"%i\t",&config.r_pid[i][j])<0) state = 1;
            if (fscanf(f,"\n")<0) state=1;
        }

        for (int i=0;i<3 && !state;i++) {
            for (int j=0;j<5 && !state;j++)
                if (fscanf(f,"%i\t",&config.s_pid[i][j])<0) state = 1;
            if (fscanf(f,"\n")<0) state=1;
        }

        for (int j=0;j<5 && !state;j++)
            if (fscanf(f,"%i\t",&config.a_pid[j])<0) state = 1;

        fclose(f);

        fflush(NULL);
    }

    //y,p,r + [min,max,kp,ki,kd]
    if (state) {
        printf("No config file or config syntax issue. New will be created!\n");
        //some default values if no config file
        config.rec[0] = 110;
        config.rec[1] = 45;
        config.rec[2] = 45;
        config.rec[3] = 1000;
        config.rec[4] = 1700;
        for (int i=0;i<3;i++) {
            for (int j=0;j<5;j++) 
                config.s_pid[i][j] = config.r_pid[i][j] = 0;
            config.s_pid[i][0] = -180;
            config.s_pid[i][1] = 180;
            config.r_pid[i][0] = -360;
            config.r_pid[i][1] = 360;
        }
        for (int j=0;j<5;j++) 
            config.a_pid[j] = 0;


        config.r_pid[0][2]=2400;  //yaw
        config.r_pid[1][2]=865;  //pitch
        config.r_pid[2][2]=865;  //roll
        config.s_pid[0][2]=9000;  //yaw
        config.s_pid[1][2]=2500;  //pitch
        config.s_pid[2][2]=2500;  //roll
    }

    return 0; 
}

int config_save() {
    f = fopen(p,"w");
    if (f == NULL) {
        printf("Error saving config file.\n");
        return -1;
    }

    for (int j=0;j<5;j++)
        fprintf(f,"%i\t",config.rec[j]);
    fprintf(f,"\n");

    for (int i=0;i<3;i++) {
        for (int j=0;j<5;j++)
            fprintf(f,"%i\t",config.r_pid[i][j]);
        fprintf(f,"\n");
    }

    for (int i=0;i<3;i++) { 
        for (int j=0;j<5;j++)
            fprintf(f,"%i\t",config.s_pid[i][j]);
        fprintf(f,"\n");
    }
    for (int j=0;j<5;j++)
        fprintf(f,"%i\t",config.a_pid[j]);

    fflush(NULL);

    fclose(f);

    return 0;
}


