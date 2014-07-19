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

	if (fscanf(f,"%i\t%i\n",&config.log_seq,&config.log_t)<0) state = 1;

	for (int i=0;i<2 && !state;i++) 
            if (fscanf(f,"%i\t",&config.rec_t[i])<0) state = 1;
        if (fscanf(f,"\n")<0) state=1;

        for (int i=0;i<2 && !state;i++) { 
            for (int j=0;j<3 && !state;j++) 
                if (fscanf(f,"%i\t",&config.rec_ypr[i][j])<0) state = 1;
            if (fscanf(f,"\n")<0) state=1;
        }

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
            if (fscanf(f,"%i\t",&config.alt_pid[j])<0) state = 1;
        if (fscanf(f,"\n")<0) state=1;
        for (int j=0;j<5 && !state;j++)
            if (fscanf(f,"%i\t",&config.vz_pid[j])<0) state = 1;

        fclose(f);

        fflush(NULL);
    }

    //y,p,r + [min,max,kp,ki,kd]
    if (state) {
        printf("No config file or config syntax issue. New will be created!\n");
        //some default values if no config file
	config.log_seq = 0;
	config.log_t = 0;
        config.rec_ypr[0][0] = 110;
        config.rec_ypr[0][1] = 30;
        config.rec_ypr[0][2] = 30;
        config.rec_ypr[1][0] = 110;
        config.rec_ypr[1][1] = 45;
        config.rec_ypr[1][2] = 45;
        config.rec_t[0] = 1000;
        config.rec_t[1] = 1650;
        for (int i=0;i<3;i++) {
            for (int j=0;j<5;j++) 
                config.s_pid[i][j] = config.r_pid[i][j] = 0;
            config.s_pid[i][0] = -180;
            config.s_pid[i][1] = 180;
            config.r_pid[i][0] = -360;
            config.r_pid[i][1] = 360;
        }
        for (int j=0;j<5;j++) { 
            config.alt_pid[j] = 0;
            config.vz_pid[j] = 0;
        }


        config.r_pid[0][2]=2400;  //yaw
        config.r_pid[1][2]=465;  //pitch
        config.r_pid[2][2]=465;  //roll
        config.s_pid[0][2]=9000;  //yaw
        config.s_pid[1][2]=2500;  //pitch
        config.s_pid[2][2]=2500;  //roll

        config.alt_pid[0] = -300;
        config.alt_pid[1] = 300;
        config.vz_pid[0] = -300;
        config.vz_pid[1] = 300;

        config.alt_pid[2] = 400;
        config.vz_pid[2] = 300;
    }

    return 0; 
}

int config_save() {
    f = fopen(p,"w");
    if (f == NULL) {
        printf("Error saving config file.\n");
        return -1;
    }

	fprintf(f,"%i\t%i\n",config.log_seq,config.log_t);

    for (int j=0;j<2;j++)
        fprintf(f,"%i\t",config.rec_t[j]);
    fprintf(f,"\n");

    for (int i=0;i<2;i++) {
        for (int j=0;j<3;j++)
            fprintf(f,"%i\t",config.rec_ypr[i][j]);
        fprintf(f,"\n");
    }

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
        fprintf(f,"%i\t",config.alt_pid[j]);
    
    fprintf(f,"\n");
    for (int j=0;j<5;j++)
        fprintf(f,"%i\t",config.vz_pid[j]);

    fflush(NULL);

    fclose(f);

    return 0;
}


