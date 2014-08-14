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

	if (fscanf(f,"%i\t%i\t%i\n",&config.log_seq,&config.log_t,&config.cam_seq)<0) state = 1;

	for (int i=0;i<3 && !state;i++) 
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
        if (fscanf(f,"\n")<0) state=1;
        for (int j=0;j<5 && !state;j++)
            if (fscanf(f,"%i\t",&config.accel_pid[j])<0) state = 1;
        if (fscanf(f,"\n")<0) state=1;

        if (fscanf(f,"%i\t",&config.a_pid[0])<0) state = 1;
        if (fscanf(f,"\n")<0) state=1;

        for (int i=0;i<9 && !state;i++)
            if (fscanf(f,"%i\t",&config.gyro_orient[i])<0) state = 1;
        if (fscanf(f,"\n")<0) state=1;

        for (int i=0;i<4 && !state;i++)
            if (fscanf(f,"%i\t",&config.motor_pin[i])<0) state = 1;
        if (fscanf(f,"\n")<0) state=1;

        if (fscanf(f,"%i\t",&config.mpu_addr)<0) state = 1;

        fclose(f);

        fflush(NULL);
    }

    //y,p,r + [min,max,kp,ki,kd]
    if (state) {
        printf("No config file or config syntax issue. New will be created!\n");
        //some default values if no config file
	config.log_seq = 0;
	config.cam_seq = 0;
	config.log_t = 0;
        config.rec_ypr[0][0] = 135;
        config.rec_ypr[0][1] = 30;
        config.rec_ypr[0][2] = 30;
        config.rec_ypr[1][0] = 135;
        config.rec_ypr[1][1] = 45;
        config.rec_ypr[1][2] = 45;
        config.rec_t[0] = 1000;
        config.rec_t[1] = 2000;
        config.rec_t[2] = 1100;
        for (int i=0;i<3;i++) {
            for (int j=0;j<5;j++) 
                config.s_pid[i][j] = config.r_pid[i][j] = 0;
            config.s_pid[i][0] = 500;
            config.s_pid[i][1] = 50;
            config.r_pid[i][0] = 500;
            config.r_pid[i][1] = 50;
        }
        for (int j=0;j<5;j++) { 
            config.alt_pid[j] = 0;
            config.vz_pid[j] = 0;
            config.accel_pid[j] = 0;
        }

	config.a_pid[0] = 4000;


        config.r_pid[0][2]=1300;  //yaw
        config.r_pid[1][2]=500;  //pitch
        config.r_pid[2][2]=500;  //roll
        config.s_pid[0][2]=2500;  //yaw
        config.s_pid[1][2]=2500;  //pitch
        config.s_pid[2][2]=2500;  //roll

        config.alt_pid[0] = 500;
        config.alt_pid[1] = 5;
        config.vz_pid[0] = 500;
        config.vz_pid[1] = 5;
        config.accel_pid[0] = 500;
        config.accel_pid[1] = 5;

        config.alt_pid[2] = 750;
        config.vz_pid[2] = 6000;
        config.accel_pid[2] = 750;
        config.accel_pid[3] = 1500;

        for (int i=0;i<9;i++) config.gyro_orient[i] = 0; 

	config.gyro_orient[0] = config.gyro_orient[4] = config.gyro_orient[8] = 1;

	config.motor_pin[0] = 3;
	config.motor_pin[1] = 5;
	config.motor_pin[2] = 6;
	config.motor_pin[3] = 9;

	config.mpu_addr = 0;
    }

    return 0; 
}

int config_save() {
    f = fopen(p,"w");
    if (f == NULL) {
        printf("Error saving config file.\n");
        return -1;
    }

	fprintf(f,"%i\t%i\t%i\n",config.log_seq,config.log_t,config.cam_seq);

    for (int j=0;j<3;j++)
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
    fprintf(f,"\n");

    for (int j=0;j<5;j++)
        fprintf(f,"%i\t",config.accel_pid[j]);
    fprintf(f,"\n");

    fprintf(f,"%i\t",config.a_pid[0]);
    fprintf(f,"\n");

	for (int i=0;i<9;i++)
	    fprintf(f,"%i\t",config.gyro_orient[i]);
    fprintf(f,"\n");

    for (int i=0;i<4;i++)
        fprintf(f,"%i\t",config.motor_pin[i]);
    fprintf(f,"\n");

    fprintf(f,"%i\t",config.mpu_addr);


    fflush(NULL);

    fclose(f);

    return 0;
}


