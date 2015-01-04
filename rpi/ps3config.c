#include "ps3config.h"
#include <stdio.h>

struct ps3_config ps3config;

static FILE *f;
static const char *p;

int ps3config_open(struct ps3_config *config,const char *path) {
    int state = 0;
    p = path;
    f = fopen(p,"r");
    if (f == NULL) state = 1; 

    if (state ==0) {

	if (fscanf(f,"%i\t%i\t%i\n",&config->log_seq,&config->log_t,&config->cam_seq)!=3) state = 1;


	if (fscanf(f,"%i\t%i\n",&config->throttle[0],&config->throttle[1])!=1) state = 1;

        for (int i=0;i<2 && !state;i++) { 
            for (int j=0;j<3 && !state;j++) 
                if (fscanf(f,"%i\t",&config->rec_ypr[i][j])!=1) state = 1;
            if (fscanf(f,"\n")<0) state=1;
        }
        fclose(f);

        fflush(NULL);
    }

    if (state) {
        printf("No config file or config syntax issue. New will be created!\n");
        //some default values if no config file
	config->log_seq = 0;
	config->cam_seq = 0;
	config->log_t = 0;
        config->rec_ypr[0][0] = 135;
        config->rec_ypr[0][1] = 30;
        config->rec_ypr[0][2] = 30;
        config->rec_ypr[1][0] = 135;
        config->rec_ypr[1][1] = 45;
        config->rec_ypr[1][2] = 45;
        config->throttle[0] = 1000;
        config->throttle[1] = 2000;
    }

    return 0; 
}

