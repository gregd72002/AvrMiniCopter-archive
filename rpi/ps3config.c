#include "ps3config.h"
#include <stdio.h>

int ps3config_open(struct ps3_config *config,const char *path) {
    static FILE *f;
    int state = 0;
    char p[128];
    sprintf(p,"%sps3.config%c",path,'\0');
    f = fopen(p,"r");
    if (f == NULL) state = 1; 

    printf("Opening config: %s\n",p);

    if (state ==0) {
	if (fscanf(f,"%i\t%i\n",&config->throttle[0],&config->throttle[1])!=2) state = 1;

        for (int i=0;i<2 && !state;i++) { 
            for (int j=0;j<3 && !state;j++) 
                if (fscanf(f,"%i\t",&config->rec_ypr[i][j])!=1) state = 1;
            if (fscanf(f,"\n")<0) state=1;
        }
        fclose(f);

        fflush(NULL);
    }

    if (state) {
        printf("No config file or config syntax issue.\n");
	return -1;
        //some default values if no config file
        config->rec_ypr[0][0] = 135;
        config->rec_ypr[0][1] = 30;
        config->rec_ypr[0][2] = 30;
        config->rec_ypr[1][0] = 135;
        config->rec_ypr[1][1] = 45;
        config->rec_ypr[1][2] = 45;
        config->throttle[0] = 1000;
        config->throttle[1] = 2000;
	
        f = fopen(p,"w");
        if (f == NULL) {
                perror("PS3CONTROLLER: Error saving config!\n");
                return 0;
        } else {
		fprintf(f,"%i\t%i\n",config->throttle[0],config->throttle[1]);
		fprintf(f,"%i\t%i\t%i\n",config->rec_ypr[0][0],config->rec_ypr[0][1],config->rec_ypr[0][2]);
		fprintf(f,"%i\t%i\t%i\n",config->rec_ypr[1][0],config->rec_ypr[1][1],config->rec_ypr[1][2]);
	}
	if (f) fclose(f);
    }

    return 0; 
}


