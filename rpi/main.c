#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <errno.h>
#include <string.h>
#include <signal.h>

#include <stdlib.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>

#include "config.h"
#include "spidev.h"
#include "ps3controller.h"
#include "gpio.h"
#include "flightlog.h"
#include "bmpsensor/interface.h"
#include "pid.h"

#define delay_ms(a) usleep(a*1000)

int ret;
int err = 0;

int trim[3] = {0,0,0};//in degrees * 1000
int mode = 0;
int gentle = 0;

int alt_hold = 0;
int constrain = 0;
int alt_target = 0;
struct s_pid pid_alt;

void sendPIDs() {
    for (int i=0;i<3;i++) 
        for (int j=2;j<5;j++) {
            spi_sendIntPacket(100+i*10+j,&config.r_pid[i][j]);
            spi_sendIntPacket(200+i*10+j,&config.s_pid[i][j]);
        }

}

void sendConfig() {
    for (int i=0;i<3;i++)
        spi_sendIntPacket(20+i,&trim[i]);

    //delay_ms(5);
    for (int i=0;i<3;i++) { 
        for (int j=0;j<5;j++) {
            spi_sendIntPacket(100+i*10+j,&config.r_pid[i][j]);
            //            delay_ms(5);
            spi_sendIntPacket(200+i*10+j,&config.s_pid[i][j]);
            //            delay_ms(5);
        }
    }

    spi_sendIntPacket(0x01,&mode);
}

void checkPIDs() {
    for (int i=0;i<3;i++) {
        if (config.r_pid[i][2]<0) config.r_pid[i][2]=0;
        if (config.r_pid[i][3]<0) config.r_pid[i][3]=0;
        if (config.r_pid[i][4]<0) config.r_pid[i][4]=0;
        if (config.s_pid[i][2]<0) config.s_pid[i][2]=0;
        if (config.s_pid[i][3]<0) config.s_pid[i][3]=0;
        if (config.s_pid[i][4]<0) config.s_pid[i][4]=0;
    }

}

void do_adjustments() {
    if (rec.aux<0) return;

    static int adj1; //for Kp
    static int adj2; //for Ki
    static int adj3 = 500; //for trim

    static int *v1,*v2,*v3,*v4;
    static int _dummy = 0;
    static int *dummy = &_dummy;

    if (mode == 0) { //normal - stabilized flight mode
        adj1 = 100;
        adj2 = 500;
        v1 = &config.s_pid[1][2];
        v2 = &config.s_pid[2][2];
        v3 = &config.s_pid[1][3];
        v4 = &config.s_pid[2][3];
    } else if (mode == 1) { //setup mode - setup yaw
        adj1 = 25;
        v1 = &config.r_pid[0][2];
        v2 = v3 = v4 = dummy;
    } else if (mode == 2) { //setup pitch
        adj1 = 25;
        v1 = &config.r_pid[1][2];
        v2 = v3 = v4 = dummy;
    } else if (mode == 3) { //setup roll
        adj1 = 25;
        v1 = &config.r_pid[2][2];
        v2 = v3 = v4 = dummy;
    }

    switch (rec.aux) {
        case 10:
            if (rec.yprt[3]<1060) {
                (*v1)+=adj1; (*v2)+=adj1; checkPIDs(); sendPIDs(); 
            } 
            if (alt_hold) alt_target+=5;
            break;
        case 8:
            if (rec.yprt[3]<1060) {
                (*v1)-=adj1; (*v2)-=adj1; checkPIDs(); sendPIDs(); 
            } 
            if (alt_hold) alt_target-=5;
            break;
        case 11:
            if (rec.yprt[3]<1060) {
                (*v3)+=adj2; (*v3)+=adj2; checkPIDs(); sendPIDs();
            } 
            break;
        case 9:
            if (rec.yprt[3]<1060) {
                (*v3)-=adj2; (*v3)-=adj2; checkPIDs(); sendPIDs();
            } 
            break;
        case 0:
            if (rec.yprt[3]<1060) {flog_save(); config_save(); sync(); fflush(NULL);}
            break;
        case 3: 
            if (rec.yprt[3]<1060) {
                ret=linuxgpio_initpin(25);
                linuxgpio_highpulsepin(25);
                linuxgpio_close();
                ret=config_open("/var/local/rpicopter.config");
                delay_ms(1000);
                mode = 0;
                sendConfig();
                bs_reset();
            }
            break;
        case 12:
            if (gentle) gentle = 0;
            else gentle = 1;

            if (gentle) {
                config.rec[1] = 25;
                config.rec[2] = 25;
            } else {
                config.rec[1] = 45;
                config.rec[2] = 45;
            }
            break;
        case 1:
            alt_hold = 0;
            break;
        case 13:
            if (constrain) constrain = 0;
            else constrain = 1;
            break;
        case 14:
            if (alt_hold) alt_hold=0;
            else alt_hold = 1;
            /*
               alt_target = bs.alt;
               pid_init(&pid_alt);
               */
            alt_target = rec.yprt[3];
            break;
        case 4:
            trim[1]+=adj3;
            spi_sendIntPacket(21,&trim[1]);
            break;
        case 6:
            trim[1]-=adj3;
            spi_sendIntPacket(21,&trim[1]);
            break;
        case 7:
            trim[2]+=adj3;
            spi_sendIntPacket(22,&trim[2]);
            break;
        case 5:
            trim[2]-=adj3;
            spi_sendIntPacket(22,&trim[2]);
            break;
        case 16:
            if (rec.yprt[3]<1060) {
                mode++;
                if (mode==4) mode=0;
                spi_sendIntPacket(0x01,&mode);
            }
            break;
        default:
            printf("Unknown command %i\n",rec.aux);
    }
    rec.aux=-1; //reset receiver command

} 

void catch_signal(int sig)
{
    printf("signal: %i\n",sig);
    err = 1;
}

long dt_ms = 0;
static struct timespec ts,t1,t2,*dt;

struct timespec *TimeSpecDiff(struct timespec *ts1, struct timespec *ts2)
{
    static struct timespec ts;
    ts.tv_sec = ts1->tv_sec - ts2->tv_sec;
    ts.tv_nsec = ts1->tv_nsec - ts2->tv_nsec;
    if (ts.tv_nsec < 0) {
        ts.tv_sec--;
        ts.tv_nsec += 1000000000;
    }
    return &ts;
}

void log() {
    flog_push(3,                                                              
            (float)t2.tv_sec-ts.tv_sec
            ,bs.alt,bs.t 
            );
    printf("T: %li\tA: %2.1f\tT: %2.1f\n",t2.tv_sec-ts.tv_sec,bs.alt,bs.t);
}

#define MAX_SAFE_STACK (MAX_LOG*MAX_VALS + 64*1024)
void stack_prefault(void) {

    unsigned char dummy[MAX_SAFE_STACK];

    memset(dummy, 0, MAX_SAFE_STACK);
    return;
}

unsigned long c = 0,k = 0;

void loop() {
    clock_gettime(CLOCK_REALTIME,&t2);                                           
    ts = t1 = t2;

    pid_init(&pid_alt);
    pid_setmode(&pid_alt,1);

    while (1 && !err) {
        ret = rec_update(); 
        // 0 - no update but read ok
        // 1 - update
        if (ret < 0) {
            printf("Receiver reading error: [%s]\n",strerror(ret));
            err = 1;
            return;
        }


        clock_gettime(CLOCK_REALTIME,&t2);                                           
        dt = TimeSpecDiff(&t2,&t1);
        dt_ms = dt->tv_sec*1000 + dt->tv_nsec/1000000;

        //if (ret == 0 && dt_ms<20) continue;
        if (dt_ms<20) continue;
        t1 = t2;
        c++;
        bs_update(c*20);
        if (!(c%10)) {
            log();
        }

        //will get here every 20ms
        do_adjustments();

        if (alt_hold) {
            rec.yprt[3] = alt_target;
        }

        for (int i=0;i<4;i++) {
            spi_sendIntPacket(0x0A+i,rec.yprt+i);
        }
        int alt = (bs.alt * 10);
        spi_sendIntPacket(0x0E,&alt);
    }
}

int main() {

    signal(SIGTERM, catch_signal);
    signal(SIGINT, catch_signal);

    struct sched_param param;

    param.sched_priority = 49;
    if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
        exit(-1);
    }
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
        perror("mlockall failed");
        exit(-2);
    }

    stack_prefault();
    ret=config_open("/var/local/rpicopter.config");
    if (ret<0) {
        printf("Failed to initiate config! [%s]\n", strerror(ret));	
        return -1;
    }

    ret=flog_open("/var/local/");                                                
    if (ret<0) {                                                                 
        printf("Failed to initiate log! [%s]\n", strerror(err));         
        return -1;                                                               
    }   

    ret=rec_open();
    if (ret<0) {
        printf("Failed to initiate receiver! [%s]\n", strerror(ret));	
        return -1;
    }

    ret=spi_init();
    if (ret <0) {
        printf("Failed to initiate spi! [%s]\n", strerror(ret));	
        return -1;
    }


    ret=bs_open();
    if (ret<0) {
        printf("Failed to initiate pressure sensor! [%s]\n", strerror(err));	
        return -1;
    }

    delay_ms(100);
    printf("int size: %i\nlong size: %i\nfloat size: %i\nyprt size: %i\n",sizeof(int),sizeof(long),sizeof(float),sizeof(rec.yprt));
    printf("Starting main loop...\n");
    loop();
    printf("Closing.\n");
    return 0;
}
