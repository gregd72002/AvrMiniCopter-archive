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

unsigned long flight_time = 0;

int trim[3] = {0,0,0};//in degrees * 1000
int mode = 0;
int rec_setting = 0;

int alt_hold = 0;
int throttle_hold = 0;
int throttle_target = 0;

void sendPIDs() {
    for (int i=0;i<3;i++) 
        for (int j=0;j<5;j++) {
            spi_sendIntPacket(100+i*10+j,&config.r_pid[i][j]);
            spi_sendIntPacket(200+i*10+j,&config.s_pid[i][j]);
        }

    for (int i=0;i<3;i++) { 
        printf("Sending pid: %i v: %i",80+i,config.alt_pid[i]);
        spi_sendIntPacket(80+i,&config.alt_pid[i]);
        spi_sendIntPacket(90+i,&config.vz_pid[i]);
    }
}

void sendConfig() {
    for (int i=0;i<3;i++)
        spi_sendIntPacket(20+i,&trim[i]);

    sendPIDs();

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
    static int adj4 = 500; //for altitude (mm)

    static int *v1,*v2,*v3,*v4;
    static int _dummy = 0;
    static int *dummy = &_dummy;

    if (mode == 0) { //normal - stabilized flight mode
        adj1 = 100;
        adj2 = 100;
        v1 = &config.s_pid[1][2];
        v2 = &config.s_pid[2][2];
        v3 = &config.s_pid[1][3];
        v4 = &config.s_pid[2][3];
    } else if (mode == 1) { //rate
        adj1 = 50;
        adj2 = 50;
        v1 = &config.r_pid[1][2];
        v2 = &config.r_pid[2][2];
        v3 = &config.r_pid[1][3];
        v4 = &config.r_pid[2][3];
    } 

    switch (rec.aux) {
        case 10:
            if (rec.yprt[3]<1060 && !alt_hold) {
                (*v1)+=adj1; (*v2)+=adj1; checkPIDs(); sendPIDs(); 
            } else {
                config.alt_pid[2] += 5;
                spi_sendIntPacket(82,&config.alt_pid[2]);
            }
            break;
        case 8:
            if (rec.yprt[3]<1060 && !alt_hold) {
                (*v1)-=adj1; (*v2)-=adj1; checkPIDs(); sendPIDs(); 
            } else {
                config.alt_pid[2] -= 5;
                spi_sendIntPacket(82,&config.alt_pid[2]);
            }
            break;
        case 11:
            if (rec.yprt[3]<1060 && !alt_hold) {
                (*v3)+=adj2; (*v4)+=adj2; checkPIDs(); sendPIDs();
            } else {
                int t = adj4;
                spi_sendIntPacket(0xA0,&t);
            }
            break;
        case 9:
            if (rec.yprt[3]<1060 && !alt_hold) {
                (*v3)-=adj2; (*v4)-=adj2; checkPIDs(); sendPIDs();
            } else {
                int t = -adj4;
                spi_sendIntPacket(0xA0,&t);
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
                alt_hold = 0;
                throttle_hold = 0;
                sendConfig();
                bs_reset();
            }
            break;
        case 12:
            if (rec_setting) rec_setting = 0;
            else rec_setting = 1;
            rec_setSetting(rec_setting);
            break;
        case 1:
            alt_hold = 0;
            throttle_hold = 0;
            spi_sendIntPacket(0x0F,&alt_hold);
            break;
        case 13:
            /*
            if (throttle_hold) throttle_hold=0;
            else throttle_hold = 1;
            throttle_target = rec.yprt[3];
            */
            break;
        case 14:
            if (alt_hold) alt_hold=0;
            else alt_hold = 1;
            spi_sendIntPacket(0x0F,&alt_hold);
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
            if (rec.yprt[3]<1060 && !alt_hold) {
                mode++;
                if (mode==2) mode=0;
                spi_sendIntPacket(0x01,&mode);
            }
            break;
        default:
            printf("Unknown command %i\n",rec.aux);
    }
    if (rec.aux!=-1) printf("Button: %i\n",rec.aux);
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
    flog_push(6, 
            (float)t2.tv_sec-ts.tv_sec
            ,bs.alt,bs.t
            ,spi_v[10]/10.0f,spi_v[11]/100.0f,spi_v[12]/100.0f
            );
#ifdef DEBUG
    //printf("T: %li\tA: %2.1f\tT: %2.1f\t\talt: %2.1f\t\tvz: %2.2f\t\the: %2.1f\n",t2.tv_sec-ts.tv_sec,bs.alt,bs.t,spi_v[10]/10.0f,spi_v[11]/100.0f,spi_v[12]/100.0f);
    printf("T: %li\tdp: %2.2f\tdr: %2.2f\t\tdip: %2.1f\tdir: %2.2f\n",
            t2.tv_sec-ts.tv_sec
            ,spi_v[0x20]/100.0f,spi_v[0x21]/100.0f,spi_v[0x22]/100.0f,spi_v[0x23]/100.0f
            );
#endif
}

void log_accel() {
    flog_push(8, 
            (float)t2.tv_sec-ts.tv_sec
            ,(float)flight_time
            ,spi_v[0x20]/1000.0f,spi_v[0x21]/1000.0f,spi_v[0x22]/1000.0f
            ,spi_v[0x30]/1000.0f,spi_v[0x31]/1000.0f,spi_v[0x32]/1000.0f
            );
#ifdef DEBUG
   //if (!(c%5)) 
       printf("T: %li\tax: %2.3f\t\ay: %2.3f\t\az: %2.3f\tbx: %2.3f\tby: %2.3f\tbz: %2.3f\n",
               flight_time,spi_v[0x20]/1000.0f,spi_v[0x21]/1000.0f,spi_v[0x22]/1000.0f,spi_v[0x30]/1000.0f,spi_v[0x31]/1000.0f,spi_v[0x32]/1000.0f);
#endif
}

#define MAX_SAFE_STACK (MAX_LOG*MAX_VALS + 64*1024)
void stack_prefault(void) {

    unsigned char dummy[MAX_SAFE_STACK];

    memset(dummy, 0, MAX_SAFE_STACK);
    return;
}

unsigned long c = 0,k = 0;

void loop() {
    sendConfig();
    bs_reset();
    clock_gettime(CLOCK_REALTIME,&t2);                                           
    ts = t1 = t2;

    while (1 && !err) {
        ret = rec_update(); 
        // 0 - no update but read ok
        // 1 - update
        if (ret < 0) {
            printf("Receiver reading error: [%s]\n",strerror(ret));
            err = 1;
            return;
        }

        if (alt_hold && abs(rec.yprt[3]) > (config.rec_t[1]-50)) {
            alt_hold = 0;
            throttle_hold = 0;
            spi_sendIntPacket(0x0F,&alt_hold);
        }
        


        clock_gettime(CLOCK_REALTIME,&t2);                                           
        dt = TimeSpecDiff(&t2,&t1);
        dt_ms = dt->tv_sec*1000 + dt->tv_nsec/1000000;

        //if (ret == 0 && dt_ms<20) continue;
        if (dt_ms<15) continue; //4 packets normally take 50ms anyway, so it should not stop in here
        t1 = t2;

        if (alt_hold || rec.yprt[3]>config.rec_t[0]+50)
            flight_time += dt_ms; 

        c++;
        bs_update(c*dt_ms);
        /*
        if (!(c%5)) {
            log();
        }*/
        //log_accel();
        log();


        //will get here every 20ms
        do_adjustments();

        if (throttle_hold) {
            rec.yprt[3] = throttle_target;
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
