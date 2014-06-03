#include "freeram.h"

#include "mpu.h"
#include "I2Cdev.h"
#include <SPI.h>
#include "SPIdev.h"
#include "crc8.h"
#include "pid.h"
#include <Servo.h>

int ret;
Servo myservo[4];
#define SERVO_FL 0
#define SERVO_BL 1
#define SERVO_FR 2
#define SERVO_BR 3
#define MOTOR_INIT_US 1000
#define INFLIGHT_THRESHOLD 1060 //below this we will be writing MOTOR_INIT_US
#define INTEGRAL_THRESHOLD 1150

struct s_pid pid_r[3];
struct s_pid pid_s[3];

void setup() {
    Fastwire::setup(600,0);
    //Serial.begin(9600);
#ifdef DEBUG
    //Serial.begin(38400);
    Serial.begin(115200);
#endif
    SPI.setDataMode(SPI_MODE0);
    pinMode(MISO,OUTPUT);
    SPCR |= _BV(SPE);
    //SPI.setClockDivider(SPI_CLOCK_DIV4);
    SPI.attachInterrupt();
#ifdef DEBUG
    Serial.print("MPU init: "); Serial.println(ret);
    Serial.print("Free mem: "); Serial.println(freeRam());
    Serial.print("int size: "); Serial.println(sizeof(int));
    Serial.print("long size: "); Serial.println(sizeof(long));
    Serial.print("float size: "); Serial.println(sizeof(float));
#endif

    myservo[SERVO_FL].attach(3);
    myservo[SERVO_FL].writeMicroseconds(MOTOR_INIT_US);
    myservo[SERVO_BL].attach(5);
    myservo[SERVO_BL].writeMicroseconds(MOTOR_INIT_US);
    myservo[SERVO_FR].attach(6);
    myservo[SERVO_FR].writeMicroseconds(MOTOR_INIT_US);
    myservo[SERVO_BR].attach(9);
    myservo[SERVO_BR].writeMicroseconds(MOTOR_INIT_US);
     for (int i=0;i<3;i++) {
        pid_init(&pid_r[i]);
        pid_init(&pid_s[i]);
    }
}

unsigned char loop_count = 0;
unsigned int c = 0; //cumulative number of successful MPU/DMP reads
unsigned int np = 0; //cumulative number of MPU/DMP reads that brought no packet back
unsigned int err_c = 0; //cumulative number of MPU/DMP reads that brought corrupted packet
unsigned int err_o = 0; //cumulative number of MPU/DMP reads that had overflow bit set

unsigned char armed = 0;
unsigned char emergency = 0;
unsigned char inflight = 0;
int mode = 0;
int fly_mode = 0;

float yaw_target = 0.0f;

union s_packet {
    byte b[3];
    struct {
        byte t;
        int v;
    };
} packet;

int yprt[4] = {0,0,0,0};
float trim[3] = {0.f,0.f,0.f}; //0-pitch; 1-roll

unsigned int mpu_err = 0;
unsigned int rec_err = 0;

int process_command() { 
    static unsigned long last_command = millis();
    if (millis() - last_command>500) {
#ifdef DEBUG
        Serial.println("No communication!"); 
#endif
        yprt[0]=yprt[1]=yprt[2]=yprt[3]=0;
//        return 0;
    }
    //each command is 4 byte long: what, value(2), crc - do it till buffer empty

    if (SPI_getPacket(packet.b)==0) {
        last_command = millis();
        //Serial.print("Type: "); Serial.print(packet.t,DEC); Serial.print(" Value: "); Serial.println(packet.v,DEC);
        switch(packet.t) {
            case 0x01: fly_mode = packet.v; break;
            case 0x0A: yprt[0] = packet.v; break;
            case 0x0B: yprt[1] = packet.v; break;
            case 0x0C: yprt[2] = packet.v; break;
            case 0x0D: yprt[3] = packet.v; break;
            case 20: trim[0] = (float)packet.v/1000.f; break;
            case 21: trim[1] = (float)packet.v/1000.f; break;
            case 22: trim[2] = (float)packet.v/1000.f; break;

            case 100: pid_r[0].min = packet.v; break; 
            case 101: pid_r[0].max = packet.v; break; 
            case 102: pid_r[0].Kp = (float)packet.v/1000.f; break; 
            case 103: pid_r[0].Ki = (float)packet.v/1000.f; break; 
            case 104: pid_r[0].Kd = (float)packet.v/1000.f; break; 
            case 110: pid_r[1].min = packet.v; break; 
            case 111: pid_r[1].max = packet.v; break; 
            case 112: pid_r[1].Kp = (float)packet.v/1000.f; break; 
            case 113: pid_r[1].Ki = (float)packet.v/1000.f; break; 
            case 114: pid_r[1].Kd = (float)packet.v/1000.f; break; 
            case 120: pid_r[2].min = packet.v; break; 
            case 121: pid_r[2].max = packet.v; break; 
            case 122: pid_r[2].Kp = (float)packet.v/1000.f; break; 
            case 123: pid_r[2].Ki = (float)packet.v/1000.f; break; 
            case 124: pid_r[2].Kd = (float)packet.v/1000.f; break; 

            case 200: pid_s[0].min = packet.v; break; 
            case 201: pid_s[0].max = packet.v; break; 
            case 202: pid_s[0].Kp = (float)packet.v/1000.f; break; 
            case 203: pid_s[0].Ki = (float)packet.v/1000.f; break; 
            case 204: pid_s[0].Kd = (float)packet.v/1000.f; break; 
            case 210: pid_s[1].min = packet.v; break; 
            case 211: pid_s[1].max = packet.v; break; 
            case 212: pid_s[1].Kp = (float)packet.v/1000.f; break; 
            case 213: pid_s[1].Ki = (float)packet.v/1000.f; break; 
            case 214: pid_s[1].Kd = (float)packet.v/1000.f; break; 
            case 220: pid_s[2].min = packet.v; break; 
            case 221: pid_s[2].max = packet.v; break; 
            case 222: pid_s[2].Kp = (float)packet.v/1000.f; break; 
            case 223: pid_s[2].Ki = (float)packet.v/1000.f; break; 
            case 224: pid_s[2].Kd = (float)packet.v/1000.f; break; 

            default: 
#ifdef DEBUG
                      Serial.print("Unknown command: "); Serial.println(packet.t);
#endif
              return 0;
        }
    }

    return 0;
}


void log() {
#ifdef DEBUG
    if (!(loop_count%100)) {
        Serial.print(np); Serial.print(" OK: "); Serial.print(c); Serial.print(" C:"); Serial.print(err_c); Serial.print(" O:"); Serial.print(err_o);

        Serial.print(" YT: "); Serial.print(yaw_target);
        Serial.print(" Y: "); Serial.print(mympu.ypr[0]);
        Serial.print(" P: "); Serial.print(mympu.ypr[1]);
        Serial.print(" R: "); Serial.print(mympu.ypr[2]);

        Serial.print("\tgy: "); Serial.print(mympu.gyro[0]);
        Serial.print(" gp: "); Serial.print(mympu.gyro[1]);
        Serial.print(" gr: "); Serial.print(mympu.gyro[2]);

        Serial.print("\try: "); Serial.print(yprt[0]);
        Serial.print(" rp: "); Serial.print(yprt[1]);
        Serial.print(" rr: "); Serial.print(yprt[2]);
        Serial.print(" rt: "); Serial.print(yprt[3]);

        Serial.println();
    }
#endif
}

float loop_s = 0.0f;
unsigned long p_millis = 0;
bool stop = 0;

void controller_loop() {
    if (stop) {
        myservo[SERVO_FL].writeMicroseconds(MOTOR_INIT_US);
        myservo[SERVO_BL].writeMicroseconds(MOTOR_INIT_US);
        myservo[SERVO_FR].writeMicroseconds(MOTOR_INIT_US);
        myservo[SERVO_BR].writeMicroseconds(MOTOR_INIT_US);
        return;
    }
    ret = mympu_update();
    if (ret < 0) {
        //mpu_err++;
        mympu.ypr[0]=mympu.ypr[1]=mympu.ypr[2] = 0.0f;
        mympu.gyro[0]=mympu.gyro[1]=mympu.gyro[2] = 0.0f;
        stop = 1;
        if (ret<=-100) stop = 1;
        return;
    }
#ifdef DEBUG
    switch (ret) {
        case 0: c++; break;
        case 1: np++; return; 
        case 2: err_o++; return;
        case 3: err_c++; return;
    }

    loop_count++;
#endif
    
    if (yprt[3]<INTEGRAL_THRESHOLD) { //use integral part only if there is some throttle
        yaw_target = mympu.ypr[0];
        for (int i=0;i<3;i++) {                                              
            pid_r[i]._KiTerm = 0.0f;                                  
            pid_s[i]._KiTerm = 0.0f;                                  
        }                                                                    
    }       

    if (yaw_target-mympu.ypr[0]<-180.0f) yaw_target*=-1;                        
    if (yaw_target-mympu.ypr[0]>180.0f) yaw_target*=-1;     

    //do STAB PID                                                            
    loop_s = (float)(millis() - p_millis)/1000.0f;
    p_millis = millis();
    for (int i=0;i<3;i++) {                                                  
        if (i==0) //keep yaw_target                                          
            pid_update(&pid_s[0],yaw_target,mympu.ypr[0],loop_s);        
        else 
            //pid_update(&pid_s[i],0,mympu.ypr[i],loop_s);
            pid_update(&pid_s[i],yprt[i]+trim[i],mympu.ypr[i],loop_s);
    }      


    //yaw requests will be fed directly to rate pid                          
    if (abs(yprt[0])>7) {                                             
        pid_s[0].value = yprt[0];                                 
        yaw_target = mympu.ypr[0];                                              
    }                                                                        

    if (fly_mode == 1) { //yaw setup                                             
        pid_s[0].value = 0.0f;                                        
        pid_s[1].value = mympu.gyro[1];                                  
        pid_s[2].value = mympu.gyro[2];                                  
    } else if (fly_mode == 2) { //pitch setup                                    
        pid_s[0].value = mympu.gyro[0];                                  
        pid_s[1].value = 0.0f;                                        
        pid_s[2].value = mympu.gyro[2];                                  
    } else if (fly_mode == 3) { //roll setup                                     
        pid_s[0].value = mympu.gyro[0];                                  
        pid_s[1].value = mympu.gyro[1];                                  
        pid_s[2].value = 0.0f;                                        
    }                                                                        

    //do RATE PID                                                            
    for (int i=0;i<3;i++) {                                                  
        pid_update(&pid_r[i],pid_s[i].value,mympu.gyro[i],loop_s);
    }                                                                        

#ifdef DEBUG
    log();
#endif

    if (yprt[3] < INFLIGHT_THRESHOLD) {
            myservo[SERVO_FL].writeMicroseconds(MOTOR_INIT_US);
            myservo[SERVO_BL].writeMicroseconds(MOTOR_INIT_US);
            myservo[SERVO_FR].writeMicroseconds(MOTOR_INIT_US);
            myservo[SERVO_BR].writeMicroseconds(MOTOR_INIT_US);
            return;
    }

    //calculate motor speeds                                                 
    myservo[SERVO_FL].writeMicroseconds(yprt[3]-pid_r[2].value-pid_r[1].value+pid_r[0].value);
    myservo[SERVO_BL].writeMicroseconds(yprt[3]-pid_r[2].value+pid_r[1].value-pid_r[0].value);
    myservo[SERVO_FR].writeMicroseconds(yprt[3]+pid_r[2].value-pid_r[1].value-pid_r[0].value);
    myservo[SERVO_BR].writeMicroseconds(yprt[3]+pid_r[2].value+pid_r[1].value+pid_r[0].value);
}

int check_init() {
    for (int i=0;i<3;i++) { //as the minimum we need min and max for pid and Kp value
        if (pid_r[i].min==0) return -1;
        if (pid_r[i].max==0) return -1;
        if (pid_r[i].Kp==0) return -1;
        if (pid_s[i].min==0) return -1;
        if (pid_s[i].max==0) return -1;
        if (pid_s[i].Kp==0) return -1;
    }

    for (int i=0;i<3;i++) {                                                      
         pid_setmode(&pid_r[i],1);                                         
         pid_setmode(&pid_s[i],1);                                         
    } 

    return 0;
}

int gyroCal() {
    ret = mympu_update();
    if (ret!=0) {
        return -1;
    }
    if (mympu.gyro[0]>-1.0f && mympu.gyro[1]>-1.0f && mympu.gyro[2]>-1.0f &&    
         mympu.gyro[0]<1.0f && mympu.gyro[1]<1.0f && mympu.gyro[2]<1.0f) {
#ifdef DEBUG
    Serial.println("Gyro calibration ok.");
#endif
        
        return 0;
    }
    return -1;
}

void loop() {
    process_command();

    switch (mode) {
        case 0:
            if (check_init()==0) mode = 1;
            break;
        case 1: 
            ret = mympu_open(200);
            if (ret == 0) mode = 2;
        break;

        case 2:
            if (gyroCal()==0) mode = 3;
        break;

        case 3:
            controller_loop();
        break;
    }
}

