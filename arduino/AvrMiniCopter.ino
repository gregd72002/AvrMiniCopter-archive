#include "freeram.h"

union s_crcerr {
    byte b[2];
    unsigned int i;
} crcerr;

#include "mpu.h"
#include "I2Cdev.h"
#include <SPI.h>
#include "SPIdev.h"
#include "crc8.h"
#include "pid.h"
#include <Servo.h>
#include <EEPROM.h>

int ret;
Servo myservo[4];
#define SERVO_FL 0
#define SERVO_BL 1
#define SERVO_FR 2
#define SERVO_BR 3

#define FL_PIN 5
#define BL_PIN 3
#define FR_PIN 9
#define BR_PIN 6

#define MOTOR_INIT_US 1000
#define INFLIGHT_THRESHOLD 1065 //below this we will be writing MOTOR_INIT_US
#define INTEGRAL_THRESHOLD 1100

struct s_pid pid_r[3];
struct s_pid pid_s[3];
struct s_pid pid_alt, pid_vz;


void initMotors() {
    myservo[SERVO_BR].writeMicroseconds(MOTOR_INIT_US);
    myservo[SERVO_FL].writeMicroseconds(MOTOR_INIT_US);
    myservo[SERVO_FR].writeMicroseconds(MOTOR_INIT_US);
    myservo[SERVO_BL].writeMicroseconds(MOTOR_INIT_US);
}

void testServo() {
    for (int i=0;i<4;i++)
    {
        myservo[i].writeMicroseconds(1090);
        delay(3000);
        myservo[i].writeMicroseconds(MOTOR_INIT_US);
        delay(2000);
    }
}

void testServo1() {
    for (int i=0;i<4;i++)
        myservo[i].writeMicroseconds(1080);

    delay(4000);
    for (int i=0;i<4;i++)
    {
        myservo[i].writeMicroseconds(1175);
    }
    delay(4000);
    for (int i=0;i<4;i++)
        myservo[i].writeMicroseconds(MOTOR_INIT_US);
}

void setup() {
    crcerr.i = 0;
    Fastwire::setup(400,0);
    //Serial.begin(9600);
#ifdef DEBUG
    //Serial.begin(38400);
    Serial.begin(115200);
#endif
    SPI.setDataMode(SPI_MODE0);
    pinMode(MISO,OUTPUT);
    SPCR |= _BV(SPE);
    SPI.attachInterrupt(); //alternatively:SPCR |= _BV(SPIE);
#ifdef DEBUG
    Serial.print("MPU init: "); Serial.println(ret);
    Serial.print("Free mem: "); Serial.println(freeRam());
    Serial.print("int size: "); Serial.println(sizeof(int));
    Serial.print("long size: "); Serial.println(sizeof(long));
    Serial.print("float size: "); Serial.println(sizeof(float));
#endif
    myservo[SERVO_FL].attach(FL_PIN);
    myservo[SERVO_BL].attach(BL_PIN);
    myservo[SERVO_FR].attach(FR_PIN);
    myservo[SERVO_BR].attach(BR_PIN);

    initMotors();

/*
    delay(3000);
    testServo1();
    testServo();
*/
    for (int i=0;i<3;i++) {
        pid_init(&pid_r[i]);
        pid_init(&pid_s[i]);
    }
    pid_init(&pid_alt);
    pid_init(&pid_vz);
}

unsigned char loop_count = 0;
unsigned int c = 0; //cumulative number of successful MPU/DMP reads
unsigned int np = 0; //cumulative number of MPU/DMP reads that brought no packet back
unsigned int err_c = 0; //cumulative number of MPU/DMP reads that brought corrupted packet
unsigned int err_o = 0; //cumulative number of MPU/DMP reads that had overflow bit set


unsigned char armed = 0;
unsigned char emergency = 0;
unsigned char inflight = 0;
int config_count = 1; //when 0 this means config has been received
int mode = 0;
int fly_mode = 0;
int log_mode = 0;
unsigned short gyro_orientation;

float yaw_target = 0.0f;

union s_packet {
    byte b[4];
    struct {
        byte t;
        int v;
        byte c;
    };
} packet;

int yprt[4] = {0,0,0,0};
float trim[3] = {0.f,0.f,0.f}; //0-pitch; 1-roll
float altitude = 0.0f;

int alt_hold = 0;
float alt_hold_altitude = 0.0f;
int alt_hold_throttle = 0;
float smooth_az = 0.0f;

float k_vz = -0.001f;
float k_h_est = -0.008f;
float vz_est = 0.f;
float h_est = 0.f;

unsigned int mpu_err = 0;
unsigned int rec_err = 0;

int m_fl,m_bl,m_fr,m_br;


int process_command() { 
    static unsigned long last_command = millis();
    if (millis() - last_command>900) {
#ifdef DEBUG
        Serial.println("No communication!"); 
#endif
        alt_hold=yprt[0]=yprt[1]=yprt[2]=yprt[3]=0;
        //        return 0;
    }
    //each command is 4 byte long: what, value(2), crc - do it till buffer empty

    if (SPI_getPacket(packet.b)==0) {
        last_command = millis();
        switch(packet.t) {
	    case 0x01: config_count = packet.v; break;
            case 0x02: log_mode = packet.v; 
		       config_count--;
			break;
            case 0x03: fly_mode = packet.v; 
		       config_count--;
                       break;
	    case 0x04: 
			gyro_orientation = packet.v;
			break;
            case 0x0A: yprt[0] = packet.v; break;
            case 0x0B: yprt[1] = packet.v; break;
            case 0x0C: yprt[2] = packet.v; break;
            case 0x0D: yprt[3] = packet.v; break;
            case 0x0E: altitude = packet.v/10.f; break;
            case 0x0F: alt_hold = packet.v; 
                       alt_hold_throttle = yprt[3]; 
                       alt_hold_altitude = h_est;
                       vz_est = 0.f;
#ifdef DEBUG
                       Serial.print("Alt hold: "); Serial.println(alt_hold);
#endif
                       break;
            case 0xA0: alt_hold_altitude += packet.v/1000.f; break;
            case 20: trim[0] = (float)packet.v/1000.f; break;
            case 21: trim[1] = (float)packet.v/1000.f; break;
            case 22: trim[2] = (float)packet.v/1000.f; break;

            case 80: pid_alt.min = packet.v; config_count--; break; 
            case 81: pid_alt.max = packet.v; config_count--; break; 
            case 82: pid_alt.Kp = (float)packet.v/10.f; config_count--; break; 
            case 83: pid_alt.Ki = (float)packet.v/10.f; config_count--; break; 
            case 84: pid_alt.Kd = (float)packet.v/10.f; config_count--; break; 
            case 90: pid_vz.min = packet.v; config_count--; break; 
            case 91: pid_vz.max = packet.v; config_count--; break; 
            case 92: pid_vz.Kp = (float)packet.v/10.f; config_count--; break; 
            case 93: pid_vz.Ki = (float)packet.v/10.f; config_count--; break; 
            case 94: pid_vz.Kd = (float)packet.v/10.f; config_count--; break; 

            case 100: pid_r[0].min = packet.v; config_count--; break; 
            case 101: pid_r[0].max = packet.v; config_count--; break; 
            case 102: pid_r[0].Kp = (float)packet.v/1000.f; config_count--; break; 
            case 103: pid_r[0].Ki = (float)packet.v/1000.f; config_count--; break; 
            case 104: pid_r[0].Kd = (float)packet.v/10000.f; config_count--; break; 
            case 110: pid_r[1].min = packet.v; config_count--; break; 
            case 111: pid_r[1].max = packet.v; config_count--; break; 
            case 112: pid_r[1].Kp = (float)packet.v/1000.f; config_count--; break; 
            case 113: pid_r[1].Ki = (float)packet.v/1000.f; config_count--; break; 
            case 114: pid_r[1].Kd = (float)packet.v/10000.f; config_count--; break; 
            case 120: pid_r[2].min = packet.v; config_count--; break; 
            case 121: pid_r[2].max = packet.v; config_count--; break; 
            case 122: pid_r[2].Kp = (float)packet.v/1000.f; config_count--; break; 
            case 123: pid_r[2].Ki = (float)packet.v/1000.f; config_count--; break; 
            case 124: pid_r[2].Kd = (float)packet.v/10000.f; config_count--; break; 

            case 200: pid_s[0].min = packet.v; config_count--; break; 
            case 201: pid_s[0].max = packet.v; config_count--; break; 
            case 202: pid_s[0].Kp = (float)packet.v/1000.f; config_count--; break; 
            case 203: pid_s[0].Ki = (float)packet.v/1000.f; config_count--; break; 
            case 204: pid_s[0].Kd = (float)packet.v/10000.f; config_count--; break; 
            case 210: pid_s[1].min = packet.v; config_count--; break; 
            case 211: pid_s[1].max = packet.v; config_count--; break; 
            case 212: pid_s[1].Kp = (float)packet.v/1000.f; config_count--; break; 
            case 213: pid_s[1].Ki = (float)packet.v/1000.f; config_count--; break; 
            case 214: pid_s[1].Kd = (float)packet.v/10000.f; config_count--; break; 
            case 220: pid_s[2].min = packet.v; config_count--; break; 
            case 221: pid_s[2].max = packet.v; config_count--; break; 
            case 222: pid_s[2].Kp = (float)packet.v/1000.f; config_count--; break; 
            case 223: pid_s[2].Ki = (float)packet.v/1000.f; config_count--; break; 
            case 224: pid_s[2].Kd = (float)packet.v/10000.f; config_count--; break; 

            default: 
#ifdef DEBUG
                      Serial.print("Unknown command: "); Serial.println(packet.t);
#endif
                      return 0;
        }
    }

    return 0;
}

void log_altitude() {
    static union s_packet p;
    if ((loop_count%20)==0) { //200Hz so 10times a sec... -> every 100ms
        p.t = 0xA;
        p.v = alt_hold_altitude*10;
        SPI_sendBytes(p.b,3);
        p.t = 0xB;
        p.v = vz_est*100;
        SPI_sendBytes(p.b,3);
        p.t = 0xC;
        p.v = h_est*100;
        SPI_sendBytes(p.b,3);
    }
}

void log_accel() {
    static union s_packet p;
    static float _accelMax[3] = {0.f,0.f,0.f};
    static float _accelMin[3] = {0.f,0.f,0.f};

    for (int i=0;i<3;i++) {
        if (mympu.accel[i]<_accelMin[i]) _accelMin[i] = mympu.accel[i];
        if (mympu.accel[i]>_accelMax[i]) _accelMax[i] = mympu.accel[i];
    }

    if ((loop_count%20)==0) { //200Hz so 10times a sec... -> every 100ms
        p.t = 0x20;
        p.v = _accelMax[0]*1000.f;
        SPI_sendBytes(p.b,3);
        p.t = 0x21;
        p.v = _accelMax[1]*1000.f;
        SPI_sendBytes(p.b,3);
        p.t = 0x22;
        p.v = _accelMax[2]*1000.f;
        SPI_sendBytes(p.b,3);

        _accelMax[0] = 0.f;
        _accelMax[1] = 0.f;
        _accelMax[2] = 0.f;
    }
    else if ((loop_count%20)==10) { //200Hz so 10times a sec... -> every 100ms
        p.t = 0x30;
        p.v = _accelMin[0]*1000.f;
        SPI_sendBytes(p.b,3);
        p.t = 0x31;
        p.v = _accelMin[1]*1000.f;
        SPI_sendBytes(p.b,3);
        p.t = 0x32;
        p.v = _accelMin[2]*1000.f;
        SPI_sendBytes(p.b,3);

        _accelMin[0] = 0.f;
        _accelMin[1] = 0.f;
        _accelMin[2] = 0.f;
    }
}

void log_mpu() {
    static union s_packet p;
    static float gyro[3] = {0.f,0.f,0.f};
    static float quat[3] = {0.f,0.f,0.f};

    if ((loop_count%20)==0) { //200Hz so 10times a sec... -> every 100ms
	    for (int i=0;i<3;i++) 
		gyro[i] = mympu.gyro[i];
        p.t = 0x20;
        p.v = gyro[0]*100.f;
        SPI_sendBytes(p.b,3);
        p.t = 0x21;
        p.v = gyro[1]*100.f;
        SPI_sendBytes(p.b,3);
        p.t = 0x22;
        p.v = gyro[2]*100.f;
        SPI_sendBytes(p.b,3);
    }
    else if ((loop_count%20)==10) { //200Hz so 10times a sec... -> every 100ms
	    for (int i=0;i<3;i++) 
		quat[i] = mympu.ypr[i];
        p.t = 0x30;
        p.v = quat[0]*100.f;
        SPI_sendBytes(p.b,3);
        p.t = 0x31;
        p.v = quat[1]*100.f;
        SPI_sendBytes(p.b,3);
        p.t = 0x32;
        p.v = quat[2]*100.f;
        SPI_sendBytes(p.b,3);
    }
}

void log_motor() {
    static union s_packet p;
    static float vA[4] = {0.f,0.f,0.f,0.f};

    if ((loop_count%10)==0) { //200Hz -> every 50ms
	vA[0] = m_fl; 
	vA[1] = m_bl;
	vA[2] = m_fr; 
	vA[3] = m_br; 

        p.t = 0x20;
        p.v = vA[0];
        SPI_sendBytes(p.b,3);
        p.t = 0x21;
        p.v = vA[1];
        SPI_sendBytes(p.b,3);
        p.t = 0x22;
        p.v = vA[2];
        SPI_sendBytes(p.b,3);
        p.t = 0x30;
        p.v = vA[2];
        SPI_sendBytes(p.b,3);
    }
}

void my_log() {
    static union s_packet p;
    static float vA[3] = {0.f,0.f,0.f};

    if ((loop_count%10)==0) { //200Hz -> every 50ms
	vA[0] = yaw_target-mympu.ypr[0];
	vA[1] = pid_s[0].value;
	vA[2] = pid_r[0].value; 

        p.t = 0x20;
        p.v = vA[0]*100.f;
        SPI_sendBytes(p.b,3);
        p.t = 0x21;
        p.v = vA[1]*100.f;
        SPI_sendBytes(p.b,3);
        p.t = 0x22;
        p.v = vA[2]*100.f;
        SPI_sendBytes(p.b,3);
    }
}



void log_debug() {

    if (!(loop_count%25)) {
        //Serial.print(pid_r[1]._d*10000.f);
        /*
           Serial.print(np); Serial.print(" OK: "); Serial.print(c); Serial.print(" C:"); Serial.print(err_c); Serial.print(" O:"); Serial.print(err_o);
         */

	   Serial.print("\t"); Serial.print(yaw_target-mympu.ypr[0]);
	   Serial.print("\t"); Serial.print(mympu.ypr[0]);
	   Serial.print("\t"); Serial.print(mympu.ypr[1]);
	   Serial.print("\t"); Serial.print(mympu.ypr[2]);
	   Serial.print("\t"); Serial.print(mympu.gyro[0]);
	   Serial.print("\t"); Serial.print(mympu.gyro[1]);
	   Serial.print("\t"); Serial.print(mympu.gyro[2]);

/*
           Serial.print("\t"); Serial.print(yprt[1]-mympu.ypr[1]);
           Serial.print("\t"); Serial.print(yprt[2]-mympu.ypr[2]);
           Serial.print("\t"); Serial.print(pid_s[0].value);
           Serial.print("\t"); Serial.print(pid_s[1].value);
           Serial.print("\t"); Serial.print(pid_s[2].value);
           Serial.print("\t"); Serial.print(pid_r[0].value);
           Serial.print("\t"); Serial.print(pid_r[1].value);
           Serial.print("\t"); Serial.print(pid_r[2].value);
           Serial.print("\t"); Serial.print(m_fl);
           Serial.print("\t"); Serial.print(m_bl);
           Serial.print("\t"); Serial.print(m_fr);
           Serial.print("\t"); Serial.print(m_br);
           Serial.print("\t"); Serial.print(config_count);
*/
/*
        Serial.print("\talt: "); Serial.print(altitude);
        Serial.print("  \tsz: "); Serial.print((int)(smooth_az*250));
        Serial.print("\tvz_est: "); Serial.print(vz_est);
        Serial.print("\th_est: "); Serial.print(h_est);
        Serial.print("\tpa: "); Serial.print(pid_alt.value);
        //Serial.print("\tpv: "); Serial.print(pid_vz.value);
        Serial.print("\t"); Serial.print(pid_alt.Kp);
*/
        /*
           Serial.print("\tc0: "); Serial.print(mympu.comp[0]);
           Serial.print("\tc1: "); Serial.print(mympu.comp[1]);
           Serial.print("\tc2: "); Serial.print(mympu.comp[2]);
         */
        /*
           Serial.print("\try: "); Serial.print(yprt[0]);
           Serial.print(" rp: "); Serial.print(yprt[1]);
           Serial.print(" rr: "); Serial.print(yprt[2]);
           Serial.print(" rt: "); Serial.print(yprt[3]);
         */
        Serial.println();
    }
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
    ret = mympu_update_compass();
    if (ret < 0) {
#ifdef DEBUG
        Serial.print("Error reading compass: "); Serial.println(ret);
#endif
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

#endif
    loop_count++;

    loop_s = (float)(millis() - p_millis)/1000.0f;
    p_millis = millis();

    //altitude hold
    int _a = mympu.accel[2]*250;
    smooth_az = 0.95f * smooth_az + 0.05f * (float)_a/250.f;
    vz_est = vz_est - smooth_az*loop_s;
    if (vz_est>0.5f) vz_est = 0.5f;
    else if (vz_est<-0.5f) vz_est = -0.5f;

    h_est = h_est + vz_est*loop_s;

    vz_est = vz_est + k_vz*(h_est - altitude);
    h_est = h_est + k_h_est*(h_est - altitude);

    pid_update(&pid_alt,alt_hold_altitude,h_est,loop_s);
    //pid_update(&pid_vz,0.f,vz_est,loop_s);

    if (alt_hold) {
        yprt[3] = alt_hold_throttle + pid_alt.value;// - pid_vz.value; 
    } 

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

    if (abs(mympu.ypr[2])>50.f) yaw_target = mympu.ypr[0]; //disable yaw if rolling
    if (abs(mympu.ypr[1])>50.f) yaw_target = mympu.ypr[0]; //disable yaw if rolling
    if (abs(mympu.ypr[1])>50.f) yprt[2] = mympu.ypr[2]; //disable roll when pitching
    //if (p_err-r_err<10.f) 
    //if (abs(mympu.ypr[1])>65) yprt[2] = mympu.ypr[2]; //disable roll when pitching
    //if (abs(mympu.ypr[2])>10.f) yaw_target = mympu.ypr[0]; //disable yaw if rolling
    //if (abs(mympu.ypr[1])>10.f) yaw_target = mympu.ypr[0]; //disable yaw if rolling
    //flip recovery start
    //flip recovery end

    if (fly_mode == 0) {

        for (int i=0;i<3;i++) {                                                  
            if (i==0) //keep yaw_target                                          
                pid_update(&pid_s[0],yaw_target,mympu.ypr[0],loop_s);        
            else 
                //pid_update(&pid_s[i],0,mympu.ypr[i],loop_s);
                pid_update(&pid_s[i],yprt[i]+trim[i],mympu.ypr[i],loop_s);
        }

        //yaw requests will be fed directly to rate pid                          
        if (abs(yprt[0])>10) {                                             
            pid_s[0].value = yprt[0];                                 
            yaw_target = mympu.ypr[0];                                              
        }                                                                       
    } else if (fly_mode == 1) {
        for (int i=0;i<3;i++) {                                                  
            if (i==0)                                          
                pid_update(&pid_s[0],yprt[0],0.f,loop_s);        
            else 
                //pid_update(&pid_s[i],0,mympu.ypr[i],loop_s);
                pid_update(&pid_s[i],yprt[i],0.f,loop_s);
        }
    } 

    //do RATE PID                                                            
    for (int i=0;i<3;i++) {                                                  
        pid_update(&pid_r[i],pid_s[i].value,mympu.gyro[i],loop_s);
    }                                                                        



    //pid_r[0].value = 0.f;
#ifdef DEBUG
	log_debug();
#else
    switch(log_mode) {
	case 0: break;
	case 1: log_accel(); break;
	case 2: log_mpu(); break;
	case 3: log_motor(); break;
	case 99: my_log(); break;
	default: break;
    };
#endif

    if (yprt[3] < INFLIGHT_THRESHOLD) {
        myservo[SERVO_FL].writeMicroseconds(MOTOR_INIT_US);
        myservo[SERVO_BR].writeMicroseconds(MOTOR_INIT_US);
        myservo[SERVO_FR].writeMicroseconds(MOTOR_INIT_US);
        myservo[SERVO_BL].writeMicroseconds(MOTOR_INIT_US);
        return;
    }

    /*
       union s_packet p;
       p.t = 1;
       p.v = yprt[3];
       SPI_sendBytes(p.b,3);
     */

    //calculate motor speeds                                        
    m_fl = yprt[3]-pid_r[2].value-pid_r[1].value+pid_r[0].value;
    m_bl = yprt[3]-pid_r[2].value+pid_r[1].value-pid_r[0].value;
    m_fr = yprt[3]+pid_r[2].value-pid_r[1].value-pid_r[0].value;
    m_br = yprt[3]+pid_r[2].value+pid_r[1].value+pid_r[0].value;

    m_fl = m_fl<1080?1080:m_fl;
    m_bl = m_bl<1080?1080:m_bl;
    m_fr = m_fr<1080?1080:m_fr;
    m_br = m_br<1080?1080:m_br;

    myservo[SERVO_FL].writeMicroseconds(m_fl);
    myservo[SERVO_BL].writeMicroseconds(m_bl);
    myservo[SERVO_BR].writeMicroseconds(m_br);
    myservo[SERVO_FR].writeMicroseconds(m_fr);
    /*
       myservo[SERVO_FL].writeMicroseconds(yprt[3]-pid_r[2].value-pid_r[1].value+pid_r[0].value);
       myservo[SERVO_BL].writeMicroseconds(yprt[3]-pid_r[2].value+pid_r[1].value-pid_r[0].value);
       myservo[SERVO_FR].writeMicroseconds(yprt[3]+pid_r[2].value-pid_r[1].value-pid_r[0].value);
       myservo[SERVO_BR].writeMicroseconds(yprt[3]+pid_r[2].value+pid_r[1].value+pid_r[0].value);
     */
}

int check_init() {
	if (config_count!=0) return -1;

    for (int i=0;i<3;i++) {                                                      
        pid_setmode(&pid_r[i],1);                                         
        pid_setmode(&pid_s[i],1);                                         
    } 

    pid_setmode(&pid_alt,1);                                         

#ifdef DEBUG
    Serial.println("Configuration received.");
#endif

    return 0;
}

int gyroCal() {
    static float accel = 0.0f;
    static unsigned int c = 0;
    ret = mympu_update();
    if (ret!=0) {
#ifdef DEBUG
        if (ret!=1) { Serial.print("MPU error! "); Serial.println(ret); }
#endif
        return -1;
    }

    if (c<1000) {
        if (c>=100) accel += mympu.accel[2];
        c++;
    }
    if (mympu.gyro[0]>-1.0f && mympu.gyro[1]>-1.0f && mympu.gyro[2]>-1.0f &&    
            mympu.gyro[0]<1.0f && mympu.gyro[1]<1.0f && mympu.gyro[2]<1.0f) {
#ifdef DEBUG
        Serial.println("Gyro calibration ok.");
#endif
        mympu.gravity = accel / (c-100);
        return 0;
    }
    return -1;
}

void loop() {
    process_command();
    if (config_count!=0) return;

    switch (mode) {
        case 0:
            if (check_init()==0) mode = 1;
	    break;
        case 1: 
            ret = mympu_open(200,gyro_orientation);
            delay(150);
            if (ret == 0) mode = 2;
            break;

        case 2:
            if (gyroCal()==0) mode = 3;
            break;

        case 3:
            controller_loop();
            break;

	default: break;
    }
}

