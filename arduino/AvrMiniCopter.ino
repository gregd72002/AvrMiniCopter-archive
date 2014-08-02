#ifdef DEBUG
#include "freeram.h"
#endif

int crc_err;
#include <avr/wdt.h>
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

int m[4]; //calculated motor thrust
byte FL_PIN,BL_PIN,FR_PIN,BR_PIN;

short mpu_addr;
unsigned int motor_min;
int inflight_threshold;

struct s_pid pid_r[3];
struct s_pid pid_s[3];
float pid_acro_p;

#ifdef ALTHOLD 
struct s_pid pid_alt, pid_vz;
float altitude = 0.0f;
int alt_hold = 0;
float alt_hold_altitude = 0.0f;
int alt_hold_throttle = 0;
float smooth_az = 0.0f;

float k_vz = -0.001f;
float k_h_est = -0.008f;
float vz_est = 0.f;
float h_est = 0.f;
#endif


unsigned char loop_count = 0;
#ifdef DEBUG
unsigned int c = 0; //cumulative number of successful MPU/DMP reads
unsigned int np = 0; //cumulative number of MPU/DMP reads that brought no packet back
unsigned int err_c = 0; //cumulative number of MPU/DMP reads that brought corrupted packet
unsigned int err_o = 0; //cumulative number of MPU/DMP reads that had overflow bit set
#endif


byte config_count; //when 0 this means config has been received
byte mode;
byte fly_mode;
byte log_mode;
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
signed char trim[3] = {0,0,0}; //0-pitch; 1-roll

unsigned int mpu_err = 0;
unsigned int rec_err = 0;

void motor_idle() {
	for (int i=0;i<4;i++)
		myservo[i].writeMicroseconds(motor_min);
}


void initMotors() {
    myservo[SERVO_FL].attach(FL_PIN);
    myservo[SERVO_BL].attach(BL_PIN);
    myservo[SERVO_FR].attach(FR_PIN);
    myservo[SERVO_BR].attach(BR_PIN);
    motor_idle();
}

void sendPacket(byte t, int v) {
        packet.t = t;
        packet.v = v;
        SPI_sendBytes(packet.b,3);
}

void setup() {
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
#endif
    for (int i=0;i<3;i++) {
        pid_init(&pid_r[i]);
        pid_init(&pid_s[i]);
    }
#ifdef ALTHOLD
    pid_init(&pid_alt);
    pid_init(&pid_vz);
#endif

	config_count = 1;
	mode = 0;
	fly_mode = 0;
	log_mode = 0;
	mpu_addr = 136; //for identity maytix see inv_mpu documentation how this is calculated; this is overwritten by a config packet
	crc_err = 0;
}

void motorReattach(int v) {
    if (myservo[v & 0x000F].attached()) {
	    myservo[v & 0x000F].detach();
    	    delay(1000);
	}
    myservo[v & 0x000F].attach(v>>4);
}

void motorTest(int v) {
        myservo[v & 0x000F].writeMicroseconds(v >> 4);
}

int process_command() { 
    static unsigned long last_command = millis();
    if (millis() - last_command>900) {
#ifdef DEBUG
        //Serial.println("No communication!"); 
#endif
#ifdef ALTHOLD
        alt_hold=0;
#endif
	yprt[0]=yprt[1]=yprt[2]=yprt[3]=0;
    }
    //each command is 4 byte long: what, value(2), crc - do it till buffer empty

    if (SPI_getPacket(packet.b)==0) {
        last_command = millis();
        switch(packet.t) {
	    case 1: config_count = packet.v; break;
            case 2: log_mode = packet.v; 
		       config_count--;
			break;
            case 3: fly_mode = packet.v; 
		       yaw_target = mympu.ypr[0];    //when changing fly_mode during flight reset the yaw_target                                           
		       config_count--;
                       break;
	    case 4: 
			config_count--;
			gyro_orientation = packet.v;
			break;

	    case 5:	config_count--;	FL_PIN = packet.v; break;
	    case 6:	config_count--; BL_PIN = packet.v; break;
	    case 7:	config_count--; FR_PIN = packet.v; break;
	    case 8:	config_count--; BR_PIN = packet.v; break;
	    case 9:	config_count--; mpu_addr = packet.v; break;
            case 10: yprt[0] = packet.v; break;
            case 11: yprt[1] = packet.v; break;
            case 12: yprt[2] = packet.v; break;
            case 13: yprt[3] = packet.v; break;
#ifdef ALTHOLD
            case 14: altitude = packet.v/10.f; break;
            case 15: alt_hold = packet.v; 
                       alt_hold_throttle = yprt[3]; 
                       alt_hold_altitude = h_est;
                       vz_est = 0.f;
#ifdef DEBUG
                       Serial.print("Alt hold: "); Serial.println(alt_hold);
#endif
                       break;
            case 16: alt_hold_altitude += packet.v/1000.f; break;
#endif
	    case 17: config_count--; motor_min = packet.v; break;
	    case 18: config_count--; inflight_threshold = packet.v; break;
            case 20: trim[0] = packet.v; break;
            case 21: trim[1] = packet.v; break;
            case 22: trim[2] = packet.v; break;

#ifdef ALTHOLD
            case 80: pid_alt.max = packet.v; config_count--; break; 
            case 81: pid_alt.imax = packet.v; config_count--; break; 
            case 82: pid_alt.Kp = (float)packet.v/1000.f; config_count--; break; 
            case 83: pid_alt.Ki = (float)packet.v/1000.f; config_count--; break; 
            case 84: pid_alt.Kd = (float)packet.v/10000.f; config_count--; break; 
            case 90: pid_vz.max = packet.v; config_count--; break; 
            case 91: pid_vz.imax = packet.v; config_count--; break; 
            case 92: pid_vz.Kp = (float)packet.v/1000.f; config_count--; break; 
            case 93: pid_vz.Ki = (float)packet.v/1000.f; config_count--; break; 
            case 94: pid_vz.Kd = (float)packet.v/10000.f; config_count--; break; 
#endif

            case 100: pid_r[0].max = packet.v; config_count--; break; 
            case 101: pid_r[0].imax = packet.v; config_count--; break; 
            case 102: pid_r[0].Kp = (float)packet.v/1000.f; config_count--; break; 
            case 103: pid_r[0].Ki = (float)packet.v/1000.f; config_count--; break; 
            case 104: pid_r[0].Kd = (float)packet.v/10000.f; config_count--; break; 
            case 110: pid_r[1].max = packet.v; config_count--; break; 
            case 111: pid_r[1].imax = packet.v; config_count--; break; 
            case 112: pid_r[1].Kp = (float)packet.v/1000.f; config_count--; break; 
            case 113: pid_r[1].Ki = (float)packet.v/1000.f; config_count--; break; 
            case 114: pid_r[1].Kd = (float)packet.v/10000.f; config_count--; break; 
            case 120: pid_r[2].max = packet.v; config_count--; break; 
            case 121: pid_r[2].imax = packet.v; config_count--; break; 
            case 122: pid_r[2].Kp = (float)packet.v/1000.f; config_count--; break; 
            case 123: pid_r[2].Ki = (float)packet.v/1000.f; config_count--; break; 
            case 124: pid_r[2].Kd = (float)packet.v/10000.f; config_count--; break; 

            case 130: pid_acro_p = packet.v/1000.f; config_count--; break; 

            case 200: pid_s[0].max = packet.v; config_count--; break; 
            case 201: pid_s[0].imax = packet.v; config_count--; break; 
            case 202: pid_s[0].Kp = (float)packet.v/1000.f; config_count--; break; 
            case 203: pid_s[0].Ki = (float)packet.v/1000.f; config_count--; break; 
            case 204: pid_s[0].Kd = (float)packet.v/10000.f; config_count--; break; 
            case 210: pid_s[1].max = packet.v; config_count--; break; 
            case 211: pid_s[1].imax = packet.v; config_count--; break; 
            case 212: pid_s[1].Kp = (float)packet.v/1000.f; config_count--; break; 
            case 213: pid_s[1].Ki = (float)packet.v/1000.f; config_count--; break; 
            case 214: pid_s[1].Kd = (float)packet.v/10000.f; config_count--; break; 
            case 220: pid_s[2].max = packet.v; config_count--; break; 
            case 221: pid_s[2].imax = packet.v; config_count--; break; 
            case 222: pid_s[2].Kp = (float)packet.v/1000.f; config_count--; break; 
            case 223: pid_s[2].Ki = (float)packet.v/1000.f; config_count--; break; 
            case 224: pid_s[2].Kd = (float)packet.v/10000.f; config_count--; break; 
	    case 250:	motorReattach(packet.v); break;
	    case 251:	motorTest(packet.v); break;
	    case 254: wdt_enable(WDTO_15MS); while (1); break;
            default: 
#ifdef DEBUG
                      Serial.print("Unknown command: "); Serial.println(packet.t);
#endif
                      return 0;
        }
    }

    return 0;
}


#ifdef ALTHOLD
void log_altitude() {
	sendPacket(30,alt_hold_altitude*10.f);
	sendPacket(31,vz_est*100.f);
	sendPacket(32,h_est*100.f);
}
#endif

void log_accel() {
    static float _accelMax[3] = {0.f,0.f,0.f};
    static float _accelMin[3] = {0.f,0.f,0.f};

    for (int i=0;i<3;i++) {
        if (mympu.accel[i]<_accelMin[i]) _accelMin[i] = mympu.accel[i];
        if (mympu.accel[i]>_accelMax[i]) _accelMax[i] = mympu.accel[i];
    }

    if ((loop_count%20)==0) { //200Hz so 10times a sec... -> every 100ms
	for (int i=0;i<3;i++) {
		sendPacket(20+i,_accelMax[i]*1000.f);
		_accelMax[i] = 0.f;
	}
    }
    else if ((loop_count%20)==10) { //200Hz so 10times a sec... -> every 100ms
	for (int i=0;i<3;i++) {
		sendPacket(25+i,_accelMin[i]*1000.f);
		_accelMin[i] = 0.f;
	}
    }
}

void log_gyro() {
	for (int i=0;i<3;i++)
		sendPacket(1+i,mympu.gyro[i]*100.f);
}

void log_quat() {

	for (int i=0;i<3;i++)
		sendPacket(5+i,mympu.ypr[i]*100.f);
	sendPacket(8,yaw_target*100.f);
}

void log_motor() {
	for (int i=0;i<4;i++)
		sendPacket(10+i,m[i]);
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

void log() {
#ifdef DEBUG
	log_debug();
#else
    switch(log_mode) {
	case 0: break;

	case 1: 
		log_accel(); 
	break;
	
	case 2: 
	    if ((loop_count%20)==0) //200Hz -> every 50ms
		log_gyro();
	    else if ((loop_count%20)==10) //200Hz -> every 50ms
		log_quat();
	break;

	case 3: 
	    if ((loop_count%10)==0) //200Hz -> every 50ms
		log_motor();
	break;

	case 4:
	    if ((loop_count%20)==0) //200Hz -> every 50ms
		log_motor();
	    else if ((loop_count%20)==10)
		log_quat();
	break;

#ifdef ALTHOLD
	case 5:
	    if ((loop_count%20)==0)  //200Hz so 10times a sec... -> every 100ms
		log_altitude();
	break;
#endif

	case 99:
	    if ((loop_count%20)==0) //200Hz -> every 50ms
		log_motor();
	    else if ((loop_count%20)==10)
		    sendPacket(254,crc_err);
	break;
	
	default: break;
    };
#endif
}

float loop_s = 0.0f;
unsigned long p_millis = 0;
bool stop = 0;

void controller_loop() {
    if (stop) {
	motor_idle();
        return;
    }
#ifdef MPU9150
    ret = mympu_update_compass();
    if (ret < 0) {
#ifdef DEBUG
        Serial.print("Error reading compass: "); Serial.println(ret);
#endif
    }
#endif
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
    if (++loop_count==200) loop_count = 0;

    loop_s = (float)(millis() - p_millis)/1000.0f;
    p_millis = millis();

#ifdef ALTHOLD
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
        yprt[3] = (int)(alt_hold_throttle + pid_alt.value);// - pid_vz.value; 
    } 
#endif

    if (yaw_target-mympu.ypr[0]<-180.0f) yaw_target*=-1;                        
    if (yaw_target-mympu.ypr[0]>180.0f) yaw_target*=-1;     

    //do STAB PID                                                            

    if (abs(mympu.ypr[2])>50.f) yaw_target = mympu.ypr[0]; //disable yaw if rolling
    if (abs(mympu.ypr[1])>50.f) yaw_target = mympu.ypr[0]; //disable yaw if rolling
    //flip recovery end

    if (fly_mode == 0) {
	
	//yaw requests will be fed directly to rate pid                          
	if (abs(yprt[0])>10) {
	    pid_s[0].value = yprt[0]*pid_acro_p;                                 
	    yaw_target = mympu.ypr[0];                                              
	} else pid_update(&pid_s[0],yaw_target,mympu.ypr[0],loop_s);        

        for (int i=1;i<3;i++)                                               
                pid_update(&pid_s[i],yprt[i]+trim[i],mympu.ypr[i],loop_s);

    } else if (fly_mode == 1) {
        for (int i=0;i<3;i++)                                                   
                pid_update(&pid_s[i],yprt[i]*pid_acro_p,0.f,loop_s);        
        
    } 

    //do RATE PID                                                            
    for (int i=0;i<3;i++) {                                                  
        pid_update(&pid_r[i],pid_s[i].value,mympu.gyro[i],loop_s);
    }                                                                        

    //calculate motor speeds                                        
    m[0] = (int)(yprt[3]+pid_r[2].value-pid_r[1].value+pid_r[0].value);
    m[1] = (int)(yprt[3]+pid_r[2].value+pid_r[1].value-pid_r[0].value);
    m[2] = (int)(yprt[3]-pid_r[2].value-pid_r[1].value-pid_r[0].value);
    m[3] = (int)(yprt[3]-pid_r[2].value+pid_r[1].value+pid_r[0].value);

    log();

    if (yprt[3] < inflight_threshold) {
	motor_idle();
        yaw_target = mympu.ypr[0];
        for (int i=0;i<3;i++) {                                              
            pid_r[i]._KiTerm = 0.0f;                                  
            pid_s[i]._KiTerm = 0.0f;                                  
        }                                                                    
        return;
    }

    for (int i=0;i<4;i++) { 
	m[i] = m[i]<inflight_threshold?inflight_threshold:m[i];
    	myservo[i].writeMicroseconds(m[i]);
    }

}

int check_init() {
	if (config_count!=0) return -1;

    initMotors();

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

    if (c<200) {
        if (c>=20) accel += mympu.accel[2];
        c++;
    }
    if (mympu.gyro[0]>-1.0f && mympu.gyro[1]>-1.0f && mympu.gyro[2]>-1.0f &&    
            mympu.gyro[0]<1.0f && mympu.gyro[1]<1.0f && mympu.gyro[2]<1.0f) {
#ifdef DEBUG
        Serial.println("Gyro calibration ok.");
#endif
        mympu.gravity = accel / (c-20);
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
            ret = mympu_open(mpu_addr,200,gyro_orientation);
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

