#include "Arduino.h"
#ifdef DEBUG
#include "freeram.h"
#endif


uint8_t crc_err;
#include "mpu.h"
#include "I2Cdev.h"
#include <SPI.h>
#include "SPIdev.h"
#include "crc8.h"
#include "pid.h"
#include "buf.h"
#include <Servo.h>

int ret;
Servo myservo[4];
#define SERVO_FL 0
#define SERVO_BL 1
#define SERVO_FR 2
#define SERVO_BR 3

int m[4]; //calculated motor thrust
byte motor_pin[4]; //FL_PIN,BL_PIN,FR_PIN,BR_PIN;

uint8_t mpu_addr; 
int motor_pwm[2]; //min, inflight threshold;

struct s_pid pid_r[3];
struct s_pid pid_s[3];
float pid_acro_p;

#define MAX_ALT 20000 //200m (ensure MAX_ALT + MAX_ALT_INC fits into signed int)
#define MAX_ALT_INC 1000 //10m
#define MAX_ACCEL  250
struct s_pid pid_accel, pid_alt, pid_vz;

struct s_buf<float> alt_buf;

float ld;
float accel_z = 0.f;

float accel_err = 0.f;
float accel_corr = 0.f;
float alt_corr = 0.f;
float vz_inc = 0.f;
float alt_err = 0.f;
float alt_base = 0.f;

float alt = 0.f;
float vz = 0.f;
float pos_err = 0.f, vz_target = 0.f;

float bc,bc1,bc2,bc3;
int8_t baro_counter = 0;

int8_t alt_hold = 0;
float alt_hold_target;
int alt_hold_throttle;



uint8_t loop_count = 0;
#ifdef DEBUG
unsigned int c = 0; //cumulative number of successful MPU/DMP reads
unsigned int np = 0; //cumulative number of MPU/DMP reads that brought no packet back
unsigned int err_c = 0; //cumulative number of MPU/DMP reads that brought corrupted packet
unsigned int err_o = 0; //cumulative number of MPU/DMP reads that had overflow bit set
#endif


uint8_t status = 0;
uint8_t fly_mode;
uint8_t log_mode;
uint8_t gyro_orientation;

float yaw_target;

byte packet[4];

int yprt[4] = {0,0,0,0};
int8_t trim[3] = {0,0,0}; //0-pitch; 1-roll

void motor_idle() {
	for (int i=0;i<4;i++)
		myservo[i].writeMicroseconds(motor_pwm[0]);
}

void initMotor(int v) {
	myservo[v].attach(motor_pin[v]);
}

void testMotor(int m,int v) {
	myservo[m].writeMicroseconds(v);
}


void armMotors() {
	for (int i=0;i<4;i++)
		initMotor(i);
	motor_idle();
}

void sendPacket(byte t, int v) {
	packet[0] = t;
	packet[1] = v & 0x00FF;
	packet[2] = (v & 0xFF00) >> 8;
	SPI_sendBytes(packet,3);
}

void initAVR() {
	for (int i=0;i<3;i++) {
		pid_init(&pid_r[i]);
		pid_init(&pid_s[i]);
	}
#ifdef ALTHOLD
	pid_init(&pid_accel);
	pid_init(&pid_vz);
	pid_init(&pid_alt);
	buf_init(&alt_buf,15);
#endif
	bc = 0.5f;
	bc1 = 0.6f;
	bc2 = 0.12f;
	bc3 = 0.1f;

	yaw_target = 0.f;
	loop_count = 0;
	status = 0;
	fly_mode = 0;
	log_mode = 0;
	gyro_orientation = 136; //for identity matrix see inv_mpu documentation how this is calculated; this is overwritten by a config packet
	crc_err = 0;
	alt_hold = 0;
	alt_hold_target = 0.0f;
	alt_hold_throttle = 0;
	accel_z = 0.f;

	accel_err = 0.f;
	accel_corr = 0.f;
	alt_corr = 0.f;
	vz_inc = 0.f;
	alt_err = 0.f;
	alt_base = 0.f;

	alt = 0.f;
	vz = 0.f;
}

void setup() {
	Fastwire::setup(400,0);

#ifdef DEBUG
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
}

int process_command() { 
	static unsigned long last_command = millis();
	if (millis() - last_command>1000) {
		alt_hold=0;
		yprt[0]=yprt[1]=yprt[2]=yprt[3]=0;
	}
	//each command is 4 byte long: what, value(2), crc - do it till buffer empty

	byte t = 0;
	int v = 0;
	if (SPI_getPacket(packet)==0) {
		t = packet[0];
		v = packet[2] << 8 | packet[1];
		switch(t) {
#ifdef DEBUG
			case 0: Serial.println("Received packet 0! Check your SPI connection!"); //this usually happens by default when SPI wiring is not right
				break;
#endif
			case 2: log_mode = v; 
				break;
			case 3: fly_mode = v; 
				yaw_target = mympu.ypr[0];    //when changing fly_mode during flight reset the yaw_target                                           
				break;
			case 4: 
				gyro_orientation = v;
				break;

			case 5: case 6: case 7: case 8: 
				motor_pin[t-5] = v; 
				break;
			case 9: mpu_addr = v; break;
			case 10: yprt[0] = v; break;
			case 11: yprt[1] = v; break;
			case 12: yprt[2] = v; break;
			case 13: last_command = millis(); yprt[3] = v; break;
#ifdef ALTHOLD
			case 14: //altitude reading in cm - convert it into altitude error 
				 static float b;
				 baro_counter = 100; //use this baro reading not more than 100 times
				 if (!buf_space(&alt_buf)) b = buf_pop(&alt_buf); //buffer full
				 else b = alt_base;
				 alt_err = v - (b + alt_corr);
				 break;
			case 15:   alt_hold = v; 
				   alt_hold_throttle = yprt[3]; 
				   break;
			case 16: 
				   if (v>MAX_ALT_INC) alt_hold_target += MAX_ALT_INC; 
				   else if (v<-MAX_ALT_INC) alt_hold_target -= MAX_ALT_INC;
				   else alt_hold_target += v;
				   if (alt_hold_target>MAX_ALT) alt_hold_target=MAX_ALT;
				   break;
#endif
			case 17: motor_pwm[0] = v; break;
			case 18: motor_pwm[1] = v; break;
			case 20: trim[0] = v; break;
			case 21: trim[1] = v; break;
			case 22: trim[2] = v; break;

			case 69: 
				 bc = v/100.f;
				 bc1 = 3.f/(bc);
				 bc2 = 3.f/(bc*bc);
				 bc3 = 1.f/(bc*bc*bc);
				 break;
			case 70: pid_accel.max = v; break; 
			case 71: pid_accel.imax = v; break; 
			case 72: pid_accel.Kp = (float)v/1000.f; break; 
			case 73: pid_accel.Ki = (float)v/1000.f; break; 
			case 74: pid_accel.Kd = (float)v/10000.f; break; 
			case 80: pid_alt.max = v; break; 
			case 81: pid_alt.imax = v; break; 
			case 82: pid_alt.Kp = (float)v/1000.f; break; 
			case 83: pid_alt.Ki = (float)v/1000.f; break; 
			case 84: pid_alt.Kd = (float)v/10000.f; break; 
			case 90: pid_vz.max = v; break; 
			case 91: pid_vz.imax = v; break; 
			case 92: pid_vz.Kp = (float)v/1000.f; break; 
			case 93: pid_vz.Ki = (float)v/1000.f; break; 
			case 94: pid_vz.Kd = (float)v/10000.f; break; 

			case 100: pid_r[0].max = v; break; 
			case 101: pid_r[0].imax = v; break; 
			case 102: pid_r[0].Kp = (float)v/1000.f; break; 
			case 103: pid_r[0].Ki = (float)v/1000.f; break; 
			case 104: pid_r[0].Kd = (float)v/10000.f; break; 
			case 110: pid_r[1].max = v; break; 
			case 111: pid_r[1].imax = v; break; 
			case 112: pid_r[1].Kp = (float)v/1000.f; break; 
			case 113: pid_r[1].Ki = (float)v/1000.f; break; 
			case 114: pid_r[1].Kd = (float)v/10000.f; break; 
			case 120: pid_r[2].max = v; break; 
			case 121: pid_r[2].imax = v; break; 
			case 122: pid_r[2].Kp = (float)v/1000.f; break; 
			case 123: pid_r[2].Ki = (float)v/1000.f; break; 
			case 124: pid_r[2].Kd = (float)v/10000.f; break; 

			case 130: pid_acro_p = v/1000.f; break; 

			case 200: pid_s[0].max = v; break; 
			case 201: pid_s[0].imax = v; break; 
			case 202: pid_s[0].Kp = (float)v/1000.f; break; 
			case 203: pid_s[0].Ki = (float)v/1000.f; break; 
			case 204: pid_s[0].Kd = (float)v/10000.f; break; 
			case 210: pid_s[1].max = v; break; 
			case 211: pid_s[1].imax = v; break; 
			case 212: pid_s[1].Kp = (float)v/1000.f; break; 
			case 213: pid_s[1].Ki = (float)v/1000.f; break; 
			case 214: pid_s[1].Kd = (float)v/10000.f; break; 
			case 220: pid_s[2].max = v; break; 
			case 221: pid_s[2].imax = v; break; 
			case 222: pid_s[2].Kp = (float)v/1000.f; break; 
			case 223: pid_s[2].Ki = (float)v/1000.f; break; 
			case 224: pid_s[2].Kd = (float)v/10000.f; break; 
			case 249: sendPacket(249,v); break;
			case 250: case 251: case 252: case 253:
				  testMotor(t-250,v); break;
			case 254: initMotor(v); break;
			case 255: 
				  switch (v) {
					  case 0: sendPacket(255,status); break;
					  case 1: sendPacket(254,crc_err); break;
					  case 2: status = 2; break;
					  case 254: break; //dummy - used for SPI queued message retrieval  
				  }
				  break;
			default: 
#ifdef DEBUG
				  byte c = packet[3];
				  Serial.print("Unknown command: "); Serial.print(t); Serial.print(" "); Serial.print(v); Serial.print(" "); Serial.println(c);
#endif
				  return 0;
		}
	}

	return 0;
}


#ifdef ALTHOLD
void log_accel_pid() {
	//	sendPacket(100,vz);
	//	sendPacket(101,pos_err*100.f);
	//	sendPacket(102,accel_err*100.f);
	/*
	   sendPacket(103,pid_alt.value);
	   sendPacket(104,pid_vz.value);
	   sendPacket(105,pid_accel.value);
	 */
}

void log_altitude() {
	sendPacket(30,alt_hold_target);
	sendPacket(31,alt);
	sendPacket(32,vz);
	//	sendPacket(33,accel_err*100.f);
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
		//
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
		case 100: 
			if ((loop_count%10)==0)  //200Hz so 10times a sec... -> every 50ms
				log_accel_pid();
			break;

#endif

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
#ifdef DEBUG
		Serial.print("mympu_update: "); Serial.println(ret);
#endif
		motor_idle();
		status = 253;
		stop = 1;
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
	if (loop_s>0.01) { 
		status = 254;
		motor_idle();
		return;
	}
#ifdef ALTHOLD

	if (baro_counter>0) { //if there is no recent baro reading dont do alt_hold
		baro_counter--;
		//maintain altitude & velocity
		accel_z = mympu.accel[2]*982.f; //convert accel to cm/s (9.82 * 100)
		accel_corr += (alt_err * bc3 * loop_s);
		vz += (alt_err * bc2 * loop_s);
		alt_corr += (alt_err * bc1 * loop_s);
		vz_inc = (accel_z + accel_corr) * loop_s;
		alt_base += (vz + vz_inc * 0.5f) * loop_s;
		alt = alt_base + alt_corr;
		vz += vz_inc;
		buf_push(&alt_buf, alt_base);
		//end maintain altitude & velocity 

		// do altitude PID
		pos_err = alt_hold_target - alt;
		ld = MAX_ACCEL / (2.f * pid_alt.Kp * pid_alt.Kp);
		if (pos_err > 2.f*ld) 
			vz_target = sqrt(2.f * MAX_ACCEL * (pos_err-ld)); 	
		else if (pos_err < -2.f*ld)
			vz_target = -sqrt(2.f * MAX_ACCEL * (-pos_err-ld)); 	
		else {
			pid_update(&pid_alt,pos_err, loop_s);
			vz_target = pid_alt.value;
		}
		// end altitude PID

		// do velocity PID
		pid_update(&pid_vz, vz_target - vz, loop_s);
		// end velocity PID

		accel_err += 0.11164f * (pid_vz.value - accel_z - accel_err);
		pid_update(&pid_accel,accel_err,loop_s);
		if (alt_hold) {
			yprt[3] = (int)(alt_hold_throttle + pid_accel.value); 
		} else {
			alt_hold_target = alt;
		} 
	} else alt_hold = 0; //baro expired
#endif
	if (yaw_target-mympu.ypr[0]<-180.0f) yaw_target*=-1;                        
	if (yaw_target-mympu.ypr[0]>180.0f) yaw_target*=-1;     

	//do STAB PID                                                            

	if (abs(mympu.ypr[2])>50.f) yaw_target = mympu.ypr[0]; //disable yaw if rolling
	if (abs(mympu.ypr[1])>50.f) yaw_target = mympu.ypr[0]; //disable yaw if rolling
	//flip recovery end

	if (fly_mode == 0) {

		//yaw requests will be fed directly to rate pid                          

		if (abs(yprt[0])>7) {
			pid_update(&pid_s[0],yprt[0]/10,loop_s);        
			yaw_target = mympu.ypr[0];                                              
		} else pid_update(&pid_s[0],yaw_target-mympu.ypr[0],loop_s);        

		for (int i=1;i<3;i++)                                               
			pid_update(&pid_s[i],yprt[i]+trim[i]-mympu.ypr[i],loop_s);

	} else if (fly_mode == 1) {
		pid_update(&pid_s[0],yprt[0]/10,loop_s);        
		pid_update(&pid_s[1],yprt[1]*pid_acro_p,loop_s);        
		pid_update(&pid_s[2],yprt[2]*pid_acro_p,loop_s);        

	} 

	//do RATE PID                                                            
	for (int i=0;i<3;i++) {                                                  
		pid_update(&pid_r[i],pid_s[i].value-mympu.gyro[i],loop_s);
	}                                                                        

	//calculate motor speeds                                        
	m[0] = (int)(yprt[3]+pid_r[2].value-pid_r[1].value+pid_r[0].value);
	m[1] = (int)(yprt[3]+pid_r[2].value+pid_r[1].value-pid_r[0].value);
	m[2] = (int)(yprt[3]-pid_r[2].value-pid_r[1].value-pid_r[0].value);
	m[3] = (int)(yprt[3]-pid_r[2].value+pid_r[1].value+pid_r[0].value);

	log();

	if (yprt[3] < motor_pwm[1]) {
		motor_idle();
		yaw_target = mympu.ypr[0];
		for (int i=0;i<3;i++) {                                              
			pid_reset(&pid_r[i]);
			pid_reset(&pid_s[i]);
		}                                                                    
		return;
	}

	for (int i=0;i<4;i++) { 
		m[i] = m[i]<motor_pwm[1]?motor_pwm[1]:m[i];
		myservo[i].writeMicroseconds(m[i]);
	}

}

int check_init() {
	armMotors();

	return 0;
}

int gyroCal() {
	static float accel = 0.0f;
	static byte c = 0;
	static unsigned int loop_c = 0;
	loop_c++;
	if (loop_c>50000) status=255;
	ret = mympu_update();
	if (ret!=0) {
#ifdef DEBUG
		if (ret!=1) { Serial.print("MPU error! "); Serial.println(ret); }
#endif
		return -1;
	}

	if (c<200) {
		if (c>=20) 
			accel += mympu.accel[2];
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

	if (status==0) {
		initAVR();
		status = 1;
		sendPacket(255,status); 
	}

	//status = 2 done by SPI

	switch (status) {
		case 2:
			if (check_init()==0)
				status = 3;
			break;
		case 3: 
			ret = mympu_open(mpu_addr,200,gyro_orientation);
			delay(150);
			if (ret == 0) 
				status = 4;
			break;
		case 4:
			if (gyroCal()==0) 
				status = 5;
			p_millis = millis();
			break;

		case 5:
			controller_loop();
			break;

		default: break;
	}
}

