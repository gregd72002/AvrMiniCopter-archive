#include "bmp085.h"

#define BMP_ADDR 0x77
#define OVERSAMPLING 3 //oversampling_setting
#define PRESSURE_DELAY 26000 //us 
#define TEMPERATURE_DELAY 5000 //us  
#define delay_ms(a)    delay(a)

#define BMP180_ADDR 0x77 // 7-bit address

#define BMP180_REG_CONTROL 0xF4
#define BMP180_REG_RESULT 0xF6

#define BMP180_COMMAND_TEMPERATURE 0x2E
#define BMP180_COMMAND_PRESSURE0 0x34
#define BMP180_COMMAND_PRESSURE1 0x74
#define BMP180_COMMAND_PRESSURE2 0xB4
#define BMP180_COMMAND_PRESSURE3 0xF4

static int state = 0;
static unsigned long dt;

struct s_bs bs; 

static int ret = 0;

static struct s_cc {
    int AC1,AC2,AC3,b1,B2,MB,MC,MD;
    unsigned int AC4,AC5,AC6;
    long X1,X2,X3,B3,B5,B6;
    unsigned long B4,B7;
} cc; //calibration coefficients


typedef unsigned char uint8_t;
typedef int int16_t;

int bs_open()
    // Initialize library for subsequent pressure measurements
{


    // The BMP180 includes factory calibration data stored on the device.
    // Each device has different numbers, these must be retrieved and
    // used in the calculations when taking pressure measurements.

    // Retrieve calibration data from device:
bs.t = bs.p = bs.p0 = bs.alt = 1.f;                                                               


	uint8_t buff[22];

    if (I2Cdev::readBytes(BMP_ADDR,0xAA,22,buff)<0) {
	return -1;
    }

    cc.AC1 = ((int16_t)buff[0] << 8) | buff[1];
    cc.AC2 = ((int16_t)buff[2] << 8) | buff[3];
    cc.AC3 = ((int16_t)buff[4] << 8) | buff[5];
    cc.AC4 = ((int16_t)buff[6] << 8) | buff[7];
    cc.AC5 = ((int16_t)buff[8] << 8) | buff[9];
    cc.AC6 = ((int16_t)buff[10] << 8) | buff[11];
    cc.b1 = ((int16_t)buff[12] << 8) | buff[13];
    cc.B2 = ((int16_t)buff[14] << 8) | buff[15];
    cc.MB = ((int16_t)buff[16] << 8) | buff[17];
    cc.MC = ((int16_t)buff[18] << 8) | buff[19];
    cc.MD = ((int16_t)buff[20] << 8) | buff[21];
        return(0);
}

int bs_update(unsigned long t_ms) {
    if (state==0 && (t_ms-dt)>(PRESSURE_DELAY/1000)) { //_DELAY provided in us thus converting to ms
        dt = t_ms;
        state = 1;
        if (getPressure(bs.p)<0) {
	    return -1;
	}
        bs.alt = altitude(bs.p, bs.p0); 
        if (prepareTemperature()<0) {
	    return -1;
	}
    }

    if (state == 1 && (t_ms-dt)>(TEMPERATURE_DELAY/1000))  {
        if (getTemperature(bs.t)<0) {
	    return -1;
	}
        if (preparePressure()<0) { 
	    return -1;
	}
        dt = t_ms;
        state = 0;
    }

    return 0;
}

static int prepareTemperature(void)
    // Begin a temperature reading.
{

    ret = I2Cdev::writeByte(BMP_ADDR,BMP180_REG_CONTROL,BMP180_COMMAND_TEMPERATURE);
    return ret;
}

static int getTemperature(float &T)
    // Retrieve a previously-started temperature reading.
    // Requires begin() to be called once prior to retrieve calibration parameters.
    // Requires startTemperature() to have been called prior and sufficient time elapsed.
    // T: external variable to hold result.
{
    unsigned char data[2];
    long tu;
    static float _temp_sum = 0.f;

    ret = I2Cdev::readBytes(BMP_ADDR,BMP180_REG_RESULT,2,data);
    if (ret==2) // good read, calculate temperature
    {
        tu = (((long)data[0]) << 8) + ((long)data[1]);
        //tu = 27898;
        cc.X1 = ((tu - cc.AC6) * cc.AC5) >> 15;
        cc.X2 = (cc.MC << 11) / (cc.X1 + cc.MD);
        cc.B5 = cc.X1 + cc.X2;
        _temp_sum = ((cc.B5 + 8) >> 4);
        T = _temp_sum/10.0f;
    }
    return ret;
}

static int preparePressure()
    // Begin a pressure reading.
    // Oversampling: 0 to 3, higher numbers are slower, higher-res outputs.
    // Will return delay in ms to wait, or 0 if I2C error.
{
    unsigned char data;

            data = BMP180_COMMAND_PRESSURE3;

    ret = I2Cdev::writeBytes(BMP_ADDR,BMP180_REG_CONTROL,1,&data);
    return ret;
}

static int getPressure(float &P)
    // Retrieve a previously started pressure reading, calculate abolute pressure in mbars.
    // Requires begin() to be called once prior to retrieve calibration parameters.
    // Requires startPressure() to have been called prior and sufficient time elapsed.
    // Requires recent temperature reading to accurately calculate pressure.

    // P: external variable to hold pressure.
    // T: previously-calculated temperature.

    // Note that calculated pressure value is absolute mbars, to compensate for altitude call sealevel().
{
    unsigned char data[3];
    long up = 0l;
    long p;

    ret = I2Cdev::readBytes(BMP_ADDR,BMP180_REG_RESULT,3,data); // good read, calculate pressure
    if (ret == 3) // good read, calculate pressure
    {
        up = (((long)(data[0]) << 16) + ((long)(data[1])<<8) + (long)(data[2])) >> (8-OVERSAMPLING);
        //up = 23843;

        cc.B6 = cc.B5-4000;
        cc.X1 = (cc.B2*((cc.B6*cc.B6)>>12))>>11;
        cc.X2 = (cc.AC2*cc.B6)>>11;
        cc.X3 = cc.X1+cc.X2;
        cc.B3 = (((cc.AC1*4+cc.X3)<<OVERSAMPLING)+2)/4;
        cc.X1 = (cc.AC3*cc.B6)>>13;
        cc.X2 = (cc.b1*((cc.B6*cc.B6)>>12))>>16;
        cc.X3 = ((cc.X1+cc.X2)+2)>>2;
        cc.B4 = (cc.AC4*(unsigned long)(cc.X3+32768))>>15;
        cc.B7 = ((unsigned long)up-cc.B3)*(50000>>OVERSAMPLING);
        if (cc.B7<0x80000000) p = (cc.B7*2)/cc.B4;
        else p = (cc.B7/cc.B4)*2;
        cc.X1 = (p>>8) * (p>>8);
        cc.X1 = (cc.X1*3038)>>16;
        cc.X2 = (-7357*p)>>16;
        P = p + ((cc.X1+cc.X2+3791)>>4);
    }
    return ret;
}


static float altitude(float P, float P0)
    // Given a pressure measurement P (mb) and the pressure at a baseline P0 (mb),
    // return altitude (meters) above baseline.
{
    return round(10.0f*44330.0f*(1.0f-pow(P/P0,1.0f/5.255f)))/10.0f;
}

