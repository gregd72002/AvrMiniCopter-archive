#include "mpu.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "helper_3dmath.h"

#define FSR 2000
//#define GYRO_SENS       ( 131.0f * 250.f / (float)FSR )
#define GYRO_SENS       16.375f 
#define ACCEL_SENS      16384.0f
#define QUAT_SENS       1073741824.f //2^30

#define EPSILON         0.0001f
#define PI_2            1.57079632679489661923f

struct s_mympu mympu;

struct s_quat { float w, x, y, z; }; 

union u_quat {
	struct s_quat _f;
	long _l[4];
} q;

static int ret;
static short gyro[3];
static short accel[3];
#ifdef MPU9150
static short comp[3];
#endif
static short sensors;
static unsigned char fifoCount;
static unsigned int _rate;
static unsigned int _c;

int8_t mympu_open(short addr,unsigned int rate, unsigned short orient) {
    _rate = rate;
    _c = 0;
  	mpu_select_device(addr);
   	mpu_init_structures();

    mympu.gravity = 0.f;

	ret = mpu_init(NULL);
#ifdef MPU_DEBUG
	if (ret) return 10+ret;
#endif
	
	ret = mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL
#ifdef MPU9150
           |INV_XYZ_COMPASS
#endif
        ); 
#ifdef MPU_DEBUG
	if (ret) return 20+ret;
#endif

        ret = mpu_set_gyro_fsr(FSR);
#ifdef MPU_DEBUG
	if (ret) return 30+ret;
#endif

        ret = mpu_set_accel_fsr(2);
#ifdef MPU_DEBUG
	if (ret) return 40+ret;
#endif

#ifdef MPU9150
        ret = mpu_set_compass_sample_rate(50);
#endif

#ifdef MPU_DEBUG
	if (ret) return 50+ret;
#endif

        mpu_get_power_state((unsigned char *)&ret);
#ifdef MPU_DEBUG
	if (!ret) return 60+ret;
#endif

        ret = mpu_configure_fifo(INV_XYZ_GYRO|INV_XYZ_ACCEL);
#ifdef MPU_DEBUG
	if (ret) return 70+ret;
#endif

	dmp_select_device(0);
	dmp_init_structures();

	ret = dmp_load_motion_driver_firmware();
#ifdef MPU_DEBUG
	if (ret) return 80+ret;
#endif

	ret = dmp_set_orientation(orient);
#ifdef MPU_DEBUG
	if (ret) return 90+ret;
#endif

	ret = dmp_set_fifo_rate(rate);
#ifdef MPU_DEBUG
	if (ret) return 100+ret;
#endif

	ret = mpu_set_dmp_state(1);
#ifdef MPU_DEBUG
	if (ret) return 110+ret;
#endif

	ret = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_SEND_CAL_GYRO|DMP_FEATURE_GYRO_CAL|DMP_FEATURE_SEND_RAW_ACCEL);
//	ret = dmp_enable_feature(DMP_FEATURE_SEND_CAL_GYRO|DMP_FEATURE_GYRO_CAL);
#ifdef MPU_DEBUG
	if (ret) return 120+ret;
#endif

	return 0;
}

static inline float rad2deg( float rad )
{
        //return (180.f/PI) * rad;
	return 57.2957795131f * rad;
}

static float test, sqy,sqz,sqw;
static void quaternionToEuler( const struct s_quat *q, float* x, float* y, float* z )
{
        sqy = q->y * q->y;
        sqz = q->z * q->z;
        sqw = q->w * q->w;

        test = q->x * q->z - q->w * q->y;

        if( test > 0.5f - EPSILON )
        {
                *x = 2.f * atan2( q->y, q->w );
                *y = PI_2;
                *z = 0;
        }
        else if( test < -0.5f + EPSILON )
        {
                *x = -2.f * atan2( q->y, q->w );
                *y = -PI_2;
                *z = 0;
        }
        else
        {
                *x = atan2( 2.f * ( q->x * q->w + q->y * q->z ), 1.f - 2.f * ( sqz + sqw ) );
                *y = asin( 2.f * test );
                *z = atan2( 2.f * ( q->x * q->y - q->z * q->w ), 1.f - 2.f * ( sqy + sqz ) );
        }
}

static inline float wrap_180(float x) {
	return (x<-180.f?x+360.f:(x>180.f?x-180.f:x));
}

static inline float shift_180(float x) {
    return x+180>180?x-180:x+180;
}

void mympu_reset_fifo() {
	mpu_reset_fifo();
}

#ifdef MPU9150
int8_t mympu_update_compass() {
    if (_c <_rate/50) return 1;

    ret = mpu_get_compass_reg(comp,NULL);
    if (ret!=0) return ret; 

    mympu.comp[0] = comp[0];
    mympu.comp[1] = comp[1];
    mympu.comp[2] = comp[2];

    _c = 0;

    return 0;
}
#endif


int8_t mympu_update() {
/*
	do {
		ret = dmp_read_fifo(gyro,NULL,q._l,NULL,&sensors,&fifoCount);
		// will return:
		//	0 - if ok
		//	1 - no packet available
		//	2 - if BIT_FIFO_OVERFLOWN is set (mpu will reset fifo automatically)
		//	3 - if frame corrupted
		//       <0 - if error
		//

		if (ret!=0) return ret; 
	} while (fifoCount>1);
*/
    sensors = 0;
    ret = dmp_read_fifo(gyro,accel,q._l,NULL,&sensors,&fifoCount);
    if (ret!=0) return ret; 
    if (fifoCount>1) { 
        do { //empty fifo
            ret = dmp_read_fifo(gyro,accel,q._l,NULL,&sensors,&fifoCount);
        } while (fifoCount>1 && ret == 0);
#ifdef DEBUG
        Serial.print("ERR Fifocount: "); Serial.println(fifoCount); 
#endif
        return -128;
    }

	q._f.w = (float)q._l[0] / (float)QUAT_SENS;
	q._f.x = (float)q._l[1] / (float)QUAT_SENS;
	q._f.y = (float)q._l[2] / (float)QUAT_SENS;
	q._f.z = (float)q._l[3] / (float)QUAT_SENS;

	quaternionToEuler( &q._f, &mympu.ypr[2], &mympu.ypr[1], &mympu.ypr[0] );
	/* need to adjust signs and do the wraps depending on the MPU mount orientation */ 
	/* if axis is no centered around 0 but around i.e 90 degree due to mount orientation */
	/* then do:  mympu.ypr[x] = wrap_180(90.f+rad2deg(mympu.ypr[x])); */
	mympu.ypr[0] = -rad2deg(mympu.ypr[0]);
	mympu.ypr[1] = -rad2deg(mympu.ypr[1]);
	mympu.ypr[2] = -shift_180(rad2deg(mympu.ypr[2]));

    static Quaternion qq;
    qq.w=q._f.w;
    qq.x=q._f.x;
    qq.y=q._f.y;
    qq.z=q._f.z;

    VectorInt16 a;
    a.x = (float)accel[0];
    a.y = (float)accel[1];
    a.z = (float)accel[2];
    a.rotate(&qq);
    mympu.accel[0] = (float)a.x/ACCEL_SENS;
    mympu.accel[1] = (float)a.y/ACCEL_SENS;
    mympu.accel[2] = (float)a.z/ACCEL_SENS - mympu.gravity;

	/* need to adjust signs depending on the MPU mount orientation */ 
	mympu.gyro[0] = (float)gyro[2] / GYRO_SENS;
	mympu.gyro[1] = (float)gyro[1] / GYRO_SENS;
	mympu.gyro[2] = (float)gyro[0] / GYRO_SENS;

    _c++;

	return 0;
}

