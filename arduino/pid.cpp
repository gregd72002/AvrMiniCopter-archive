/* Based on: http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/ */

#include "pid.h"

#define PI 3.141592653589793f
static float _d_lpf_alpha = 0.0f;

int pid_init(struct s_pid *pid) {
    pid->Kp=pid->Ki=pid->Kd=0.0f;
    pid->max=pid->imax=0.0f;
    pid->value = 0.0f;
    pid->_lastInput = pid->_lastDInput = 0.0f;
    pid->_KiTerm = 0.0f;
    pid->_dInput = 0.0f;

    float rc = 1/(2*PI*20);
    _d_lpf_alpha = 0.05f / (0.05f + rc);

    return 0;
}

void pid_reset(struct s_pid *pid) {
    pid->_KiTerm = pid->value;
    if (pid->_KiTerm>pid->imax) pid->_KiTerm=pid->imax;
    else if (pid->_KiTerm<-pid->imax) pid->_KiTerm=-pid->imax;
}

void pid_update(struct s_pid *pid, float input, float dt_s) {
    pid->_KiTerm += (pid->Ki * input * dt_s);
    if (pid->_KiTerm>pid->imax) pid->_KiTerm=pid->imax;
    else if (pid->_KiTerm<-pid->imax) pid->_KiTerm=-pid->imax;

    pid->_dInput = input - (pid->_lastInput) / dt_s;
    pid->_dInput = pid->_lastDInput + _d_lpf_alpha * (pid->_dInput - pid->_lastDInput);
    
    pid->value = input*pid->Kp + pid->_KiTerm - pid->_dInput*pid->Kd;

    if (pid->value>pid->max) pid->value=pid->max;
    if (pid->value<-pid->max) pid->value=-pid->max;
    
    pid->_lastInput = input;
    pid->_lastDInput = pid->_dInput;
}

