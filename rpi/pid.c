/* Based on: http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/ */

#include "pid.h"

int pid_init(struct s_pid *pid) {
	pid->Kp=pid->Ki=pid->Kd=0.0f;
	pid->max=pid->min=0.0f;
	pid->_output = 0.0f;
	pid->value = 0.0f;
	pid->_err=0.0f;
	pid->_lastInput = 0.0f;
	pid->_mode = 0;
	pid->_KiTerm = 0.0f;
	pid->_dInput = 0.0f;
	return 0;
}

void pid_reset(struct s_pid *pid) {
	pid->_KiTerm = pid->_output;
	if (pid->_KiTerm>pid->max) pid->_KiTerm=pid->max;
	else if (pid->_KiTerm<pid->min) pid->_KiTerm=pid->min;
}

void pid_setmode(struct s_pid *pid, int mode) {
	if (pid->_mode==0 && mode==1) {
		pid_reset(pid);
	}
	pid->_mode = mode;
}

int pid_update(struct s_pid *pid, float desired, float actual, float dt_s) {
	if (!pid->_mode) {
		//pid->value = (desired-actual)*pid->Kp;
		return 0;
	}
	pid->_err = desired-actual;

	pid->_KiTerm += (pid->Ki * pid->_err * dt_s);
	if (pid->_KiTerm>pid->max) pid->_KiTerm=pid->max;
	else if (pid->_KiTerm<pid->min) pid->_KiTerm=pid->min;
	
	pid->_dInput = actual - pid->_lastInput;

	pid->_output = pid->_err*pid->Kp + pid->_KiTerm - pid->_dInput*pid->Kd/dt_s;
	if (pid->_output>pid->max) pid->_output=pid->max;
	if (pid->_output<pid->min) pid->_output=pid->min;
	
	pid->value = pid->_output;
	pid->_lastInput = actual;
	
	return 0;
}

