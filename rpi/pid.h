#ifndef PID_H
#define PID_H

struct s_pid {
	int _mode;
	float Kp,Ki,Kd;
	float _KiTerm,_dInput;
	float min,max;
	float _output;
	float value;
	float _err;
        float _lastInput;
};

int pid_init(struct s_pid *pid);

void pid_setmode(struct s_pid *pid,int mode);

int pid_update(struct s_pid *pid, float desired, float actual, float dt_s);


#endif

