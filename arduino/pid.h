#ifndef PID_H
#define PID_H

struct s_pid {
    float Kp,Ki,Kd;
    float _KiTerm,_dInput;
    float max,imax;
    float value;
    float _lastInput,_lastDInput;
#ifdef DEBUG
    float _d;
#endif
};

int pid_init(struct s_pid *pid);

void pid_reset(struct s_pid *pid);

void pid_update(struct s_pid *pid, float input, float dt_s);


#endif

