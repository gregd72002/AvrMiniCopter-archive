#ifndef _BMP_INT_H_
#define _BMP_INT_H_


struct s_bs {
	float t;
	float p;
	float p0;
	float alt;
};

extern struct s_bs bs;

extern int bs_open();
extern int bs_reset();
extern int bs_update(unsigned long t_ms);
extern int bs_close();

#endif 
