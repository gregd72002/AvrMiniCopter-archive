#include <linux/joystick.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <error.h>

#include "ps3controller.h"
#include "config.h"


static struct js_event js_e[0xff];
static int fd = 0;
static int ret;
static int i;
static int throttle_threshold;
static int nc=0;
struct s_rec rec;

static int map(int x, int in_min, int in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int rec_open() {
	fd = open ("/dev/input/js0", O_RDONLY | O_NONBLOCK | O_NOATIME);	
	if (fd < 0) {
		printf("can't open js0: [%i] [%s]\n", fd, strerror(errno));
		return -1;
	}
	throttle_threshold = config.rec[4] - config.rec[3];
    rec.yprt[0] = rec.yprt[1] = rec.yprt[2] = rec.yprt[3] = 0.0f;
    rec.aux=0;
	return rec_update();
}

int process_jsevent(struct js_event *e) {
	if ((e->type & JS_EVENT_INIT)==JS_EVENT_INIT) {
		//printf("JS buffer full?\n");	
		return 0;
	}

	if (e->type==JS_EVENT_BUTTON && e->value) {
		//printf("Button pressed: %2u value: %4i\n",e->number,e->value);
		rec.aux = e->number;
        nc = 1;
	}
    else if ((e->type==JS_EVENT_AXIS) && (e->number<4)) {
		//printf("A %2u VAL: %4i\n",e->number,e->value);
		switch(e->number) {
			case 0: rec.yprt[0] = map(e->value,-32767,32767,config.rec[0],-config.rec[0]); nc=1; break;
			case 1: rec.yprt[3] = map(e->value,-32767,32767,config.rec[4],config.rec[4]-2*throttle_threshold); nc=1; break;
			case 2: rec.yprt[2] = map(e->value,-32767,32767,config.rec[2],-config.rec[2]); nc=1; break;
			case 3: rec.yprt[1] = map(e->value,-32767,32767,config.rec[1],-config.rec[1]); nc=1; break;
		}
	}
	return 0;	
}

int rec_update() {
	ret = read (fd, js_e, sizeof(js_e));
	if (ret<0 && errno != EAGAIN) {
		printf("Error reading js0: [%i] [%s]\n",errno,strerror(errno));
		return -1;
	} 
	if (ret>0) {
        nc = 0;
		for (i=0;i<ret/sizeof(struct js_event);i++)
		    process_jsevent(&js_e[i]);
		if (nc) return 1;
	}	
	
	return 0;
}

int rec_close() {
	close(fd);
	return 0;
}

