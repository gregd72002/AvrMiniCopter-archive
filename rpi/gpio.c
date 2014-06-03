#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

/*
 * GPIO user space helpers
 *
 * Copyright 2009 Analog Devices Inc.
 * Michael Hennerich (hennerich@blackfin.uclinux.org)
 *
 * Licensed under the GPL-2 or later
 */

/*
 * GPIO user space helpers
 * The following functions are acting on an "unsigned gpio" argument, which corresponds to the 
 * gpio numbering scheme in the kernel (starting from 0).  
 * The higher level functions use "int pin" to specify the pins with an offset of 1:
 * gpio = pin - 1;
 */

#define GPIO_DIR_IN 0
#define GPIO_DIR_OUT    1
#define PIN_MASK    (UINT_MAX>>1)
#define PIN_INVERSE (~(PIN_MASK))   /* flag for inverted pin in serbb */
#define PIN_MIN     0   /* smallest allowed pin number */
#define PIN_MAX     31  /* largest allowed pin number */

int linuxgpio_export(unsigned int gpio)
{
    int fd, len, r;
    char buf[11];

    fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) {
        perror("Can't open /sys/class/gpio/export");
        return fd;
    }

    len = snprintf(buf, sizeof(buf), "%d", gpio);
    r = write(fd, buf, len);
    close(fd);

    return r;
}

int linuxgpio_unexport(unsigned int gpio)
{
    int fd, len, r;
    char buf[11];

    fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (fd < 0) {
        perror("Can't open /sys/class/gpio/unexport");
        return fd;
    }

    len = snprintf(buf, sizeof(buf), "%d", gpio);
    r = write(fd, buf, len);
    close(fd);

    return r;
}

int linuxgpio_openfd(unsigned int gpio)
{
    char filepath[60];

    snprintf(filepath, sizeof(filepath), "/sys/class/gpio/gpio%d/value", gpio);
    return (open(filepath, O_RDWR));
}

int linuxgpio_dir(unsigned int gpio, unsigned int dir)
{
    int fd, r;
    char buf[60];

    snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/direction", gpio);

    fd = open(buf, O_WRONLY);
    if (fd < 0) {
        printf("/sys/class/gpio/gpio%d/direction\n",gpio);
        printf("/sys/class/gpio/gpio%i/direction\n",gpio);
        perror("Can't open gpioX/direction");
        return fd;
    }

    if (dir == GPIO_DIR_OUT)
        r = write(fd, "out", 4);
    else
        r = write(fd, "in", 3);

    close(fd);

    return r;
}

int linuxgpio_dir_out(unsigned int gpio)
{
    return linuxgpio_dir(gpio, GPIO_DIR_OUT);
}

int linuxgpio_dir_in(unsigned int gpio)
{
    return linuxgpio_dir(gpio, GPIO_DIR_IN);
}

/*
 * End of GPIO user space helpers
 */

#define N_GPIO (PIN_MAX + 1)

/*
 * an array which holds open FDs to /sys/class/gpio/gpioXX/value for all needed pins
 */
static int linuxgpio_fds[N_GPIO] ;


int linuxgpio_setpin(int pin, int value)
{
    int r;

    if (pin & PIN_INVERSE)
    {
        value  = !value;
        pin   &= PIN_MASK;
    }

    if ( linuxgpio_fds[pin] < 0 )
        return -1;

    if (value)
        r = write(linuxgpio_fds[pin], "1", 1);
    else
        r = write(linuxgpio_fds[pin], "0", 1);

    if (r!=1) return -1;

    return 0;
}

int linuxgpio_getpin(int pin)
{
    unsigned char invert=0;
    char c;

    if (pin & PIN_INVERSE)
    {
        invert = 1;
        pin   &= PIN_MASK;
    }

    if ( linuxgpio_fds[pin] < 0 )
        return -1;

    if (lseek(linuxgpio_fds[pin], 0, SEEK_SET)<0)
        return -1;

    if (read(linuxgpio_fds[pin], &c, 1)!=1)
        return -1;

    if (c=='0')
        return 0+invert;
    else if (c=='1')
        return 1-invert;
    else
        return -1;

}

int linuxgpio_highpulsepin(int pin)
{

    if ( linuxgpio_fds[pin & PIN_MASK] < 0 )
        return -1;

    linuxgpio_setpin(pin, 1);
    linuxgpio_setpin(pin, 0);

    return 0;
}

int linuxgpio_initpin(int p) {
    int r, i, pin;

    for (i=0; i<N_GPIO; i++)
        linuxgpio_fds[i] = -1;

    r = linuxgpio_export(p);
    if (r<0) {
        fprintf(stderr, "Can't export GPIO %d, already exported/busy?: %s",
                p, strerror(errno));
        return r;
    }
    r=linuxgpio_dir_out(p);
    if (r<0) {
        fprintf(stderr, "Can't set pin direction to out: GPIO %d, already exported/busy?: %s",
                p, strerror(errno));
        return r;
    }

    if ((linuxgpio_fds[p]=linuxgpio_openfd(p)) < 0) {
        printf("?????\n");
        return linuxgpio_fds[p];
    }
    return 0;
}

void linuxgpio_close()
{
    int i, reset_pin;

    for (i=0; i<N_GPIO; i++) {
        if (linuxgpio_fds[i]==-1) continue;
        close(linuxgpio_fds[i]);
        linuxgpio_dir_in(i);
        //if (i==25) linuxgpio_dir_out(i);
        linuxgpio_unexport(i);
    }
    /*
    //configure RESET as input, if there's external pull up it will go high
    if (linuxgpio_fds[reset_pin] >= 0) {
    close(linuxgpio_fds[reset_pin]);
    linuxgpio_dir_in(reset_pin);
    linuxgpio_unexport(reset_pin);
    }
    */
}

