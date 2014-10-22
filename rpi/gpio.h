#ifndef GPIO_H
#define GPIO_H

int linuxgpio_export(unsigned int gpio);

int linuxgpio_unexport(unsigned int gpio);

int linuxgpio_openfd(unsigned int gpio);

int linuxgpio_dir(unsigned int gpio, unsigned int dir);

int linuxgpio_dir_out(unsigned int gpio);

int linuxgpio_dir_in(unsigned int gpio);

int linuxgpio_setpin(int pin, int value);

int linuxgpio_getpin(int pin);

int linuxgpio_highpulsepin(int pin, int delay);

int linuxgpio_initpin(int p);

void linuxgpio_close();

#endif
