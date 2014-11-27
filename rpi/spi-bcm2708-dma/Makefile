obj-m := spi-bcm2708-dma.o
KVERSION ?= $(shell uname -r)
KERNEL_SRC ?= /lib/modules/$(KVERSION)/build
PWD := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules

install:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules_install

clean:
	rm *.ko
	rm *.o
	rm *.mod.c
	rm modules.order
	rm Module.symvers
