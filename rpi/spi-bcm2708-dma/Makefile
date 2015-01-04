obj-m := spi-bcm2708-dma.o
KVERSION ?= $(shell uname -r)
KERNEL_SRC ?= /lib/modules/$(KVERSION)/build
PWD := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules

install:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules_install

clean:
	rm -f *.ko
	rm -f *.o
	rm -f *.mod.c
	#rm -rdf .*
	rm -f modules.order
	rm -f Module.symvers
