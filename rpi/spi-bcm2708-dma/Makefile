obj-m := spi-bcm2708-dma.o
KVERSION ?= $(shell uname -r)
KERNEL_SRC ?= /lib/modules/$(KVERSION)/build
PWD := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules

install:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules_install

clean:
	rm -rf *.ko
	rm -rf *.o
	rm -rf *.mod.c
	rm -rdf .tmp_versions
	rm -rf .spi*
	rm -rf modules.order
	rm -rf Module.symvers
