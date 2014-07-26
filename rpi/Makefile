CXX=g++
CXXFLAGS= -Wall -g -O2 -DDEBUG
CXX_OPTS= -Wall -g -O2 -DDEBUG
LDFLAGS=-lpthread -pthread
LD_OPTS=-lpthread -lrt

PROG=avrcontroller

INSTALL=install

%.o: %.c                                                                         
	$(CXX) -c $(CXXFLAGS) $(CXX_OPTS) $< -o $@ 


all: controller.o avrspi.o 
	$(CXX) $(CXXFLAGS) $(CXX_OPTS) $(LD_OPTS) -o $(PROG) \
		main.c \
		config.o \
		ps3controller.o \
		gpio.o \
		mpu.o \
		flightlog.o \
		bmpsensor/libbs.a \
		libs/libi2cdev.a \
		spidev.o
	$(CXX) $(CXXFLAGS) $(CXX_OPTS) $(LD_OPTS) -o avrspi \
		avrspi.o
	

controller.o: mpu.o flightlog.o bmpsensor/libbs.a libs/libi2cdev.a spidev.o config.o gpio.o ps3controller.o 

speedcontroller/libspeedcontroller.a:
	$(MAKE) -C speedcontroller/ 
	
motionsensor/libmotionsensor.a:
	$(MAKE) -C motionsensor/ 

receiver/libreceiver.a:
	$(MAKE) -C receiver/ 

bmpsensor/libbs.a:
	$(MAKE) -C bmpsensor/ 

libs/libi2cdev.a:
	$(MAKE) -C libs/i2cdev

install:
	$(INSTALL) -m 0755 -d $(DESTDIR)/usr/local/bin
	$(INSTALL) -m 755 $(PROG) $(DESTDIR)/usr/local/bin/

clean:
	cd bmpsensor && $(MAKE) clean
	cd libs/i2cdev && $(MAKE) clean
	rm -rf *.o *~ *.mod
	rm -rf $(PROG)
