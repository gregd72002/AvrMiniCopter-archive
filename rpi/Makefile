CXX=g++
CXX_OPTS= -Wall -g -O2 -DDEBUG
CXXFLAGS=$(CXX_OPTS)

CC=cc
CFLAGS=
CC_OPTS=-lstdc++ -lm
LDFLAGS=$(CC_OPTS)
#LDFLAGS=-lpthread -pthread -lstdc++ -lsupc++ 
#LD_OPTS=-lpthread -lrt -lstdc++

INSTALL=install

OBJ_AVRCONTROLLER=avrcontroller.o mpu.o flightlog.o bmpsensor/libbs.a libs/libi2cdev.a routines.o config.o ps3controller.o
OBJ_AVRSPI=avrspi.o routines.o gpio.o spidev.o
OBJ_AVRSPI_CMD=avrspi_cmd.o routines.o

%.o: %.c                                                                         
	$(CXX) -c $(CXXFLAGS) $(CXX_OPTS) $< -o $@ 

all: avrcontroller avrspi avrspi_cmd

avrcontroller: $(OBJ_AVRCONTROLLER)
	$(CC) $(CFLAGS) $(OBJ_AVRCONTROLLER) -o avrcontroller $(LDFLAGS) $(CC_OPTS) 

avrspi: $(OBJ_AVRSPI)
	$(CC) $(CFLAGS) $(OBJ_AVRSPI) -o avrspi $(LDFLAGS) $(CC_OPTS) 

avrspi_cmd: $(OBJ_AVRSPI_CMD)
	$(CC) $(CFLAGS) $(OBJ_AVRSPI_CMD) -o avrspi_cmd $(LDFLAGS) $(CC_OPTS) 

receiver/libreceiver.a:
	$(MAKE) -C receiver/ 

bmpsensor/libbs.a:
	$(MAKE) -C bmpsensor/ 

libs/libi2cdev.a:
	$(MAKE) -C libs/i2cdev

install:
	$(INSTALL) -m 0755 -d $(DESTDIR)/usr/local/bin
	$(INSTALL) -m 755 avrcontroller $(DESTDIR)/usr/local/bin/
	$(INSTALL) -m 755 avrspi $(DESTDIR)/usr/local/bin/
	$(INSTALL) -m 755 avrspi_cmd $(DESTDIR)/usr/local/bin/

clean:
	cd bmpsensor && $(MAKE) clean
	cd libs/i2cdev && $(MAKE) clean
	rm -rf avrcontroller 
	rm -rf avrspi_cmd
	rm -rf avrspi 
	rm -rf *.o *~ *.mod
