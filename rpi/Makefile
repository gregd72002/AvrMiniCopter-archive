CXX=g++
CXX_OPTS= -Wall -g -O2 -DDEBUG1
CXXFLAGS=$(CXX_OPTS)

CC=cc
CFLAGS=
CC_OPTS=-lstdc++ -lm
LDFLAGS=$(CC_OPTS)
#LDFLAGS=-lpthread -pthread -lstdc++ -lsupc++ 
#LD_OPTS=-lpthread -lrt -lstdc++

INSTALL=install

OBJ_AVRCONTROLLER=avrcontroller.o mpu.o flightlog.o routines.o config.o ps3controller.o
OBJ_AVRSPI=avrspi.o routines.o gpio.o spidev.o
OBJ_AVRSPI_CMD=avrspi_cmd.o routines.o
OBJ_AVRSPI_BUF=avrspi_buf.o routines.o
OBJ_AVRPING=avrping.o routines.o
OBJ_AVRCONFIG=avrconfig.o routines.o config.o mpu.o
OBJ_AVRBARO=avrbaro.o routines.o bmpsensor/libbs.a libs/libi2cdev.a

%.o: %.c                                                                         
	$(CXX) -c $(CXXFLAGS) $(CXX_OPTS) $< -o $@ 

all: _websockify avrcontroller avrspi avrping avrconfig avrspi_cmd avrspi_buf avrbaro _remote

modules:
	$(MAKE) -C spi-bcm2708-dma/

_websockify:
	$(MAKE) -C websockify/

_remote:
	$(MAKE) -C remote/

avrcontroller: $(OBJ_AVRCONTROLLER)
	$(CC) $(CFLAGS) $(OBJ_AVRCONTROLLER) -o avrcontroller $(LDFLAGS) $(CC_OPTS) 

avrconfig: $(OBJ_AVRCONFIG)
	$(CC) $(CFLAGS) $(OBJ_AVRCONFIG) -o avrconfig $(LDFLAGS) $(CC_OPTS) 

avrspi: $(OBJ_AVRSPI)
	$(CC) $(CFLAGS) $(OBJ_AVRSPI) -o avrspi $(LDFLAGS) $(CC_OPTS) 

avrping: $(OBJ_AVRPING)
	$(CC) $(CFLAGS) $(OBJ_AVRPING) -o avrping $(LDFLAGS) $(CC_OPTS) 

avrspi_cmd: $(OBJ_AVRSPI_CMD)
	$(CC) $(CFLAGS) $(OBJ_AVRSPI_CMD) -o avrspi_cmd $(LDFLAGS) $(CC_OPTS) 

avrspi_buf: $(OBJ_AVRSPI_BUF)
	$(CC) $(CFLAGS) $(OBJ_AVRSPI_BUF) -o avrspi_buf $(LDFLAGS) $(CC_OPTS) 

avrbaro: $(OBJ_AVRBARO)
	$(CC) $(CFLAGS) $(OBJ_AVRBARO) -o avrbaro $(LDFLAGS) $(CC_OPTS) 

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
	$(INSTALL) -m 755 avrspi_buf $(DESTDIR)/usr/local/bin/
	$(INSTALL) -m 755 avrbaro $(DESTDIR)/usr/local/bin/
	$(INSTALL) -m 755 avrping $(DESTDIR)/usr/local/bin/
	$(INSTALL) -m 755 avrconfig $(DESTDIR)/usr/local/bin/
	$(INSTALL) -m 755 websockify/websockify $(DESTDIR)/usr/local/bin/
	$(MAKE) -C remote/ install

clean:
	cd bmpsensor && $(MAKE) clean
	cd libs/i2cdev && $(MAKE) clean
	cd websockify && $(MAKE) clean
	cd remote && $(MAKE) clean
	cd spi-bcm2708-dma && $(MAKE) clean
	rm -rf avrcontroller 
	rm -rf avrspi_cmd
	rm -rf avrspi_buf
	rm -rf avrspi 
	rm -rf avrping 
	rm -rf avrconfig 
	rm -rf avrbaro 
	rm -rf *.o *~ *.mod

