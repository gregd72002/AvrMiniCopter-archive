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

OBJ_PS3CONTROLLER=ps3controller.o routines.o ps3config.o ps3dev.o
OBJ_AVRSPI=avrspi.o routines.o gpio.o spidev.o avrconfig.o flightlog.o
OBJ_AVRSPI_CMD=avrspi_cmd.o routines.o
OBJ_AVRBARO=avrbaro.o routines.o bmpsensor/libbs.a libs/libi2cdev.a

%.o: %.c                                                                         
	$(CXX) -c $(CXXFLAGS) $(CXX_OPTS) $< -o $@ 

all: _websockify ps3controller avrspi avrspi_cmd avrbaro _remote

#modules:
#	$(MAKE) -C spi-bcm2708-dma/

_websockify:
	$(MAKE) -C websockify/

_remote:
	$(MAKE) -C remote/

ps3controller: $(OBJ_PS3CONTROLLER)
	$(CC) $(CFLAGS) $(OBJ_PS3CONTROLLER) -o ps3controller $(LDFLAGS) $(CC_OPTS) 

avrspi: $(OBJ_AVRSPI)
	$(CC) $(CFLAGS) $(OBJ_AVRSPI) -o avrspi $(LDFLAGS) $(CC_OPTS) 

avrspi_cmd: $(OBJ_AVRSPI_CMD)
	$(CC) $(CFLAGS) $(OBJ_AVRSPI_CMD) -o avrspi_cmd $(LDFLAGS) $(CC_OPTS) 

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
	$(INSTALL) -m 755 ps3controller $(DESTDIR)/usr/local/bin/
	$(INSTALL) -m 755 avrspi $(DESTDIR)/usr/local/bin/
	$(INSTALL) -m 755 avrspi_cmd $(DESTDIR)/usr/local/bin/
	$(INSTALL) -m 755 avrbaro $(DESTDIR)/usr/local/bin/
	$(INSTALL) -m 755 websockify/websockify $(DESTDIR)/usr/local/bin/
	$(MAKE) -C remote/ install

clean:
	cd bmpsensor && $(MAKE) clean
	cd libs/i2cdev && $(MAKE) clean
	cd websockify && $(MAKE) clean
	cd remote && $(MAKE) clean
#	cd spi-bcm2708-dma && $(MAKE) clean
	rm -rf ps3controller 
	rm -rf avrspi_cmd
	rm -rf avrspi 
	rm -rf avrbaro 
	rm -rf *.o *~ *.mod

