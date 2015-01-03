CXX=g++
CXX_OPTS= -Wall -g -O2 -I../ 

CC=cc
CC_OPTS=

INSTALL=install

OBJS=camera_server.o ../routines.o

%.o: %.c                                                                         
	$(CXX) -c $(CXX_OPTS) $< -o $@ 

all: $(OBJS)  
	$(CC) $(CFLAGS) $(OBJS)  -o camera_server $(LDFLAGS) $(CC_OPTS) 

install:
	$(INSTALL) -m 755 camera_server $(DESTDIR)/usr/local/bin/
	$(INSTALL) -m 755 camera_streamer.sh $(DESTDIR)/usr/local/bin/

clean:
	rm -rf camera_server
	rm -rf *.o *~ *.mod

