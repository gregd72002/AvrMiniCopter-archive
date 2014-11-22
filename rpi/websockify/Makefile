CXX=g++
AR=ar
CXXFLAGS=-Wall -g -O2
CXX_OPTS=-c -DBMP_DEBUGO

all: websockify 

websockify: websockify.o websocket.o b64_ntop.o b64_pton.o
	$(CC) $(LDFLAGS) $^ -lssl -lcrypto -lresolv -o $@

websockify1: websockify.o websocket.o
	$(CC) $(LDFLAGS) $^ -lssl -lcrypto -lresolv -o $@

b64_ntop.o: b64_ntop.c 
b64_pton.o: b64_pton.c 
websocket.o: websocket.c websocket.h
websockify.o: websockify.c websocket.h


clean:
	rm -f websockify *.o
