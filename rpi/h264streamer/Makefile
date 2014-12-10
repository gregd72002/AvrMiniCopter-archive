CXX=g++
AR=ar
CXXFLAGS=-Wall -g -O2 

all: h264streamer 

OBJS=h264streamer.o ../routines.o

%.o: %.c
	$(CXX) -c $(CXXFLAGS) -I../ $< -o $@ 

h264streamer: $(OBJS) 
	$(CC) $(CFLAGS) $(OBJS) -o h264streamer $(LDFLAGS) $(CC_OPTS)

clean:
	rm -f h264streamer *.o
