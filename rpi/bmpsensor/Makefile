CXX=g++
AR=ar
CXXFLAGS=-Wall -g -O2
CXX_OPTS=-I../libs/ -c -DBMP_DEBUGO

SOURCES=bmp180.c
OBJECTS=$(SOURCES:.c=.o)

LIB=libbs.a

%.o: %.c
	$(CXX) $(CXXFLAGS) $(CXX_OPTS) $< -o $@ 
	

$(LIB): $(OBJECTS) 
	$(AR) rcs $(LIB) $(OBJECTS) 

all: $(LIB)

clean:
	rm -rf *.o
	rm -rf $(LIB)
