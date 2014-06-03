CXX=g++
AR=ar
CXXCFLAGS=-Wall -g -O2
CXX_OPTS=-c
LDFLAGS= -lm -lrt

LIB=../libi2cdev.a
OBJ=i2cdev.o

%.o: %.c                                                                         
	$(CXX) $(CXXFLAGS) $(CXX_OPTS) $< -o $@ 

$(LIB): $(OBJ)
	$(AR) rcs $(LIB) $(OBJ)

all: $(LIB)

clean:
	rm -rf *.o
	rm -rf $(LIB)
