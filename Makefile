
INCLUDES = $(QUAN_ROOT)
CC = g++
LD = g++

APPNAME = playuavosd-util

INCLUDE_ARGS = $(patsubst %,-I%,$(INCLUDES))
CFLAGS = -std=c++11 -Wall

local_objects = main.o crc.o

objects = $(local_objects) serial_port.o 

all: $(APPNAME)

$(APPNAME) : $(objects)
	$(LD) $(objects) -o $(APPNAME)

$(local_objects) : %.o : %.cpp
	$(CC) $(CFLAGS) $(INCLUDE_ARGS) -c $< -o $@

serial_port.o : $(QUAN_ROOT)/quan_matters/src/serial_port.cpp
	$(CC) $(CFLAGS) $(INCLUDE_ARGS) -c $< -o $@ 
	
clean:
	-rm -rf *.o *.exe
