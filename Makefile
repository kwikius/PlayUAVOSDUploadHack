# PlayUAV OSD command line uploader
# for more info visit http://www.playuav.com

# usage make QUAN_ROOT=<path to quan root>
# download quan dependency from https://github.com/kwikius/quan-trunk.git
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>
# 
# Authors
#  Andy Little
#  Tom Ren

APPNAME = playuavosd-util

ifneq ($(MAKECMDGOALS),clean)
ifeq ($(QUAN_ROOT),)
$(error ERROR: to build $(APPNAME) Requires QUAN_ROOT to be set to the path to quan library available \
 at https://github.com/kwikius/quan-trunk. Either export or 'make QUAN_ROOT=<path to quan>')
endif
endif

INCLUDES = $(QUAN_ROOT)
CC = g++
LD = g++

INCLUDE_ARGS = $(patsubst %,-I%,$(INCLUDES))
CFLAGS = -std=c++11 -Wall

local_objects = main.o crc.o params.o osdconn.o

objects = $(local_objects) serial_port.o 

all: $(APPNAME)

$(APPNAME) : $(objects)
	$(LD) $(objects) -o $(APPNAME)
	./$(APPNAME)

$(local_objects) : %.o : %.cpp
	$(CC) $(CFLAGS) $(INCLUDE_ARGS) -c $< -o $@

serial_port.o : $(QUAN_ROOT)/quan_matters/src/serial_port.cpp
	$(CC) $(CFLAGS) $(INCLUDE_ARGS) -c $< -o $@ 
	
clean:
	-rm -rf *.o $(APPNAME)
