#
# Makefile:
#	wiringPi - Wiring Compatable library for the Raspberry Pi
#	https://projects.drogon.net/wiring-pi
#
#	Copyright (c) 2012 Gordon Henderson
#################################################################################
# This file is part of wiringPi:
#	Wiring Compatable library for the Raspberry Pi
#
#    wiringPi is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    wiringPi is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
#################################################################################


#DEBUG	  = -g -O0
DEBUG	  = -O3
CC	  = gcc
INCLUDE	  = -I/usr/local/include
CFLAGS	  = $(DEBUG) -Wall $(INCLUDE) -Winline -pipe
AR	  = ar

LDFLAGS	  = -L/usr/local/lib
LDLIBS    = -lwiringPi -lwiringPiDev -lpthread -lm -lnmea -lgps

BUILD_DIR = ../build

# Should not alter anything below this line
###############################################################################

OBJ	=	$(SRC:.c=.o)

BINS	=	$(SRC:.c=) nmealib/lib/libnmea.a

all:	blinkgreen wind rain mcp3008 temp pressure airqual rain wind libgps position
	@echo "    $(BINS)" | fmt
	@echo ""

blinkgreen:	blinkgreen.o
	@echo [link]
	@$(CC) -o ${BUILD_DIR}/$@ blinkgreen.o $(LDFLAGS) $(LDLIBS)

mcp3008:	mcp3008.o
	@echo [link]
	@$(CC) -o ${BUILD_DIR}/$@ mcp3008.o $(LDFLAGS) $(LDLIBS)

temp:	temp.o
	@echo [link]
	@$(CC) -o ${BUILD_DIR}/$@ temp.o $(LDFLAGS) $(LDLIBS)

pressure:	pressure.o
	@echo [link]
	@$(CC) -o ${BUILD_DIR}/$@ pressure.o $(LDFLAGS) $(LDLIBS)

airqual:	airqual.o
	@echo [link]
	@$(CC) -o ${BUILD_DIR}/$@ airqual.o $(LDFLAGS) $(LDLIBS)

rain:		rain.o
	@echo [link]
	@$(CC) -o ${BUILD_DIR}/$@ rain.o $(LDFLAGS) $(LDLIBS)

wind:		wind.o
	@echo [link]
	@$(CC) -o ${BUILD_DIR}/$@ wind.o $(LDFLAGS) $(LDLIBS)

libgps:	
	${CC} ${CFLAGS} -I. -c -o gps.o gps.c
	${CC} ${CFLAGS} -I. -c -o serial.o serial.c
	${CC} ${CFLAGS} -I. -c -o nmea.o nmea.c
	${AR} -rsc libgps.a gps.o serial.o nmea.o

position:		position.o
	@echo [link]
	@$(CC) -o ${BUILD_DIR}/$@ position.o $(LDFLAGS) $(LDLIBS)

.c.o:
	@echo [CC] $<
	@$(CC) -c $(CFLAGS) $< -o $@

install:
	sudo cp ${BUILD_DIR}/libgps.a /usr/lib/libgps.a
	sudo cp /${BUILD_DIR}/gps.h /usr/include/gps.h

clean:
	@echo "[Clean]"
	@rm -f $(OBJ) *~ core tags $(BINS)

tags:	$(SRC)
	@echo [ctags]
	@ctags $(SRC)

depend:
	makedepend -Y $(SRC)


# DO NOT DELETE
