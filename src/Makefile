#
# Makefile:
#

#DEBUG	  = -g -O0
DEBUG	  = -O3
CC	  = gcc
INCLUDE	  = -I/usr/local/include
CFLAGS	  = $(DEBUG) -Wall $(INCLUDE) -Winline -pipe
AR	  = ar

LDFLAGS	  = -L/usr/local/lib
LDLIBS    = -lwiringPi -lpthread -lwiringPiDev -lm -lgps

BUILD_DIR = ../build

# Name of text file containing build number.
BUILD_NUMBER_FILE=build-number.txt

# Should not alter anything below this line
###############################################################################

OBJ	=	$(SRC:.c=.o)

BINS	=	$(SRC:.c=)

all:	start libgps.a blink wind rain mcp3008 temperature pressure \
	rain wind gps-gpsd gps-serial oswabox oswaboxd
	@echo "    $(BINS)" | fmt
	@echo ""

start:
	@echo "Building OSWABox"

libgps.a:	gps.o serial.o nmea.o
	@$(AR) -rsc libgps.a gps.o serial.o nmea.o



blink:		blink.o
	@$(CC) -o ${BUILD_DIR}/$@ blink.o $(LDFLAGS) $(LDLIBS)

temperature:	temperature.o locking.o
	@$(CC) -o ${BUILD_DIR}/$@ temperature.o locking.o $(LDFLAGS) $(LDLIBS)

pressure:	pressure.o bmp085.o
	@$(CC) -o ${BUILD_DIR}/$@ pressure.o bmp085.o $(LDFLAGS) $(LDLIBS)

rain:		rain.o
	@$(CC) -o ${BUILD_DIR}/$@ rain.o $(LDFLAGS) $(LDLIBS)

wind:		wind.o
	@$(CC) -o ${BUILD_DIR}/$@ wind.o $(LDFLAGS) $(LDLIBS)

gps-gpsd:	gps-gpsd.o
	@$(CC) -o ${BUILD_DIR}/$@ gps-gpsd.o libgps.a $(LDFLAGS) $(LDLIBS)

gps-serial:	gps-serial.o
	@$(CC) -o ${BUILD_DIR}/$@ gps-serial.o libgps.a $(LDFLAGS) $(LDLIBS)

mcp3008:	mcp3008.o
	@$(CC) -o ${BUILD_DIR}/$@ mcp3008.o $(LDFLAGS) $(LDLIBS)

oswabox:	oswabox.o bmp085.o
	@$(CC) -o ${BUILD_DIR}/$@ oswabox.o libgps.a bmp085.o $(LDFLAGS) $(LDLIBS)

oswaboxd:	oswaboxd.o bmp085.o 
	@$(CC) -o ${BUILD_DIR}/$@ oswaboxd.o bmp085.o $(LDFLAGS) $(LDLIBS) $(BUILD_NUMBER_LDFLAGS)

install:
	sudo cp ${BUILD_DIR}/$(BINS) /usr/bin

.c.o:
	@echo [CC] $<
	@$(CC) -c $(CFLAGS) $< -o $@

# Include build number rules.
#include ./buildnumber.mak

# DO NOT DELETE
