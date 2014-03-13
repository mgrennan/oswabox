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


# Should not alter anything below this line
###############################################################################

OBJ	=	$(SRC:.c=.o)

BINS	=	$(SRC:.c=)

all:	start libgps.a LED wind rain MCP3008 DHT22 BMP085 \
	rain wind gps-gpsd gps-serial oswabox oswaboxd
	@echo "    $(BINS)" | fmt
	@echo ""

start:
	@echo "Building OSWABox"
	./buildnumber.sh

libgps.a:	gps.o serial.o nmea.o
	@$(AR) -rsc libgps.a gps.o serial.o nmea.o



LED:		LED.o
	@$(CC) -o ${BUILD_DIR}/$@ LED.o $(LDFLAGS) $(LDLIBS)

DHT22:	DHT22.o locking.o
	@$(CC) -o ${BUILD_DIR}/$@ DHT22.o locking.o $(LDFLAGS) $(LDLIBS)

BMP085: BMP085.o bmp085_chip.o
	@$(CC) -o ${BUILD_DIR}/$@ BMP085.o bmp085_chip.o $(LDFLAGS) $(LDLIBS)

rain:		rain.o
	@$(CC) -o ${BUILD_DIR}/$@ rain.o $(LDFLAGS) $(LDLIBS)

wind:		wind.o
	@$(CC) -o ${BUILD_DIR}/$@ wind.o $(LDFLAGS) $(LDLIBS)

gps-gpsd:	gps-gpsd.o
	@$(CC) -o ${BUILD_DIR}/$@ gps-gpsd.o libgps.a $(LDFLAGS) $(LDLIBS)

gps-serial:	gps-serial.o
	@$(CC) -o ${BUILD_DIR}/$@ gps-serial.o libgps.a $(LDFLAGS) $(LDLIBS)

MCP3008:	MCP3008.o
	@$(CC) -o ${BUILD_DIR}/$@ MCP3008.o $(LDFLAGS) $(LDLIBS) 

oswabox:	oswabox.o bmp085_chip.o
	@$(CC) -o ${BUILD_DIR}/$@ oswabox.o libgps.a bmp085_chip.o $(LDFLAGS) $(LDLIBS)

oswaboxd:	oswaboxd.o bmp085_chip.o 
	@$(CC) -o ${BUILD_DIR}/$@ oswaboxd.o bmp085_chip.o $(LDFLAGS) $(LDLIBS)

install:
	sudo cp ${BUILD_DIR}/$(BINS) /usr/bin

.c.o:
	@echo [CC] $<
	@$(CC) -c $(CFLAGS) $< -o $@


# DO NOT DELETE
