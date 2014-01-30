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
LDLIBS    = -lwiringPi -lpthread -lm -lgps

BUILD_DIR = ../build

# Should not alter anything below this line
###############################################################################

OBJ	=	$(SRC:.c=.o)

BINS	=	$(SRC:.c=) nmealib/lib/libnmea.a

all:	libgps install blinkgreen wind rain mcp3008 temperature pressure \
	rain wind gps-serial oswabox
	@echo "    $(BINS)" | fmt
	@echo ""

blinkgreen:	blinkgreen.o
	@$(CC) -o ${BUILD_DIR}/$@ blinkgreen.o $(LDFLAGS) $(LDLIBS)

temperature:	temperature.o locking.o
	@$(CC) -o ${BUILD_DIR}/$@ temperature.o locking.o $(LDFLAGS) $(LDLIBS)

pressure:	pressure.o bmp085.o
	@$(CC) -o ${BUILD_DIR}/$@ pressure.o bmp085.o $(LDFLAGS) $(LDLIBS)

rain:		rain.o
	@$(CC) -o ${BUILD_DIR}/$@ rain.o $(LDFLAGS) $(LDLIBS)

wind:		wind.o
	@$(CC) -o ${BUILD_DIR}/$@ wind.o $(LDFLAGS) $(LDLIBS)


libgps:		gps.o serial.o nmea.o
	${AR} -rsc libgps.a gps.o serial.o nmea.o

gps-gpsd:	gps-gpsd.o
	@$(CC) -o ${BUILD_DIR}/$@ gps-gpsd.o $(LDFLAGS) $(LDLIBS)

gps-serial:	gps-serial.o
	@$(CC) -o ${BUILD_DIR}/$@ gps-serial.o libgps.a $(LDFLAGS) $(LDLIBS)

mcp3008:	mcp3008.o
	@$(CC) -o ${BUILD_DIR}/$@ mcp3008.o $(LDFLAGS) $(LDLIBS)


oswabox:	oswabox.o bmp085.o
	@$(CC) -o ${BUILD_DIR}/$@ oswabox.o bmp085.o $(LDFLAGS) $(LDLIBS)


.c.o:
	@echo [CC] $<
	@$(CC) -c $(CFLAGS) $< -o $@

install:
	sudo cp ./libgps.a /usr/lib/libgps.a


# DO NOT DELETE
