RM=/bin/rm
CC=/usr/bin/gcc
BUILD_DIR=./build
APP=${BUILD_DIR}/app
CFLAGS=-g -Wall -O3
MAKE=/usr/bin/make
MKDIR=/bin/mkdir

all:	init
	export LANG=C
	$(MAKE) -C src all

init:
	${MKDIR} -p ${BUILD_DIR}

install:
	$(MAKE) -C src install

clean: 
	$(RM) -Rf $(BUILD_DIR)
	$(RM) -f src/*.o
