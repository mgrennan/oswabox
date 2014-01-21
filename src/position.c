/*
//  position.c
//  Mark Grennan - 2014-01-17
//
//  Using the MCP3008 AD converter (ADC) read the Air Quality
//  insterment and print the values.
//
//  based on isr.c from the WiringPi library, authored by Gordon Henderson
//  https://github.com/WiringPi/WiringPi/blob/master/examples/isr.c
//
//  Compile as follows:
//
//      gcc -o position position.c -lwiringPi
//
//  Run as follows:
//
//      sudo ./position
*/
#include <stdio.h>
#include <stdlib.h>
#include "gps.h"

int main(void) {

    printf("Display the GPS information.\n");
    gps_init(); 		// Open serial port

    loc_t data; 		// Create structure

    gps_location(&data);	// Read and parse the GPS info

    printf("Date: 20%02d-%02d-%02d %02d:%02d:%02d Lat: %lf lon: %lf Atl: %lf\n", 
	data.year, data.month, data.day, data.hour, data.minute, data.second, data.latitude, data.longitude, data.altitude);

    return EXIT_SUCCESS;
}

