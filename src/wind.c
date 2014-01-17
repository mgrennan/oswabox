/*
//  windspeed.c
//  Mark Grennan - 2014-01-02
//
// based on isr.c from the WiringPi library, authored by Gordon Henderson
// https://github.com/WiringPi/WiringPi/blob/master/examples/isr.c
//
// Compile as follows:
//
//	gcc -o windspeed windspeed.c -lwiringPi
//
//  Run as follows:
//
//	sudo ./windspeed
//
 */
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <wiringPi.h>

// Use GPIO Pin 17 Wind Speed, which is Pin 0 in wiringPi
// Use GPIO Pin 18 Rain Guage, which is Pin 1 in wiringPi

#define WIND_PIN 0

// the event counter
volatile int eventCounter = 0;

// -------------------------------------------------------------------------
// myInterrupt:  called every time an event occurs
void myInterrupt(void)
{
	eventCounter++;
}


// -------------------------------------------------------------------------
// main
int main(void)
{
	// sets up the wiringPi library
	if (wiringPiSetup () < 0)
	{
		fprintf (stderr, "Unable to setup wiringPi: %s\n", strerror (errno));
		return 1;
	}

	// set Pin 17/0 generate an interrupt on high-to-low transitions
	// and attach myInterrupt() to the interrupt
	if ( wiringPiISR (WIND_PIN, INT_EDGE_FALLING, &myInterrupt) < 0 )
	{
		fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
		return 1;
	}

	// display counter value every second.
	while ( 1 )
	{
								 /* Wind is 1.492 mph per event */
		printf( "%4.2f\n", eventCounter * 1.492 );
		eventCounter = 0;
		delay( 1000 );			 // wait 1 second
	}

	return 0;
}
