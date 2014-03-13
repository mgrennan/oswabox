/*
// rainfall.c
// Mark Grennan - 2014-01-02
//
// based on isr.c from the WiringPi library, authored by Gordon Henderson
// https://github.com/WiringPi/WiringPi/blob/master/examples/isr.c
//
// Compile as follows:
//
//	gcc -o rainfall rainfall.c -lwiringPi
//
// Run as follows:
//
//	sudo ./rainfall
//
*/
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <wiringPi.h>

// Use GPIO Pin 17 Wind Speed, which is Pin 0 in wiringPi
// Use GPIO Pin 18 Rain Guage, which is Pin 1 in wiringPi

#define RAIN_PIN 0

// the event counter
volatile long eventCount = 0;

// -------------------------------------------------------------------------
// myInterrupt:  called every time an event occurs
void myInterrupt(void)
{
    eventCount++;
}


// -------------------------------------------------------------------------
// main
int main(void)
{
    long count ;                                            // Count the minutes run

    if (wiringPiSetup () < 0) {                             // sets up the wiringPi library
        fprintf (stderr, "Unable to setup wiringPi: %s\n", strerror (errno));
        return 1;
    }

    // set Pin 17/0 generate an interrupt on high-to-low transitions
    // and attach myInterrupt() to the interrupt
    if ( wiringPiISR (RAIN_PIN, INT_EDGE_FALLING, &myInterrupt) < 0 ) {
        fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
        return 1;
    }

    count = 0;

    while ( 1 ) {                                           // display counter value every second.
        // event rate is 0.011 inches per event
        printf("Minuts: %04ld %lf Avg: %lf ipm\n", count, eventCount * 0.011, (eventCount * 0.011) / count);
        eventCount = 0;
        delay( 60000 );                                     // wait 60 second

    }

    return 0;
}
