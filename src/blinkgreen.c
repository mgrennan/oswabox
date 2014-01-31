/*
//  blinkgreen.c
//  Mark Grennan - 2014-01-14
//
//  Using the MCP3008 AD converter (ADC) read the Air Quality
//  insterment and print the values.
//
//  based on isr.c from the WiringPi library, authored by Gordon Henderson
//  https://github.com/WiringPi/WiringPi/blob/master/examples/isr.c
//
//  Compile as follows:
//
//      gcc -o blinkgreen blinkgreen.c -lwiringPi
//
//  Run as follows:
//
//      sudo ./blinkgreen
*/
#include <stdio.h>
#include <wiringPi.h>

#define LED 3

int main (void)
{
	wiringPiSetup () ;
	pinMode (LED, OUTPUT) ;

	for (;;)
	{
		digitalWrite (LED, HIGH) ;	// On
		printf("On\n");
		delay (500) ;			// mS
		digitalWrite (LED, LOW) ;	// Off
		printf("Off\n");
		delay (500) ;
	}
	return 0 ;
}
