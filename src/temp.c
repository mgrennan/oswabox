/*
//  temp.c:
//  Mark Grennan - 2014-01-14
//
//  Read the RHT03 tempiture in Centagrade and the 
//  humidity.
//
//  Compile as follows:
//
//      gcc -o temp temp.c -lwiringPi
//
//  Run as follows:
//
//      sudo ./temp
*/
#include <stdio.h>
#include <wiringPi.h>
#include <maxdetect.h>

#define RHT03_PIN   7

/*
 ***********************************************************************
 * The main program
 ***********************************************************************
 */
int main (void)
{
	int temp, rh ;
	int newTemp, newRh ;
	float dp ;

	temp = rh = newTemp = newRh = 0 ;

	wiringPiSetup () ;
	piHiPri       (55) ;

	sleep (1) ;

	readRHT03 (RHT03_PIN, &newTemp, &newRh) ;

	if ((temp != newTemp) || (rh != newRh))
	{
		temp = newTemp ;
		rh   = newRh ;

		printf ("Temperature: %5.1f c = %5.2f f\n", temp / 10.0, ((temp /10.0) * 1.8) + 32) ;
		printf ("   Humidity: %5.1f%%\n", rh / 10.0, (rh / 10.0) * 3.2808) ;
	}

	return 0 ;
}
