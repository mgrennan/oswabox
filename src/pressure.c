/*
//  pressure.c
//  Mark Grennan - 2014-01-09
//
//  Read the BMP085 data on the I2C bus and print the current observations.
//
//  Compile as follows:
//
//	gcc -o pressure pressure.c -lwiringPi
//
//  Run as follows:
//
//	sudo ./pressure
//
*/
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <wiringPi.h>

#define currentBMP 102350

int main()
{
	float temp = 24.9;
	float pressure = 95693.0;
	float rawAltitude = 480.17;
	float altitude = 400.72;

	// read current temp
	//    temp = readTemperature();

	// Read the current barometric pressure level
	//    pressure = readPresure();

	// To calculate altitude based on an estimated mean sea level pressure
	// (1013.25 hPa) call the function as follows, but this won't be very accurate
	//    rawAltitude = readAltitude();

	// To specify a more accurate altitude, enter the correct mean sea level
	// pressure level.  For example, if the current pressure level is 1023.50 hPa
	// enter 102350 since we include two decimal places in the integer value
	//    altitude = bmp.readAltitude(currentBMP);

	printf("Temperature:  % 4.2f C  \t= % 4.2f F\n", temp, (temp * 1.8) + 32);
	printf("Pressure:     % 4.2f hPa  \t= % 4.2f inch of mercury\n", pressure / 100.0, (pressure /100) * 0.02953);
	printf("Raw Altitude: % 4.2f m  \t= % 4.2f f\n", rawAltitude, rawAltitude * 3.2808);
	printf("Altitude:     % 4.2f m  \t= % 4.2f f\n", altitude, altitude * 3.2808);
}
